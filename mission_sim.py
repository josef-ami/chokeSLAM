"""
A simulated robot for the mission (checkpoint F2): the dashboard's mission mock,
and the field of test_run_control.py.

    World        a rulebook layout (layouts.py, parking lot included), the car's
                 true pose (kinematic bicycle at the rear axle, steering and
                 speed lag) and its sensors: $IMU samples (simulation.SimStm32
                 through the real parser), LIDAR frames (one revolution at the
                 current pose, stamped as lidar_source stamps them) and camera
                 frames (camera_sim).
    FirmwareModel  the drive firmware's logic in Python, for the mock only:
                 DRIVE and PARAM frames (drive_protocol.h byte formats), the run
                 states (READY / RUNNING / STOPPED / FINISHED, the start button),
                 the 250 ms watchdog, the $PAR echo. The C++ original is what
                 test_drive_firmware.py and test_run_control.py test; this copy
                 mirrors it so the dashboard can run without a compiler.
    MissionSim   World + FirmwareModel in real time (optionally faster) on a
                 background thread, offering the same surfaces the Pi program
                 uses on the robot:
                     .link    drain(), status(), write(), is_alive()  (stm32_link.Stm32Link)
                     .lidar   get_latest_scan(), get_latest_scan_timed(), status(), timing_status()
                     .camera  get_latest_frame(), status()
                 plus press() (the start button), carry_to_start() (a person
                 carrying the car back to its start zone) and truth() for the
                 dashboard's overlay.
The run_control.RunSupervisor and the Mission run unchanged on top of it.
"""
from __future__ import annotations

import math
import random
import struct
import threading
import time

import numpy as np

import camera_sim
import config
import field_map as fm
import lane_frame as lf
import layouts
import seat_occupancy as so
import simulation as sim
from drive_link import fw_wanted
from stm32_link import ImuSample, LinkStats, parse_line

WATCHDOG_S = 0.25
RUN_READY, RUN_RUNNING, RUN_STOPPED, RUN_FINISHED = 0, 1, 2, 3


class World:
    """A rulebook layout, the car's true pose, and the simulated sensors."""
    def __init__(self, layout, seed):
        self.lay, self.rng = layout, random.Random(seed)
        d = layout.direction
        self.pillars, self.barriers = [], []
        for k in range(4):
            sec = layout.slot_section(k)
            for i, col in layout.pillars.get(sec, []):
                s = so.seats()[i]
                self.pillars.append(sim.Pillar(*lf.lane_to_global(sec, d, s.x_mm, s.y_mm), col))
        for (x0, y0, x1, y1) in fm.parking_barriers(d):
            a = lf.lane_to_global(layout.start_section, d, x0, y0)
            b = lf.lane_to_global(layout.start_section, d, x1, y1)
            self.barriers.append((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
        self.place_at_start()
        self.dist, self.v, self.delta = 0.0, 0.0, 0.0
        self.stm = sim.SimStm32(self.rng, ticks_per_cm=config.ENCODER_TICKS_PER_CM, yaw_noise_deg=0.0,
                                yaw_drift_deg_s=0.0)
        self.cam_rng = np.random.default_rng(seed)
        self.path = []

    def start_pose(self):
        d, sec = self.lay.direction, self.lay.start_section
        gx, gy = lf.lane_to_global(sec, d, self.lay.start_x, self.lay.start_y)
        return gx, gy, lf.yaw_to_heading(0.0, sec, d)

    def place_at_start(self):
        self.gx, self.gy, self.brg = self.start_pose()

    def sensor(self):
        r = math.radians(self.brg)
        F, L = config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM
        return self.gx + F * math.sin(r) - L * math.cos(r), self.gy + F * math.cos(r) + L * math.sin(r)

    def drive(self, steer, speed, dt):
        self.v += (speed - self.v) * min(1.0, dt / 0.10)
        self.delta += (steer - self.delta) * min(1.0, dt / 0.05)
        if speed == 0.0 and abs(self.v) < 5.0:
            self.v = 0.0
        ds = self.v * dt
        dth = ds * math.tan(math.radians(self.delta)) / config.WHEELBASE_MM
        bm = math.radians(self.brg - math.degrees(dth) / 2)
        self.gx += ds * math.sin(bm)
        self.gy += ds * math.cos(bm)
        self.brg = (self.brg - math.degrees(dth)) % 360.0
        self.dist += ds
        self.path.append((self.gx, self.gy, self.brg))

    def imu(self, t):
        smp, _ = parse_line(self.stm.line(int(round(t * 1000)), self.dist, self.brg))
        return ImuSample(smp.seq, smp.t_ms, smp.enc, smp.yaw_deg, t)

    def scan_timed(self, t):
        lx, ly = self.sensor()
        raw = sim.simulate_scan(lx, ly, self.brg, self.pillars, self.barriers, n_points=720, rng_noise=self.rng,
                                blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                                blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
        # stamped like sim_closed_loop: the LIDAR's clock offset the tracker corrects for
        return [(sim.to_sensor_raw(a), dd, q, t + config.LIDAR_TIME_OFFSET_S) for a, dd, q in raw]

    def camera_frame(self):
        cx, cy = camera_sim.camera_global(self.gx, self.gy, self.brg)
        return camera_sim.render(cx, cy, self.brg, self.pillars, self.cam_rng)

    def lane_pose(self):
        return lf.global_to_lane(self.lay.start_section, self.lay.direction, self.gx, self.gy)


class FirmwareModel:
    """drive_protocol.h's run logic, in Python (see the module docstring)."""

    def __init__(self):
        self.buf = bytearray()
        self.state, self.run_id = RUN_READY, 0
        self.cmd = None                    # (enable, closed, mode, steer_deg, speed, ready, over)
        self.cmd_t = -1e9
        self.seq = 0
        self.params = fw_wanted()          # the "compiled defaults" = config.py at start-up
        self.echo: dict = {}
        self.log: list = []

    def feed(self, data: bytes, t: float):
        self.buf += data
        while len(self.buf) >= 2:
            if self.buf[0] != 0xAA or self.buf[1] not in (0x55, 0x56):
                del self.buf[0]
                continue
            n = 11 if self.buf[1] == 0x55 else 8
            if len(self.buf) < n:
                return
            f = bytes(self.buf[:n])
            x = 0
            for v in f[2:n - 1]:
                x ^= v
            if x != f[n - 1]:
                del self.buf[0]
                continue
            del self.buf[:n]
            if n == 8:
                pid, val = f[2], struct.unpack("<f", f[3:7])[0]
                if pid == 0xFF:
                    self.echo.update(self.params)
                elif pid in self.params and math.isfinite(val):
                    self.params[pid] = val
                    self.echo[pid] = val
                continue
            fl = f[3]
            steer, speed = struct.unpack("<hh", f[4:8])
            self.seq = f[2]
            self.cmd = (bool(fl & 1), bool(fl & 2), (fl >> 2) & 3, steer / 10.0, float(speed),
                        bool(fl & 16), bool(fl & 32))
            self.cmd_t = t
            if self.state == RUN_RUNNING and self.cmd[6]:
                self.state = RUN_FINISHED
                self.log.append(f"# FINISHED (Pi) run {self.run_id}")

    def fresh(self, t):
        return self.cmd is not None and t - self.cmd_t <= WATCHDOG_S

    def press(self, t):
        if self.state == RUN_RUNNING:
            self.state = RUN_STOPPED
            self.log.append(f"# STOPPED (button) run {self.run_id}")
        elif self.fresh(t) and self.cmd[5]:
            self.state = RUN_RUNNING
            self.run_id += 1
            self.log.append(f"# START (button) run {self.run_id}")
        else:
            self.log.append(f"# press ignored: Pi not ready run {self.run_id}")

    def output(self, t):
        """(motor on, steer deg, speed mm/s)"""
        if self.state != RUN_RUNNING or not self.fresh(t):
            return False, 0.0, 0.0
        en, _, mode, steer, speed, _, _ = self.cmd
        if not en or mode != 0:
            return False, 0.0, 0.0
        lock_l, lock_r = self.params[4], self.params[5]
        return True, max(-lock_r, min(lock_l, steer)), speed

    def status_bits(self, t):
        on = self.output(t)[0]
        wd = self.cmd is not None and not self.fresh(t)
        return (1 if on else 0) | (2 if wd else 0) | (8 if on else 0) | 16


class MissionSim:
    def __init__(self, direction: str | None = None, seed: int = 1, time_scale: float = 1.0,
                 lidar_hz: float = 10.0, camera_fps: float = 15.0):
        self.seed, self.scale, self.lidar_hz, self.camera_fps = seed, time_scale, lidar_hz, camera_fps
        self.layout = layouts.draw(random.Random(seed), direction)
        self.direction = self.layout.direction
        self.world = World(self.layout, seed)
        self.fw = FirmwareModel()
        self.stats = LinkStats()
        self._lock = threading.RLock()
        self._pending: list = []
        self._t = 0.0
        self._carry = None                 # (t_end, dx, dy, dbrg per 10 ms step)
        self._scan = (None, None)
        self._frame = None
        self._n_frames = 0
        self._stop = threading.Event()
        self._t0 = time.monotonic()
        self._thread = threading.Thread(target=self._run, daemon=True, name="mission_sim")
        self._thread.start()
        self.link, self.lidar, self.camera = _Link(self), _Lidar(self), _Camera(self)

    # -- clock and the simulated robot ---------------------------------------------------
    def now(self) -> float:
        return self.scale * (time.monotonic() - self._t0)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            target = self.now()
            while self._t + 0.01 <= target:
                self._step()
            time.sleep(0.005)

    def _step(self):
        with self._lock:
            self._t = round(self._t + 0.01, 6)
            w = self.world
            if self._carry is not None:
                t_end, dx, dy, db = self._carry
                w.gx += dx
                w.gy += dy
                w.brg = (w.brg + db) % 360.0
                w.v = w.delta = 0.0
                if self._t >= t_end:
                    self._carry = None
            else:
                on, steer, speed = self.fw.output(self._t)
                w.drive(steer if on else 0.0, speed if on else 0.0, 0.01)
                if len(w.path) > 30000:
                    del w.path[:10000]
            s = w.imu(self._t)
            self.stats.note(s)
            self._pending.append(s)

    # -- what a person does ------------------------------------------------------------------
    def press(self):
        """The start button (debounced press)."""
        with self._lock:
            self.fw.press(self._t)

    def carry_to_start(self, seconds: float = 1.5):
        """Lift the car and set it down in its start zone over `seconds` (the
        wheels don't turn; the heading swings, so the Pi sees the motion)."""
        with self._lock:
            gx, gy, brg = self.world.start_pose()
            n = max(1, int(round(seconds / 0.01)))
            w = self.world
            db = ((brg - w.brg + 180.0) % 360.0 - 180.0)
            if abs(db) < 5.0:
                db += 360.0 if db >= 0 else -360.0        # a visible swing even when already straight
            self._carry = (self._t + seconds, (gx - w.gx) / n, (gy - w.gy) / n, db / n)

    # -- the dashboard's truth overlay (live_sim.LiveSim.truth's format) ---------------------
    def truth(self, t: float | None = None) -> dict:
        lay = self.layout
        with self._lock:
            pose = (self.world.gx, self.world.gy, self.world.brg)
        seats, colors = {}, {}
        for k in range(4):
            sec = lay.slot_section(k)
            seats[sec] = sorted(i for i, _ in lay.pillars.get(sec, []))
            colors[sec] = {i: c for i, c in lay.pillars.get(sec, [])}
        return {"pose": pose, "pillars": [(p.x_mm, p.y_mm) for p in self.world.pillars],
                "start_section": lay.start_section, "direction": lay.direction, "seats": seats, "colors": colors,
                "slot_sections": [lay.slot_section(k) for k in range(4)],
                "driving": self.fw.output(self._t)[0], "finished": self.fw.state == RUN_FINISHED,
                "layout": lay.describe()}


class _Link:
    """stm32_link.Stm32Link's surface."""

    def __init__(self, s: MissionSim):
        self._s = s
        self.port = "(simulated STM32, mission mock)"

    def drain(self):
        with self._s._lock:
            out, self._s._pending = self._s._pending, []
        return out

    def write(self, data: bytes) -> bool:
        with self._s._lock:
            self._s.fw.feed(bytes(data), self._s._t)
        return True

    def is_alive(self) -> bool:
        return self._s._thread.is_alive()

    def stop(self):
        self._s.stop()

    def status(self) -> dict:
        s = self._s
        with s._lock:
            fw, t = s.fw, s._t
            st, last = s.stats, s.stats.last_sample
            sta = {"seq_ack": fw.seq, "status": fw.status_bits(t), "age_s": 0.0, "run_state": fw.state,
                   "run_id": fw.run_id, "pwm": 0, "speed_mm_s": round(s.world.v)}
            log = list(fw.log[-20:])
            echo = dict(fw.echo)
        return {"port": self.port, "thread_alive": self.is_alive(), "error": None,
                "lines_ok": st.lines_ok, "lines_bad": 0, "last_bad_reason": "", "lines_log": len(log),
                "recent_log": log, "seq_gaps": st.seq_gaps, "seq_resets": st.seq_resets,
                "rate_hz": 100.0 if last is not None else 0.0, "last_age_s": 0.0,
                "stale": last is None, "sta": sta, "fw_params": echo,
                "last": None if last is None else {"seq": last.seq, "t_ms": last.t_ms, "enc": last.enc,
                                                   "yaw_deg": last.yaw_deg}}


class _Lidar:
    def __init__(self, s: MissionSim):
        self._s = s

    def get_latest_scan_timed(self):
        s = self._s
        with s._lock:
            t = s._t
            if s._scan[0] is not None and t - s._scan[0] < 1.0 / s.lidar_hz:
                return s._scan[1]
            raw = s.world.scan_timed(t)
            s._scan = (t, raw)
            return raw

    def get_latest_scan(self):
        return [(a, d, q) for a, d, q, _ in self.get_latest_scan_timed()]

    def status(self) -> dict:
        return {"thread_alive": self._s._thread.is_alive(), "error": None, "simulated": True}

    def timing_status(self) -> dict:
        return {"spin_hz": self._s.lidar_hz, "backwards_steps": 0, "last_return_age_s": 0.0}


class _Camera:
    def __init__(self, s: MissionSim):
        self._s = s

    def get_latest_frame(self):
        s = self._s
        with s._lock:
            t = s._t
            if s._frame is not None and t - s._frame[1] < 1.0 / s.camera_fps:
                return s._frame
            s._n_frames += 1
            s._frame = (s.world.camera_frame(), t, s._n_frames)
            return s._frame

    def status(self) -> dict:
        return {"source": "simulated", "size": [config.CAMERA_WIDTH, config.CAMERA_HEIGHT],
                "thread_alive": self._s._thread.is_alive(), "error": None, "frames": self._s._n_frames,
                "fps": self._s.camera_fps, "last_age_s": None, "stamp_fallbacks": 0, "simulated": True}
