"""
The dashboard's mock hardware (decision #42): the run_mock world in real time.

A background thread advances a simulated clock (the mock's stand-in for the
Pi's time.monotonic(), optionally faster than real time) in 10 ms steps and
emits the STM32 lines a real robot would send (simulation.SimStm32 through the
real parser). The robot stands at its start until drive() is called, then
follows simulation.LoopPath for `laps` laps at `speed_mm_s` and stops.

It offers the same two interfaces the dashboard uses for the real hardware:
    .link   drain(), status()                          (like stm32_link.Stm32Link)
    .lidar  get_latest_scan(), get_latest_scan_timed(),
            status(), timing_status()                  (like lidar_source.RPLidarC1Source)
get_latest_scan_timed() is one real revolution (simulation.cast_revolution)
ending now, each ray cast from the pose at the time it was measured and
stamped as lidar_source would stamp it (measurement + 2 ms +
config.LIDAR_TIME_OFFSET_S), so the dashboard's de-skew runs exactly as on
the robot.

truth() gives what the real robot can't know (pillars, true pose) for the
dashboard's faint truth overlay.
"""
from __future__ import annotations

import math
import random
import threading
import time
from collections import deque

import numpy as np

import config
import simulation as sim
from stm32_link import ImuSample, LinkStats, parse_line

IMU_LATENCY_S = 0.002


class LiveSim:
    def __init__(self, direction: str = "CCW", start_slot: int = 0, seed: int = 1, speed_mm_s: float = 600.0,
                 laps: int = 3, time_scale: float = 1.0, y_start: float = 1400.0, lidar_hz: float = 10.0):
        self.direction, self.start_slot, self.seed = direction, start_slot, seed
        self.speed, self.laps, self.scale, self.lidar_hz = speed_mm_s, laps, time_scale, lidar_hz
        world_rng = random.Random(seed)
        self.path, self.start_section, self.s0, self.pillars, self.truth_seats = sim.make_world(
            direction, start_slot, y_start, world_rng)
        self._stm = sim.SimStm32(random.Random(seed + 1000))
        self._scan_rng = random.Random(seed + 2000)
        self._lock = threading.Lock()
        self._pending: list[ImuSample] = []
        self._track: deque = deque(maxlen=400)          # (sim time, arc length s), 4 s
        self._t = 0.0                                   # sim time of the newest step
        self._s = self.s0
        self._dist = 0.0
        self._drive_at: float | None = None
        self._s_end = self.s0 + laps * self.path.length
        self._frame = None                              # cached (t, raw4)
        self.stats = LinkStats()
        self._stop = threading.Event()
        self._t0 = time.monotonic()
        self._track.append((0.0, self.s0))
        self._thread = threading.Thread(target=self._run, daemon=True, name="live_sim")
        self._thread.start()
        self.link, self.lidar = _Link(self), _Lidar(self)

    # -- control ---------------------------------------------------------------
    def now(self) -> float:
        return self.scale * (time.monotonic() - self._t0)

    def drive(self, after_s: float = 1.0):
        """Start driving `after_s` (simulated) seconds from now."""
        with self._lock:
            self._drive_at = self._t + after_s

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    @property
    def driving(self) -> bool:
        return self._drive_at is not None and self._t >= self._drive_at and self._s < self._s_end

    @property
    def finished(self) -> bool:
        return self._s >= self._s_end

    # -- the simulated robot ------------------------------------------------------
    def _run(self):
        while not self._stop.is_set():
            target = self.now()
            while self._t + 0.01 <= target:
                self._step()
            time.sleep(0.005)

    def _step(self):
        with self._lock:
            self._t = round(self._t + 0.01, 6)
            if self._drive_at is not None and self._t >= self._drive_at and self._s < self._s_end:
                gx0, gy0, _ = self.path.pose(self._s)
                self._s = min(self._s + self.speed * 0.01, self._s_end)
                gx1, gy1, _ = self.path.pose(self._s)
                self._dist += math.hypot(gx1 - gx0, gy1 - gy0)
            self._track.append((self._t, self._s))
            _, _, brg = self.path.pose(self._s)
            t_ms = int(round(self._t * 1000))
            smp, _ = parse_line(self._stm.line(t_ms, self._dist, brg))
            smp = ImuSample(smp.seq, smp.t_ms, smp.enc, smp.yaw_deg, self._t + IMU_LATENCY_S)
            self.stats.note(smp)
            self._pending.append(smp)

    def truth(self, t: float | None = None) -> dict:
        """t: the simulated time to give the true pose at (e.g. the time of the
        tracker's newest sample, so the two are compared at the same instant);
        None = now."""
        with self._lock:
            if t is None or not self._track:
                gx, gy, brg = self.path.pose(self._s)
            else:
                ts = [a for a, _ in self._track]
                ss = [b for _, b in self._track]
                gx, gy, brg = self.path.pose(float(np.interp(t, ts, ss)))
        return {"pose": (gx, gy, brg), "pillars": [(p.x_mm, p.y_mm) for p in self.pillars],
                "start_section": self.start_section, "direction": self.direction,
                "seats": {k: sorted(v) for k, v in self.truth_seats.items()},
                "slot_sections": [self.path.section(self.start_slot + k) for k in range(4)],
                "driving": self.driving, "finished": self.finished}


class _Link:
    """stm32_link.Stm32Link's consumer side."""

    def __init__(self, s: LiveSim):
        self._s = s
        self.port = "(simulated STM32)"

    def drain(self) -> list[ImuSample]:
        with self._s._lock:
            out, self._s._pending = self._s._pending, []
        return out

    def is_alive(self) -> bool:
        return self._s._thread.is_alive()

    def status(self) -> dict:
        st, last = self._s.stats, self._s.stats.last_sample
        age = None if last is None else max(0.0, self._s.now() - last.rx_time)
        return {"port": self.port, "thread_alive": self.is_alive(), "error": None,
                "lines_ok": st.lines_ok, "lines_bad": st.lines_bad, "last_bad_reason": st.last_bad_reason,
                "seq_gaps": st.seq_gaps, "seq_resets": st.seq_resets,
                "rate_hz": 100.0 if last is not None else 0.0,          # simulated clock
                "last_age_s": None if age is None else round(age, 3),
                "stale": age is None or age > config.IMU_STALE_S,
                "last": None if last is None else {"seq": last.seq, "t_ms": last.t_ms, "enc": last.enc,
                                                   "yaw_deg": last.yaw_deg}}


class _Lidar:
    """lidar_source.RPLidarC1Source's consumer side."""

    def __init__(self, s: LiveSim):
        self._s = s

    def get_latest_scan(self) -> list[tuple[float, float, int]]:
        return [(a, d, q) for a, d, q, _ in self.get_latest_scan_timed()]

    def get_latest_scan_timed(self) -> list[tuple[float, float, int, float]]:
        s = self._s
        with s._lock:
            t_end = s._t
            if s._frame is not None and t_end - s._frame[0] < 0.5 / s.lidar_hz:
                return s._frame[1]
            track = list(s._track)
        ts = [a for a, _ in track]
        ss = [b for _, b in track]
        raw = sim.cast_revolution(lambda t: s.path.pose(float(np.interp(t, ts, ss))), t_end, 1.0 / s.lidar_hz,
                                  s.pillars, s._scan_rng)
        raw = [(a, d, q, t + IMU_LATENCY_S + config.LIDAR_TIME_OFFSET_S) for a, d, q, t in raw]
        with s._lock:
            s._frame = (t_end, raw)
        return raw

    def status(self) -> dict:
        return {"thread_alive": self._s._thread.is_alive(), "error": None, "simulated": True}

    def timing_status(self) -> dict:
        return {"spin_hz": self._s.lidar_hz, "backwards_steps": 0, "last_return_age_s": 0.0}
