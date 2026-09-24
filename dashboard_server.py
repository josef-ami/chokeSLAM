"""
The dashboard (checkpoint C; docs/CHANGES.md section 10, decisions #16,
#21, #22, #41-#44): a Flask page that draws the loop lane by lane as the
robot drives it, with the obstacles found, the robot, its IMU-traced path and
the LIDAR scans, plus the numbers behind them and a tuning panel.

Run:
    python3 dashboard_server.py         then open http://<pi-or-localhost>:5056/

config.MODE picks the hardware:
    "real"  the STM32 link (stm32_link.Stm32Link) and the RPLIDAR
            (lidar_source.RPLidarC1Source) start with the server;
    "mock"  live_sim.LiveSim: the run_mock world in real time.

Nothing is initialised until Initialise is pressed (#41). Initialise, with
the robot standing still at its start:
    real: takes the newest STM32 sample and the current scan, runs
          lane_init.initialise, and starts a lane_tracker.LaneTracker from
          them (the same pairing as run_track.py --real);
    mock: rebuilds the simulated world from the panel's settings (direction,
          start lane, seed, speed, time scale), then does the same, and the
          simulated robot drives off 1 s later.
Pressing it again (Re-initialise) throws away every lane, seat and path and
starts over from lane 1.

The runtime loop (a background thread, ~50 Hz) feeds every STM32 sample to the
tracker, reads a LIDAR frame with its measurement times ~20 times a second,
hands it to the tracker while the entry re-check wants it (de-skewed there),
and de-skews it once more for display. The page gets the state over
Server-Sent Events at STREAM_HZ; the init scan and the mock's truth, which
don't change during a run, come from /api/static once per initialisation.

Everything drawn is in display.py's fixed full-loop frame.

Checkpoint F2 (owner: "keep as many of the tuning values and parameters as
possible modifiable live from the dashboard; continuously draw the planned path
my path planner builds after perception"):
    PLANNED PATH   while a tracker exists, a background thread re-plans every
                   PLAN_PREVIEW_S what the mission would plan from the tracked
                   pose and the seats and colours perceived so far
                   (mission.plan_preview: lap 1 -> the next lane's viewing pose;
                   laps 2-3 -> the rest of the round and the finish), and the
                   page draws it with the planner's goals and pass-side lines.
                   In mission mode, while a run is going, the page draws the
                   mission's own current path instead (it changes on every re-plan).
    MISSION MODE   python3 dashboard_server.py --mission   (config.MODE mock: the
                   simulated car of mission_sim.py, driven by the real run
                   supervisor + mission; buttons for the start button and for
                   carrying the car back)   or   python3 run_mission.py --dashboard
                   (on the robot: the competition program with this page).
    TUNING         every group of config.py values the Pi reads at use time,
                   plus the drive firmware's values (servo map, steering lock,
                   speed loop, encoder scale), which the Pi sends to the STM32
                   (drive_link.sync_params) -- the panel shows the STM32's echo.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

from flask import Flask, Response, jsonify, render_template, request

import config
import display as dp
import lane_init as li
import plan_view as pv_mod
import seat_occupancy as so
from lane_tracker import LaneTracker
from plan_view import _r, plan_view, preview_snapshot
from scan_processing import clean_and_project, clean_and_project_timed

app = Flask(__name__)
RUNS_DIR = "runs"                      # Save-run writes runs/<date-time>/scan.json + imu.log
LIDAR_READ_S = 0.05                    # read a LIDAR frame every 50 ms
LOOP_S = 0.02
MAX_LIVE_POINTS = 720
MAX_PATH_POINTS = 2000


class PlanPreview:
    """Re-plans every PLAN_PREVIEW_S. The plan itself runs in a worker process
    (plan_view.preview_job; why: plan_view's docstring); this thread only takes
    the snapshot under the runtime's lock and waits for the answer.
    rt.preview_input() gives a snapshot (or None: nothing to preview),
    rt.plan_view receives the result."""

    TIMEOUT_S = 20.0

    def __init__(self, rt):
        self.rt = rt
        self.runs = 0
        self._pool = None
        self._thread = threading.Thread(target=self._loop, daemon=True, name="plan_preview")
        self._thread.start()

    def _executor(self):
        if self._pool is None:
            import multiprocessing as mp
            from concurrent.futures import ProcessPoolExecutor
            self._pool = ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"))
        return self._pool

    def _loop(self):
        while True:
            time.sleep(max(0.05, float(config.PLAN_PREVIEW_S)))
            try:
                if not config.PLAN_PREVIEW_ENABLED:
                    continue
                with self.rt.lock:
                    snap = self.rt.preview_input()
                    tag = self.rt.init_id
                if snap is None:
                    continue
                view = self._executor().submit(pv_mod.preview_job, pv_mod.config_values(), snap).result(
                    timeout=self.TIMEOUT_S)
                view["at"] = time.monotonic()
                with self.rt.lock:
                    if self.rt.init_id == tag and self.rt.preview_input_ok():
                        self.rt.plan_view = view
                self.runs += 1
            except Exception as e:                      # keep previewing; show it
                if self._pool is not None and not isinstance(e, ValueError):
                    self._pool.shutdown(wait=False, cancel_futures=True)   # a broken or stuck worker: start afresh
                    self._pool = None
                with self.rt.lock:
                    self.rt.plan_view = plan_view("preview", False, f"preview error: {type(e).__name__}: {e}",
                                                  None, None, [])


class Runtime:
    def __init__(self, mode: str):
        self.mode = mode
        self.lock = threading.RLock()
        self.seat_params = so.DetectParams()
        self.mock = {"direction": "CCW", "start_slot": 0, "seed": 1, "speed_mm_s": 600.0, "time_scale": 1.0}
        self.sim = self.link = self.lidar = self.camera = None
        self._last_cam_no = None
        self.camera_ms = None                 # time the last colour-ID frame took to process
        self.trk: LaneTracker | None = None
        self.init: li.InitResult | None = None
        self.init_raw = None
        self.first = None
        self.samples: list = []               # every STM32 sample since initialisation (Save run)
        self.last_sample = None
        self.init_id = 0
        self.message = "Press Initialise with the robot standing still at its start."
        self.static = {"init_id": 0, "init_scan": [], "truth_pillars": []}
        self.live, self.live_frame = [], "robot"
        self._last_lidar = self._last_view = 0.0
        self.error = None
        self.plan_view = None
        self.drive = None                      # real mode: only to keep the STM32's tuning values = config.py
        self._start_sources()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="dashboard_runtime")
        self._thread.start()
        self.preview = PlanPreview(self)

    # -- planner preview (checkpoint F2) ---------------------------------------
    def preview_input(self):
        return None if self.trk is None else preview_snapshot(self.trk)

    def preview_input_ok(self) -> bool:
        return self.trk is not None

    # -- hardware ---------------------------------------------------------------
    def _start_sources(self):
        if self.mode == "mock":
            from live_sim import LiveSim
            if self.sim is not None:
                self.sim.stop()
            m = self.mock
            self.sim = LiveSim(m["direction"], int(m["start_slot"]), int(m["seed"]), float(m["speed_mm_s"]),
                               time_scale=float(m["time_scale"]))
            self.link, self.lidar, self.camera = self.sim.link, self.sim.lidar, self.sim.camera
        else:
            from lidar_source import RPLidarC1Source
            from stm32_link import Stm32Link
            from drive_link import DriveLink
            self.link = Stm32Link()
            self.link.start()
            self.drive = DriveLink(self.link)          # never sends DRIVE frames here: sync_params only
            self.lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
            self.lidar.start()
            if config.CAMERA_ENABLED:
                from camera_source import Picamera2Source
                self.camera = Picamera2Source()
                self.camera.start()

    # -- initialise ---------------------------------------------------------------
    def initialise(self) -> dict:
        with self.lock:
            if self.mode == "mock":
                self._start_sources()                 # a fresh world from the panel's settings
                t0 = time.monotonic()
                while self.sim.stats.last_sample is None and time.monotonic() - t0 < 2.0:
                    time.sleep(0.01)
            self.trk, self.init, self.samples = None, None, []
            self.plan_view = None
            batch = self.link.drain()
            t0 = time.monotonic()
            while not batch and time.monotonic() - t0 < 1.0:
                time.sleep(0.01)
                batch = self.link.drain()
            if not batch:
                self.message = f"Initialise failed: no STM32 samples ({self.link.status().get('error')})"
                return {"ok": False, "reason": self.message}
            self.first = batch[-1]                    # the sample current at the scan (as run_track.py --real)
            raw = self.lidar.get_latest_scan()
            self.init_raw = raw
            pts = clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            self.init = li.initialise(pts, params=self.seat_params)
            self.init_id += 1
            self.live = []
            if not self.init.ok:
                self.message = f"Initialise failed: {self.init.reason}. Move the robot or check the scan, then retry."
                self.static = {"init_id": self.init_id, "init_scan": [], "truth_pillars": self._truth_pillars()}
                return {"ok": False, "reason": self.init.reason}
            self.trk = LaneTracker(self.init, self.first, params=self.seat_params)
            self.samples = [self.first]
            self.static = {"init_id": self.init_id, "init_scan": self._init_scan_display(pts),
                           "truth_pillars": self._truth_pillars()}
            self.message = (f"Tracking: {self.init.reason}. "
                            + ("The simulated robot drives off 1 s after Initialise." if self.mode == "mock" else
                               "Drive the robot."))
            if self.mode == "mock":
                self.sim.drive(1.0)
            return {"ok": True, "reason": self.init.reason}

    def _init_scan_display(self, pts) -> list:
        trk = self.trk
        X0, Y0 = dp.lane_to_display(0, trk.x, trk.y, trk.direction)
        F, L = trk.params.lidar_offset_forward_mm, trk.params.lidar_offset_lateral_mm
        return [[round(v, 1) for v in dp.robot_to_display(X0, Y0, trk.psi0, p.fwd_mm + F, p.right_mm - L)]
                for p in pts]

    def _truth_pillars(self) -> list:
        if self.sim is None:
            return []
        t = self.sim.truth()
        g2d = dp.GlobalToDisplay(t["start_section"], t["direction"])
        return [[round(v, 1) for v in g2d.point(x, y)] for x, y in t["pillars"]]

    # -- the loop ---------------------------------------------------------------------
    def _feed_imu(self):
        batch = self.link.drain()
        if batch:
            self.last_sample = batch[-1]
        if self.trk is not None:
            for s in batch:
                self.trk.on_imu(s)
            self.samples.extend(batch)

    def _feed_camera(self):
        """Pillar colour ID (checkpoint D): a NEW frame goes to the tracker only
        while a PRESENT seat is waiting for its colour; the frame's own capture
        time picks the pose (#62). After _feed_imu, so the history reaches it."""
        if self.trk is None or self.camera is None or not self.trk.wants_camera:
            return
        fr = self.camera.get_latest_frame()
        if fr is None or fr[2] == self._last_cam_no:
            return
        self._last_cam_no = fr[2]
        t0 = time.perf_counter()
        self.trk.on_camera_frame(fr[0], fr[1])
        self.camera_ms = round((time.perf_counter() - t0) * 1000.0, 2)

    def _loop(self):
        while True:
            try:
                self._tick()
                self.error = None
            except Exception as e:                    # keep serving; show it on the page
                self.error = f"{type(e).__name__}: {e}"
            time.sleep(LOOP_S)

    def _tick(self):
        with self.lock:
            now = time.monotonic()
            raw4 = None
            if now - self._last_lidar >= LIDAR_READ_S:
                self._last_lidar = now
                # the LIDAR frame is read BEFORE the STM32 samples are taken, so the pose history the
                # de-skew uses already reaches the frame's newest returns (no extrapolation over the gap)
                raw4 = self.lidar.get_latest_scan_timed()
            self._feed_imu()
            self._feed_camera()
            if self.drive is not None:
                self.drive.sync_params(now)
            if not raw4:
                return
            pts, times = clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            if self.trk is not None and self.trk.wants_lidar:
                self.trk.on_lidar_frame(pts, times)
            if now - self._last_view >= 1.0 / max(1, config.STREAM_HZ):
                self._last_view = now
                self._live_view(pts, times)

    def _live_view(self, pts, times):
        trk = self.trk
        if trk is None:                               # not initialised: robot-centred view, facing up
            X0, Y0, b, F, L, view = 1500.0, 1500.0, 0.0, config.LIDAR_OFFSET_FORWARD_MM, \
                config.LIDAR_OFFSET_LATERAL_MM, "robot"
        else:
            pts = trk.deskew_view(pts, times)
            X0, Y0 = dp.lane_to_display(trk.slot, trk.x, trk.y, trk.direction)
            b, F, L, view = trk.heading, trk.params.lidar_offset_forward_mm, trk.params.lidar_offset_lateral_mm, "track"
        step = max(1, len(pts) // MAX_LIVE_POINTS)
        self.live = [[round(v, 1) for v in dp.robot_to_display(X0, Y0, b, p.fwd_mm + F, p.right_mm - L)]
                     for p in pts[::step]]
        self.live_frame = view

    # -- state for the page -------------------------------------------------------------
    def state(self) -> dict:
        with self.lock:
            trk = self.trk
            out = {"mode": self.mode, "message": self.message, "error": self.error, "init_id": self.init_id,
                   "tracking": trk is not None, "live": self.live, "live_frame": self.live_frame,
                   "mock": dict(self.mock) if self.mode == "mock" else None,
                   "imu": self._imu_status(), "lidar": self._lidar_status(), "camera": self._camera_status(),
                   "init": self._init_summary()}
            if trk is not None:
                X, Y = dp.lane_to_display(trk.slot, trk.x, trk.y, trk.direction)
                out["direction"] = trk.direction
                out["robot"] = {"X": _r(X), "Y": _r(Y), "bearing": _r(trk.heading % 360.0, 2)}
                out["tracker"] = {"lane_index": trk.lane_index, "slot": trk.slot, "lap": trk.lap,
                                  "x": _r(trk.x), "y": _r(trk.y), "psi": _r(trk.psi, 2), "psi0": _r(trk.psi0, 2),
                                  "heading": _r(trk.heading, 2), "distance_mm": _r(trk.distance_mm),
                                  "recheck_active": trk.wants_lidar,
                                  "events": [{"t": _r(e.t_ms / 1000.0, 2), "kind": e.kind, "detail": e.detail}
                                             for e in trk.events[-30:]]}
                out["lanes"] = [self._lane(k, rec) for k, rec in sorted(trk.lanes.items())]
                path = trk.path
                step = max(1, len(path) // MAX_PATH_POINTS)
                out["path"] = [[round(v, 1) for v in dp.lane_to_display(k % 4, x, y, trk.direction)]
                               for k, x, y in path[::step]] + \
                              [[round(v, 1) for v in dp.lane_to_display(trk.slot, trk.x, trk.y, trk.direction)]]
            out["plan"] = self.plan_view
            out["fw_params"] = self._fw_params()
            if self.sim is not None:
                # the true pose at the instant of the tracker's newest sample, so the two compare like for like
                t = self.sim.truth(None if trk is None else trk.last_sample.t_ms / 1000.0)
                g2d = dp.GlobalToDisplay(t["start_section"], t["direction"])
                gx, gy, brg = t["pose"]
                X, Y = g2d.point(gx, gy)
                truth_lanes, truth_colors = {}, {}
                for k, sec in enumerate(t["slot_sections"]):
                    truth_lanes[k] = t["seats"].get(sec, [])
                    truth_colors[k] = t["colors"].get(sec, {})
                out["truth"] = {"robot": {"X": _r(X), "Y": _r(Y), "bearing": _r(g2d.bearing(brg), 2)},
                                "seats_by_slot": truth_lanes, "colors_by_slot": truth_colors, "driving": t["driving"], "finished": t["finished"],
                                "start_section": t["start_section"]}
            return out

    def _lane(self, slot, rec) -> dict:
        d = self.trk.direction
        seats = []
        names = {s.index: s for s in so.seats()}
        for i in sorted(rec.seats):
            st, seat = rec.seats[i], names[i]
            X, Y = dp.lane_to_display(slot, seat.x_mm, seat.y_mm, d)
            seats.append({"index": i, "name": seat.name, "X": _r(X), "Y": _r(Y), "state": st.state,
                          "source": st.source, "reason": st.reason, "at_y": _r(st.at_y_mm),
                          "color": st.color, "color_reason": st.color_reason})
        ol = dp.lane_outline(slot, d)
        return {"slot": slot, "lane_number": slot + 1, "source": rec.source, "frozen": rec.frozen,
                "label": [_r(v) for v in dp.lane_to_display(slot, 800.0, 1250.0, d)],
                "frames_used": rec.frames_used, "frames_skipped_align": rec.frames_skipped_align,
                "returns_dropped_old": rec.returns_dropped_old, "max_shift_mm": _r(rec.max_shift_mm),
                "frames_skipped_old": rec.frames_skipped_old,
                "outline": {k: [[_r(a), _r(b)] for a, b in v] for k, v in ol.items()}, "seats": seats}

    def _fw_params(self):
        """The drive firmware's tuning values: config.py's, the STM32's echo, and the ones that differ."""
        if self.drive is None:
            return None
        from drive_link import FW_PARAMS, fw_wanted
        have = self.drive.fw_params()
        return {"values": {FW_PARAMS[k][0]: {"config": _r(v, 4), "stm32": _r(have.get(k), 4)}
                           for k, v in fw_wanted().items()},
                "mismatch": sorted(self.drive.param_mismatch()), "sent": self.drive.params_sent}

    def _init_summary(self):
        r = self.init
        if r is None:
            return None
        d = r.direction

        def side(sc):
            if sc is None:
                return None
            fit = sc.fit
            return {"side": sc.side, "d_mm": _r(sc.d_side_mm), "opening_mm": _r(sc.opening_mm),
                    "opening_from_mm": _r(sc.opening_start_mm), "opening_to_mm": _r(sc.opening_end_mm),
                    "n_wall": sc.n_wall, "n_through": sc.n_through, "n_blocked": sc.n_blocked,
                    "fit_deg": _r(fit.angle_deg, 2) if fit is not None and fit.ok else None,
                    "fit_inliers": fit.n_inliers if fit is not None and fit.ok else None,
                    "fit_rms_mm": _r(fit.rms_mm) if fit is not None and fit.ok else None}

        out = {"ok": r.ok, "reason": r.reason, "direction": d.direction, "direction_reason": d.reason,
               "wall_angle_deg": _r(d.wall_angle_deg, 2), "left": side(d.left), "right": side(d.right)}
        if r.x is not None:
            out["x"] = {"x_mm": _r(r.x.x_mm), "d90_mm": _r(r.x.d90_mm), "d270_mm": _r(r.x.d270_mm),
                        "sum_mm": _r(r.x.lane_sum_mm), "reason": r.x.reason}
        if r.y is not None:
            out["y"] = {"y_mm": _r(r.y.y_mm), "front_mm": _r(r.y.front_mm), "n_front": r.y.n_front,
                        "n_fan": r.y.n_fan, "reason": r.y.reason}
        out["seats"] = [{"index": s.seat.index, "name": s.seat.name, "state": s.state.value, "reason": s.reason}
                        for s in r.seats]
        if self.trk is not None:
            out["psi0"] = _r(self.trk.psi0, 2)
        return out

    def _imu_status(self) -> dict:
        st = dict(self.link.status()) if self.link is not None else {}
        if self.trk is not None:
            st["heading_deg"] = _r(self.trk.heading, 2)
            st["distance_mm"] = _r(self.trk.distance_mm)
        return st

    def _camera_status(self) -> dict:
        st = dict(self.camera.status()) if self.camera is not None else {"source": "off (config.CAMERA_ENABLED)"}
        if self.trk is not None:
            st.update({"frames_used": self.trk.camera_frames, "frames_no_pose": self.trk.camera_frames_no_pose,
                       "pending": len(self.trk._color_pending)})
        st["process_ms"] = self.camera_ms
        return st

    def _lidar_status(self) -> dict:
        if self.lidar is None:
            return {}
        st = dict(self.lidar.status())
        ts = self.lidar.timing_status()
        st.update({"spin_hz": _r(ts.get("spin_hz"), 2), "backwards_steps": ts.get("backwards_steps"),
                   "last_return_age_s": _r(ts.get("last_return_age_s"), 3)})
        return st

    # -- Save run -----------------------------------------------------------------------------
    def save_run(self) -> dict:
        with self.lock:
            if self.init is None or self.init_raw is None or not self.samples:
                return {"ok": False, "reason": "nothing to save: press Initialise first"}
            d = os.path.join(RUNS_DIR, time.strftime("%Y-%m-%d_%H%M%S"))
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, "scan.json"), "w") as fh:
                json.dump({"raw": [list(p) for p in self.init_raw], "angle_sign": config.LIDAR_ANGLE_SIGN,
                           "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG,
                           "imu_seq_at_scan": self.first.seq}, fh)
            with open(os.path.join(d, "imu.log"), "w") as fh:
                for s in self.samples:
                    fh.write(f"$IMU,{s.seq},{s.t_ms},{s.enc},{s.yaw_deg:.2f}\n")
            return {"ok": True, "dir": d, "files": ["scan.json", "imu.log"], "samples": len(self.samples),
                    "replay": f"python3 run_track.py --replay-scan {d}/scan.json --replay-imu {d}/imu.log"}


# =============================================================================
# mission mode (checkpoint F2): the run supervisor + mission behind the same page
# =============================================================================
class MissionRuntime(Runtime):
    """run_control.RunSupervisor on the robot's hardware (run_mission.py
    --dashboard) or on mission_sim.MissionSim (the mock). The page shows the
    tracker the supervisor holds (between runs: the one initialised at rest;
    during a run: the mission's), the mission's current path while it runs and
    the planner preview between runs. Runs start and stop only from the start
    button (in the mock: the page's button stands in for it)."""

    def __init__(self, mode: str, link, drive, lidar, camera=None, sim=None, dump=None, clock=None):
        from collections import deque
        self.mode = f"mission-{mode}"
        self.lock = threading.RLock()
        self.seat_params = so.DetectParams()
        self.sim, self.link, self.drive, self.lidar, self.camera = sim, link, drive, lidar, camera
        self.clock = clock or (sim.now if sim is not None else time.monotonic)
        self.dump = dump
        self.mock = ({"direction": sim.direction, "start_slot": 0, "seed": sim.seed, "speed_mm_s": 0.0,
                      "time_scale": 1.0} if sim is not None else None)
        self.log_lines = deque(maxlen=80)
        self._last_trk = None
        self._shown = None
        self._mission_path = None
        self.init_id = 0
        self.static = {"init_id": 0, "init_scan": [], "truth_pillars": self._truth_pillars()}
        self.live, self.live_frame = [], "robot"
        self._last_view = 0.0
        self.error = None
        self.plan_view = None
        self.camera_ms, self._last_cam_no = None, None
        self.samples, self.init_raw, self.first = [], None, None
        self.message = "Waiting: stand the car still in a start zone; the LED goes solid when a press will start."
        self._new_supervisor()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="mission_runtime")
        self._thread.start()
        self.preview = PlanPreview(self)

    def _new_supervisor(self):
        from mission import Mission
        from run_control import RunSupervisor
        self.sup = RunSupervisor(self.link, self.drive, self.lidar, self.camera,
                                 make_mission=lambda trk: Mission(trk, log=None), log=self._log,
                                 dump=self.dump, seat_params=self.seat_params)

    def _log(self, line):
        self.log_lines.append((round(self.clock(), 2), line))
        self.message = line
        print(line)

    # the tracker and initialisation shown: the supervisor's, else the last run's
    @property
    def trk(self):
        cur = self.sup.tracker()
        if cur is not None:
            self._last_trk = cur
        return self._last_trk

    @property
    def init(self):
        return self.sup.init_result()

    def preview_input(self):
        cur = self.sup.tracker()
        if cur is None or self.sup.mode != "WAIT":
            return None
        return preview_snapshot(cur)

    def preview_input_ok(self) -> bool:
        return self.sup.mode == "WAIT" and self.sup.tracker() is not None

    def initialise(self) -> dict:
        return {"ok": False, "reason": "mission mode: the car initialises by itself whenever it stands still; "
                                      "runs start from the start button"}

    def save_run(self) -> dict:
        return {"ok": False, "reason": "mission mode: use run_mission.py --log imu.log --dump scan.json"}

    # -- the loop -------------------------------------------------------------------------
    def _loop(self):
        while True:
            tick = time.monotonic()
            try:
                with self.lock:
                    self.sup.step(self.clock())
                    self._after_step()
                self.error = None
            except Exception as e:
                import traceback
                self.error = f"{type(e).__name__}: {e}"
                traceback.print_exc()
            time.sleep(max(0.0, 1.0 / max(1.0, float(config.DRIVE_HZ)) - (time.monotonic() - tick)))

    def _after_step(self):
        cur = self.sup.tracker()
        if cur is not None and cur is not self._shown:           # a new initialisation (at rest)
            self._shown = cur
            self._last_trk = cur
            self.init_id += 1
            self.plan_view = None
            pts = []
            if self.sup.prep_raw is not None:
                pts = clean_and_project(self.sup.prep_raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            self.static = {"init_id": self.init_id, "init_scan": self._init_scan_display(pts) if pts else [],
                           "truth_pillars": self._truth_pillars()}
        mis = self.sup.mis
        if self.sup.mode == "RUN" and mis is not None and mis.path is not None and mis.path is not self._mission_path:
            self._mission_path = mis.path
            plans = [e for e in mis.events if e.kind == "plan"]
            self.plan_view = plan_view("mission", True, plans[-1].detail if plans else "", mis.path,
                                       mis.last_world, mis.last_goals)
        now = time.monotonic()
        if now - self._last_view >= 1.0 / max(1, config.STREAM_HZ):
            self._last_view = now
            raw4 = self.lidar.get_latest_scan_timed()
            if raw4:
                pts, times = clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
                if self.trk is not None and self.sup.tracker() is None:
                    self.live, self.live_frame = [], "track"          # between runs, before a new rest: no pose
                else:
                    self._live_view(pts, times)

    def state(self) -> dict:
        from drive_link import RUN_STATE_NAMES
        out = super().state()
        with self.lock:
            sup, mis = self.sup, self.sup.mis
            rs, rid = self.drive.run_state()
            ev = [] if mis is None else [{"t": _r(e.t, 2), "kind": e.kind, "detail": e.detail} for e in mis.events[-40:]]
            out["mission"] = {
                "run_state": RUN_STATE_NAMES.get(rs, "unknown" if rs is None else str(rs)), "run_id": rid,
                "ready": self.drive.ready, "supervisor": sup.mode,
                "phase": None if mis is None else mis.phase,
                "target_lane": None if mis is None else mis.target_lane,
                "plans": None if mis is None else mis.plans, "replans": None if mis is None else mis.replans,
                "runs": [list(r) for r in sup.runs], "events": ev,
                "log": [{"t": t, "line": ln} for t, ln in list(self.log_lines)[-30:]],
                "mock": self.sim is not None,
                "layout": self.sim.layout.describe() if self.sim is not None else None,
            }
        return out

    # -- the mock's hands -------------------------------------------------------------------
    def press(self):
        if self.sim is None:
            return {"ok": False, "reason": "on the robot, use the start button"}
        self.sim.press()
        return {"ok": True}

    def carry_to_start(self):
        if self.sim is None:
            return {"ok": False, "reason": "on the robot, carry it yourself"}
        self.sim.carry_to_start()
        return {"ok": True}

    def new_layout(self, direction, seed):
        if self.sim is None:
            return {"ok": False, "reason": "not in the mock"}
        from drive_link import DriveLink
        from mission_sim import MissionSim
        with self.lock:
            self.sim.stop()
            self.sim = MissionSim(direction, seed)
            self.link, self.lidar, self.camera = self.sim.link, self.sim.lidar, self.sim.camera
            self.drive = DriveLink(self.link)
            self.clock = self.sim.now
            self.mock = {"direction": self.sim.direction, "start_slot": 0, "seed": seed, "speed_mm_s": 0.0,
                         "time_scale": 1.0}
            self._last_trk = self._shown = self._mission_path = None
            self.plan_view = None
            self.init_id += 1
            self.static = {"init_id": self.init_id, "init_scan": [], "truth_pillars": self._truth_pillars()}
            self._new_supervisor()
            self.message = f"New layout: {self.sim.layout.describe()}"
        return {"ok": True, "layout": self.sim.layout.describe()}


# =============================================================================
# tuning panel (#43): four groups, every value editable, with its meaning and
# when it takes effect. Edits live in memory only; "Reset" restores config.py.
# =============================================================================
NEXT_INIT = "next Initialise"
LIVE = "live"
NEXT_PLAN = "next plan (and the preview)"
STM32 = "live: sent to the STM32 within 0.5 s"
NEXT_REST = "next initialisation at rest"


def _cfg(name, kind, unit, when, meaning, options=None):
    return {"name": name, "kind": kind, "unit": unit, "when": when, "meaning": meaning, "options": options,
            "get": lambda: getattr(config, name), "set": lambda v: setattr(config, name, v)}


def _seat(name, kind, unit, meaning):
    rt = lambda: RT
    return {"name": name, "kind": kind, "unit": unit, "when": LIVE + " (and the next Initialise)",
            "meaning": meaning, "options": None,
            "get": lambda: getattr(rt().seat_params, name), "set": lambda v: setattr(rt().seat_params, name, v)}


def registry():
    return [
        {"group": "LIDAR mount + calibration", "params": [
            _cfg("LIDAR_ANGLE_SIGN", "int", "", LIVE, "+1 if the sensor's raw angles run clockwise seen from above, "
                 "-1 if counter-clockwise (measured -1: the LIDAR is mounted upside down). Wrong = left and right "
                 "swapped, so CW reads as CCW.", options=[1, -1]),
            _cfg("LIDAR_ANGLE_ZERO_OFFSET_DEG", "float", "deg", LIVE, "Added after the sign so that 0 = the "
                 "chassis' forward direction. An object dead ahead must read about 0."),
            _cfg("LIDAR_OFFSET_FORWARD_MM", "float", "mm", NEXT_INIT, "How far the LIDAR sits ahead of the pose "
                 "reference point. Used by x/y, the seat check and the de-skew."),
            _cfg("LIDAR_OFFSET_LATERAL_MM", "float", "mm", NEXT_INIT, "How far the LIDAR sits to the LEFT of the "
                 "pose reference point."),
            _cfg("REAR_BLIND_ARC_CENTER_DEG", "float", "deg", NEXT_INIT, "Centre of the chassis-blocked wedge "
                 "(clockwise robot angle; 180 = straight back). Seats inside it are always UNKNOWN."),
            _cfg("REAR_BLIND_ARC_WIDTH_DEG", "float", "deg", NEXT_INIT, "Total width of that wedge (measured 105)."),
            _cfg("LIDAR_TIME_OFFSET_S", "float", "s", LIVE, "How much later a LIDAR return reaches the Pi than an "
                 "STM32 line, each from when it was measured. Measure with measure_lidar_delay.py. About +/-10 ms "
                 "of error is harmless; 30 ms costs wrong entry verdicts. (The mock's simulated LIDAR follows this value, "
                 "so changing it has no effect in mock mode.)"),
        ]},
        {"group": "Initialisation thresholds", "params": [
            _cfg("LANE_WIDTH_MM", "float", "mm", NEXT_INIT, "Outer wall to island wall. Rulebook 1000. "
                 "d(90) + d(270) must match it within the tolerance. (The mock world keeps 1000.)"),
            _cfg("LANE_WIDTH_TOLERANCE_MM", "float", "mm", NEXT_INIT, "Margin on that sum. Too tight rejects a "
                 "good start; too loose accepts a pillar as a wall."),
            _cfg("SIDE_WALL_HALF_DEG", "float", "deg", NEXT_INIT, "Returns within this of 90/270 are searched "
                 "for the side walls."),
            _cfg("SIDE_WALL_BIN_MM", "float", "mm", NEXT_INIT, "Histogram bin for finding the wall's distance."),
            _cfg("SIDE_WALL_BAND_MM", "float", "mm", NEXT_INIT, "Returns within this of the wall's peak go into "
                 "the line fit."),
            _cfg("SIDE_WALL_INLIER_MM", "float", "mm", NEXT_INIT, "Returns farther than this off the fitted line "
                 "are dropped and the line refitted."),
            _cfg("SIDE_WALL_MIN_POINTS", "int", "", NEXT_INIT, "A wall peak needs at least this many returns."),
            _cfg("GAP_MARGIN_MM", "float", "mm", NEXT_INIT, "A forward return this far beyond the side-wall "
                 "line counts as 'passed through' the gap."),
            _cfg("GAP_OPEN_MIN_MM", "float", "mm", NEXT_INIT, "The island side needs at least this much opening."),
            _cfg("INIT_USE_WALL_YAW", "bool", "", NEXT_INIT, "P2: measure y along the lane using the fitted wall "
                 "yaw (the car need not be placed exactly straight).", options=[True, False]),
            _cfg("GAP_CLOSED_MAX_MM", "float", "mm", NEXT_INIT, "...and the other side at most this much, or the "
                 "direction is UNDETERMINED."),
            _cfg("GAP_MIN_ANGLE_FROM_FWD_DEG", "float", "deg", NEXT_INIT, "Rays closer to dead ahead than this "
                 "are skipped by the gap test."),
            _cfg("GAP_FIT_AGREE_DEG", "float", "deg", NEXT_INIT, "The two side walls' fitted tilts must agree "
                 "within this (they are parallel)."),
            _cfg("FRONT_FAN_HALF_DEG", "float", "deg", NEXT_INIT, "y uses returns within this of dead ahead."),
            _cfg("FRONT_BAND_MM", "float", "mm", NEXT_INIT, "The wall ahead = returns within this of the farthest "
                 "forward distance (anything nearer is standing in front of it)."),
        ]},
        {"group": "Tracker + IMU", "params": [
            _cfg("IMU_YAW_SIGN", "int", "", LIVE, "-1: the chip's yaw decreases when the robot turns clockwise "
                 "(owner, #36). Wrong = every turn goes the wrong way. (In mock mode the simulated chip is built "
                 "with the value in force at Initialise.)", options=[1, -1]),
            _cfg("ENCODER_TICKS_PER_CM", "float", "ticks/cm", NEXT_INIT, "Hall-encoder ticks per cm (owner: "
                 "14.853). 2% off gives about 50 mm of error over a lap. Also sent to the STM32 (its speed loop's "
                 "speed estimate) within 0.5 s."),
            _cfg("MAX_SPEED_MM_S", "float", "mm/s", LIVE, "An encoder step faster than this is a glitch and is not "
                 "integrated. Must be above the robot's real top speed."),
            _cfg("TURN_MIN_DEG", "float", "deg", LIVE, "A turn needs this much rotation toward the round "
                 "direction..."),
            _cfg("TURN_GATE_Y_MM", "float", "mm", LIVE, "...and the robot past this y (the island ends at 2000)."),
            _cfg("TURN_FAILSAFE_DEG", "float", "deg", LIVE, "This much rotation alone is always a turn."),
            _cfg("RECHECK_Y_MAX_MM", "float", "mm", LIVE, "The entry re-check runs until the robot passes this y "
                 "in the new lane."),
            _cfg("RECHECK_ALIGN_DEG", "float", "deg", LIVE, "Re-check frames are used only while the heading is "
                 "within this of the lane's direction."),
            _cfg("RECHECK_EXTEND", "bool", "", LIVE, "Past RECHECK_Y_MAX_MM, keep re-checking seats still "
                 "at least RECHECK_AHEAD_MIN_MM ahead (#81).", options=[True, False]),
            _cfg("RECHECK_AHEAD_MIN_MM", "float", "mm", LIVE, "...a seat must be at least this far ahead to be "
                 "re-checked in the extension."),
            _cfg("RECHECK_START_LANE_ON_RETURN", "bool", "", LIVE, "Re-check the start lane once when the car "
                 "comes back to it after lap 1 (#82).", options=[True, False]),
            _cfg("DESKEW_HISTORY_S", "float", "s", NEXT_INIT, "Pose history kept for the de-skew; LIDAR returns "
                 "older than this are dropped as stale."),
            _cfg("IMU_STALE_S", "float", "s", LIVE, "No STM32 line for this long = link STALE."),
        ]},
        {"group": "Seat detector", "params": [
            _seat("angular_margin_deg", "float", "deg", "Search half-window around each seat's predicted bearing, on "
                  "top of the pillar's own width. Covers heading error."),
            _seat("range_tol_mm", "float", "mm", "Range agreement, fixed part. Covers position error."),
            _seat("range_tol_frac", "float", "", "Range agreement, proportional part (far seats judged more "
                  "leniently)."),
            _seat("min_points", "int", "", "Returns needed on a pillar face."),
            _seat("max_width_factor", "float", "x", "A run wider than this many pillar widths is a wall, not a "
                  "pillar."),
            _seat("max_range_step_mm", "float", "mm", "Consecutive returns farther apart than this are different "
                  "surfaces."),
            _seat("min_expected_hits", "float", "returns", "EMPTY needs a pillar there to have produced at least "
                  "this many returns; otherwise UNKNOWN."),
            _seat("min_observable_face_mm", "float", "mm", "Seats nearer than this are UNKNOWN (sensor dead zone)."),
            _seat("seat_position_slack_mm", "float", "mm", "Slack for a sign nudged inside its circle, and mat "
                  "tolerance."),
            _seat("min_range_mm", "float", "mm", "Returns nearer than this are ignored by the seat check."),
            _seat("max_range_mm", "float", "mm", "Returns farther than this are ignored by the seat check."),
        ]},
        {"group": "Steering + drive firmware (STM32)", "params": [
            _cfg("SERVO_STRAIGHT_DEG", "float", "servo deg", STM32, "Servo angle that puts the wheels straight "
                 "(OpenRound 76.5). Trim it until the car drives straight with steering 0."),
            _cfg("SERVO_LEFT_STOP_DEG", "float", "servo deg", STM32, "Servo angle at full LEFT lock (below "
                 "straight steers left; OpenRound 20)."),
            _cfg("SERVO_RIGHT_STOP_DEG", "float", "servo deg", STM32, "Servo angle at full RIGHT lock (OpenRound 140)."),
            _cfg("STEER_LOCK_LEFT_DEG", "float", "deg", STM32 + "; " + NEXT_PLAN, "PLACEHOLDER. Road-wheel "
                 "(bicycle) angle at the left stop: the planner's tightest left arc and the servo map's scale. "
                 "From the 27 cm radius read at the outer front wheel; 31.8 if at the outer rear wheel, 26.7 at "
                 "the rear-axle midpoint. Measure it (RUNNING_ON_THE_ROBOT 6.5)."),
            _cfg("STEER_LOCK_RIGHT_DEG", "float", "deg", STM32 + "; " + NEXT_PLAN, "PLACEHOLDER. The same to the "
                 "right (25 cm: 40.5; outer rear wheel 34.3, rear-axle midpoint 28.5)."),
            _cfg("SPEED_KFF", "float", "PWM per mm/s", STM32, "PLACEHOLDER. Speed feed-forward slope "
                 "(drive_calibrate.py measures it)."),
            _cfg("SPEED_OFFSET_PWM", "float", "PWM", STM32, "PLACEHOLDER. PWM the motor needs to start turning."),
            _cfg("SPEED_KP", "float", "PWM per mm/s", STM32, "PLACEHOLDER. Speed loop proportional gain."),
            _cfg("SPEED_KI", "float", "PWM per mm", STM32, "PLACEHOLDER. Speed loop integral gain."),
        ]},
        {"group": "Planner", "params": [
            _cfg("PLAN_RADIUS_FACTOR", "float", "x", NEXT_PLAN, "Planned arcs are the lock radius times this "
                 "(1.25 leaves the follower steering to correct errors). Lower = tighter corners, less margin."),
            _cfg("PLAN_INFLATION_MM", "float", "mm", NEXT_PLAN, "Obstacles are grown by this for the "
                 "visibility graph (car half-width 57.2 + clearance)."),
            _cfg("PLAN_CLEARANCE_MM", "float", "mm", NEXT_PLAN, "The car's real footprint must stay this far from "
                 "everything along the whole path."),
            _cfg("PLAN_INFLATION_STEP_MM", "float", "mm", NEXT_PLAN, "An obstacle the footprint check hits is "
                 "inflated by this much more and the path re-planned."),
            _cfg("PLAN_MAX_ITER", "int", "", NEXT_PLAN, "...at most this many times."),
            _cfg("PLAN_MAX_TURN_DEG", "float", "deg", NEXT_PLAN, "No single arc turns more than this."),
            _cfg("START_LANE_OUTER_SEATS_EMPTY", "bool", "", NEXT_PLAN, "Rulebook Fig. 8e: an UNKNOWN outer seat of "
                 "the start lane is not treated as a possible pillar.", options=[True, False]),
            _cfg("PLAN_PREVIEW_ENABLED", "bool", "", LIVE, "Dashboard: re-plan and draw the planner preview.",
                 options=[True, False]),
            _cfg("PLAN_PREVIEW_S", "float", "s", LIVE, "Dashboard: how often the preview re-plans."),
        ]},
        {"group": "Mission + speeds", "params": [
            _cfg("SPEED_LAP1_MM_S", "float", "mm/s", NEXT_PLAN, "Cruise speed on lap 1."),
            _cfg("SPEED_LAPS23_MM_S", "float", "mm/s", NEXT_PLAN, "Cruise speed on laps 2-3."),
            _cfg("SPEED_MIN_MM_S", "float", "mm/s", LIVE, "The follower never asks for less than this while moving."),
            _cfg("LAT_ACCEL_MM_S2", "float", "mm/s²", NEXT_PLAN, "Arc speed limit: v <= sqrt(this x radius)."),
            _cfg("DECEL_MM_S2", "float", "mm/s²", NEXT_PLAN, "Braking into a stop."),
            _cfg("VIEW_X_MM", "list", "mm", NEXT_PLAN, "Viewing-pose candidates across the new lane (x from the "
                 "outer wall), first preferred. JSON list, e.g. [500, 350, 650]."),
            _cfg("VIEW_Y_MM", "float", "mm", NEXT_PLAN, "Viewing pose: how far into the new lane (500 = the corner "
                 "square's centre)."),
            _cfg("LOOK_SETTLE_S", "float", "s", LIVE, "At a viewing pose: stand at least this long before planning."),
            _cfg("LOOK_TIMEOUT_S", "float", "s", LIVE, "...and at most this long waiting for seats and colours."),
            _cfg("COLOR_LOOK_DIST_MM", "float", "mm", LIVE, "Stop when a pillar of unknown colour is this close ahead."),
            _cfg("COLOR_LOOK_RETRIES", "int", "", LIVE, "Colour re-requests before the pillar is passed either side."),
            _cfg("REPLAN_DEVIATION_MM", "float", "mm", LIVE, "Laps 2-3: re-plan when the car is this far off the path."),
            _cfg("REVERSE_SPEED_MM_S", "float", "mm/s", LIVE, "Backing up when no forward path exists."),
            _cfg("REVERSE_STEPS_MM", "list", "mm", LIVE, "Back-up distances tried, shortest first. JSON list."),
            _cfg("REVERSE_MAX_PER_STOP", "int", "", LIVE, "Back-ups allowed per stop."),
            _cfg("REVERSE_CLEARANCE_MM", "float", "mm", LIVE, "A back-up must stay this far from everything."),
            _cfg("FINISH_MARGIN_MM", "float", "mm", NEXT_PLAN, "Stop with the whole outline this far inside the "
                 "start section."),
        ]},
        {"group": "Path follower", "params": [
            _cfg("FOLLOWER_MODE", "str", "", NEXT_PLAN, "rwf = rear-wheel feedback (default), pp = pure pursuit.",
                 options=["rwf", "pp"]),
            _cfg("RWF_LENGTH_MM", "float", "mm", LIVE, "rwf: a lateral error dies out over about this distance."),
            _cfg("RWF_DAMPING", "float", "", LIVE, "rwf: damping of the lateral error."),
            _cfg("RWF_PREVIEW_S", "float", "s", LIVE, "rwf: curvature is taken this far ahead (servo lag + link latency)."),
            _cfg("PP_LOOKAHEAD_S", "float", "s", LIVE, "pp: lookahead = speed x this ..."),
            _cfg("PP_LOOKAHEAD_MIN_MM", "float", "mm", LIVE, "pp: ... at least this ..."),
            _cfg("PP_LOOKAHEAD_MAX_MM", "float", "mm", LIVE, "pp: ... at most this."),
            _cfg("DRIVE_HZ", "float", "Hz", LIVE, "DRIVE frames (control periods) per second."),
        ]},
        {"group": "Run control (start button)", "params": [
            _cfg("START_STILL_WINDOW_S", "float", "s", LIVE, "At rest = over this long ..."),
            _cfg("START_STILL_ENC_TICKS", "int", "ticks", LIVE, "... the encoder moved at most this ..."),
            _cfg("START_STILL_YAW_DEG", "float", "deg", LIVE, "... and the yaw at most this."),
            _cfg("START_SCAN_MARGIN_S", "float", "s", NEXT_REST, "Use LIDAR returns measured this long after the "
                 "rest began."),
            _cfg("START_MIN_RETURNS", "int", "", NEXT_REST, "Fewer fresh returns: wait for more."),
            _cfg("START_RETRY_S", "float", "s", LIVE, "A failed initialisation is retried this often at rest."),
        ]},
        {"group": "Pillar colour + camera", "params": [
            _cfg("COLOR_ID_MIN_FRACTION", "float", "", LIVE, "P3: the winning colour's share of the pillar's ROI."),
            _cfg("COLOR_ID_MARGIN_RATIO", "float", "x", LIVE, "...and at least this many times the other colour's."),
            _cfg("COLOR_ID_ROI_MARGIN_FACTOR", "float", "x", LIVE, "ROI = the pillar's projected box widened by this."),
            _cfg("COLOR_ID_WINDOW_S", "float", "s", LIVE, "A colour request stays open this long after the first "
                 "in-view frame ..."),
            _cfg("COLOR_ID_MAX_ATTEMPTS", "int", "", LIVE, "... for at most this many in-view frames."),
            _cfg("COLOR_MIN_SAT", "int", "0-255", LIVE, "HSV: pixels less saturated than this count for no colour."),
            _cfg("COLOR_MIN_VAL", "int", "0-255", LIVE, "HSV: pixels darker than this count for no colour."),
            _cfg("COLOR_RED_HUE", "list", "OpenCV hue", LIVE, "Red hue ranges (0-179, wraps). JSON, e.g. "
                 "[[0, 10], [170, 179]]."),
            _cfg("COLOR_GREEN_HUE", "list", "OpenCV hue", LIVE, "Green hue ranges. JSON, e.g. [[40, 85]]."),
            _cfg("CAMERA_BEARING_SIGN", "int", "", LIVE, "Flip if boxes land mirrored (camera_check.py).",
                 options=[1, -1]),
            _cfg("CAMERA_ROTATE_180", "bool", "", LIVE, "The image is upside down.", options=[True, False]),
            _cfg("CAMERA_HEIGHT_MM", "float", "mm", LIVE, "Lens height above the floor."),
            _cfg("CAMERA_OFFSET_FORWARD_MM", "float", "mm", LIVE, "Lens ahead of the rear axle."),
            _cfg("CAMERA_OFFSET_LATERAL_MM", "float", "mm", LIVE, "Lens to the left of the centre line."),
            _cfg("CAMERA_TIME_OFFSET_S", "float", "s", LIVE, "Camera timestamps relative to the STM32 clock."),
        ]},
    ]


def _flat():
    return {p["name"]: p for g in registry() for p in g["params"]}


RT: Runtime | None = None
DEFAULTS: dict = {}


def _tuple(v):
    return tuple(_tuple(x) for x in v) if isinstance(v, (list, tuple)) else v


def _coerce(p, v):
    if p["kind"] == "int":
        v = int(float(v))
    elif p["kind"] == "float":
        v = float(v)
        if not math.isfinite(v):
            raise ValueError("not a finite number")
    elif p["kind"] == "bool":
        if isinstance(v, str):
            if v.strip().lower() not in ("true", "false", "1", "0"):
                raise ValueError("must be true or false")
            v = v.strip().lower() in ("true", "1")
        v = bool(v)
    elif p["kind"] == "list":
        if isinstance(v, str):
            v = json.loads(v)
        if not isinstance(v, (list, tuple)) or not v:
            raise ValueError("must be a non-empty JSON list")
        flat = json.dumps(v)
        if any(c.isalpha() for c in flat.replace("e", "")):
            raise ValueError("numbers only")
        old = p["get"]()
        if isinstance(old, (list, tuple)) and old and isinstance(old[0], (list, tuple)) != isinstance(v[0], (list, tuple)):
            raise ValueError(f"must have the same shape as {json.dumps(old)}")
        v = _tuple(v)
    elif p["kind"] == "str":
        v = str(v)
    if p["options"] is not None and v not in p["options"]:
        raise ValueError(f"must be one of {p['options']}")
    return v


# =============================================================================
# routes
# =============================================================================
@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/stream")
def stream():
    def gen():
        while True:
            yield f"data: {json.dumps(RT.state())}\n\n"
            time.sleep(1.0 / max(1, config.STREAM_HZ))
    return Response(gen(), mimetype="text/event-stream")


@app.route("/api/state")
def api_state():
    return jsonify(RT.state())


@app.route("/api/static")
def api_static():
    with RT.lock:
        return jsonify(RT.static)


@app.route("/api/initialise", methods=["POST"])
def api_initialise():
    return jsonify(RT.initialise())


@app.route("/api/mock", methods=["POST"])
def api_mock():
    if RT.mode != "mock":
        return jsonify({"ok": False, "reason": "not in mock mode"}), 400
    body = request.get_json(force=True, silent=True) or {}
    m = dict(RT.mock)
    try:
        if "direction" in body:
            if body["direction"] not in ("CCW", "CW"):
                raise ValueError("direction must be CCW or CW")
            m["direction"] = body["direction"]
        if "start_slot" in body:
            m["start_slot"] = int(body["start_slot"]) % 4
        if "seed" in body:
            m["seed"] = int(body["seed"])
        if "speed_mm_s" in body:
            m["speed_mm_s"] = min(1500.0, max(100.0, float(body["speed_mm_s"])))
        if "time_scale" in body:
            m["time_scale"] = min(8.0, max(0.25, float(body["time_scale"])))
    except (TypeError, ValueError) as e:
        return jsonify({"ok": False, "reason": str(e)}), 400
    with RT.lock:
        RT.mock = m
    return jsonify({"ok": True, "mock": m, "note": "applies at the next Initialise"})


@app.route("/api/tuning")
def api_tuning():
    groups = []
    for g in registry():
        ps = []
        for p in g["params"]:
            v = p["get"]()
            ps.append({"name": p["name"], "value": v, "default": DEFAULTS.get(p["name"]), "unit": p["unit"],
                       "when": p["when"], "meaning": p["meaning"], "kind": p["kind"], "options": p["options"],
                       "text": json.dumps(v) if p["kind"] == "list" else None,
                       "changed": DEFAULTS.get(p["name"]) != v})
        groups.append({"group": g["group"], "params": ps})
    with RT.lock:
        fw = RT._fw_params()
    return jsonify({"groups": groups, "fw": fw})


@app.route("/api/param", methods=["POST"])
def api_param():
    body = request.get_json(force=True, silent=True) or {}
    p = _flat().get(body.get("name"))
    if p is None:
        return jsonify({"ok": False, "error": f"unknown parameter {body.get('name')!r}"}), 400
    try:
        v = _coerce(p, body.get("value"))
        with RT.lock:
            p["set"](v)
    except (TypeError, ValueError, json.JSONDecodeError) as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "name": p["name"], "value": p["get"](), "when": p["when"]})


@app.route("/api/tuning/reset", methods=["POST"])
def api_tuning_reset():
    n = 0
    with RT.lock:
        for name, p in _flat().items():
            if name in DEFAULTS:
                p["set"](DEFAULTS[name])
                n += 1
    return jsonify({"ok": True, "restored": n})


@app.route("/api/save-run", methods=["POST"])
def api_save_run():
    return jsonify(RT.save_run())


@app.route("/api/mission/<action>", methods=["POST"])
def api_mission(action):
    if not isinstance(RT, MissionRuntime):
        return jsonify({"ok": False, "reason": "not in mission mode"}), 400
    if action == "press":
        return jsonify(RT.press())
    if action == "carry":
        return jsonify(RT.carry_to_start())
    if action == "layout":
        body = request.get_json(force=True, silent=True) or {}
        d = body.get("direction")
        if d not in (None, "CCW", "CW"):
            return jsonify({"ok": False, "reason": "direction must be CCW or CW"}), 400
        try:
            seed = int(body.get("seed", 1))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "reason": "seed must be an integer"}), 400
        return jsonify(RT.new_layout(d, seed))
    return jsonify({"ok": False, "reason": f"unknown action {action!r}"}), 400


def _capture_defaults():
    global DEFAULTS
    DEFAULTS = {name: p["get"]() for name, p in _flat().items()}


def create_runtime(mode: str | None = None) -> Runtime:
    """Start the runtime (hardware or simulation) and capture the tuning
    defaults. Used by main() and by the tests."""
    global RT
    RT = Runtime(mode or config.MODE)
    _capture_defaults()
    return RT


def create_mission_runtime(mode: str | None = None, link=None, drive=None, lidar=None, camera=None,
                           dump=None, seed: int = 1, direction: str | None = None) -> MissionRuntime:
    """Mission mode. mock: a fresh mission_sim.MissionSim. real: the hardware the
    caller opened (run_mission.py --dashboard)."""
    global RT
    mode = mode or config.MODE
    if mode == "mock":
        from drive_link import DriveLink
        from mission_sim import MissionSim
        sim = MissionSim(direction, seed)
        RT = MissionRuntime("mock", sim.link, DriveLink(sim.link), sim.lidar, sim.camera, sim=sim)
    else:
        RT = MissionRuntime("real", link, drive, lidar, camera, dump=dump)
    _capture_defaults()
    return RT


def serve():
    print(f"[dashboard] {RT.mode} on http://{config.DASHBOARD_HOST}:{config.DASHBOARD_PORT}/")
    app.run(host=config.DASHBOARD_HOST, port=config.DASHBOARD_PORT, threaded=True, debug=False)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", action="store_true",
                    help="the run supervisor + mission (mock: the simulated car; real: use run_mission.py --dashboard)")
    ap.add_argument("--seed", type=int, default=1, help="mission mock: the rulebook layout")
    ap.add_argument("--direction", choices=["CCW", "CW"], help="mission mock: force the direction")
    a = ap.parse_args()
    if a.mission:
        if config.MODE != "mock":
            raise SystemExit("on the robot, mission mode is: python3 run_mission.py --dashboard")
        create_mission_runtime("mock", seed=a.seed, direction=a.direction)
    else:
        create_runtime()
    serve()


if __name__ == "__main__":
    main()
