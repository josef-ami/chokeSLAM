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
import seat_occupancy as so
from lane_tracker import LaneTracker
from scan_processing import clean_and_project, clean_and_project_timed

app = Flask(__name__)
RUNS_DIR = "runs"                      # Save-run writes runs/<date-time>/scan.json + imu.log
LIDAR_READ_S = 0.05                    # read a LIDAR frame every 50 ms
LOOP_S = 0.02
MAX_LIVE_POINTS = 720
MAX_PATH_POINTS = 2000


def _r(v, nd=1):
    return None if v is None else round(float(v), nd)


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
        self._start_sources()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="dashboard_runtime")
        self._thread.start()

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
            self.link = Stm32Link()
            self.link.start()
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
        if self.mode != "mock":
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
            if self.mode == "mock" and self.sim is not None:
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
# tuning panel (#43): four groups, every value editable, with its meaning and
# when it takes effect. Edits live in memory only; "Reset" restores config.py.
# =============================================================================
NEXT_INIT = "next Initialise"
LIVE = "live"


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
                 "14.853). 2% off gives about 50 mm of error over a lap."),
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
    ]


def _flat():
    return {p["name"]: p for g in registry() for p in g["params"]}


RT: Runtime | None = None
DEFAULTS: dict = {}


def _coerce(p, v):
    if p["kind"] == "int":
        v = int(float(v))
    elif p["kind"] == "float":
        v = float(v)
        if not math.isfinite(v):
            raise ValueError("not a finite number")
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
                       "changed": DEFAULTS.get(p["name"]) != v})
        groups.append({"group": g["group"], "params": ps})
    return jsonify({"groups": groups})


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
    except (TypeError, ValueError) as e:
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


def create_runtime(mode: str | None = None) -> Runtime:
    """Start the runtime (hardware or simulation) and capture the tuning
    defaults. Used by main() and by the tests."""
    global RT, DEFAULTS
    RT = Runtime(mode or config.MODE)
    DEFAULTS = {name: p["get"]() for name, p in _flat().items()}
    return RT


def main():
    create_runtime()
    print(f"[dashboard] {config.MODE} mode on http://{config.DASHBOARD_HOST}:{config.DASHBOARD_PORT}/")
    app.run(host=config.DASHBOARD_HOST, port=config.DASHBOARD_PORT, threaded=True, debug=False)


if __name__ == "__main__":
    main()
