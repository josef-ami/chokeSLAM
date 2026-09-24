"""
The Pi half of the run (checkpoint F): start, stop and restart on the one
start button, which the STM32 owns (firmware/drive_bridge/drive_protocol.h).

    READY / STOPPED / FINISHED --press, Pi ready--> RUNNING   (the STM32 decides)
    RUNNING --press--> STOPPED                                 (the STM32 decides)
    RUNNING --the mission ends (DONE / FAILED)--> FINISHED     (the Pi sends RUN_OVER)

While no run is going the Pi keeps itself READY for the next press:

    WAIT     every IMU sample goes to a stillness test. Once the car has stood
             still for START_STILL_WINDOW_S (encoder and yaw), the Pi takes the
             LIDAR returns measured since it came to rest, initialises
             (lane_init) and builds the lane tracker from that moment. It then
             sends PI_READY on every frame (status LED solid) and keeps feeding
             the tracker. Any motion clears PI_READY; the next rest re-initialises.
             A failed initialisation is retried every START_RETRY_S while the
             car stands still (LED blinking 250 ms: place the car again).
    RUN      the STM32 reports RUNNING with a new run id: the mission runs with
             the tracker held at the press. The STM32 leaving RUNNING (a press)
             ends it at once; the car is re-initialised when it next stands still.
    ENDING   the mission ended by itself: RUN_OVER is sent until the STM32
             reports FINISHED.

Every run starts from nothing: a new initialisation, tracker and mission (lap 1
maps the field again). A RUNNING state found when this program starts belongs
to an earlier Pi process and is ended with RUN_OVER.

The supervisor only talks to interfaces (link.drain / drive.* / lidar /
camera / the mission factory), so test_run_control.py drives it with the real
firmware logic compiled on the host and a simulated field.
"""
from __future__ import annotations

import time
from collections import deque

import config
import lane_init as li
from drive_link import RUN_RUNNING, RUN_STATE_NAMES
from follower import DriveCmd
from lane_tracker import LaneTracker
from scan_processing import clean_and_project, clean_and_project_timed

STOP = DriveCmd(0.0, 0.0, stop=True)


class Stillness:
    """Still = over the last `window_s` of STM32 time the encoder moved at most
    `enc_ticks` and the yaw at most `yaw_deg`. `since` is the Pi receive time of
    the first sample of the current still stretch (None while moving)."""

    def __init__(self, window_s=None, enc_ticks=None, yaw_deg=None):
        self.window_ms = 1000.0 * (config.START_STILL_WINDOW_S if window_s is None else window_s)
        self.enc_ticks = config.START_STILL_ENC_TICKS if enc_ticks is None else enc_ticks
        self.yaw_deg = config.START_STILL_YAW_DEG if yaw_deg is None else yaw_deg
        self.buf: deque = deque()
        self.since = None

    def add(self, s):
        self.buf.append(s)
        while len(self.buf) > 2 and s.t_ms - self.buf[1].t_ms >= self.window_ms:
            self.buf.popleft()
        if self.still():
            if self.since is None:
                self.since = self.buf[0].rx_time
        else:
            self.since = None

    def still(self) -> bool:
        if len(self.buf) < 2 or self.buf[-1].t_ms - self.buf[0].t_ms < self.window_ms:
            return False
        encs = [s.enc for s in self.buf]
        y0 = self.buf[0].yaw_deg
        dy = [(s.yaw_deg - y0 + 180.0) % 360.0 - 180.0 for s in self.buf]
        return max(encs) - min(encs) <= self.enc_ticks and max(dy) - min(dy) <= self.yaw_deg


class RunSupervisor:
    def __init__(self, link, drive, lidar, camera=None, make_mission=None, log=print, dump=None, seat_params=None):
        self.link, self.drive, self.lidar, self.camera = link, drive, lidar, camera
        self.seat_params = seat_params    # seat_occupancy.DetectParams (the dashboard edits it live), or None
        self.make_mission = make_mission
        self.log = log
        self.dump = dump                  # callable(run_id, raw, init) or None
        self.mode = "WAIT"
        self.still = Stillness()
        self.prep = None                  # (init, tracker) held for the next press
        self.prep_raw = None
        self.moved = True                 # motion seen since prep was taken
        self.last_try = -1e9
        self.handled_id = None            # run id this program has acted on
        self.seen_state = False
        self.trk = self.mis = None
        self.t_start = 0.0
        self.last_cam = None
        self.runs = []                    # (run_id, outcome) for the record
        self.last_init = None             # the initialisation of the current / last run

    # -- one control period ------------------------------------------------
    def step(self, now=None):
        now = time.monotonic() if now is None else now
        if hasattr(self.drive, "sync_params"):
            self.drive.sync_params(now)           # the STM32's tuning values = config.py (checkpoint F2)
        state, rid = self.drive.run_state()
        if state is not None and not self.seen_state:
            self.seen_state = True
            if state == RUN_RUNNING:
                self.log(f"[run] STM32 is RUNNING run {rid} from an earlier program: ending it")
                self.handled_id = rid
                self._end("stale")
        if self.mode == "WAIT":
            self._wait(now, state, rid)
        elif self.mode == "RUN":
            self._run(now, state, rid)
        else:
            self._ending(state)

    def tracker(self):
        """The tracker to show: the running mission's, else the one held for the next press."""
        if self.trk is not None:
            return self.trk
        return None if self.prep is None else self.prep[1]

    def init_result(self):
        if self.prep is not None:
            return self.prep[0]
        return self.last_init

    # -- WAIT: keep a valid initialisation for the next press ---------------
    def _wait(self, now, state, rid):
        for s in self.link.drain():
            self.still.add(s)
            if self.prep is not None:
                self.prep[1].on_imu(s)
        if self.still.since is None:
            if not self.moved and self.prep is not None:
                self.log("[run] car moved: not ready until it stands still again")
            self.moved = True
        elif (self.prep is None or self.moved) and now - self.last_try >= config.START_RETRY_S:
            self.last_try = now
            self._prepare()
        self.drive.ready = self.prep is not None and not self.moved
        if state == RUN_RUNNING and rid != self.handled_id:
            self.handled_id = rid
            if self.prep is None:
                self.log(f"[run] run {rid} started but no initialisation is held: ending it")
                self._end("no init")
                return
            self._start(now, rid)
            return
        self.drive.send(STOP)

    def _prepare(self):
        since = self.still.since + config.START_SCAN_MARGIN_S + config.LIDAR_TIME_OFFSET_S
        raw4 = [r for r in self.lidar.get_latest_scan_timed() if r[3] >= since]
        if len(raw4) < config.START_MIN_RETURNS:
            return
        raw = [(a, d, q) for a, d, q, _ in raw4]
        first = self.still.buf[-1]
        init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG),
                             params=self.seat_params)
        if not init.ok:
            if self.prep is not None or self.moved:
                self.log(f"[run] initialisation failed ({init.reason}): place the car again")
            self.prep, self.moved = None, False
            return
        self.prep = (init, LaneTracker(init, first, params=self.seat_params))
        self.prep_raw = raw
        self.moved = False
        self.log(f"[run] ready: {init.direction.direction} x {init.x.x_mm:.0f} y {init.y.y_mm:.0f} mm "
                 f"-- press the button to start")

    # -- RUN ---------------------------------------------------------------
    def _start(self, now, rid):
        init, self.trk = self.prep
        self.last_init = init
        self.drive.ready = False
        if self.dump is not None:
            self.dump(rid, self.prep_raw, init)
        self.prep = None
        self.mis = self.make_mission(self.trk)
        self.t_start = now
        self.mode = "RUN"
        self.log(f"[run] run {rid} START")
        self._run(now, RUN_RUNNING, rid)

    def _run(self, now, state, rid):
        trk, mis = self.trk, self.mis
        if state is not None and (state != RUN_RUNNING or rid != self.handled_id):
            self.log(f"[run] run {self.handled_id} {RUN_STATE_NAMES.get(state, state)} by the button "
                     f"after {now - self.t_start:.1f} s")
            self.runs.append((self.handled_id, "stopped"))
            self._to_wait()
            self.drive.send(STOP)
            return
        raw4 = self.lidar.get_latest_scan_timed() if trk.wants_lidar else None
        samples = self.link.drain()
        for s in samples:
            trk.on_imu(s)
            self.still.add(s)
        if raw4 and trk.wants_lidar:
            pts, times = clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            trk.on_lidar_frame(pts, times)
        if self.camera is not None and trk.wants_camera:
            fr = self.camera.get_latest_frame()
            if fr is not None and fr[2] != self.last_cam:
                self.last_cam = fr[2]
                trk.on_camera_frame(fr[0], fr[1])
        v = 0.0
        if len(samples) >= 2:
            dt = (samples[-1].t_ms - samples[0].t_ms) / 1000.0
            if dt > 0:
                v = (samples[-1].enc - samples[0].enc) * 10.0 / config.ENCODER_TICKS_PER_CM / dt
        cmd = mis.update(now - self.t_start, v)
        if mis.phase in ("DONE", "FAILED"):
            self.log(f"[run] run {self.handled_id} mission {mis.phase} after {now - self.t_start:.1f} s")
            self.runs.append((self.handled_id, mis.phase))
            self._end(mis.phase)
            return
        self.drive.send(cmd)

    # -- ENDING: RUN_OVER until the STM32 leaves RUNNING --------------------
    def _end(self, why):
        self.mode = "ENDING"
        self.drive.ready = False
        self.drive.run_over = True
        self.drive.send(STOP)

    def _ending(self, state):
        self.link.drain()
        if state is not None and state != RUN_RUNNING:
            self.drive.run_over = False
            self._to_wait()
        self.drive.send(STOP)

    def _to_wait(self):
        self.mode = "WAIT"
        self.trk = self.mis = None
        self.prep = None
        self.moved = True
        self.still = Stillness()
