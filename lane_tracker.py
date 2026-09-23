"""
Lane tracker: everything after initialisation. Moves the robot through the
lane frame from the STM32 feed only (BNO08x heading + drive-motor hall
encoder), detects the turns, switches to the next lane's frame at each turn,
and -- on lap 1 only, for the lanes after the start lane -- re-checks that
lane's six seats with the LIDAR in its entry corner.

Design: docs/CHANGES.md section 9 (decisions #13-#20, #23, #25, #26, #35, #36).

STATE (all in the CURRENT lane's frame, see lane_frame.py)
    x    distance from the outer wall (mm)
    y    along the lane from the wall behind (mm)
    psi  heading relative to the lane's grid north, clockwise + (deg)
    lane_index  0 = the start lane, +1 per turn; slot = lane_index % 4;
                lap = lane_index // 4 + 1

START (decision #35)
    x, y   from initialisation (lane_init.InitResult)
    psi0   the placement yaw measured by the direction test's wall fit,
           psi0 = -DirectionResult.wall_angle_deg (the walls appear rotated the
           opposite way to the robot). Initialisation's own x/y/seats keep
           yaw 0 as agreed (#4); this only seeds the tracker's heading.
    references: `first_sample`, which must be the STM32 sample current when the
           start scan was taken (run_track.py pairs them); every later sample
           is integrated from there.

EACH STM32 SAMPLE
    heading += IMU_YAW_SIGN * wrap180(yaw - yaw_prev)      (unwrapped, clockwise +)
    ds       = (enc - enc_prev) * 10 / ENCODER_TICKS_PER_CM (mm, forward +)
    psi      = wrap180(heading - lane_north)
    psi_mid  = midpoint of the step's heading
    y += ds * cos(psi_mid)
    x += ds * sin(psi_mid) * (-1 for CCW, +1 for CW)        (x from the OUTER wall:
                                                             +x is the robot's left
                                                             for CCW, right for CW)
    Guards: seq going backwards (STM32 restarted) or an encoder step faster than
    MAX_SPEED_MM_S -> not integrated, references re-based, event logged; the
    tracked pose is kept.

TURN RULE (decision #25)
    toward = -psi (CCW, left turns) | +psi (CW, right turns)
    turn when (toward >= TURN_MIN_DEG and y >= TURN_GATE_Y_MM)
           or  toward >= TURN_FAILSAFE_DEG
    On a turn: (x, y, psi) -> lane_frame.corner_transform (new_y = old_x,
    new_x = 3000 - old_y, psi +/- 90), lane_north moves 90 deg toward the round
    direction, lane_index += 1. A pure change of coordinates, so switching a
    little late costs nothing; the y gate and threshold only guard against a
    false turn (a swerve, the parking manoeuvre).

SEATS PER LANE (decisions #15, #19, #20, #23, #26)
    slot 0 (start lane): the initialisation result, all run.
    slots 1-3, first visit (lap 1): entry re-check. On every LIDAR frame while
        y < RECHECK_Y_MAX_MM and |psi| <= RECHECK_ALIGN_DEG, run the seat check
        at the tracked pose (x, y, psi -- the IMU yaw, not 0); the first
        decided verdict per seat is kept, later frames only fill UNKNOWNs.
        Frozen once y >= RECHECK_Y_MAX_MM.
    later visits (laps 2-3): the lap-1 result, no re-check.

PILLAR COLOUR (checkpoint D, decisions #53-#65, color_id.py)
    Every seat that becomes PRESENT (initialisation or entry re-check) gets a
    colour request, once. on_camera_frame(frame, t) serves the requests: for
    each pending seat, the lane pose at the frame's capture time t (#62) gives
    the camera's view of the seat; frames where it is out of view don't count
    (#63); the first in-view frame opens a COLOR_ID_WINDOW_S window with up to
    COLOR_ID_MAX_ATTEMPTS in-view frames; the first confident read wins and is
    never overwritten; otherwise UNKNOWN. A request still open when its lane is
    left (the next turn) closes as UNKNOWN. Timed by the frames' own
    timestamps only (#61), not by the LIDAR or STM32 loops.

DE-SKEW (decision #38, deskew.py)
    Each STM32 sample is also integrated into an odometry pose (ox, oy,
    heading) in one fixed frame, and kept for DESKEW_HISTORY_S, timed on the
    Pi clock (timing.LinkClock). A LIDAR frame given with its returns'
    measurement times is de-skewed to the current pose before the seat check.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np

import color_id
import config
import lane_frame as lf
import seat_occupancy as so
from deskew import FRAME_SPAN_S, DeskewStats, PoseHistory, deskew
from lane_init import InitResult, detect_params_from_config
from stm32_link import ImuSample
from timing import LinkClock


def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class SeatState:
    state: str = "unknown"          # "occupied" | "empty" | "unknown"
    source: str = ""                # "init" | "entry"
    reason: str = ""
    at_y_mm: float | None = None    # tracked y when decided (entry re-check)
    at_lane_index: int | None = None
    # pillar colour (checkpoint D): None = not a PRESENT seat; "pending", "red", "green", "unknown"
    color: str | None = None
    color_reason: str = ""
    color_attempts: int = 0          # in-view frames tried
    color_out_of_view: int = 0       # frames in which the seat was out of the camera's view
    color_window_t: float | None = None   # capture time of the first in-view frame (window start)
    color_lane_index: int | None = None   # the lane (index) whose frame the seat is in


@dataclass
class LaneRecord:
    slot: int                       # 0 = start lane, then 1, 2, 3 in driving order
    first_lane_index: int
    source: str                     # "init" or "entry"
    seats: dict = field(default_factory=dict)   # seat index -> SeatState
    frozen: bool = False
    frames_used: int = 0            # LIDAR frames that went into the re-check
    frames_skipped_align: int = 0   # frames skipped by the +/-RECHECK_ALIGN_DEG gate
    returns_dropped_old: int = 0    # de-skew: returns older than the pose history (stale buckets)
    frames_skipped_old: int = 0     # frames that ended before this lane began (or before the history)
    frozen_t: float | None = None   # Pi time the re-check froze (a frame taken before it still counts)
    empty_downgraded: int = 0       # EMPTY verdicts turned UNKNOWN by the coverage check (P10)
    max_shift_mm: float = 0.0       # de-skew: the largest distance a return was moved


@dataclass
class TrackerEvent:
    t_ms: int
    kind: str                       # "start", "turn", "recheck_frozen", "glitch", "reset", ...
    detail: str
    lane_index: int


class LaneTracker:
    def __init__(self, init: InitResult, first_sample: ImuSample,
                 params: so.DetectParams | None = None):
        if not init.ok:
            raise ValueError(f"initialisation did not succeed: {init.reason}")
        if config.IMU_YAW_SIGN not in (1, -1):
            raise ValueError("config.IMU_YAW_SIGN must be +1 or -1")
        if not config.ENCODER_TICKS_PER_CM or config.ENCODER_TICKS_PER_CM <= 0:
            raise ValueError("config.ENCODER_TICKS_PER_CM must be a positive number")
        self.direction = init.direction.direction
        self._h = -1.0 if self.direction == "CCW" else 1.0     # sign of +x relative to the robot's right
        self.mm_per_tick = 10.0 / config.ENCODER_TICKS_PER_CM
        self.params = detect_params_from_config(params)

        self.x = float(init.x.x_mm)
        self.y = float(init.y.y_mm)
        wall = init.direction.wall_angle_deg
        self.psi0 = -wall if wall is not None else 0.0
        self.heading = self.psi0            # unwrapped, clockwise +, relative to the START lane's grid north
        self.lane_north = 0.0               # grid north of the current lane, same reference
        self.psi = _wrap180(self.heading - self.lane_north)
        self.lane_index = 0
        self.distance_mm = 0.0

        self._last = first_sample
        # odometry in one fixed frame (the start lane's orientation, never switched) for the de-skew
        self._clock = LinkClock()
        self._ox = self._oy = 0.0
        self.t_now = self._clock.update(first_sample.t_ms, first_sample.rx_time)
        self._hist = PoseHistory(config.DESKEW_HISTORY_S)
        self._hist.append(self.t_now, 0.0, 0.0, self.heading, self._lane_pose())
        self.last_deskew: DeskewStats | None = None
        self.events: list[TrackerEvent] = []
        self.path: list[tuple[int, float, float]] = [(0, self.x, self.y)]

        start = LaneRecord(slot=0, first_lane_index=0, source="init", frozen=True)
        for r in init.seats:
            start.seats[r.seat.index] = SeatState(r.state.value, "init", r.reason, init.y.y_mm, 0)
        self.lanes: dict[int, LaneRecord] = {0: start}
        self._color_pending: list[tuple[int, int]] = []     # (slot, seat index)
        self.camera_frames = 0            # frames offered while a colour request was open
        self.camera_frames_no_pose = 0    # ... skipped: capture time outside the pose history
        for i, st in start.seats.items():
            if st.state == so.Occupancy.OCCUPIED.value:
                self._request_color(0, i, 0)
        self._recheck_active = False
        self._event("start", f"{self.direction}, x {self.x:.0f}, y {self.y:.0f}, psi0 {self.psi0:+.2f} "
                             f"(wall-fit placement yaw)", first_sample.t_ms)

    # -- properties -----------------------------------------------------------
    @property
    def slot(self) -> int:
        return self.lane_index % 4

    @property
    def lap(self) -> int:
        return self.lane_index // 4 + 1

    @property
    def last_sample(self) -> ImuSample:
        """The newest STM32 sample integrated (the tracked pose is as of this one)."""
        return self._last

    @property
    def wants_camera(self) -> bool:
        """True while any PRESENT seat is waiting for its colour."""
        return bool(self._color_pending)

    @property
    def wants_lidar(self) -> bool:
        """True while the current lane's entry re-check is still open."""
        return self._recheck_active

    # -- STM32 samples ----------------------------------------------------------
    def on_imu(self, s: ImuSample) -> None:
        last = self._last
        self.t_now = self._clock.update(s.t_ms, s.rx_time)
        if s.seq <= last.seq:
            self._event("reset", f"seq went {last.seq} -> {s.seq} (STM32 restarted?); "
                                 f"references re-based, pose kept", s.t_ms)
            self._last = s
            self._hist.append(self.t_now, self._ox, self._oy, self.heading, self._lane_pose())
            return
        dt = max((s.t_ms - last.t_ms) / 1000.0, 0.001)
        ds = (s.enc - last.enc) * self.mm_per_tick
        dyaw = config.IMU_YAW_SIGN * _wrap180(s.yaw_deg - last.yaw_deg)
        if abs(ds) > config.MAX_SPEED_MM_S * dt + 50.0:
            self._event("glitch", f"encoder step {ds:.0f} mm in {dt * 1000:.0f} ms ignored "
                                  f"(> {config.MAX_SPEED_MM_S:.0f} mm/s)", s.t_ms)
            ds = 0.0
        psi_prev = self.psi
        h_mid = math.radians(self.heading + dyaw / 2.0)
        self.heading += dyaw
        self.psi = _wrap180(self.heading - self.lane_north)
        psi_mid = math.radians(psi_prev + _wrap180(self.psi - psi_prev) / 2.0)
        self.y += ds * math.cos(psi_mid)
        self.x += ds * math.sin(psi_mid) * self._h
        self._ox += ds * math.sin(h_mid)
        self._oy += ds * math.cos(h_mid)
        self.distance_mm += abs(ds)
        self._last = s
        self._check_turn(s.t_ms)
        if self._recheck_active and self.y >= config.RECHECK_Y_MAX_MM:
            self._freeze(s.t_ms)
        self._hist.append(self.t_now, self._ox, self._oy, self.heading, self._lane_pose())
        px = self.path[-1]
        if px[0] != self.lane_index or math.hypot(px[1] - self.x, px[2] - self.y) >= 20.0:
            self.path.append((self.lane_index, self.x, self.y))

    def _lane_pose(self) -> tuple[int, float, float, float]:
        return (self.lane_index, self.x, self.y, self.psi)

    def _toward_round(self) -> float:
        return -self.psi if self.direction == "CCW" else self.psi

    def _check_turn(self, t_ms: int) -> None:
        toward = self._toward_round()
        gated = toward >= config.TURN_MIN_DEG and self.y >= config.TURN_GATE_Y_MM
        failsafe = toward >= config.TURN_FAILSAFE_DEG
        if not (gated or failsafe):
            return
        if self._recheck_active:
            self._freeze(t_ms)
        old = (self.x, self.y, self.psi)
        self.x, self.y, _ = lf.corner_transform(self.x, self.y, self.psi, self.direction)
        self.lane_north += -90.0 if self.direction == "CCW" else 90.0
        self.psi = _wrap180(self.heading - self.lane_north)
        self.lane_index += 1
        self._close_colors_of_old_lanes(t_ms)
        why = "failsafe" if failsafe and not gated else "heading+y gate"
        self._event("turn", f"lane {self.lane_index - 1} -> {self.lane_index} (slot {self.slot}, lap {self.lap}) "
                            f"by {why}: old (x {old[0]:.0f}, y {old[1]:.0f}, psi {old[2]:+.1f}) -> "
                            f"new (x {self.x:.0f}, y {self.y:.0f}, psi {self.psi:+.1f})", t_ms)
        if self.slot not in self.lanes:
            self.lanes[self.slot] = LaneRecord(slot=self.slot, first_lane_index=self.lane_index,
                                               source="entry",
                                               seats={s.index: SeatState() for s in so.seats()})
            self._recheck_active = self.y < config.RECHECK_Y_MAX_MM
            self.lanes[self.slot].frozen = not self._recheck_active
        else:
            self._recheck_active = False

    def _freeze(self, t_ms: int) -> None:
        rec = self.lanes[self.slot]
        rec.frozen = True
        rec.frozen_t = self.t_now
        self._recheck_active = False
        decided = sum(1 for st in rec.seats.values() if st.state != "unknown")
        self._event("recheck_frozen", f"slot {self.slot}: {decided}/6 seats decided from "
                                      f"{rec.frames_used} frames ({rec.frames_skipped_align} skipped by "
                                      f"the +/-{config.RECHECK_ALIGN_DEG:.0f} deg gate; de-skew moved returns "
                                      f"up to {rec.max_shift_mm:.0f} mm, dropped {rec.returns_dropped_old} stale"
                                      + (f"; {rec.frames_skipped_old} frames from before the lane skipped"
                                         if rec.frames_skipped_old else "") + ")",
                    t_ms)

    # -- LIDAR frames -------------------------------------------------------------
    def on_lidar_frame(self, points, times=None) -> list | None:
        """points: mount-corrected clockwise ScanPoints. times: each return's
        measurement time on the Pi clock (lidar_source.get_latest_scan_timed ->
        scan_processing.clean_and_project_timed). With times, the frame is
        checked from the pose at its END (its newest return, or now if that is
        newer than the newest STM32 sample): the returns are de-skewed to that
        pose (deskew.py) and the seat check runs with the lane pose of that
        moment, taken from the history -- so a frame processed late is still
        judged from where it was taken. A frame that ends before the current
        lane began (or before the history) is skipped. Without times the frame
        is taken as a snapshot at the current pose (no de-skew).
        Returns the frame's SeatReadings if it was used, else None."""
        t_end = None
        if times is None or not len(times):
            if not self._recheck_active:
                return None
            rec = self.lanes[self.slot]
            x, y, psi = self.x, self.y, self.psi
        else:
            # Judged at the frame's END: the lane, pose and re-check window of that moment. A frame
            # processed late still counts if it was taken inside its lane's window.
            t_end = min(max(times) - config.LIDAR_TIME_OFFSET_S, self.t_now)
            span = self._hist.span
            lp = None if span is None or t_end - FRAME_SPAN_S < span[0] else self._hist.lane_pose_at(t_end)
            if lp is None:                               # too late: the history no longer covers the whole frame
                if self._recheck_active:
                    self.lanes[self.slot].frames_skipped_old += 1
                return None
            lane_index, x, y, psi = lp
            rec = self.lanes.get(lane_index % 4)
            if (rec is None or rec.source != "entry" or rec.first_lane_index != lane_index
                    or (rec.frozen and (rec.frozen_t is None or t_end > rec.frozen_t))
                    or y >= config.RECHECK_Y_MAX_MM):
                if self._recheck_active and lane_index != self.lane_index:
                    self.lanes[self.slot].frames_skipped_old += 1
                return None
        if abs(psi) > config.RECHECK_ALIGN_DEG:
            rec.frames_skipped_align += 1
            return None
        if t_end is not None:
            points, st = deskew(points, times, self._hist, config.LIDAR_TIME_OFFSET_S,
                                self.params.lidar_offset_forward_mm, self.params.lidar_offset_lateral_mm,
                                t_ref=t_end)
            self.last_deskew = st
            rec.returns_dropped_old += st.dropped_old
            rec.max_shift_mm = max(rec.max_shift_mm, st.max_shift_mm)
        readings = so.detect_seat_occupancy(points, x, y, self.direction, robot_yaw_deg=psi, params=self.params)
        readings = self._coverage_check(points, readings, rec)
        rec.frames_used += 1
        for r in readings:
            st = rec.seats[r.seat.index]
            if st.state == "unknown" and r.state is not so.Occupancy.UNKNOWN:
                rec.seats[r.seat.index] = SeatState(r.state.value, "entry", r.reason,
                                                    round(y, 1), rec.first_lane_index)
                if r.state is so.Occupancy.OCCUPIED:
                    self._request_color(rec.slot, r.seat.index, rec.first_lane_index)
        return readings

    def _coverage_check(self, points, readings, rec) -> list:
        """EMPTY needs the seat's whole search window covered. Where consecutive
        returns (by angle) are further apart than the pillar's own predicted
        angular width -- and than 2.5x the frame's typical spacing -- a pillar
        could sit in the hole unseen while the returns either side reach the
        wall behind: that EMPTY becomes UNKNOWN for this frame (another frame
        may decide the seat). The hole this catches in practice is the
        revolution's SEAM while the robot turns: returns just ahead of the beam
        are a whole revolution old and those just behind it are new, so a
        sector as wide as the rotation during the revolution was seen by
        neither (P10, docs/CHANGES.md section 10). OCCUPIED verdicts are
        positive evidence and are left alone."""
        angs = sorted(p.angle_deg for p in points)
        if len(angs) < 10:
            return readings
        steps = [b - a for a, b in zip(angs, angs[1:])] + [angs[0] + 360.0 - angs[-1]]
        typical = float(np.median(steps))
        gaps = [(a, s) for a, s in zip(angs, steps)]                # hole from a to a + s
        out = []
        for r in readings:
            if r.state is so.Occupancy.EMPTY:
                half = r.search_half_width_deg
                width = 2.0 * max(half - self.params.angular_margin_deg, 0.0)
                limit = max(width, 2.5 * typical)
                c = r.predicted_lidar_angle_deg
                worst = 0.0
                for a, s in gaps:
                    if s <= limit:
                        continue
                    lo = (a - c + 180.0) % 360.0 - 180.0                # hole start relative to the seat bearing
                    if lo < half and lo + s > -half:                    # the hole overlaps the window
                        worst = max(worst, s)
                if worst > 0.0:
                    rec.empty_downgraded += 1
                    r = dataclasses.replace(r, state=so.Occupancy.UNKNOWN,
                                            reason=f"coverage hole of {worst:.1f} deg inside the search window "
                                                   f"(wider than the pillar's {width:.1f} deg): it could hide the "
                                                   f"pillar -- was: {r.reason}")
            out.append(r)
        return out

    # -- pillar colour (checkpoint D) ---------------------------------------------------
    def _request_color(self, slot: int, seat_index: int, lane_index: int) -> None:
        st = self.lanes[slot].seats[seat_index]
        if st.color is not None:                  # one request per seat, ever
            return
        st.color, st.color_lane_index = "pending", lane_index
        st.color_reason = "pending: waiting for a camera frame with the seat in view"
        self._color_pending.append((slot, seat_index))

    def _close_color(self, slot: int, seat_index: int, color: str, reason: str) -> None:
        st = self.lanes[slot].seats[seat_index]
        st.color, st.color_reason = color, reason
        self._color_pending.remove((slot, seat_index))

    def _close_colors_of_old_lanes(self, t_ms: int) -> None:
        for slot, i in list(self._color_pending):
            st = self.lanes[slot].seats[i]
            if st.color_lane_index is None or st.color_lane_index >= self.lane_index:
                continue
            if st.color_attempts:
                why = f"lane left after {st.color_attempts} attempt(s) without a confident read ({st.color_reason})"
            elif st.color_out_of_view:
                why = f"never in the camera's view ({st.color_out_of_view} frames; last: {st.color_reason})"
            else:
                why = "no camera frames while in its lane"
            self._close_color(slot, i, color_id.UNKNOWN, why)
            self._event("color", f"slot {slot} seat {i}: UNKNOWN -- {why}", t_ms)

    def on_camera_frame(self, frame, t_capture: float) -> int:
        """One camera frame (raw, as captured) with its capture time on the Pi
        clock. Serves the open colour requests; returns how many seats were
        tried in it. Cheap when nothing is pending (returns 0 at once)."""
        if not self._color_pending:
            return 0
        self.camera_frames += 1
        t = t_capture - config.CAMERA_TIME_OFFSET_S
        span = self._hist.span
        lp = None if span is None or t < span[0] else self._hist.lane_pose_at(t)
        if lp is None:                            # too old for the pose history, or across a lane switch
            self.camera_frames_no_pose += 1
            return 0
        lane_index, x, y, psi = lp
        seats = {s.index: s for s in so.seats()}
        img, tried = None, 0
        for slot, i in list(self._color_pending):
            st = self.lanes[slot].seats[i]
            if st.color_lane_index != lane_index:
                continue
            if st.color_window_t is not None and t - st.color_window_t > config.COLOR_ID_WINDOW_S:
                self._close_color(slot, i, color_id.UNKNOWN,
                                  f"{config.COLOR_ID_WINDOW_S:.2f} s window ended after {st.color_attempts} "
                                  f"attempt(s) without a confident read ({st.color_reason})")
                continue
            seat = seats[i]
            v = color_id.seat_view(x, y, psi, self.direction, seat.x_mm, seat.y_mm)
            roi, why = color_id.pillar_roi(v.theta_deg, v.face_mm)
            if roi is None:
                st.color_out_of_view += 1
                if st.color_window_t is None:
                    st.color_reason = f"pending: {why}"
                continue
            if st.color_window_t is None:
                st.color_window_t = t
            if img is None:
                img = color_id.correct_frame(frame)
            st.color_attempts += 1
            tried += 1
            colour, rf, gf = color_id.classify(img, roi)
            w, h = roi.size
            desc = (f"attempt {st.color_attempts}/{config.COLOR_ID_MAX_ATTEMPTS}: red {rf:.0%}, green {gf:.0%} "
                    f"in a {w}x{h} px box at {v.theta_deg:+.1f} deg, face {v.face_mm:.0f} mm "
                    f"(lane pose x {x:.0f}, y {y:.0f}, psi {psi:+.1f})")
            if colour is not None:
                self._close_color(slot, i, colour, desc)
                self._event("color", f"slot {slot} seat {i}: {colour.upper()} -- {desc}", self._last.t_ms)
            elif st.color_attempts >= config.COLOR_ID_MAX_ATTEMPTS:
                self._close_color(slot, i, color_id.UNKNOWN, f"no confident read; last {desc}")
                self._event("color", f"slot {slot} seat {i}: UNKNOWN -- no confident read; last {desc}",
                            self._last.t_ms)
            else:
                st.color_reason = f"pending: {desc}"
        return tried

    def deskew_view(self, points, times) -> list:
        """A LIDAR frame de-skewed to the current pose, for display only (the
        dashboard's live scan); no seat check, no counters."""
        pts, _ = deskew(points, times, self._hist, config.LIDAR_TIME_OFFSET_S,
                        self.params.lidar_offset_forward_mm, self.params.lidar_offset_lateral_mm)
        return pts

    # -- reporting ------------------------------------------------------------------
    def _event(self, kind: str, detail: str, t_ms: int) -> None:
        self.events.append(TrackerEvent(t_ms, kind, detail, self.lane_index))

    def state(self) -> dict:
        return {
            "direction": self.direction, "lane_index": self.lane_index, "slot": self.slot, "lap": self.lap,
            "x_mm": round(self.x, 1), "y_mm": round(self.y, 1), "psi_deg": round(self.psi, 2),
            "psi0_deg": round(self.psi0, 2), "distance_mm": round(self.distance_mm, 1),
            "recheck_active": self._recheck_active,
            "camera": {"frames": self.camera_frames, "frames_no_pose": self.camera_frames_no_pose,
                       "pending": len(self._color_pending)},
            "lanes": {k: {"source": v.source, "frozen": v.frozen, "frames_used": v.frames_used,
                          "frames_skipped_align": v.frames_skipped_align,
                          "returns_dropped_old": v.returns_dropped_old, "max_shift_mm": round(v.max_shift_mm, 1),
                          "frames_skipped_old": v.frames_skipped_old, "empty_downgraded": v.empty_downgraded,
                          "seats": {i: {"state": s.state, "source": s.source, "reason": s.reason,
                                        "at_y_mm": s.at_y_mm, "color": s.color, "color_reason": s.color_reason}
                                    for i, s in v.seats.items()}}
                      for k, v in self.lanes.items()},
        }
