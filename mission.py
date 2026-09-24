"""
The obstacle-round mission: chokeSLAM's tracker + the visibility-graph planner
+ the path follower (checkpoint E, decisions #66-#83; additions E-A1..E-A9 await approval).

    LAP 1, lane by lane (decision #70)
        LOOK    standing still: the entry re-check (LIDAR) and the camera decide
                the lane's seats; wait at least LOOK_SETTLE_S, at most
                LOOK_TIMEOUT_S (longer while a colour is pending / re-requested).
        PLAN    from the tracked pose to the NEXT lane's viewing pose (the corner
                square's centre, facing the next lane: its lane (VIEW_X, VIEW_Y),
                psi 0), through the current lane's midline if it is still ahead.
                World = this lane + the next lane (field_map.build_world).
        DRIVE   pure pursuit. Re-planned from the current pose whenever a seat
                verdict or colour of those lanes changes (the re-check keeps
                deciding seats while driving, decision #81). A PRESENT pillar of
                unknown colour ahead within COLOR_LOOK_DIST_MM stops the car and
                the colour is asked for again (decision #72, "stop and look
                again"), up to COLOR_LOOK_RETRIES times; then it is passed as
                "either" (logged).
        Driving into the viewing pose makes the tracker switch lanes (the turn
        rule) and open the next lane's re-check. After 4 lanes the car is at the
        START lane's viewing pose, where the start lane is re-checked once
        (decision #82).
    LAPS 2-3 (decision #73)
        One final map (all four lanes); one closed lap path from the start lane's
        viewing pose V0 back to V0 (checkpoints: the four midlines in order),
        followed TWICE, then the finish path -- all as one continuous path, so the
        car does not stop between laps. Re-planned from the current pose if the
        car is more than REPLAN_DEVIATION_MM off it.
    REVERSE (E-A4, found necessary in simulation, awaiting approval)
        When no forward path exists from a standstill (typically the start: the
        car stands ~300 mm behind the start section's far inner pillar, which
        must be passed on the island side), the car backs up straight by the
        shortest of REVERSE_STEPS_MM that is clear, then looks and plans again
        (rule 9.21: driving against the round direction is allowed within this
        section and the next).
    FINISH (decision #76)
        Lap 3 is complete once the car has driven out of the last corner section
        into the start section; the sign rules then no longer apply (App. A.5,
        last paragraph), except that pillars ON the start section's entry line
        (lane y = 1000) keep their side (the lap only completes as the car crosses
        that line; judges settle doubt against the team, 9.26). Stop with the
        whole outline inside the start section. Parking: not yet (decision #76).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import config
import field_map as fm
import seat_occupancy as so
import vg_planner as vp
from follower import DriveCmd, PurePursuit

STOP = DriveCmd(0.0, 0.0, stop=True)


@dataclass
class MissionEvent:
    t: float
    kind: str
    detail: str


def concat_paths(paths) -> vp.Path:
    rows, s0, prims, wps = [], 0.0, [], []
    for p in paths:
        S = p.samples.copy()
        S[:, 4] += s0 - S[0, 4]
        rows.append(S if not rows else S[1:])
        s0 = S[-1, 4]
        prims += p.prims
        wps += list(p.waypoints)
    S = np.vstack(rows)
    return vp.Path(prims, float(S[-1, 4]), S, wps, {"parts": len(paths)})


# -- planning requests, shared by Mission and plan_preview (checkpoint F2), so the
# dashboard's preview is exactly what the mission would plan from the same state --------
def view_goals(lane_index: int, d: str):
    slot = lane_index % 4
    return [fm.lane_pose(slot, x, config.VIEW_Y_MM, 0.0, d) for x in config.VIEW_X_MM]


def lap1_request(d: str, tab: dict, slot: int, y: float, target_lane: int):
    """(world, goals, checkpoints, slots) for a lap-1 plan from lane `slot` at lane y
    to the viewing pose of `target_lane`."""
    slots = {slot, (slot + 1) % 4}
    world = fm.build_world(d, tab, slots)
    cps = [fm.checkpoint(slot, d)] if y < 1500.0 - 1.0 else []
    # forward only: every other lane's midline (and this lane's once it is behind) may not be
    # crossed, so a lane plan can never go the wrong way round the island (found in simulation)
    for k in range(4):
        if k != slot or not cps:
            a, b = fm.checkpoint(k, d)
            world.gates.append(fm.Gate(a, b, ("no_reverse", k)))
    return world, view_goals(target_lane, d), cps, slots


def final_world(d: str, tab: dict, finish: bool = False):
    side_free = (lambda slot, y: slot == 0 and y > 1000.0 + 1.0) if finish else None
    return fm.build_world(d, tab, range(4), side_free=side_free)


def finish_goals(d: str):
    out = []
    for y in (1450.0, 1300.0, 1600.0):
        for x in (500.0, 350.0, 650.0):
            if (y - config.OUTLINE_REAR_MM >= 1000.0 + config.FINISH_MARGIN_MM
                    and y + config.OUTLINE_FRONT_MM <= 2000.0 - config.FINISH_MARGIN_MM):
                out.append(fm.lane_pose(0, x, y, 0.0, d))
    return out


def laps_left_and_cps(d: str, lane_index: int, y: float):
    """How many V0 arrivals are still ahead, and the checkpoints (midlines) still
    to cross before the next one."""
    slot = lane_index % 4
    cps = [fm.checkpoint(k, d) for k in range(slot, 4) if k > slot or y < 1500.0 - 1.0]
    # V0 arrivals are at lane indices 8 (end of lap 2) and 12 (end of lap 3)
    laps_left = 2 if lane_index < 8 else (1 if lane_index < 12 else 0)
    return laps_left, cps


def plan_with_retry(world, start, goals, checkpoints):
    """vp.plan, and once more with less inflation / clearance. (path, None) or (None, error text)."""
    try:
        return vp.plan(world, start, goals, checkpoints), None
    except vp.PlanError as e:
        first = str(e)
    try:
        return vp.plan(world, start, goals, checkpoints, inflation=config.PLAN_INFLATION_MM - 15.0,
                       clearance=config.PLAN_CLEARANCE_MM / 2.0), None
    except vp.PlanError as e:
        return None, f"{first}; retried with less inflation / clearance: {e}"


@dataclass
class Preview:
    ok: bool
    what: str                          # what was planned, or why nothing was
    path: object = None                # vp.Path
    world: object = None               # the WorldMap it was planned in
    goals: list = field(default_factory=list)


def plan_preview(d: str, tab: dict, pose, slot: int, lane_index: int, y: float) -> Preview:
    """What the mission would plan if it looked from this state now (lap 1: to the
    next lane's viewing pose; laps 2-3: the rest of the round and the finish).
    Takes a snapshot, not the tracker, so it can run outside the tracker's lock."""
    if lane_index < 4:
        target = lane_index + 1
        world, goals, cps, _ = lap1_request(d, tab, slot, y, target)
        path, err = plan_with_retry(world, pose, goals, cps)
        if path is None:
            return Preview(False, f"lap 1 -> lane {target}: {err}", None, world, goals)
        return Preview(True, f"lap 1 -> viewing pose of lane {target}: {path.length:.0f} mm", path, world, goals)
    world = final_world(d, tab)
    laps_left, cps = laps_left_and_cps(d, lane_index, y)
    V0 = fm.lane_pose(0, config.VIEW_X_MM[0], config.VIEW_Y_MM, 0.0, d)
    parts, fstart = [], pose
    if laps_left > 0:
        first, err = plan_with_retry(world, pose, [V0], cps)
        if first is None:
            return Preview(False, f"laps -> V0: {err}", None, world, [V0])
        parts.append(first)
        if laps_left > 1:
            lap, err = plan_with_retry(world, V0, [V0], [fm.checkpoint(k, d) for k in range(4)])
            if lap is None:
                return Preview(False, f"full lap: {err}", None, world, [V0])
            parts.append(lap)
        fstart = V0
    fin, err = plan_with_retry(final_world(d, tab, finish=True), fstart, finish_goals(d), [])
    if fin is None:
        return Preview(False, f"finish: {err}", None, world, finish_goals(d))
    parts.append(fin)
    path = concat_paths(parts)
    return Preview(True, f"laps: {laps_left} lap part(s) + finish, {path.length:.0f} mm", path, world,
                   [V0] + finish_goals(d))


class Mission:
    def __init__(self, trk, log=None):
        self.trk = trk
        self.d = trk.direction
        self.phase = "LOOK"
        self.events: list[MissionEvent] = []
        self.pp: PurePursuit | None = None
        self.path: vp.Path | None = None
        self.look_since: float | None = None
        self.target_lane = 1              # lap 1: the lane whose viewing pose we drive to
        self.plan_snapshot = None
        self.color_tries: dict = {}       # (slot, seat) -> re-requests made
        self.color_wait_since: float | None = None
        self.lap_path: vp.Path | None = None
        self.final_world = None
        self.plans = 0
        self.plan_failures = 0
        self.replans = 0
        self.reverse_left_mm = 0.0        # REVERSE phase: distance still to back up
        self.reverse_from = 0.0
        self.reverses = 0
        self.reverses_here = 0
        self.last_world = None            # the world and goals of the latest plan (dashboard)
        self.last_goals = []
        self._log = log

    # -- helpers ------------------------------------------------------------------
    def _event(self, t, kind, detail):
        self.events.append(MissionEvent(t, kind, detail))
        if self._log:
            self._log(f"[mission {t:7.2f}] {kind}: {detail}")

    def _view_goals(self, lane_index: int):
        return view_goals(lane_index, self.d)

    def _seat_snapshot(self, slots):
        tab = fm.seat_table_from_tracker(self.trk)
        return tuple(sorted((k, v) for k, v in tab.items() if k[0] in slots))

    def _plan(self, t, world, start, goals, checkpoints):
        self.plans += 1
        self.last_world, self.last_goals = world, goals
        try:
            return vp.plan(world, start, goals, checkpoints)
        except vp.PlanError as e:
            self._event(t, "plan_retry", f"{e}; retrying with less inflation / clearance")
        try:
            return vp.plan(world, start, goals, checkpoints, inflation=config.PLAN_INFLATION_MM - 15.0,
                           clearance=config.PLAN_CLEARANCE_MM / 2.0)
        except vp.PlanError as e:
            self.plan_failures += 1
            self._event(t, "plan_failed", str(e))
            return None

    def _midline_ahead(self) -> bool:
        return self.trk.y < 1500.0 - 1.0

    # -- lap 1 --------------------------------------------------------------------
    def _lap1_world(self):
        slots = {self.trk.slot, (self.trk.slot + 1) % 4}
        return fm.build_world(self.d, fm.seat_table_from_tracker(self.trk), slots), slots

    def _plan_lap1(self, t) -> bool:
        world, goals, cps, slots = lap1_request(self.d, fm.seat_table_from_tracker(self.trk), self.trk.slot,
                                                self.trk.y, self.target_lane)
        start = fm.tracker_pose(self.trk)
        path = self._plan(t, world, start, goals, cps)
        if path is None:
            return False
        self.path = path
        self.pp = PurePursuit(path, config.SPEED_LAP1_MM_S, stop_at_end=True)
        self.plan_snapshot = self._seat_snapshot(slots)
        self._event(t, "plan", f"lap 1 -> viewing pose of lane {self.target_lane}: {path.length:.0f} mm, "
                               f"goal {path.info['goal']}, {path.info['iterations']} iteration(s)")
        return True

    def _unknown_colour_ahead(self):
        """PRESENT seats of the current lane with colour pending / unknown, ahead
        within COLOR_LOOK_DIST_MM (rear-axle y), not yet given up on."""
        trk, out = self.trk, []
        rec = trk.lanes.get(trk.slot)
        if rec is None:
            return out
        seats = {s.index: s for s in so.seats()}
        for i, st in rec.seats.items():
            if st.state != "occupied" or st.color in ("red", "green"):
                continue
            dy = seats[i].y_mm - trk.y
            if 0.0 < dy <= config.COLOR_LOOK_DIST_MM and self.color_tries.get((trk.slot, i), 0) <= config.COLOR_LOOK_RETRIES:
                out.append((i, st))
        return out

    def _serve_colours(self, t, lst) -> bool:
        """Re-request colours that ended UNKNOWN; True while still waiting."""
        waiting = False
        for i, st in lst:
            key = (self.trk.slot, i)
            if st.color == "unknown" or st.color is None:
                n = self.color_tries.get(key, 0)
                if n < config.COLOR_LOOK_RETRIES and self.trk.rerequest_color(self.trk.slot, i):
                    self.color_tries[key] = n + 1
                    self._event(t, "look_again", f"slot {key[0]} seat {i}: colour re-requested ({n + 1})")
                    waiting = True
                else:
                    self.color_tries[key] = config.COLOR_LOOK_RETRIES + 1
                    self._event(t, "colour_given_up", f"slot {key[0]} seat {i}: still unknown -> passed as either side")
            else:
                waiting = True             # pending
        return waiting

    @staticmethod
    def _path_still_valid(pp, world) -> bool:
        S = pp.S[pp.i:]
        if len(S) < 2:
            return True
        idx, _ = vp.body_hits(S, world.rects, config.PLAN_CLEARANCE_MM / 2.0)
        if idx is not None:
            return False
        for g in world.gates:
            for a, b in zip(S[:-1:3, :2], S[3::3, :2]):
                if vp.segs_cross(tuple(a), tuple(b), g.a, g.b):
                    return False
        return True

    # -- laps 2-3 -----------------------------------------------------------------
    def _final_world(self, finish: bool = False):
        return final_world(self.d, fm.seat_table_from_tracker(self.trk), finish)

    def _finish_goals(self):
        return finish_goals(self.d)

    def _plan_laps(self, t, start, laps_left: int, cps_now) -> bool:
        """Path: start -> V0 (checkpoints cps_now), then laps_left - 1 more full laps, then the finish."""
        V0 = fm.lane_pose(0, config.VIEW_X_MM[0], config.VIEW_Y_MM, 0.0, self.d)
        world = self.final_world
        parts = []
        if laps_left > 0:
            first = self._plan(t, world, start, [V0], cps_now)
            if first is None:
                return False
            parts.append(first)
            if self.lap_path is None:
                full = [fm.checkpoint(k, self.d) for k in range(4)]
                self.lap_path = self._plan(t, world, V0, [V0], full)
                if self.lap_path is None:
                    return False
            parts += [self.lap_path] * (laps_left - 1)
            fstart = V0
        else:
            fstart = start
        fin = self._plan(t, self._final_world(finish=True), fstart, self._finish_goals(), [])
        if fin is None:
            return False
        parts.append(fin)
        self.path = concat_paths(parts)
        self.pp = PurePursuit(self.path, config.SPEED_LAPS23_MM_S, stop_at_end=True)
        self._event(t, "plan", f"laps: {laps_left} lap part(s) + finish, {self.path.length:.0f} mm")
        return True

    def _laps_left_and_cps(self):
        return laps_left_and_cps(self.d, self.trk.lane_index, self.trk.y)

    # -- main --------------------------------------------------------------------------
    def update(self, t: float, v_meas: float = 0.0) -> DriveCmd:
        trk = self.trk
        if self.phase in ("DONE", "FAILED"):
            return STOP

        if self.phase == "LOOK":
            if self.look_since is None:
                self.look_since = t
                self._event(t, "look", f"lane {trk.lane_index} (slot {trk.slot}), pose x {trk.x:.0f} y {trk.y:.0f}")
            waited = t - self.look_since
            rec = trk.lanes.get(trk.slot)
            unknown = sum(1 for st in rec.seats.values() if st.state == "unknown") if rec else 6
            # only colours of pillars within COLOR_LOOK_DIST_MM are waited for here: a pillar further on
            # may be hidden behind a nearer one from this pose, and re-asking from the same place cannot
            # help -- it gets its own stop and re-requests when the car is COLOR_LOOK_DIST_MM short of it
            seats_y = {s_.index: s_.y_mm for s_ in so.seats()}
            pending = [(i, st) for i, st in (rec.seats.items() if rec else [])
                       if st.state == "occupied" and st.color not in ("red", "green")
                       and 0.0 < seats_y[i] - trk.y <= config.COLOR_LOOK_DIST_MM
                       and self.color_tries.get((trk.slot, i), 0) <= config.COLOR_LOOK_RETRIES]
            if waited < config.LOOK_SETTLE_S:
                return STOP
            colour_wait = self._serve_colours(t, pending) if pending else False
            limit = config.LOOK_TIMEOUT_S * (1 + (config.COLOR_LOOK_RETRIES if colour_wait else 0))
            if (unknown > 0 or colour_wait) and waited < limit:
                return STOP
            self.look_since = None
            ok = self._plan_from_look(t)
            if ok is None:                    # reversing first
                return STOP
            if not ok:
                self.phase = "FAILED"
            return STOP

        if self.phase == "REVERSE":
            done = trk.distance_mm - self.reverse_from
            if done >= self.reverse_left_mm - 5.0:
                self.phase = "LOOK"
                self._event(t, "reversed", f"backed up {done:.0f} mm")
                return STOP
            return DriveCmd(0.0, -config.REVERSE_SPEED_MM_S)

        X, Y, th = fm.tracker_pose(trk)
        return self._drive(t, X, Y, th, v_meas)

    def _plan_from_look(self, t):
        """Plan from the standstill at a LOOK. True = planned, False = failed,
        None = no forward path, so the car backs up first (E-A4, awaiting approval)."""
        trk = self.trk
        if trk.lane_index >= 4:
            self.final_world = self._final_world()
            laps_left, cps = self._laps_left_and_cps()
            self._event(t, "final_map", f"{len(self.final_world.pillars)} pillar(s) / unknown seat(s), "
                                        f"{len(self.final_world.gates)} gate(s)")
            if self._plan_laps(t, fm.tracker_pose(trk), laps_left, cps):
                self.phase, self.reverses_here = "LAPS", 0
                return True
        elif self._plan_lap1(t):
            self.phase, self.reverses_here = "LAP1", 0
            return True
        return self._start_reverse(t)

    def _start_reverse(self, t):
        """No forward path from here: back up straight (rule 9.21 allows driving against the round
        direction within this section and the next) by the shortest of REVERSE_STEPS_MM that is clear
        of everything, then look and plan again. At most REVERSE_MAX_PER_STOP times per stop."""
        if self.reverses_here >= config.REVERSE_MAX_PER_STOP:
            return False
        X, Y, th = fm.tracker_pose(self.trk)
        world = self.final_world if self.trk.lane_index >= 4 else self._lap1_world()[0]
        for D in config.REVERSE_STEPS_MM:
            n = max(2, int(D / 10.0))
            S = np.array([[X - D * i / n * math.cos(th), Y - D * i / n * math.sin(th), th, 0.0, 0.0]
                          for i in range(n + 1)])
            if vp.body_hits(S[1:], world.rects, config.REVERSE_CLEARANCE_MM)[0] is None:
                self.reverses_here += 1
                self.reverses += 1
                self.reverse_left_mm = D * (self.reverses_here)          # back further on a second try
                self.reverse_from = self.trk.distance_mm
                self.phase = "REVERSE"
                self._event(t, "reverse", f"no forward path: backing up {self.reverse_left_mm:.0f} mm")
                return None
        return False

    def _drive(self, t, X, Y, th, v_meas) -> DriveCmd:
        trk = self.trk
        if self.phase == "LAP1":
            # re-plan when what we know about this lane or the next changed
            slots = {trk.slot, (trk.slot + 1) % 4}
            snap = self._seat_snapshot(slots)
            if snap != self.plan_snapshot and trk.lane_index < self.target_lane:
                self.replans += 1
                self._event(t, "replan", "seat map changed")
                old_pp, old_path = self.pp, self.path
                if not self._plan_lap1(t):
                    # no new path from the moving pose: keep the current one if it is still valid in
                    # the updated map (clear of every obstacle, on the right side of every gate)
                    world, slots = self._lap1_world()
                    if self._path_still_valid(old_pp, world):
                        self.pp, self.path = old_pp, old_path
                        self.plan_snapshot = snap
                        self._event(t, "replan_kept", "the current path is still valid in the updated map")
                    else:
                        self.phase = "FAILED"
                        return STOP
            # a pillar of unknown colour close ahead: stop and look again
            lst = self._unknown_colour_ahead() if trk.lane_index < self.target_lane or trk.lane_index == 0 else []
            if lst:
                if self.color_wait_since is None:
                    self.color_wait_since = t
                    self._event(t, "stop_for_colour", ", ".join(f"seat {i} ({st.color})" for i, st in lst))
                if v_meas > 30.0:
                    return STOP
                if self._serve_colours(t, lst) and t - self.color_wait_since < config.LOOK_TIMEOUT_S * (config.COLOR_LOOK_RETRIES + 1):
                    return STOP
                for i, _ in lst:
                    self.color_tries[(trk.slot, i)] = config.COLOR_LOOK_RETRIES + 1
            self.color_wait_since = None
            cmd = self.pp.update(X, Y, th, v_meas)
            if self.pp.done:
                if trk.lane_index != self.target_lane:
                    self._event(t, "error", f"at the viewing pose but the tracker is in lane {trk.lane_index}, "
                                            f"expected {self.target_lane}")
                    self.phase = "FAILED"
                    return STOP
                self.target_lane += 1
                self.phase = "LOOK"
            return cmd

        if self.phase == "LAPS":
            cmd = self.pp.update(X, Y, th, v_meas)
            if self.pp.cross_track_mm > config.REPLAN_DEVIATION_MM and not self.pp.done:
                self.replans += 1
                laps_left, cps = self._laps_left_and_cps()
                self._event(t, "replan", f"{self.pp.cross_track_mm:.0f} mm off the path "
                                         f"(lane {trk.lane_index}, {laps_left} lap part(s) left)")
                if not self._plan_laps(t, (X, Y, th), laps_left, cps):
                    self.phase = "FAILED"
                    return STOP
                cmd = self.pp.update(X, Y, th, v_meas)
            if self.pp.done:
                self.phase = "DONE"
                self._event(t, "done", f"stopped at lane {trk.lane_index}, x {trk.x:.0f}, y {trk.y:.0f}")
            return cmd
        return STOP
