"""
The broadside wall-fix (front=outer wall, back=inner wall) and a pose
estimator that fuses it with continuous odometry, corrected once at the
corner-turn events your own drive FSM tells it about.

Back is INFERRED from front (back_d = LANE_WIDTH_MM - front_d), not
independently read -- on this robot most of the LIDAR's rear is
permanently blocked by its own chassis (confirmed on real hardware: a
~105deg dead zone centred almost exactly on 180deg robot-relative,
present regardless of where on the mat the robot is, not just near
corners). This trades away the old front+back cross-check for a
plausibility bound on front_d alone (must land inside the lane) -- see
the comment in compute_broadside_fix for the detail, and the README's
"Back reading dropped" section for how this was diagnosed.

IMPORTANT GAP THIS MODULE DOES NOT SOLVE: the broadside fix gives you your
LATERAL position within whichever section you're currently in -- it cannot
tell you *which of the 4 sections* that is. Nothing about "front wall is
1000mm away, back wall is 0mm away" distinguishes the south lane from the
east lane. You (or your team, watching the referee place the robot) have to
know the starting section going in and supply it as `initial_section` --
see PoseEstimator.__init__. Likewise, advancing from one section to the next
is driven by YOUR drive/turn FSM calling `on_corner_completed()`, not
inferred automatically from raw IMU angle here -- corner detection from
heading alone is exactly the kind of thing that's already living in your
STM32 control code and shouldn't be duplicated/guessed at in this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import config
import mat_geometry as geo
from scan_processing import Cluster


@dataclass
class BroadsideFix:
    ok: bool
    reason: str = ""
    front_distance_mm: float | None = None   # corrected, from reference point to outer wall
    back_distance_mm: float | None = None     # corrected, from reference point to inner wall
    lane_sum_mm: float | None = None
    lateral_from_outer_mm: float | None = None
    front_flatness_mm: float | None = None
    back_flatness_mm: float | None = None


def _angle_diff(a: float, b: float) -> float:
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def _angle_distance_to_cluster(c: Cluster, target_angle_deg: float) -> float:
    """Angular distance from target_angle_deg to the cluster's actual
    covered arc (0 if the target falls inside it). Uses the cluster's real
    angular extent (first point to last point, in the circular order the
    clustering already produced), NOT the centroid angle -- a wide,
    asymmetric cluster's centroid can sit well outside a tight search
    window even though the cluster's points do cover the target bearing
    (found during testing: this was silently excluding the correct wall)."""
    start = c.points[0].angle_deg
    span = c.angular_span_deg
    offset = (target_angle_deg - start) % 360.0
    if offset <= span:
        return 0.0
    return min(offset - span, 360.0 - offset)


def _find_wall_near(clusters: list[Cluster], target_angle_deg: float, window_deg: float) -> Cluster | None:
    candidates = []
    for c in clusters:
        if c.kind != "wall" or c.line_normal is None:
            continue
        if _angle_distance_to_cluster(c, target_angle_deg) <= window_deg:
            candidates.append(c)
    if not candidates:
        return None
    # Prefer the NEAREST candidate, not the one whose centroid angle happens
    # to average out closest to the target bearing. Both can exist at once
    # (e.g. the island's true near face at a slight angle, and a much
    # farther wall glimpsed through the gap beside it whose wide cluster
    # centroid lands closer to dead-on) -- physically, the sensor is
    # obstructed by whatever is nearest along that general direction, so
    # nearest-wins is the correct tie-break, confirmed necessary by testing.
    return min(candidates, key=lambda c: c.line_distance_mm)


def compute_broadside_fix(clusters: list[Cluster], heading_deg: float, section: geo.Section) -> BroadsideFix:
    target_heading = geo.BROADSIDE_HEADING_DEG[section]
    if abs(_angle_diff(heading_deg, target_heading)) > config.BROADSIDE_HEADING_TOLERANCE_DEG:
        return BroadsideFix(ok=False, reason=(
            f"not broadside: heading {heading_deg:.1f} deg, need {target_heading:.1f} "
            f"+/- {config.BROADSIDE_HEADING_TOLERANCE_DEG} for section {section}"))

    front = _find_wall_near(clusters, 0.0, config.FRONT_BACK_SEARCH_WINDOW_DEG)
    if front is None:
        return BroadsideFix(ok=False, reason="no wall cluster found for front (occluded, or thresholds too tight)")

    # line_distance_mm is the perpendicular distance from the LIDAR's own
    # origin to the fitted wall line -- apply the forward lever-arm offset.
    offset = config.LIDAR_OFFSET_FORWARD_MM
    front_d = front.line_distance_mm + offset

    # Back is no longer independently read. On real hardware, most of the
    # LIDAR's rear is permanently blocked by the robot's own chassis -- a
    # ~105deg dead zone centred almost exactly on 180deg robot-relative,
    # confirmed against a real scan, present regardless of where on the mat
    # the robot is (see README's "Back reading dropped" section). Inferred
    # instead: front+back == LANE_WIDTH_MM by definition of the lane, so
    # back_d = LANE_WIDTH_MM - front_d. This trades away the old front+back
    # cross-check -- it would now always trivially pass, since back is
    # DEFINED from front rather than independently measured -- for a
    # plausibility bound on front_d instead: a genuine outer-wall reading
    # has to land inside the lane.
    back_d = geo.LANE_WIDTH_MM - front_d
    if not (-config.LANE_WIDTH_TOLERANCE_MM < front_d < geo.LANE_WIDTH_MM + config.LANE_WIDTH_TOLERANCE_MM):
        return BroadsideFix(ok=False, reason=(
            f"front={front_d:.0f}mm is outside the lane (0..{geo.LANE_WIDTH_MM:.0f}mm, "
            f"+/- {config.LANE_WIDTH_TOLERANCE_MM:.0f} noise margin) -- can't be a genuine "
            f"outer-wall reading, likely a pillar or corner return mistaken for a wall"),
            front_distance_mm=front_d, back_distance_mm=back_d, lane_sum_mm=front_d + back_d)

    return BroadsideFix(
        ok=True,
        front_distance_mm=front_d,
        back_distance_mm=back_d,   # inferred, not measured -- see comment above
        lane_sum_mm=front_d + back_d,   # == LANE_WIDTH_MM always now; kept for API/dashboard shape
        lateral_from_outer_mm=front_d,
        front_flatness_mm=front.flatness_residual_mm,
        back_flatness_mm=None,   # no longer measured
    )


def _forward_range_mm(clusters: list[Cluster], window_deg: float) -> float | None:
    """Range of the ray pointing straight down the lane (0 deg robot-relative,
    the direction of travel). Unlike the side rays this is NOT taken from a
    fitted wall -- straight ahead the robot is usually looking down the lane
    at the far corner, where the return is a corner-blend (curved, rejected by
    the wall classifier), not a flat wall. What we actually want is just the
    RANGE dead-ahead, so take the median distance of the raw points closest to
    0 deg (within +/- a few degrees), which is robust to that corner-blend and
    to the odd dropout. Returns None if nothing is seen forward at all."""
    near0 = []
    for c in clusters:
        for p in c.points:
            da = abs((p.angle_deg + 180.0) % 360.0 - 180.0)  # angular dist to 0 deg
            if da <= window_deg:
                near0.append((da, p.dist_mm))
    if not near0:
        return None
    near0.sort()
    tight = [d for da, d in near0 if da <= 5.0] or [d for _, d in near0[:5]]
    tight.sort()
    return tight[len(tight) // 2]  # median of the dead-ahead points


@dataclass
class StartOfRunFix:
    """Result of the ONE-TIME start-of-run reading, taken with the robot
    stationary in its real start orientation: PARALLEL TO THE WALLS, FACING
    THE DIRECTION OF TRAVEL (down the lane). NOT broadside -- an earlier
    version assumed the robot faced the outer wall, which put the side rays
    down the lane (expecting left+right ~ 3000mm); on the real robot the
    start orientation is along-the-lane, so the side rays instead hit the two
    lane walls ~1000mm apart. See the module docstring / README.

    In this orientation:
      - the two side rays (90 deg = left, 270 deg = right) hit the OUTER and
        INNER lane walls, so they resolve CROSS-LANE (lateral) position and
        sum to ~LANE_WIDTH_MM (not the section length);
      - the forward ray (0 deg, down the lane) gives range to the wall ahead,
        which resolves ALONG-TRACK position.
    Which of left/right is the OUTER wall depends only on the driving
    direction (CCW keeps the outer wall on one hand, CW the other) -- not on
    which leg -- so lateral is resolved without knowing the leg. WHICH of the
    4 legs is still unresolvable from LIDAR alone (see module docstring) --
    candidate_start_positions() enumerates all 4."""
    ok: bool
    reason: str = ""
    forward_distance_mm: float | None = None   # 0deg range down the lane
    left_distance_mm: float | None = None      # 90deg wall (perpendicular distance)
    right_distance_mm: float | None = None     # 270deg wall
    lane_sum_mm: float | None = None           # left+right, expected ~LANE_WIDTH_MM
    lateral_from_outer_mm: float | None = None # resolved cross-lane position
    along_mm: float | None = None              # resolved along-track position


def compute_start_of_run_fix(clusters: list[Cluster], driving_direction: str = "CCW") -> StartOfRunFix:
    """Call once, at the very start of the run, with the robot stationary and
    placed in its real start orientation: parallel to the walls, facing the
    direction of travel (down the lane). `driving_direction` ("CCW"/"CW") is
    the randomised round direction your team supplies -- it fixes which hand
    the outer wall is on.

    Geometry (verified against the simulator on all 4 legs, both directions):
      - lateral (cross-lane) comes from the two side walls. CCW keeps the
        OUTER wall on the LEFT (90 deg) and the inner wall on the RIGHT
        (270 deg); CW is mirrored. left+right must sum to ~LANE_WIDTH_MM.
      - along-track comes from the forward (0 deg) range down the lane:
        CCW -> along = forward; CW -> along = OUTER_SIZE_MM - forward
        (the robot faces opposite ends of the section in the two directions).

    Offset correction: LIDAR_OFFSET_LATERAL_MM shifts the side (cross-lane)
    reads; LIDAR_OFFSET_FORWARD_MM shifts the forward (along-track) read.
    """
    if driving_direction not in ("CCW", "CW"):
        return StartOfRunFix(ok=False, reason=f"bad driving_direction {driving_direction!r} (want CCW/CW)")

    left = _find_wall_near(clusters, 90.0, config.SIDE_SEARCH_WINDOW_DEG)
    right = _find_wall_near(clusters, 270.0, config.SIDE_SEARCH_WINDOW_DEG)
    missing = [name for name, c in (("left(90)", left), ("right(270)", right)) if c is None]
    if missing:
        return StartOfRunFix(ok=False, reason=(
            f"no wall cluster found for {', '.join(missing)} -- the two lane walls should be "
            f"~{geo.LANE_WIDTH_MM:.0f}mm apart on the robot's sides; occluded, or the robot "
            f"isn't parallel to the walls (facing the direction of travel)"))

    lat_offset = config.LIDAR_OFFSET_LATERAL_MM
    fwd_offset = config.LIDAR_OFFSET_FORWARD_MM
    left_d = left.line_distance_mm
    right_d = right.line_distance_mm
    lane_sum = left_d + right_d
    # The lateral offset cancels out of this sum (two parallel walls a fixed
    # gap apart), so the sanity check doesn't need it.
    if abs(lane_sum - geo.LANE_WIDTH_MM) > config.LANE_WIDTH_TOLERANCE_MM:
        return StartOfRunFix(ok=False, reason=(
            f"left+right = {lane_sum:.0f}mm, expected the lane width ~{geo.LANE_WIDTH_MM:.0f}mm "
            f"+/- {config.LANE_WIDTH_TOLERANCE_MM:.0f}. If it's ~{geo.OUTER_SIZE_MM:.0f}mm the robot is "
            f"broadside (facing the outer wall) instead of along the lane; if it's way off, a pillar is "
            f"blocking a side ray"),
            left_distance_mm=left_d, right_distance_mm=right_d, lane_sum_mm=lane_sum)

    forward = _forward_range_mm(clusters, config.FRONT_BACK_SEARCH_WINDOW_DEG)
    if forward is None:
        return StartOfRunFix(ok=False, reason="nothing seen forward (0 deg) -- can't resolve along-track position",
                             left_distance_mm=left_d, right_distance_mm=right_d, lane_sum_mm=lane_sum)
    forward += fwd_offset

    # CCW: outer wall on the LEFT; along = forward.  CW mirrors both.
    if driving_direction == "CCW":
        lateral_from_outer = left_d - lat_offset
        along = forward
    else:
        lateral_from_outer = right_d - lat_offset
        along = geo.OUTER_SIZE_MM - forward

    return StartOfRunFix(
        ok=True,
        forward_distance_mm=forward,
        left_distance_mm=left_d, right_distance_mm=right_d, lane_sum_mm=lane_sum,
        lateral_from_outer_mm=lateral_from_outer, along_mm=along,
    )


def driving_heading_deg(section: geo.Section, driving_direction: str) -> float:
    """The heading a robot faces while DRIVING along `section` in
    `driving_direction` -- i.e. facing the direction of travel, parallel to
    the walls. This is the section's broadside heading (facing the outer wall)
    rotated -90 deg for CCW / +90 deg for CW, matching the simulator's
    true_heading_deg and verified against the along-lane scan geometry."""
    broadside = geo.BROADSIDE_HEADING_DEG[section]
    return (broadside - 90.0) % 360.0 if driving_direction == "CCW" else (broadside + 90.0) % 360.0


@dataclass
class CandidatePose:
    """One of the possible global poses for the start-of-run fix. `section`
    is which of the 4 legs this candidate assumes. `heading_variant` is
    "primary" (that leg's real DRIVING heading -- facing the direction of
    travel, the genuine solution IF this is the right leg) or "secondary"
    (the same point rotated +90 deg, shown only because you asked for a
    marker on both axes at every candidate -- a visual reference, not a
    physically valid start heading here)."""
    section: geo.Section
    heading_variant: str
    x_mm: float
    y_mm: float
    heading_deg: float


def candidate_start_positions(along_mm: float, lateral_mm: float,
                              driving_direction: str = "CCW") -> list[CandidatePose]:
    """Expands one (along_mm, lateral_mm) pair -- leg-independent, from
    compute_start_of_run_fix() -- into all 8 candidate global poses: one
    "primary" (the leg's real driving heading) and one "secondary" (+90 deg
    from primary, for the both-axes display) for each of the 4 legs. Which
    one (if any) matches reality still has to come from your team (e.g.
    watching the referee place the robot) -- see module docstring."""
    out: list[CandidatePose] = []
    for section in ("S", "E", "N", "W"):
        x, y = geo.local_to_global(section, along_mm, lateral_mm)
        primary_heading = driving_heading_deg(section, driving_direction)
        out.append(CandidatePose(section=section, heading_variant="primary",
                                  x_mm=x, y_mm=y, heading_deg=primary_heading))
        out.append(CandidatePose(section=section, heading_variant="secondary",
                                  x_mm=x, y_mm=y, heading_deg=(primary_heading + 90.0) % 360.0))
    return out


@dataclass
class PoseState:
    section: geo.Section
    along_mm: float
    lateral_mm: float          # distance from the OUTER wall
    heading_deg: float
    x_mm: float
    y_mm: float
    initialized: bool
    last_fix_ok: bool | None = None
    last_fix_reason: str = ""


class PoseEstimator:
    def __init__(self, initial_section: geo.Section, driving_direction: str = "CCW",
                 initial_along_mm: float = 0.0):
        """initial_section: which of the 4 sides the robot starts on -- YOU
        supply this (see module docstring), it can't be inferred from the
        broadside fix alone.
        driving_direction: "CCW" or "CW", as randomised for this round.
        initial_along_mm: how far into initial_section the robot's start
        position is. The broadside fix only ever resolves LATERAL position
        (front/back), never along-track position -- if you leave this at
        the default 0.0 but the robot actually started, say, 1500mm into
        the section (per the zone/dice draw your team watched get placed),
        your x/y estimate will be off by that much until the first corner
        turn resets along-track tracking to 0 and resynchronises it. Pass
        the real value here if you know it (even a rough guess from the
        zone number beats 0); otherwise treat position during the first
        section, before the first corner, as lateral-only-reliable.
        """
        assert driving_direction in ("CCW", "CW")
        self._next_section = geo.NEXT_SECTION_CCW if driving_direction == "CCW" else geo.NEXT_SECTION_CW
        self.state = PoseState(
            section=initial_section, along_mm=initial_along_mm, lateral_mm=geo.LANE_WIDTH_MM / 2,
            heading_deg=geo.BROADSIDE_HEADING_DEG[initial_section],
            x_mm=0.0, y_mm=0.0, initialized=False,
        )
        self._recompute_xy()

    def _recompute_xy(self):
        x, y = geo.local_to_global(self.state.section, self.state.along_mm, self.state.lateral_mm)
        self.state.x_mm, self.state.y_mm = x, y

    def update_heading(self, heading_deg: float):
        self.state.heading_deg = heading_deg % 360.0

    def update_odometry(self, delta_forward_mm: float):
        """Call this continuously (e.g. every encoder tick / control cycle)
        with the distance travelled since the last call. Along-section
        progress is dead-reckoned from this; lateral position is held at
        its last LIDAR-corrected value between fixes (a car driving mostly
        straight along a lane doesn't change its lane offset much between
        corners -- if your steering wanders a lot within a section you may
        want to also integrate lateral drift from heading error here)."""
        self.state.along_mm = max(0.0, self.state.along_mm + delta_forward_mm)
        self._recompute_xy()

    def in_safe_fix_zone(self) -> bool:
        """True once along_mm is far enough into the current section that a
        perpendicular 'back' ray will actually hit the island rather than
        sail past its corner (see the note by SAFE_FIX_ALONG_MIN/MAX_MM in
        mat_geometry.py). Check this before calling apply_lidar_fix -- don't
        attempt a fix right at a corner-turn completion, it will usually
        just fail the lane-width sanity check rather than succeed.

        NOTE: back is no longer independently read (see module docstring),
        so this specific rationale -- the back ray sailing past the
        island's corner -- no longer strictly applies to compute_broadside_fix,
        which only reads front now. Left in place as a conservative gate
        rather than removed outright, since front's own cluster can still
        get corner-blended with a side wall close enough to a corner (the
        same effect documented for compute_start_of_run_fix) and that
        hasn't been specifically characterised as safe to skip. Worth
        re-testing if you want fixes available closer to a corner-turn."""
        return geo.SAFE_FIX_ALONG_MIN_MM <= self.state.along_mm <= geo.SAFE_FIX_ALONG_MAX_MM

    def apply_lidar_fix(self, clusters: list[Cluster]) -> BroadsideFix:
        """Call once at the very start (robot stationary, already placed
        broadside per your start procedure) and again each time you're
        broadside AND in_safe_fix_zone() is True -- typically: drive a short
        distance into the new section after finishing a corner turn, THEN
        briefly go broadside and take this reading, rather than trying to
        fix at the corner itself."""
        fix = compute_broadside_fix(clusters, self.state.heading_deg, self.state.section)
        self.state.last_fix_ok = fix.ok
        self.state.last_fix_reason = fix.reason
        if fix.ok:
            self.state.lateral_mm = fix.lateral_from_outer_mm
            self.state.initialized = True
            self._recompute_xy()
        return fix

    def on_corner_completed(self) -> None:
        """Call this from your drive FSM the moment a corner turn finishes.
        Advances the section and resets along-track progress to 0.

        Does NOT take a LIDAR fix itself -- despite the car being briefly
        broadside right after the turn, that position is right at
        along_mm=0, which is exactly the corner-adjacent zone where the
        back reading misses the island (see mat_geometry.py). Keep driving
        (continue calling update_odometry/update_heading) until
        in_safe_fix_zone() is True, THEN go broadside and call
        apply_lidar_fix()."""
        self.state.section = self._next_section[self.state.section]
        self.state.along_mm = 0.0
        self._recompute_xy()
