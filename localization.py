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


@dataclass
class StartOfRunFix:
    """Result of the ONE-TIME start-of-run reading: uses all four cardinal
    robot-relative directions (front=0, back=180, left=90, right=270) to
    resolve BOTH lateral (cross-lane) and along-track position, without
    assuming which of the 4 sections the robot is on.

    Deliberately separate from BroadsideFix/compute_broadside_fix (which
    needs a known `section` to check heading against, and which is what
    every RE-fix after a corner turn keeps using): at true start-of-run we
    don't have a section yet -- that's exactly what we can't resolve here
    (see module docstring) -- so there's no specific target heading to
    check against either. This trusts your team's placement procedure
    (robot stationary, parallel to the inner wall, facing the outer wall)
    rather than gating on IMU heading."""
    ok: bool
    reason: str = ""
    front_distance_mm: float | None = None
    back_distance_mm: float | None = None
    lane_sum_mm: float | None = None
    left_distance_mm: float | None = None
    right_distance_mm: float | None = None
    section_length_sum_mm: float | None = None
    lateral_from_outer_mm: float | None = None
    along_mm: float | None = None              # resolved along-track position (averaged)
    along_mm_from_left: float | None = None    # diagnostic: independent estimate from the left (90deg) ray
    along_mm_from_right: float | None = None   # diagnostic: independent estimate from the right (270deg) ray


def compute_start_of_run_fix(clusters: list[Cluster]) -> StartOfRunFix:
    """Call once, at the very start of the run, with the robot stationary
    and already placed broadside per your team's start procedure.

    Along-track derivation: for EVERY one of the 4 sections (verified
    against mat_geometry._section_axes), a broadside robot's LEFT side
    (90 deg robot-relative) points toward the far corner of whichever
    section it's on (increasing along_mm), and its RIGHT side (270 deg)
    points toward the near corner (along_mm=0) -- because "left"/"right"
    for a broadside robot are exactly the section's own along-track axis.
    So the same formula resolves along_mm regardless of which of the 4
    legs the robot is actually on; it just can't tell you WHICH leg (see
    module docstring) -- use candidate_start_positions() for that part.

    Offset correction: only LIDAR_OFFSET_LATERAL_MM matters here (not
    _FORWARD_MM) -- config.py's comment on that field already flags this.
    A forward/back shift of the sensor doesn't change its perpendicular
    distance to a wall running parallel to the direction of travel; a
    left/right shift does. (Symmetric to compute_broadside_fix, which
    only needs _FORWARD_MM for the same reason, just on the other axis.)

    Back is inferred from front, not independently read -- see the
    comment in compute_broadside_fix. That also changes what "narrow
    safe start window" meant in earlier testing: the along-track window
    used to be constrained by BOTH the back ray's and a side ray's
    corner-blending risk at once (see mat_geometry.py's note on why a
    back ray near a corner sails past the island). With back no longer
    read at all, only the front and the two side rays need to avoid
    corner-blending, which is a real but smaller constraint -- re-verify
    against your own geometry if you rely on a specific starting spot;
    don't assume the old narrow numbers still apply as-is now that one
    of the four requirements is gone.
    """
    front = _find_wall_near(clusters, 0.0, config.FRONT_BACK_SEARCH_WINDOW_DEG)
    left = _find_wall_near(clusters, 90.0, config.SIDE_SEARCH_WINDOW_DEG)
    right = _find_wall_near(clusters, 270.0, config.SIDE_SEARCH_WINDOW_DEG)

    missing = [name for name, c in (("front", front), ("left", left), ("right", right)) if c is None]
    if missing:
        return StartOfRunFix(ok=False, reason=(
            f"no wall cluster found for {', '.join(missing)} (occluded, or thresholds too tight)"))

    fwd_offset = config.LIDAR_OFFSET_FORWARD_MM
    lat_offset = config.LIDAR_OFFSET_LATERAL_MM

    front_d = front.line_distance_mm + fwd_offset
    back_d = geo.LANE_WIDTH_MM - front_d   # inferred, see compute_broadside_fix
    if not (-config.LANE_WIDTH_TOLERANCE_MM < front_d < geo.LANE_WIDTH_MM + config.LANE_WIDTH_TOLERANCE_MM):
        return StartOfRunFix(ok=False, reason=(
            f"front={front_d:.0f}mm is outside the lane (0..{geo.LANE_WIDTH_MM:.0f}mm, "
            f"+/- {config.LANE_WIDTH_TOLERANCE_MM:.0f} noise margin)"),
            front_distance_mm=front_d, back_distance_mm=back_d, lane_sum_mm=front_d + back_d)

    # Raw, uncorrected -- the lateral offset cancels out of this sum (any
    # point between two parallel walls has front+back-style distances that
    # sum to the fixed gap between them, regardless of exactly where on
    # that line the point sits), so this check doesn't need lat_offset.
    left_d_raw = left.line_distance_mm
    right_d_raw = right.line_distance_mm
    section_length_sum = left_d_raw + right_d_raw
    if abs(section_length_sum - geo.OUTER_SIZE_MM) > config.SECTION_LENGTH_TOLERANCE_MM:
        return StartOfRunFix(ok=False, reason=(
            f"left+right = {section_length_sum:.0f}mm, expected {geo.OUTER_SIZE_MM:.0f}mm "
            f"+/- {config.SECTION_LENGTH_TOLERANCE_MM:.0f} -- likely a pillar blocking one of the side rays"),
            front_distance_mm=front_d, back_distance_mm=back_d, lane_sum_mm=front_d + back_d,
            left_distance_mm=left_d_raw, right_distance_mm=right_d_raw, section_length_sum_mm=section_length_sum)

    along_from_right = right_d_raw - lat_offset
    along_from_left = geo.OUTER_SIZE_MM - left_d_raw - lat_offset
    along = (along_from_right + along_from_left) / 2.0

    return StartOfRunFix(
        ok=True,
        front_distance_mm=front_d, back_distance_mm=back_d, lane_sum_mm=front_d + back_d,
        left_distance_mm=left_d_raw, right_distance_mm=right_d_raw, section_length_sum_mm=section_length_sum,
        lateral_from_outer_mm=front_d,
        along_mm=along, along_mm_from_left=along_from_left, along_mm_from_right=along_from_right,
    )


@dataclass
class CandidatePose:
    """One of the possible global poses for the start-of-run fix. `section`
    is which of the 4 legs this candidate assumes. `heading_variant` is
    "primary" (that section's real broadside heading -- a genuine, valid
    broadside solution IF this is the right section) or "secondary" (the
    same point rotated +90 deg, shown only because you asked for both a
    north-south- and east-west-facing marker at every candidate -- this is
    NOT a physically valid broadside reading at this point, it's a visual
    reference only)."""
    section: geo.Section
    heading_variant: str
    x_mm: float
    y_mm: float
    heading_deg: float


def candidate_start_positions(along_mm: float, lateral_mm: float) -> list[CandidatePose]:
    """Expands one (along_mm, lateral_mm) pair -- section-independent, from
    compute_start_of_run_fix() -- into all 8 candidate global poses: one
    "primary" (true broadside heading) and one "secondary" (+90 deg from
    primary, purely for the both-axes display) for each of the 4 sections.
    Which one (if any) matches reality still has to come from your team
    (e.g. watching the referee place the robot) -- see module docstring."""
    out: list[CandidatePose] = []
    for section in ("S", "E", "N", "W"):
        x, y = geo.local_to_global(section, along_mm, lateral_mm)
        primary_heading = geo.BROADSIDE_HEADING_DEG[section]
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
