"""
The broadside wall-fix (front=outer wall, back=inner wall) and a pose
estimator that fuses it with continuous odometry, corrected once at the
corner-turn events your own drive FSM tells it about.

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
    back = _find_wall_near(clusters, 180.0, config.FRONT_BACK_SEARCH_WINDOW_DEG)
    if front is None or back is None:
        missing = "front" if front is None else "back"
        return BroadsideFix(ok=False, reason=f"no wall cluster found for {missing} (occluded, or thresholds too tight)")

    # line_distance_mm is the perpendicular distance from the LIDAR's own
    # origin to the fitted wall line -- apply the forward lever-arm offset.
    # If the LIDAR sits FORWARD_MM ahead of the reference point, the true
    # distance from the reference point to the front wall is larger by that
    # amount, and to the back wall smaller by that amount.
    offset = config.LIDAR_OFFSET_FORWARD_MM
    front_d = front.line_distance_mm + offset
    back_d = back.line_distance_mm - offset

    lane_sum = front_d + back_d
    if abs(lane_sum - geo.LANE_WIDTH_MM) > config.LANE_WIDTH_TOLERANCE_MM:
        return BroadsideFix(ok=False, reason=(
            f"front+back = {lane_sum:.0f}mm, expected {geo.LANE_WIDTH_MM:.0f}mm "
            f"+/- {config.LANE_WIDTH_TOLERANCE_MM:.0f} -- likely a pillar or corner "
            f"return mistaken for a wall"),
            front_distance_mm=front_d, back_distance_mm=back_d, lane_sum_mm=lane_sum)

    return BroadsideFix(
        ok=True,
        front_distance_mm=front_d,
        back_distance_mm=back_d,
        lane_sum_mm=lane_sum,
        lateral_from_outer_mm=front_d,
        front_flatness_mm=front.flatness_residual_mm,
        back_flatness_mm=back.flatness_residual_mm,
    )


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
        just fail the lane-width sanity check rather than succeed."""
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
