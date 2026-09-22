"""
Traffic-sign seat occupancy from a single LIDAR scan, in LANE-LOCAL coordinates.

WHAT THIS DOES
--------------
Rather than segmenting the scan into blobs and asking "is this blob a pillar?"
(which the existing scan_processing classifier does, and which misfires -- on a
completely empty field, 45% of simulated scans produce at least one cluster
classified "pillar", because grazing-angle slivers off the outer wall land in
the same arc-length band as a real 50 mm pillar), this module runs the check
the other way round:

    For each of the 6 traffic-sign seats whose map position we already know,
    predict the bearing and the range at which its pillar WOULD appear, then
    ask the scan whether anything is actually there.

A random wall sliver only fools that test if it happens to sit at the right
bearing AND the right range AND have the right angular width -- a far narrower
coincidence than merely "looks pillar-sized".


COORDINATE FRAME (lane-local, per WRO's "grid north" convention)
----------------------------------------------------------------
Each of the four lanes gets its own frame. Nothing here needs to know WHICH
lane it is on, and nothing here needs to know the round's driving direction.

    +y  = grid north = the direction of travel, along the lane.
          y = 0   at the wall behind the robot (the corner it entered from)
          y = 3000 at the wall ahead (the corner it is driving toward)
          This is the user's `y = 3000 - d(0 deg)`.

    +x  = to the robot's right (grid bearing 90).
          x = 0    at the LEFT-hand wall
          x = 1000 at the RIGHT-hand wall
          This is the user's `x` from the 90 deg / 270 deg rays:
          x = d(270 deg), cross-checked by d(90 deg) = 1000 - x.

    Bearings are measured CLOCKWISE from +y, so bearing = atan2(dx, dy)
    -- east over north, NOT the usual atan2(y, x).

Because the frame is anchored to the direction of travel and the lane is
symmetric about its own centreline, the seat table below is the SAME for all
four lanes and for both driving directions. That is the whole reason the
lane-local choice pays off: no CW/CCW branch, no S/E/N/W branch, no outer-wall
side to get backwards. (For reference: mapping this frame back to global mat
coordinates reproduces exactly the 24 global seat positions of Figure 11, for
every lane and both directions -- see test_seat_occupancy.py::test_frame_maps_to_global.)


SEAT GEOMETRY -- from the rulebook, not from mat_geometry.py
-------------------------------------------------------------
WRO 2026 General Rules, Figure 11 (the dimensioned field map), read at the NE
corner, cross-checked against Figure 3 and section 13:

  - 13.1  the racetrack's inner size is 3000 x 3000 mm
  - 13.x  Obstacle Challenge lane width is always 1000 mm (+/- 10 at Worlds)
  - Fig 2 the track is a 3x3 grid of 1000 mm blocks: 4 corner sections,
          4 straightforward sections, and the 1000 x 1000 island
  - Fig 11 the `400 mm | 200 mm | 400 mm` chain spans the 1000 mm lane
          -> seats sit 400 mm and 600 mm from either wall, i.e. +/-100 mm
             about the lane centreline
  - Fig 11 the `500 mm` chain spans half the 1000 mm straightforward section
          -> seats sit on the section's two boundary lines and its midpoint,
             i.e. at y = 1000, 1500, 2000 in this frame
  - 13.13 seat is 50 x 50 mm; 13.19 pillar is 50 x 50 x 100 mm
  - 13.15 the "was it moved" circle around a seat is 85 mm diameter

NOTE: this deliberately does NOT use mat_geometry.all_slots(). That table
(SLOT_COL_FRACS = (0.25, 0.75), SLOT_ROW_FRACS = (1/6, 0.5, 5/6)) places the
seats at 250/750 across the lane and 500/1500/2500 along the full 3000 mm
edge, which disagrees with Figure 11 on every one of the 24 slots, some by
over a metre. Left untouched here as instructed; this module carries its own
rulebook-derived table so that a later fix to mat_geometry.py cannot silently
change the answers this module gives.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable, Sequence

# ---------------------------------------------------------------------------
# Field constants (rulebook-derived -- see module docstring)
# ---------------------------------------------------------------------------
LANE_LENGTH_MM = 3000.0   # wall-to-wall along the direction of travel
LANE_WIDTH_MM = 1000.0    # wall-to-wall across the lane

SEAT_X_MM = (400.0, 600.0)                 # from the LEFT-hand wall
SEAT_Y_MM = (1000.0, 1500.0, 2000.0)       # along travel, from the entry wall

PILLAR_SIDE_MM = 50.0
PILLAR_HALF_MM = PILLAR_SIDE_MM / 2.0                    # 25.0
PILLAR_HALF_DIAG_MM = PILLAR_SIDE_MM * math.sqrt(2) / 2  # 35.36, corner-on


class Occupancy(Enum):
    """Three states, not two.

    EMPTY means "the ray reached past this seat and hit something further
    away", which is positive evidence of absence. UNKNOWN means "this seat was
    not observed" -- occluded by something nearer, inside the chassis blind
    wedge, or simply no returns at that bearing. Collapsing UNKNOWN into EMPTY
    is how a planner drives into a pillar it never looked at, so they are kept
    distinct and the caller has to decide what to do with UNKNOWN.
    """
    OCCUPIED = "occupied"
    EMPTY = "empty"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Seat:
    index: int      # 0..5, stable ordering: near->far, then left->right
    name: str       # e.g. "near-left", for logs and the dashboard
    x_mm: float     # lane-local, from the left-hand wall
    y_mm: float     # lane-local, along travel from the entry wall


def seats() -> list[Seat]:
    """The 6 traffic-sign seats of one straightforward section, in lane-local
    coordinates. Ordering is fixed: index = 2*row + col, row running from the
    entry end of the section (y=1000) to the exit end (y=2000), col running
    left (x=400) to right (x=600)."""
    row_names = ("near", "mid", "far")
    col_names = ("left", "right")
    out: list[Seat] = []
    for row, y in enumerate(SEAT_Y_MM):
        for col, x in enumerate(SEAT_X_MM):
            out.append(Seat(index=2 * row + col,
                            name=f"{row_names[row]}-{col_names[col]}",
                            x_mm=x, y_mm=y))
    assert len(out) == 6
    return out


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
@dataclass
class DetectParams:
    """Every threshold that matters, in one place.

    The dominant error term is NOT sensor noise -- it is pose error. A 50 mm
    pillar at 1500 mm subtends only ~1.9 deg, so a 3 deg yaw error moves the
    pillar clean out of its own predicted bearing. Size angular_margin_deg
    from your heading uncertainty and range_tol_mm from your position
    uncertainty, not from the LIDAR datasheet.
    """
    # Angular half-window searched around the predicted bearing. The pillar's
    # own subtended half-width is added to this automatically, so this value
    # only has to cover pose/heading uncertainty.
    angular_margin_deg: float = 4.0

    # Range agreement. Absolute term plus a term proportional to range, so a
    # far seat is judged more leniently than a near one.
    range_tol_mm: float = 70.0
    range_tol_frac: float = 0.03

    # A candidate pillar must be at least this many points wide, and its
    # angular extent must not exceed the pillar's own predicted subtense
    # times this factor (a much wider flat return at the right range is a
    # wall seen edge-on, not a pillar).
    min_points: int = 2
    max_width_factor: float = 2.5

    # Consecutive points further apart than this in range are not the same
    # surface.
    max_range_step_mm: float = 40.0

    # Rear chassis blind wedge, robot-relative, in the LIDAR's own angle
    # convention (0 = forward, 90 = left). Seats falling inside it are
    # reported UNKNOWN rather than EMPTY. Defaults mirror config.py; measure
    # your own unit off a raw scan dump.
    blind_arc_center_deg: float = 180.0
    blind_arc_width_deg: float = 105.0

    # Sensor limits -- returns outside this band are discarded before anything
    # else, matching scan_processing's own cleaning.
    min_range_mm: float = 60.0
    max_range_mm: float = 6000.0

    # The pillar sits on its seat at round start, but allow a little slack for
    # print tolerance and for a sign nudged inside its 85 mm circle.
    seat_position_slack_mm: float = 10.0

    # LIDAR lever arm, in the robot's own frame, from the pose reference point
    # (whatever robot_x_mm / robot_y_mm refer to -- typically the rear-axle
    # midpoint) to the sensor. forward is positive ahead, lateral positive to
    # the robot's LEFT, matching config.LIDAR_OFFSET_*. Ranges are measured
    # from the sensor, so the seat vector has to be taken from there; with a
    # vehicle up to 300 x 200 mm (rule 9.17) this is not a rounding error.
    lidar_offset_forward_mm: float = 0.0
    lidar_offset_lateral_mm: float = 0.0

    # Before declaring a seat EMPTY, require that a pillar standing on it would
    # have produced at least this many returns, given the scan's own measured
    # angular resolution. A 50 mm pillar at 1.7 m subtends 1.7 deg; at 1 deg
    # sampling that is one or two returns, and a single dropout then hides it
    # completely -- the window fills with the wall behind and the seat looks
    # empty. Below this standard the detector cannot tell "nothing there" from
    # "the one return that would have proved it was lost", so it says UNKNOWN.
    # It matters more than it looks: EMPTY is the verdict a caller holds onto,
    # so a false EMPTY is sticky in a way a false UNKNOWN is not.
    min_expected_hits: float = 2.0

    # A seat closer to the sensor than this cannot be observed at all -- the
    # pillar's face falls inside the LIDAR's dead zone, so the only returns in
    # the window come from whatever is behind it. Without this gate a seat
    # passing right beside the robot reads as EMPTY, which is the worst
    # possible error: it un-sets a seat that an earlier, valid scan had
    # already called OCCUPIED. (Found in test_drive_through_resolves_every_
    # seat_in_time -- it produced exactly this occupied->empty flip.)
    min_observable_face_mm: float = 120.0


@dataclass
class SeatReading:
    """Everything the detector concluded about one seat, and why.

    The predicted_* and observed_* fields are kept so the dashboard can draw
    the search window against the live scan -- when this misfires on the real
    robot, that overlay is what tells you whether the bearing was wrong (pose
    error) or the range was wrong (calibration).
    """
    seat: Seat
    state: Occupancy
    reason: str = ""

    predicted_bearing_deg: float = 0.0        # grid bearing, robot -> seat
    predicted_rel_bearing_deg: float = 0.0    # same, minus robot yaw
    predicted_lidar_angle_deg: float = 0.0    # in the LIDAR's own angle sense
    search_half_width_deg: float = 0.0

    expected_centre_mm: float = 0.0           # range to the seat centre
    expected_face_mm: float = 0.0             # range to the pillar's near face
    range_tolerance_mm: float = 0.0

    observed_range_mm: float | None = None    # nearest plausible surface found
    observed_width_deg: float | None = None
    expected_hits: float | None = None        # returns a pillar here would give
    residual_mm: float | None = None          # observed - expected_face
    n_points_in_window: int = 0

    @property
    def occupied(self) -> bool:
        """Convenience for the boolean form in the original sketch. Note that
        this maps UNKNOWN to False -- read `state` instead wherever the
        difference between "confirmed empty" and "never saw it" matters."""
        return self.state is Occupancy.OCCUPIED


# ---------------------------------------------------------------------------
# Angle helpers
# ---------------------------------------------------------------------------
def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def bearing_to(dx_mm: float, dy_mm: float) -> float:
    """Grid bearing (0 = grid north = direction of travel, increasing
    CLOCKWISE) of the vector (dx, dy) in lane-local coordinates.

    This is atan2(EAST, NORTH) -- dx first, dy second -- not the usual
    atan2(y, x). Sanity: straight ahead (0, +) -> 0; to the right (+, 0) -> 90;
    behind (0, -) -> 180; to the left (-, 0) -> 270.

    Two-argument atan2 is essential. One-argument atan(dx/dy) only spans
    +/-90 deg, so it folds a seat behind the robot onto a seat ahead of it and
    divides by zero for a seat exactly abeam.
    """
    return math.degrees(math.atan2(dx_mm, dy_mm)) % 360.0


def bearing_to_lidar_angle(rel_bearing_deg: float) -> float:
    """Convert a robot-relative grid bearing (clockwise) into this codebase's
    LIDAR angle convention (0 = forward, 90 = left, i.e. counter-clockwise).

    The two axes run in opposite senses, so the conversion is a mirror:
    lidar = -bearing, mod 360. That is the `2*pi - theta` in the original
    sketch -- correct as written, once theta is a genuine robot-relative
    bearing from atan2 rather than a world bearing from one-argument atan.

    If your unit's raw angles run the other way, do NOT patch it here: set
    config.LIDAR_ANGLE_SIGN / LIDAR_ANGLE_ZERO_OFFSET_DEG and feed this
    function points that scan_processing.clean_and_project has already
    corrected, so there is exactly one place in the codebase where mount
    calibration lives.
    """
    return (-rel_bearing_deg) % 360.0


def _in_blind_arc(lidar_angle_deg: float, p: DetectParams) -> bool:
    d = abs(_wrap180(lidar_angle_deg - p.blind_arc_center_deg))
    return d <= p.blind_arc_width_deg / 2.0


# ---------------------------------------------------------------------------
# Scan input
# ---------------------------------------------------------------------------
def _normalise_scan(scan: Iterable, p: DetectParams) -> list[tuple[float, float]]:
    """Accepts any of:
      - (angle_deg, range_mm) pairs
      - (angle_deg, range_mm, quality) triples  [raw rplidar style]
      - scan_processing.ScanPoint objects
    and returns cleaned (angle_deg, range_mm) sorted by angle.

    IMPORTANT: angles must ALREADY be in the corrected robot frame -- i.e. run
    through scan_processing.clean_and_project with your LIDAR_ANGLE_SIGN /
    LIDAR_ANGLE_ZERO_OFFSET_DEG. This module does not apply mount calibration.
    """
    out: list[tuple[float, float]] = []
    for item in scan:
        if hasattr(item, "angle_deg") and hasattr(item, "dist_mm"):
            a, r = float(item.angle_deg), float(item.dist_mm)
        else:
            seq = tuple(item)
            a, r = float(seq[0]), float(seq[1])
        if r < p.min_range_mm or r > p.max_range_mm:
            continue
        out.append((a % 360.0, r))
    out.sort()
    return out


def _angular_step_deg(pts: Sequence[tuple[float, float]]) -> float:
    """The scan's own angular resolution, measured rather than assumed: the
    median gap between consecutive returns. Taken from the data so the same
    thresholds work on a 360-point sweep, a 720-point sweep, or a real unit
    whose rate drifts with rotation speed."""
    if len(pts) < 8:
        return 1.0
    gaps = sorted((pts[i + 1][0] - pts[i][0]) % 360.0 for i in range(len(pts) - 1))
    step = gaps[len(gaps) // 2]
    return step if step > 1e-3 else 1.0


def _points_in_window(pts: Sequence[tuple[float, float]], centre_deg: float,
                      half_width_deg: float) -> list[tuple[float, float]]:
    """All points whose angle is within half_width of centre, returned in
    increasing angular order relative to the window's left edge (so the
    0/360 seam is handled)."""
    picked = []
    for a, r in pts:
        off = _wrap180(a - centre_deg)
        if abs(off) <= half_width_deg:
            picked.append((off, r))
    picked.sort()
    return picked


def _find_pillar_run(window: Sequence[tuple[float, float]], expected_face_mm: float,
                     tol_mm: float, max_width_deg: float, p: DetectParams):
    """Look for a contiguous run of points that behaves like the front face of
    a pillar standing at the expected place: consistent range, range close to
    expected_face_mm, and angular extent no wider than a pillar would be.

    Returns (mean_range_mm, width_deg, n_points) for the best run, or None.

    Contiguity is required so that two unrelated points that merely happen to
    straddle the right range cannot be read as an object; a real 50 mm face
    returns a short, solid, flat run.
    """
    best = None
    i = 0
    n = len(window)
    while i < n:
        j = i + 1
        while j < n and abs(window[j][1] - window[j - 1][1]) <= p.max_range_step_mm:
            j += 1
        run = window[i:j]
        i = j
        if len(run) < p.min_points:
            continue
        ranges = [r for _, r in run]
        mean_r = sum(ranges) / len(ranges)
        if abs(mean_r - expected_face_mm) > tol_mm:
            continue
        width = run[-1][0] - run[0][0]
        if width > max_width_deg:
            # Right range, but far too wide to be a 50 mm pillar -- this is a
            # wall surface that happens to pass through the expected range.
            continue
        if best is None or abs(mean_r - expected_face_mm) < abs(best[0] - expected_face_mm):
            best = (mean_r, width, len(run))
    return best


# ---------------------------------------------------------------------------
# The detector
# ---------------------------------------------------------------------------
def detect_seat_occupancy(scan: Iterable,
                          robot_x_mm: float,
                          robot_y_mm: float,
                          robot_yaw_deg: float = 0.0,
                          params: DetectParams | None = None,
                          seat_list: Sequence[Seat] | None = None) -> list[SeatReading]:
    """Decide OCCUPIED / EMPTY / UNKNOWN for each of the 6 seats of the
    straightforward section the robot is currently in.

    Args:
        scan:          one LIDAR revolution, angles already mount-corrected
                       (see _normalise_scan).
        robot_x_mm:    lane-local x -- distance from the LEFT-hand wall,
                       i.e. the 270 deg reading of the localization step.
        robot_y_mm:    lane-local y -- 3000 minus the 0 deg reading.
        robot_yaw_deg: the robot's heading relative to grid north, positive
                       CLOCKWISE (grid-bearing sense), 0 when pointing exactly
                       down the lane. This is NOT optional in practice: the
                       seat table is in the lane frame while the scan is in the
                       chassis frame, and a car wanders several degrees within
                       a lane. Feed it your IMU yaw minus the lane's own grid
                       north.
        params:        thresholds, see DetectParams.
        seat_list:     override the seat table (tests only).

    Returns 6 SeatReadings in stable seat order.
    """
    p = params or DetectParams()
    pts = _normalise_scan(scan, p)
    step_deg = _angular_step_deg(pts)
    out: list[SeatReading] = []

    # Ranges are measured from the SENSOR, not from the pose reference point.
    # Move to the sensor's own lane-local position before taking any vector.
    # Robot forward in lane-local is bearing `robot_yaw_deg`, i.e. the unit
    # (sin yaw, cos yaw); the robot's left is that turned 90 deg anticlockwise
    # in the compass sense, i.e. (-cos yaw, sin yaw).
    yaw_rad = math.radians(robot_yaw_deg)
    sensor_x = (robot_x_mm
                + p.lidar_offset_forward_mm * math.sin(yaw_rad)
                - p.lidar_offset_lateral_mm * math.cos(yaw_rad))
    sensor_y = (robot_y_mm
                + p.lidar_offset_forward_mm * math.cos(yaw_rad)
                + p.lidar_offset_lateral_mm * math.sin(yaw_rad))

    for seat in (seat_list if seat_list is not None else seats()):
        dx = seat.x_mm - sensor_x
        dy = seat.y_mm - sensor_y

        expected_centre = math.hypot(dx, dy)
        # The sensor sees the pillar's NEAR FACE, which is up to half a pillar
        # closer than the seat centre. Face-on that is 25 mm; corner-on the
        # nearest point is 35.4 mm closer. Split the difference and widen the
        # tolerance to cover the rest, rather than pretending to know the
        # pillar's yaw.
        expected_face = expected_centre - PILLAR_HALF_MM

        bearing = bearing_to(dx, dy)
        rel_bearing = (bearing - robot_yaw_deg) % 360.0
        lidar_angle = bearing_to_lidar_angle(rel_bearing)

        # The pillar's own angular half-width at this range, plus pose margin.
        subtense_half = math.degrees(math.atan2(PILLAR_HALF_DIAG_MM,
                                                max(expected_centre, 1.0)))
        half_window = subtense_half + p.angular_margin_deg
        tol = (p.range_tol_mm + p.range_tol_frac * expected_centre
               + p.seat_position_slack_mm)

        rd = SeatReading(
            seat=seat, state=Occupancy.UNKNOWN,
            predicted_bearing_deg=bearing,
            predicted_rel_bearing_deg=rel_bearing,
            predicted_lidar_angle_deg=lidar_angle,
            search_half_width_deg=half_window,
            expected_centre_mm=expected_centre,
            expected_face_mm=expected_face,
            range_tolerance_mm=tol,
            # Deliberately computed from the pillar's FLAT face (25 mm
            # half-width), not the diagonal the search window uses. The window
            # asks "where might it be?" and should be generous; this asks
            # "would I have seen it at all?" and must be the worst case, since
            # a face-on pillar is the narrowest target it can present.
            expected_hits=round(
                2.0 * math.degrees(math.atan2(PILLAR_HALF_MM,
                                              max(expected_centre, 1.0))) / step_deg, 2),
        )

        # --- reachability gates, before looking at any data ---------------
        # These all leave the state at UNKNOWN. Each one is a case where the
        # seat is not observable, and "not observable" must never be allowed
        # to fall through into the EMPTY branch below.
        if expected_face < max(p.min_observable_face_mm, p.min_range_mm):
            rd.reason = (f"seat is {expected_face:.0f} mm away -- inside the "
                         f"sensor's dead zone, nothing there can return")
            out.append(rd)
            continue
        if _in_blind_arc(lidar_angle, p):
            rd.reason = (f"seat falls in the rear chassis blind wedge "
                         f"(lidar {lidar_angle:.1f} deg)")
            out.append(rd)
            continue

        window = _points_in_window(pts, lidar_angle, half_window)
        rd.n_points_in_window = len(window)
        if not window:
            rd.reason = "no returns at the predicted bearing"
            out.append(rd)
            continue

        nearest = min(r for _, r in window)
        rd.observed_range_mm = nearest

        # --- is there a pillar-shaped thing at the expected place? --------
        max_width = 2.0 * subtense_half * p.max_width_factor
        hit = _find_pillar_run(window, expected_face, tol, max_width, p)
        if hit is not None:
            mean_r, width, npts = hit
            rd.state = Occupancy.OCCUPIED
            rd.observed_range_mm = mean_r
            rd.observed_width_deg = width
            rd.residual_mm = mean_r - expected_face
            rd.n_points_in_window = len(window)
            rd.reason = (f"{npts} pts at {mean_r:.0f} mm across {width:.1f} deg "
                         f"(expected face {expected_face:.0f} +/- {tol:.0f} mm)")
            out.append(rd)
            continue

        # --- nothing certifiable at the seat: was it actually observed? ---
        if nearest < expected_face - tol:
            rd.residual_mm = nearest - expected_face
            rd.reason = (f"line of sight blocked at {nearest:.0f} mm, "
                         f"{expected_face - nearest:.0f} mm short of the seat")
            out.append(rd)          # stays UNKNOWN
            continue

        if nearest <= expected_face + tol:
            # Something IS sitting at the seat's range, it just could not be
            # certified as a pillar -- too few points to form a run, or a run
            # too wide to be 50 mm. That is ambiguous, NOT absence.
            #
            # This is the common case for a far seat at coarse angular
            # resolution: a 50 mm pillar at 1.7 m subtends 1.7 deg, so a 1 deg
            # sweep yields one or two returns and a dropout can leave one.
            # Reporting EMPTY there would be a confident wrong answer about a
            # seat that has a pillar on it, so it reports UNKNOWN and waits
            # for a closer look. (Caught by comparing against the mock's
            # ground truth on the dashboard's lane view, at 1 deg sampling.)
            rd.residual_mm = nearest - expected_face
            rd.reason = (f"return at {nearest:.0f} mm is within {tol:.0f} mm of the "
                         f"seat but didn't form a pillar-shaped run "
                         f"({rd.n_points_in_window} pts in the window) -- ambiguous, "
                         f"not empty")
            out.append(rd)          # stays UNKNOWN
            continue

        # Everything in the window is CLEARLY further away than the seat. The
        # ray flew over an empty seat and hit whatever is behind it -- but only
        # call that EMPTY if a pillar standing here would have been resolvable
        # in the first place (see min_expected_hits).
        if rd.expected_hits < p.min_expected_hits:
            rd.residual_mm = nearest - expected_face
            rd.reason = (f"nothing at the seat, but a pillar here would only give "
                         f"{rd.expected_hits:.1f} returns at this scan's {step_deg:.2f} deg "
                         f"resolution -- too few to call it empty")
            out.append(rd)          # stays UNKNOWN
            continue

        rd.state = Occupancy.EMPTY
        rd.residual_mm = nearest - expected_face
        rd.reason = (f"nearest return {nearest:.0f} mm is "
                     f"{nearest - expected_face:.0f} mm beyond the seat")
        out.append(rd)

    return out


def as_pairs(readings: Sequence[SeatReading]) -> list[tuple[tuple[float, float], Occupancy]]:
    """The shape of the original sketch: [((x, y), state), ...] in seat order,
    with the boolean widened to the three-state Occupancy."""
    return [((r.seat.x_mm, r.seat.y_mm), r.state) for r in readings]


def summary(readings: Sequence[SeatReading]) -> str:
    """One-line-per-seat dump for logs and the dashboard side panel."""
    lines = []
    for r in readings:
        lines.append(
            f"[{r.seat.index}] {r.seat.name:10} ({r.seat.x_mm:.0f},{r.seat.y_mm:.0f}) "
            f"brg {r.predicted_bearing_deg:6.1f} lidar {r.predicted_lidar_angle_deg:6.1f} "
            f"exp {r.expected_face_mm:7.1f} obs "
            f"{'  n/a ' if r.observed_range_mm is None else format(r.observed_range_mm, '7.1f')} "
            f"-> {r.state.value.upper():8} {r.reason}")
    return "\n".join(lines)
