"""
Conversion between the global mat frame and the LANE-LOCAL frame that
seat_occupancy.py works in.

GLOBAL frame (as mat_geometry.py defines it): origin at the outer wall's
bottom-left corner, +X right, +Y up, headings are grid bearings measured
CLOCKWISE from +Y.

LANE-LOCAL frame (the "grid north" convention): one frame per lane, anchored
to the direction of travel.
    +y = direction of travel;  y = 0 at the wall behind, 3000 at the wall ahead
    +x = to the robot's right; x = 0 at the LEFT-hand wall, 1000 at the right
    yaw = heading relative to that lane's grid north, positive clockwise

*** A NOTE ON THE CCW/CW CONVENTION ***
This module encodes the geometrically correct rule:

    driving CCW (S->E->N->W), every corner is a LEFT turn, so the island is on
    the robot's left and the OUTER WALL IS ON ITS RIGHT.  CW mirrors it.

localization.py currently encodes the opposite (its compute_start_of_run_fix
takes the outer wall to be on the LEFT for CCW, and its driving_heading_deg
returns broadside + 90 for CCW where the geometry gives broadside - 90).
Those are the unfixed Findings 2 and 3. This module does NOT follow them --
it would be wrong on the mat if it did. `convention_disagreements()` below
reports the mismatch so the dashboard can show it rather than let two
conventions quietly coexist.
"""
from __future__ import annotations

import math

OUTER_SIZE_MM = 3000.0
LANE_WIDTH_MM = 1000.0

# section -> inward normal of that section's OUTER wall
_OUTER_NORMAL = {
    "S": (0.0, 1.0),
    "E": (-1.0, 0.0),
    "N": (0.0, -1.0),
    "W": (1.0, 0.0),
}

# (section, direction) -> (entry corner on the outer wall, travel unit vector).
# The entry corner is the end of this section the robot arrives at; travelling
# CCW the sections chain S -> E -> N -> W, each one's exit corner being the
# next one's entry corner.
_ENTRY = {
    ("S", "CCW"): ((0.0, 0.0), (1.0, 0.0)),
    ("S", "CW"): ((OUTER_SIZE_MM, 0.0), (-1.0, 0.0)),
    ("E", "CCW"): ((OUTER_SIZE_MM, 0.0), (0.0, 1.0)),
    ("E", "CW"): ((OUTER_SIZE_MM, OUTER_SIZE_MM), (0.0, -1.0)),
    ("N", "CCW"): ((OUTER_SIZE_MM, OUTER_SIZE_MM), (-1.0, 0.0)),
    ("N", "CW"): ((0.0, OUTER_SIZE_MM), (1.0, 0.0)),
    ("W", "CCW"): ((0.0, OUTER_SIZE_MM), (0.0, -1.0)),
    ("W", "CW"): ((0.0, 0.0), (0.0, 1.0)),
}

SECTIONS = ("S", "E", "N", "W")
DIRECTIONS = ("CCW", "CW")


def outer_wall_is_on_the_right(direction: str) -> bool:
    """CCW -> True. See the module docstring."""
    return direction == "CCW"


def grid_north_bearing(section: str, direction: str) -> float:
    """The global grid bearing of this lane's grid north -- i.e. the heading a
    robot faces while driving down it."""
    _, (ux, uy) = _ENTRY[(section, direction)]
    return (90.0 - math.degrees(math.atan2(uy, ux))) % 360.0


def lane_to_global(section: str, direction: str,
                   x_local: float, y_local: float) -> tuple[float, float]:
    (ox, oy), (ux, uy) = _ENTRY[(section, direction)]
    nx, ny = _OUTER_NORMAL[section]
    d_outer = (LANE_WIDTH_MM - x_local) if outer_wall_is_on_the_right(direction) else x_local
    return (ox + ux * y_local + nx * d_outer,
            oy + uy * y_local + ny * d_outer)


def global_to_lane(section: str, direction: str,
                   gx: float, gy: float) -> tuple[float, float]:
    (ox, oy), (ux, uy) = _ENTRY[(section, direction)]
    nx, ny = _OUTER_NORMAL[section]
    px, py = gx - ox, gy - oy
    y_local = px * ux + py * uy
    d_outer = px * nx + py * ny
    x_local = (LANE_WIDTH_MM - d_outer) if outer_wall_is_on_the_right(direction) else d_outer
    return x_local, y_local


def lane_affine(section: str, direction: str) -> list[float]:
    """The same map as global_to_lane(), flattened to six affine coefficients
    [a11, a12, c1, a21, a22, c2] such that

        x_local = a11*gx + a12*gy + c1
        y_local = a21*gx + a22*gy + c2

    Sent to the dashboard so the browser can re-project the point cloud it has
    already received for the mat view into the lane view, instead of the server
    streaming a second copy of every scan point at 10 Hz. It is a rigid
    transform, so six numbers is the whole of it.
    """
    (ox, oy), (ux, uy) = _ENTRY[(section, direction)]
    nx, ny = _OUTER_NORMAL[section]
    if outer_wall_is_on_the_right(direction):
        s, k = -1.0, LANE_WIDTH_MM
    else:
        s, k = 1.0, 0.0
    return [s * nx, s * ny, k - s * (nx * ox + ny * oy),
            ux, uy, -(ux * ox + uy * oy)]


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def heading_to_yaw(heading_bearing_deg: float, section: str, direction: str) -> float:
    """Global grid bearing -> yaw relative to this lane's grid north."""
    return wrap180(heading_bearing_deg - grid_north_bearing(section, direction))


def pose_to_lane(section: str, direction: str, gx: float, gy: float,
                 heading_bearing_deg: float) -> tuple[float, float, float]:
    """Global pose -> (x_local, y_local, yaw_deg).

    Deliberately derived from the estimator's GLOBAL x/y rather than from its
    along_mm / lateral_mm, so the lane view is a pure re-projection of exactly
    the point the mat view draws. If the pose is wrong, both views are wrong
    identically and stay comparable -- which is what you want while the
    localization findings are still open.
    """
    x, y = global_to_lane(section, direction, gx, gy)
    return x, y, heading_to_yaw(heading_bearing_deg, section, direction)


def robot_to_lane(fx_mm: float, fy_mm: float, origin_x: float, origin_y: float,
                  yaw_deg: float) -> tuple[float, float]:
    """A point in the ROBOT frame (fx = forward, fy = left, the convention
    scan_processing uses) -> lane-local, given the sensor's lane-local origin
    and the robot's yaw.

    Robot forward in lane-local is bearing `yaw`, the unit (sin yaw, cos yaw);
    the robot's left is that turned 90 deg anticlockwise in the compass sense,
    i.e. (-cos yaw, sin yaw).
    """
    yr = math.radians(yaw_deg)
    s, c = math.sin(yr), math.cos(yr)
    return (origin_x + fx_mm * s - fy_mm * c,
            origin_y + fx_mm * c + fy_mm * s)


def sensor_origin(robot_x: float, robot_y: float, yaw_deg: float,
                  offset_forward_mm: float = 0.0,
                  offset_lateral_mm: float = 0.0) -> tuple[float, float]:
    """The LIDAR's own lane-local position, given the pose reference point and
    the mount lever arm (forward positive ahead, lateral positive to the left,
    matching config.LIDAR_OFFSET_*)."""
    return robot_to_lane(offset_forward_mm, offset_lateral_mm,
                         robot_x, robot_y, yaw_deg)


def convention_disagreements() -> list[str]:
    """Compare this module's geometry against localization.py's, and return a
    human-readable list of every place they disagree. Empty list == agreement.

    Surfaced in the dashboard so that two conflicting conventions cannot sit in
    one codebase unremarked. Import failures are swallowed -- this is a
    diagnostic, it must never be the thing that stops the dashboard booting.
    """
    out: list[str] = []
    try:
        import localization as loc
    except Exception as e:  # pragma: no cover
        return [f"could not import localization.py to compare: {e}"]

    for direction in DIRECTIONS:
        for section in SECTIONS:
            try:
                theirs = loc.driving_heading_deg(section, direction)
            except Exception as e:  # pragma: no cover
                out.append(f"driving_heading_deg({section},{direction}) raised {e}")
                continue
            ours = grid_north_bearing(section, direction)
            if abs(wrap180(theirs - ours)) > 1e-6:
                out.append(
                    f"driving heading {direction}/{section}: localization.py says "
                    f"{theirs:.0f} deg, geometry says {ours:.0f} deg "
                    f"({abs(wrap180(theirs - ours)):.0f} deg apart)")

    # localization.compute_start_of_run_fix uses left_d for CCW, i.e. it takes
    # the outer wall to be on the LEFT when driving CCW.
    if outer_wall_is_on_the_right("CCW"):
        out.append("outer wall side: localization.py takes it to be on the LEFT "
                   "when driving CCW; geometry puts it on the RIGHT "
                   "(lateral comes back mirrored about the lane centreline)")
    return out
