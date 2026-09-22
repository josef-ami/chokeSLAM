"""
The LANE frame, and its relation to the global mat frame.

LANE frame (one per lane, anchored to the direction of travel). Agreed conventions:
    y  = along the lane in the direction of travel.
         y = 0 at the wall behind (the corner the robot entered from),
         y = 3000 at the wall ahead.  At initialisation y = 3000 - (front range).
    x  = distance from the OUTER wall, in both driving directions.
         x = 0 at the outer wall, x = 1000 at the island wall.
    Bearings are measured CLOCKWISE from the lane's grid north (= +y, the
    direction of travel): 0 = ahead, 90 = right, 180 = behind, 270 = left.

Because x is always "distance from the outer wall", the direction +x points
depends on the round direction:
    CCW (every corner a left turn): island on the robot's LEFT, outer wall on
        its RIGHT -> +x points to the robot's LEFT.        handedness h = -1
    CW  (every corner a right turn): island on the robot's RIGHT, outer wall
        on its LEFT -> +x points to the robot's RIGHT.     handedness h = +1

A clockwise bearing b (relative to grid north) is therefore the lane-frame unit
vector (h * sin b, cos b), and a lane-frame vector (dx, dy) has bearing
atan2(h * dx, dy). For CCW that is 360 - atan2(dx, dy) -- the agreed
"2*pi - atan" form -- and for CW it is atan2(dx, dy).

The global frame (mat_geometry.py) is only used to build worlds for the
simulator and the tests.
"""
from __future__ import annotations

import math

from mat_geometry import LANE_WIDTH_MM, OUTER_SIZE_MM

SECTIONS = ("S", "E", "N", "W")
DIRECTIONS = ("CCW", "CW")

# section -> inward unit normal of that section's OUTER wall (global frame).
# Moving along it from the outer wall is moving in +x of the lane frame.
_OUTER_NORMAL = {
    "S": (0.0, 1.0),
    "E": (-1.0, 0.0),
    "N": (0.0, -1.0),
    "W": (1.0, 0.0),
}

# (section, direction) -> (lane origin, travel unit vector), global frame.
# The lane origin is the point on the OUTER wall line at y = 0, i.e. the
# outer corner behind the robot (the corner it enters the lane from).
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

# Section order in the direction of travel.
NEXT_SECTION = {
    "CCW": {"S": "E", "E": "N", "N": "W", "W": "S"},
    "CW": {"S": "W", "W": "N", "N": "E", "E": "S"},
}


def _check_direction(direction: str) -> None:
    if direction not in DIRECTIONS:
        raise ValueError(f"direction must be 'CCW' or 'CW', got {direction!r}")


def handedness(direction: str) -> int:
    """-1 for CCW (+x points to the robot's left), +1 for CW (+x points right)."""
    _check_direction(direction)
    return -1 if direction == "CCW" else +1


def outer_wall_side(direction: str) -> str:
    """Which side of the robot the outer wall is on while driving."""
    _check_direction(direction)
    return "right" if direction == "CCW" else "left"


def outer_wall_lidar_angle(direction: str) -> float:
    """Clockwise LIDAR angle that points at the outer wall when aligned:
    CCW -> 90 (right), CW -> 270 (left)."""
    _check_direction(direction)
    return 90.0 if direction == "CCW" else 270.0


def bearing_of(dx_mm: float, dy_mm: float, direction: str) -> float:
    """Clockwise bearing (0 = grid north = +y) of the lane-frame vector (dx, dy).

    CCW: 360 - atan2(dx, dy)   (+x points to the robot's left)
    CW :       atan2(dx, dy)   (+x points to the robot's right)

    Two-argument atan2 so that a vector beside or behind the robot (dy <= 0)
    is not folded forward and dy = 0 does not divide by zero.
    """
    _check_direction(direction)
    a = math.degrees(math.atan2(dx_mm, dy_mm))
    if direction == "CCW":
        return (360.0 - a) % 360.0
    return a % 360.0


def unit_of_bearing(bearing_deg: float, direction: str) -> tuple[float, float]:
    """Lane-frame unit vector (dx, dy) of a clockwise bearing. Inverse of bearing_of."""
    h = handedness(direction)
    b = math.radians(bearing_deg)
    return h * math.sin(b), math.cos(b)


def grid_north_bearing(section: str, direction: str) -> float:
    """Global grid bearing of this lane's grid north (its direction of travel)."""
    _check_direction(direction)
    _, (ux, uy) = _ENTRY[(section, direction)]
    return (90.0 - math.degrees(math.atan2(uy, ux))) % 360.0


def lane_to_global(section: str, direction: str,
                   x_from_outer: float, y_along: float) -> tuple[float, float]:
    _check_direction(direction)
    (ox, oy), (ux, uy) = _ENTRY[(section, direction)]
    nx, ny = _OUTER_NORMAL[section]
    return (ox + ux * y_along + nx * x_from_outer,
            oy + uy * y_along + ny * x_from_outer)


def global_to_lane(section: str, direction: str,
                   gx: float, gy: float) -> tuple[float, float]:
    """Returns (x_from_outer, y_along)."""
    _check_direction(direction)
    (ox, oy), (ux, uy) = _ENTRY[(section, direction)]
    nx, ny = _OUTER_NORMAL[section]
    px, py = gx - ox, gy - oy
    return px * nx + py * ny, px * ux + py * uy


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def heading_to_yaw(heading_bearing_deg: float, section: str, direction: str) -> float:
    """Global grid bearing -> yaw relative to this lane's grid north (clockwise +)."""
    return wrap180(heading_bearing_deg - grid_north_bearing(section, direction))


def yaw_to_heading(yaw_deg: float, section: str, direction: str) -> float:
    return (grid_north_bearing(section, direction) + yaw_deg) % 360.0


def offset_in_lane(x_mm: float, y_mm: float, yaw_deg: float, direction: str,
                   forward_mm: float = 0.0, left_mm: float = 0.0) -> tuple[float, float]:
    """A point fixed to the robot (forward_mm ahead of and left_mm to the left
    of the point (x_mm, y_mm), robot yawed yaw_deg clockwise from grid north)
    expressed in the lane frame. Used for the LIDAR lever arm."""
    fx, fy = unit_of_bearing(yaw_deg, direction)
    lx, ly = unit_of_bearing(yaw_deg - 90.0, direction)
    return (x_mm + forward_mm * fx + left_mm * lx,
            y_mm + forward_mm * fy + left_mm * ly)


def corner_transform(x_mm: float, y_mm: float, yaw_deg: float,
                     direction: str) -> tuple[float, float, float]:
    """Re-express a pose from the lane being left in the lane being entered.

    The corner square is shared by both lanes. The new lane's wall behind
    (its y = 0) is the old lane's outer wall (old x = 0), and the new lane's
    outer wall (its x = 0) is the old lane's wall ahead (old y = 3000):
        new_y = old_x
        new_x = 3000 - old_y
    This holds for both round directions because x is always measured from
    the outer wall. The new lane's grid north is the old one turned 90 deg
    toward the round direction (left for CCW, right for CW), so
        new_yaw = old_yaw + 90   (CCW)
        new_yaw = old_yaw - 90   (CW)
    It is an exact change of coordinates, valid at any instant.
    """
    _check_direction(direction)
    new_yaw = yaw_deg + 90.0 if direction == "CCW" else yaw_deg - 90.0
    return OUTER_SIZE_MM - y_mm, x_mm, wrap180(new_yaw)
