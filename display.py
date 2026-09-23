"""
The dashboard's fixed full-loop frame (decisions #16, #22; docs/CHANGES.md
section 10).

DISPLAY frame: the 3000 x 3000 mm loop square, X to the right, Y UP (the
page flips Y when drawing), bearings clockwise from +Y ("up the screen").
It never rescales. Lane 1 (the start lane, tracker slot 0) is drawn with its
direction of travel UP the screen, at the place it holds when the loop
closes:

    CCW (outer wall on the robot's right): the RIGHT-hand strip,
        X = 3000 - x,  Y = y
    CW  (outer wall on the robot's left):  the LEFT-hand strip,
        X = x,         Y = y

where (x, y) are lane coordinates (x from the outer wall, y along travel).

Every later lane is placed with the corner transform the tracker uses
(lane_frame.corner_transform: new_y = old_x, new_x = 3000 - old_y), run
backwards: a point in slot k is taken back to slot k-1 by
    old_x = new_y,  old_y = 3000 - new_x
and so on down to slot 0. So the drawing agrees with the tracker by
construction: a pose re-expressed at a turn lands on the same screen point.

Headings: the tracker's unwrapped heading is already relative to the start
lane's grid north, i.e. to "up": it is the display bearing directly.
"""
from __future__ import annotations

import math

OUTER = 3000.0


def _to_slot0(slot: int, x: float, y: float) -> tuple[float, float]:
    for _ in range(slot % 4):
        x, y = y, OUTER - x
    return x, y


def lane_to_display(slot: int, x: float, y: float, direction: str) -> tuple[float, float]:
    """Lane coordinates of tracker slot `slot` (0 = start lane) -> display (X, Y)."""
    x0, y0 = _to_slot0(slot, x, y)
    if direction == "CCW":
        return OUTER - x0, y0
    if direction == "CW":
        return x0, y0
    raise ValueError(f"direction must be CCW or CW, got {direction!r}")


def lane_north_display_bearing(slot: int, direction: str) -> float:
    """Display bearing (clockwise from up) of slot `slot`'s direction of travel."""
    return ((-90.0 if direction == "CCW" else 90.0) * (slot % 4)) % 360.0


def robot_to_display(px: float, py: float, bearing_deg: float, fwd: float, right: float) -> tuple[float, float]:
    """A robot-frame vector (fwd, right) from display point (px, py), robot
    facing display bearing `bearing_deg` -> display point."""
    b = math.radians(bearing_deg)
    return px + fwd * math.sin(b) + right * math.cos(b), py + fwd * math.cos(b) - right * math.sin(b)


def lane_outline(slot: int, direction: str) -> dict:
    """The lane's walls in display coordinates: the outer wall (x = 0, the
    whole 3000 mm) and the island wall (x = 1000, y 1000..2000), plus the
    strip's corners for shading."""
    d = lambda x, y: lane_to_display(slot, x, y, direction)
    return {"outer": [d(0, 0), d(0, OUTER)],
            "island": [d(1000, 1000), d(1000, 2000)],
            "strip": [d(0, 0), d(0, OUTER), d(1000, OUTER), d(1000, 0)]}


class GlobalToDisplay:
    """Mock mode only (the truth): the simulator's global mat frame -> display.
    Slot 0's lane frame, extended to the whole mat, is an affine map of the
    global plane, so global -> slot-0 lane coordinates -> display is exact
    everywhere (the corner transform matches global geometry,
    test_lane_frame.py)."""

    def __init__(self, start_section: str, direction: str):
        import lane_frame as lf
        self._lf, self.sec0, self.direction = lf, start_section, direction
        self.north0 = lf.grid_north_bearing(start_section, direction)

    def point(self, gx: float, gy: float) -> tuple[float, float]:
        x, y = self._lf.global_to_lane(self.sec0, self.direction, gx, gy)
        return lane_to_display(0, x, y, self.direction)

    def bearing(self, grid_bearing_deg: float) -> float:
        return (grid_bearing_deg - self.north0) % 360.0
