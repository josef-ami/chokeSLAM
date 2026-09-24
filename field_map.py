"""
The path planner's world, in ONE right-handed frame for the whole loop
(checkpoint E, decisions #68, #71, #72).

LOOP frame = the dashboard's fixed full-loop frame (display.py): mm, X to the
right, Y up, the start lane (tracker slot 0) travelling up the screen. It is
exactly the tracker's lane frames glued together by its own corner transform,
so a tracker pose converts into it without any approximation. Headings in the
planner are MATHS angles th (radians, counter-clockwise from +X, left turns
positive); the tracker's unwrapped heading is a clockwise bearing from "up",
so th = 90 deg - heading.

What is in the world (all axis-aligned in the loop frame):
    outer wall      the 3000 x 3000 square (rule 13.1, walls square: 13.16)
    island          [1000, 2000]^2 (obstacle round: lane always 1000, section 8)
    pillars         50 x 50 mm at the seat's RULEBOOK centre (Fig. 11), one per
                    seat that is OCCUPIED -- or UNKNOWN (decision #72: an unknown
                    seat is a pillar that may be passed on either side)
    pass gates      for a pillar whose colour is known, a line from the pillar
                    to beyond the wall on its FORBIDDEN side, at the pillar's y
                    (the rulebook's "radius", App. A.5). No path may cross it.
                    Red keeps right, green keeps left (9.19), each relative to
                    its own lane's direction of travel.
    parking lot     two 200 x 20 mm limitations against the outer wall of the
                    start lane (Fig. 4 / 8d, 13.25), see parking_barriers().
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import config
import display
import lane_frame as lf
import seat_occupancy as so

OUTER = 3000.0
ISLAND = (1000.0, 1000.0, 2000.0, 2000.0)
PILLAR_HALF = 25.0


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float
    kind: str                 # "pillar" | "island" | "barrier"
    key: tuple                # e.g. ("pillar", slot, seat), ("island",), ("barrier", 0)


@dataclass
class Gate:
    a: tuple[float, float]
    b: tuple[float, float]
    key: tuple


@dataclass
class PillarInfo:
    slot: int
    seat: int
    X: float
    Y: float
    state: str                # "occupied" | "unknown"
    color: str                # "red" | "green" | "unknown" (includes pending / None)
    side: str                 # "right" | "left" | "either" -- the side of the pillar the car passes on


@dataclass
class WorldMap:
    direction: str
    rects: list[Rect] = field(default_factory=list)
    gates: list[Gate] = field(default_factory=list)
    pillars: list[PillarInfo] = field(default_factory=list)
    extra_nodes: list = field(default_factory=list)     # free-space graph nodes (corner squares)


# --- frame conversions -----------------------------------------------------------
def lane_point(slot: int, x: float, y: float, direction: str) -> tuple[float, float]:
    return display.lane_to_display(slot, x, y, direction)


def lane_heading(slot: int, psi_deg: float, direction: str) -> float:
    """Lane-relative clockwise yaw psi of slot `slot` -> maths heading (rad) in the loop frame."""
    b = display.lane_north_display_bearing(slot, direction) + psi_deg
    return math.radians(90.0 - b)


def lane_pose(slot: int, x: float, y: float, psi_deg: float, direction: str) -> tuple[float, float, float]:
    X, Y = lane_point(slot, x, y, direction)
    return X, Y, lane_heading(slot, psi_deg, direction)


def tracker_pose(trk) -> tuple[float, float, float]:
    """The tracker's pose (rear-axle midpoint) in the loop frame."""
    X, Y = lane_point(trk.slot, trk.x, trk.y, trk.direction)
    return X, Y, math.radians(90.0 - trk.heading)


def lane_rect(slot: int, x0: float, y0: float, x1: float, y1: float, direction: str) -> tuple[float, float, float, float]:
    ax, ay = lane_point(slot, x0, y0, direction)
    bx, by = lane_point(slot, x1, y1, direction)
    return min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)


def forbidden_x_end(side: str, direction: str) -> float:
    """Lane x the pass gate runs to for a pillar passed on `side`: the gate lies
    on the OTHER side, from the pillar to beyond that side's wall. +x points to
    the robot's left for CCW (h = -1) and to its right for CW (h = +1)."""
    h = lf.handedness(direction)
    forbidden = "left" if side == "right" else "right"
    dx = -h if forbidden == "left" else h          # +1: toward the island (x up), -1: toward the outer wall
    return 1100.0 if dx > 0 else -100.0


def side_of_color(color: str) -> str:
    return {"red": "right", "green": "left"}.get(color, "either")


def parking_barriers(direction: str) -> list[tuple[float, float, float, float]]:
    """The two limitations in START-LANE (slot 0) lane coordinates (x0, y0, x1, y1).
    Fig. 8d: in every section the lot sits at the end a CCW car reaches last;
    Fig. 4: the "right" limitation is next to the section boundary and the lot
    is 1.5 x the robot's length (taken between the limitations' inner faces,
    with the robot's full outline, wing included -- ASSUMPTION, see CHANGES E)."""
    t, lot, d = config.PARKING_BARRIER_MM, 1.5 * config.OUTLINE_LENGTH_MM, config.PARKING_DEPTH_MM
    if direction == "CCW":
        a = (2000.0 - t, 2000.0)
        b = (a[0] - lot - t, a[0] - lot)
    else:
        a = (1000.0, 1000.0 + t)
        b = (a[1] + lot, a[1] + lot + t)
    return [(0.0, a[0], d, a[1]), (0.0, b[0], d, b[1])]


def seat_table_from_tracker(trk) -> dict:
    """{(slot, seat index): (state, colour)} from the tracker's lane records."""
    out = {}
    for slot, rec in trk.lanes.items():
        for i, st in rec.seats.items():
            col = st.color if st.color in ("red", "green") else "unknown"
            out[(slot, i)] = (st.state, col)
    return out


def build_world(direction: str, seat_table: dict, slots, parking: bool = True,
                side_free=None) -> WorldMap:
    """seat_table: {(slot, seat): (state, colour)} ; slots: the lanes to include
    (a lane with no record counts as six UNKNOWN seats). side_free(slot, seat_y)
    -> True where the pass-side rule no longer applies (after lap 3, App. A.5)."""
    w = WorldMap(direction)
    w.rects.append(Rect(*ISLAND, "island", ("island",)))
    seats = so.seats()
    for slot in sorted(set(int(s) % 4 for s in slots)):
        for seat in seats:
            state, color = seat_table.get((slot, seat.index), ("unknown", "unknown"))
            if state == "empty":
                continue
            if (slot == 0 and state == "unknown" and seat.x_mm < 500.0
                    and config.START_LANE_OUTER_SEATS_EMPTY):
                continue          # rulebook Fig. 8e: the start section's signs all stand on the inner row
            side = side_of_color(color) if state == "occupied" else "either"
            if side_free is not None and side_free(slot, seat.y_mm):
                side = "either"
            X, Y = lane_point(slot, seat.x_mm, seat.y_mm, direction)
            w.pillars.append(PillarInfo(slot, seat.index, X, Y, state, color, side))
            w.rects.append(Rect(X - PILLAR_HALF, Y - PILLAR_HALF, X + PILLAR_HALF, Y + PILLAR_HALF,
                                "pillar", ("pillar", slot, seat.index)))
            if side != "either":
                a = lane_point(slot, seat.x_mm, seat.y_mm, direction)
                b = lane_point(slot, forbidden_x_end(side, direction), seat.y_mm, direction)
                w.gates.append(Gate(a, b, ("gate", slot, seat.index)))
    w.extra_nodes = corner_nodes(direction)
    if parking:
        for k, (x0, y0, x1, y1) in enumerate(parking_barriers(direction)):
            w.rects.append(Rect(*lane_rect(0, x0, y0, x1, y1, direction), "barrier", ("barrier", k)))
    return w


def corner_nodes(direction: str) -> list[tuple[float, float]]:
    """Free-space visibility-graph nodes: a 3 x 3 grid in every corner square
    (lane x 250 / 500 / 750, lane y 2250 / 2500 / 2750 of the lane before it).
    A corner square has no obstacles, hence no obstacle corners; without these
    the only place to turn 90 deg round the island would be the island's own
    inflated corner, and the arc-fit rule (B7) rejects that turn whenever a
    pillar near the section boundary makes the leg into it short."""
    out = []
    for slot in range(4):
        for x in (250.0, 500.0, 750.0):
            for y in (2250.0, 2500.0, 2750.0):
                out.append(lane_point(slot, x, y, direction))
    return out


def checkpoint(slot: int, direction: str, y: float = 1500.0) -> tuple[tuple[float, float], tuple[float, float]]:
    """A line across lane `slot` at lane y (default: the straight's midline).
    The planner must cross its checkpoints in order (lap progress)."""
    return lane_point(slot, -50.0, y, direction), lane_point(slot, 1050.0, y, direction)
