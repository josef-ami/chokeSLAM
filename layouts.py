"""
WRO 2026 obstacle-round layouts, drawn exactly as the rulebook describes
(section 8, "Obstacle Challenge rounds", Figs. 8b-8e). Simulation only.

    1. coin x2   -> the section with the single sign (Fig. 8b): its middle seat
                    on the OUTER row, lane (400, 1500)
    2. coin      -> its colour (heads green, tails red)
    3. 36 cards (Fig. 8c) minus card 9 (green chosen) or 10 (red chosen); one card
       per straightforward section for the next three sections CLOCKWISE,
       without replacement
    4. coin x2   -> the parking / start section (independent of 1-3); its signs
                    move to the inner row (Fig. 8e)
    start        -> the middle zone above the parking lot (the team's choice, §8),
                    facing the round direction (9.8)

Cards are read off Fig. 8c: each card is a straightforward section drawn with
the inner wall (thick line) at the top, i.e. as seen on the map with the
island above it. Row "U" = the dashed line nearer the inner wall (lane x = 600),
"L" = the one nearer the outer wall (x = 400). Columns l / m / r = the
section's left boundary, middle, right boundary. Rotating every section to the
bottom of the map, a CCW car drives left -> right, so for CCW l / m / r =
lane y 1000 / 1500 / 2000 and for CW l / m / r = y 2000 / 1500 / 1000.
The card list keeps the figure's duplicates (e.g. 11 = 5, 14 = 16).
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field

import config
import lane_frame as lf
import seat_occupancy as so

CARDS = {
    1: ["GlU"], 2: ["RlU"], 3: ["GmU"], 4: ["RmU"], 5: ["GrU"], 6: ["RrU"],
    7: ["GlL"], 8: ["RlL"], 9: ["GmL"], 10: ["RmL"], 11: ["GrU"], 12: ["RrU"],
    13: ["GlL", "GrU"], 14: ["GlL", "RrU"], 15: ["RlL", "GrU"], 16: ["GlL", "RrU"], 17: ["RlL", "GrU"],
    18: ["RlL", "RrU"], 19: ["GlU", "GrL"], 20: ["GlU", "RrL"], 21: ["RlU", "GrL"], 22: ["GlU", "RrL"],
    23: ["RlU", "GrL"], 24: ["RlU", "RrL"], 25: ["GlU", "GrU"], 26: ["GlU", "RrU"], 27: ["RlU", "GrU"],
    28: ["GlU", "RrU"], 29: ["RlU", "GrU"], 30: ["RlU", "RrU"], 31: ["GlL", "GrL"], 32: ["GlL", "RrL"],
    33: ["RlL", "GrL"], 34: ["GlL", "RrL"], 35: ["RlL", "GrL"], 36: ["RlL", "RrL"],
}
COLOR = {"G": "green", "R": "red"}


def _seat_index(x: float, y: float) -> int:
    for s in so.seats():
        if abs(s.x_mm - x) < 1 and abs(s.y_mm - y) < 1:
            return s.index
    raise ValueError((x, y))


def card_seats(card: int, direction: str) -> list[tuple[int, str]]:
    out = []
    for code in CARDS[card]:
        col, c, row = COLOR[code[0]], code[1], code[2]
        y = {"l": 1000.0, "m": 1500.0, "r": 2000.0}[c]
        if direction == "CW":
            y = 3000.0 - y
        x = 600.0 if row == "U" else 400.0
        out.append((_seat_index(x, y), col))
    return out


@dataclass
class Layout:
    direction: str
    start_section: str
    pillars: dict = field(default_factory=dict)      # section -> [(seat index, colour)]
    single_section: str = ""
    cards: dict = field(default_factory=dict)        # section -> card number
    start_x: float = 500.0                           # rear-axle midpoint, start lane frame
    start_y: float = 0.0

    def slot_section(self, slot: int) -> str:
        sec = self.start_section
        for _ in range(slot % 4):
            sec = lf.NEXT_SECTION[self.direction][sec]
        return sec

    def describe(self) -> str:
        parts = []
        for k in range(4):
            sec = self.slot_section(k)
            ps = ", ".join(f"{so.seats()[i].name}:{c[0].upper()}" for i, c in self.pillars[sec])
            parts.append(f"{sec}[{ps}]")
        return f"{self.direction} start {self.start_section}: " + " ".join(parts)


def draw(rng: random.Random, direction: str | None = None) -> Layout:
    direction = direction or rng.choice(["CCW", "CW"])
    single = rng.choice(lf.SECTIONS)
    colour = rng.choice(["green", "red"])
    deck = [c for c in CARDS if c != (9 if colour == "green" else 10)]
    rng.shuffle(deck)
    pillars = {single: [(_seat_index(400.0, 1500.0), colour)]}
    cards = {}
    sec = single
    for k in range(3):
        sec = lf.NEXT_SECTION["CW"][sec]             # "next (considered clockwise)"
        cards[sec] = deck[k]
        pillars[sec] = card_seats(deck[k], direction)
    start = rng.choice(lf.SECTIONS)
    moved = []
    for i, col in pillars[start]:
        s = so.seats()[i]
        moved.append((_seat_index(600.0, s.y_mm), col))
    pillars[start] = moved
    lay = Layout(direction, start, pillars, single, cards)
    # the middle zone above the lot (x 400-600), in the half of the section holding the lot
    zone_y = 1750.0 if direction == "CCW" else 1250.0
    lay.start_x = 500.0
    lay.start_y = zone_y - (config.OUTLINE_FRONT_MM - config.OUTLINE_REAR_MM) / 2.0
    return lay
