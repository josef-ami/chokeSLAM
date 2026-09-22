"""
Geometry of the WRO 2026 Future Engineers Obstacle Challenge field.

Coordinate convention (all millimetres):
    - Origin (0, 0) is the outer wall's bottom-left corner, as if the two
      straight outer walls were extended to meet it (this field has sharp,
      perpendicular outer corners, so that's also its literal physical corner).
    - +X points right ("east"), +Y points up ("north").
    - Heading is a GRID / COMPASS BEARING in degrees, 0..360, measured
      CLOCKWISE from grid north: 0=north(+Y), 90=east(+X), 180=south(-Y),
      270=west(-X). This matches a compass / IMU, not the maths convention.
      Convert to the maths angle used for x/y trig with
      math_deg = (90 - bearing) % 360. (Robot-relative LIDAR angles are a
      SEPARATE frame and unchanged: 0=forward, 90=left, 180=back, 270=right.)

Field layout: a square lane (constant width, per the Obstacle Challenge rule
"distance between the track borders will be always 1000mm") running around a
square island of the same wall style. Because the island is an INWARD offset
of a square with sharp corners, it stays a sharp-cornered square too -- no
corner rounding is needed anywhere for a constant-width lane on a square
outer wall (rounding is only needed when offsetting *outward*, or around a
shape that isn't rectilinear).

*** VERIFY THESE TWO NUMBERS AGAINST YOUR ACTUAL MAT / THE RULES PDF BEFORE
*** TRUSTING THIS MODULE. OUTER_SIZE_MM in particular was read off the mat
*** artwork, not stated explicitly in the rules pages we reviewed.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

# ---------------------------------------------------------------------------
# Core field dimensions -- CONFIRM THESE
# ---------------------------------------------------------------------------
OUTER_SIZE_MM = 3000.0          # outer wall: square, sharp corners
LANE_WIDTH_MM = 1000.0          # Obstacle Challenge only (+/- 10mm at Worlds).
                                 # Open Challenge lane width varies 600/1000mm
                                 # PER SECTION -- this module models the
                                 # Obstacle Challenge field only.
INNER_SIZE_MM = OUTER_SIZE_MM - 2 * LANE_WIDTH_MM   # island size (sharp corners)

# Island occupies the centred square [ISLAND_MIN, ISLAND_MAX] on both axes.
ISLAND_MIN_MM = (OUTER_SIZE_MM - INNER_SIZE_MM) / 2.0
ISLAND_MAX_MM = ISLAND_MIN_MM + INNER_SIZE_MM

Section = Literal["S", "E", "N", "W"]

# Going around the loop with every corner a LEFT turn (mathematically
# counter-clockwise) visits the sections in this order. Reverse the dict for
# a right-turn (clockwise) round -- the challenge direction is randomised
# per round, so read this from wherever your code records the chosen
# direction, don't hardcode one.
NEXT_SECTION_CCW: dict[Section, Section] = {"S": "E", "E": "N", "N": "W", "W": "S"}
NEXT_SECTION_CW: dict[Section, Section] = {v: k for k, v in NEXT_SECTION_CCW.items()}

# Heading (GRID BEARING, deg) the robot must be at to be "broadside": front
# pointing at the OUTER wall, back at the inner wall (island). Each section
# faces its own outer wall: S->south(180), E->east(90), N->north(0), W->west(270).
BROADSIDE_HEADING_DEG: dict[Section, float] = {"S": 180.0, "E": 90.0, "N": 0.0, "W": 270.0}

# IMPORTANT, found while testing this module: the island is a square (an
# inward offset of the outer square by LANE_WIDTH_MM on each side), so it
# only directly faces the MIDDLE portion of each 3000mm edge -- exactly the
# region where along_mm falls between ISLAND_MIN_MM and ISLAND_MAX_MM. Near
# a corner (roughly the first/last 1000mm of a section), a perpendicular
# "back" ray sails past the island's corner and travels much further before
# hitting anything, so front+back no longer sums to ~LANE_WIDTH_MM -- the
# broadside fix's own sanity check correctly REJECTS a reading taken there
# (verified: it does not fail silently), but that also means a fix attempted
# right at a corner-turn completion will usually just fail, not just be
# less precise. Wait until along_mm is inside this safe range before
# attempting a fix -- e.g. drive a short distance into the new section
# after a corner turn before calling apply_lidar_fix().
# Margin needed so that even a ray at the EDGE of the front/back search
# window (config.FRONT_BACK_SEARCH_WINDOW_DEG either side of straight
# ahead), not just the centre ray, still lands on the island's flat face
# rather than skimming past its corner. At a plausible ~600mm back-distance
# and a 15 degree window, lateral deviation is ~600*tan(15deg)=~160mm;
# this adds real headroom on top of that (confirmed by testing -- 100mm
# was NOT enough and produced silent-looking failures right at the zone's
# edge). Re-check this if you change FRONT_BACK_SEARCH_WINDOW_DEG or your
# typical lane lateral position.
SAFE_FIX_MARGIN_MM = 250.0
SAFE_FIX_ALONG_MIN_MM = ISLAND_MIN_MM + SAFE_FIX_MARGIN_MM
SAFE_FIX_ALONG_MAX_MM = ISLAND_MAX_MM - SAFE_FIX_MARGIN_MM


@dataclass(frozen=True)
class Slot:
    """One of the 24 candidate pillar placeholders (4 sections x 6 slots)."""
    section: Section
    row: int          # 0..2, position along the section's length
    col: int           # 0..1, 0 = nearer the inner wall, 1 = nearer the outer wall
    x_mm: float
    y_mm: float


# Approximate slot layout within a section: a 2 (across the lane) x 3 (along
# the section) grid, matching the 2x3 zone grid shown in the rules figures.
# *** These fractions are a reasonable schematic guess, not measured off an
# *** official dimensioned drawing -- adjust SLOT_COL_FRACS / SLOT_ROW_FRACS
# *** once you have exact card/zone spacing.
SLOT_COL_FRACS = (0.25, 0.75)   # lateral position within the lane, as a
                                 # fraction of LANE_WIDTH_MM from the outer wall
SLOT_ROW_FRACS = (1 / 6, 0.5, 5 / 6)  # position along the section length


def _section_axes(section: Section):
    """Returns (start_corner, along_unit, lateral_unit) for a section, where
    'along' runs the length of the section (corner to corner) and 'lateral'
    runs across the lane from the OUTER wall towards the inner wall -- i.e.
    lateral=0 is the outer wall, lateral=LANE_WIDTH_MM is the inner wall."""
    if section == "S":
        return (0.0, 0.0), (1.0, 0.0), (0.0, 1.0)
    if section == "E":
        return (OUTER_SIZE_MM, 0.0), (0.0, 1.0), (-1.0, 0.0)
    if section == "N":
        return (OUTER_SIZE_MM, OUTER_SIZE_MM), (-1.0, 0.0), (0.0, -1.0)
    if section == "W":
        return (0.0, OUTER_SIZE_MM), (0.0, -1.0), (1.0, 0.0)
    raise ValueError(section)


def local_to_global(section: Section, along_mm: float, lateral_mm: float) -> tuple[float, float]:
    """along_mm: 0..OUTER_SIZE_MM along the section, from its CCW-first corner.
    lateral_mm: 0 (outer wall) .. LANE_WIDTH_MM (inner wall)."""
    (sx, sy), (ax, ay), (lx, ly) = _section_axes(section)
    x = sx + ax * along_mm + lx * lateral_mm
    y = sy + ay * along_mm + ly * lateral_mm
    return x, y


def all_slots() -> list[Slot]:
    slots = []
    for section in ("S", "E", "N", "W"):
        for row, row_frac in enumerate(SLOT_ROW_FRACS):
            for col, col_frac in enumerate(SLOT_COL_FRACS):
                along = row_frac * OUTER_SIZE_MM
                lateral = col_frac * LANE_WIDTH_MM
                x, y = local_to_global(section, along, lateral)
                slots.append(Slot(section=section, row=row, col=col, x_mm=x, y_mm=y))
    assert len(slots) == 24
    return slots


def nearest_slot(x_mm: float, y_mm: float, slots: list[Slot] | None = None) -> tuple[Slot, float]:
    """Returns (slot, distance_mm) for the placeholder slot closest to a
    detected obstacle position -- use this for the occupancy classification
    step, with a tolerance check against distance_mm before accepting it."""
    if slots is None:
        slots = all_slots()
    best = min(slots, key=lambda s: (s.x_mm - x_mm) ** 2 + (s.y_mm - y_mm) ** 2)
    dist = ((best.x_mm - x_mm) ** 2 + (best.y_mm - y_mm) ** 2) ** 0.5
    return best, dist
