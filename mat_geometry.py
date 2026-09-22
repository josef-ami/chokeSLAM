"""
Global (whole-mat) geometry of the WRO 2026 Future Engineers Obstacle Challenge
field. Used only to BUILD a world: by the mock simulator and by the tests'
ray-casters. Nothing on the robot's own decision path works in this frame --
initialisation, tracking and obstacle checks all work in the lane frame
(see lane_frame.py / seat_occupancy.py).

Global frame (all millimetres):
    - origin (0, 0) at the outer wall's bottom-left inside corner
    - +X right ("east"), +Y up ("north")
    - headings are GRID BEARINGS: degrees clockwise from +Y (0 = north,
      90 = east, 180 = south, 270 = west). Maths angle = 90 - bearing.

Dimensions, all from the WRO 2026 General Rules:
    - 13.1   racetrack inner size 3000 x 3000 mm (+/- 5 mm)
    - Sec 8  Obstacle Challenge: distance between the track borders is always
             1000 mm (+/- 10 mm at the International Final)
    - Fig 2 / Fig 11: the island is the centred 1000 x 1000 square
"""
from __future__ import annotations

OUTER_SIZE_MM = 3000.0          # rule 13.1
LANE_WIDTH_MM = 1000.0          # rulebook section 8, Obstacle Challenge rounds
INNER_SIZE_MM = OUTER_SIZE_MM - 2 * LANE_WIDTH_MM   # island side length (1000)

# The island occupies the centred square [ISLAND_MIN, ISLAND_MAX] on both axes.
ISLAND_MIN_MM = (OUTER_SIZE_MM - INNER_SIZE_MM) / 2.0   # 1000
ISLAND_MAX_MM = ISLAND_MIN_MM + INNER_SIZE_MM           # 2000

# Axis-aligned boxes (x0, y0, x1, y1), handy for ray-casters.
OUTER_BOX = (0.0, 0.0, OUTER_SIZE_MM, OUTER_SIZE_MM)
ISLAND_BOX = (ISLAND_MIN_MM, ISLAND_MIN_MM, ISLAND_MAX_MM, ISLAND_MAX_MM)
