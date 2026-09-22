"""
Verification for lane_frame.py against GLOBAL geometry (grid bearings in the
mat frame), for all 4 lanes x both round directions.

Run:  python3 test_lane_frame.py
"""
from __future__ import annotations

# The simulated worlds in this file are RULEBOOK fields: pin the lane width
# before anything imports mat_geometry, whatever config.py is set to.
import config
config.LANE_WIDTH_MM = 1000.0

import math
import random

import lane_frame as lf

rng = random.Random(3)


def _gbearing(dx, dy):
    """global grid bearing (clockwise from +Y) of a global vector"""
    return math.degrees(math.atan2(dx, dy)) % 360.0


def _adiff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def test_handedness_and_outer_side():
    """Stand in each lane facing grid north: the OUTER wall (x = 0) must be on
    the robot's right for CCW and on its left for CW."""
    for lane in lf.SECTIONS:
        for d in lf.DIRECTIONS:
            north = lf.grid_north_bearing(lane, d)
            p = lf.lane_to_global(lane, d, 500.0, 1500.0)
            q = lf.lane_to_global(lane, d, 0.0, 1500.0)        # the outer wall, abeam
            rel = (_gbearing(q[0] - p[0], q[1] - p[1]) - north) % 360.0
            want = 90.0 if d == "CCW" else 270.0
            assert _adiff(rel, want) < 1e-9, (lane, d, rel)
            assert lf.outer_wall_lidar_angle(d) == want
    print("PASS  test_handedness_and_outer_side (outer wall at 90 for CCW, 270 for CW, all lanes)")


def test_bearing_roundtrip():
    worst = 0.0
    for d in lf.DIRECTIONS:
        for _ in range(500):
            b = rng.uniform(0, 360)
            ux, uy = lf.unit_of_bearing(b, d)
            worst = max(worst, _adiff(lf.bearing_of(ux, uy, d), b))
    assert worst < 1e-9, worst
    print(f"PASS  test_bearing_roundtrip        (bearing_of(unit_of_bearing(b)) == b, worst {worst:.1e})")


def test_offset_in_lane_vs_global():
    worst = 0.0
    for lane in lf.SECTIONS:
        for d in lf.DIRECTIONS:
            for _ in range(50):
                x, y, yaw = rng.uniform(100, 900), rng.uniform(200, 2800), rng.uniform(-30, 30)
                fwd, left = rng.uniform(-150, 150), rng.uniform(-100, 100)
                ox, oy = lf.offset_in_lane(x, y, yaw, d, fwd, left)
                # global: heading H = north + yaw; unit of bearing b is (sin b, cos b)
                H = lf.grid_north_bearing(lane, d) + yaw
                gx, gy = lf.lane_to_global(lane, d, x, y)
                ex = gx + fwd * math.sin(math.radians(H)) + left * math.sin(math.radians(H - 90))
                ey = gy + fwd * math.cos(math.radians(H)) + left * math.cos(math.radians(H - 90))
                got = lf.lane_to_global(lane, d, ox, oy)
                worst = max(worst, math.hypot(got[0] - ex, got[1] - ey))
    assert worst < 1e-6, worst
    print(f"PASS  test_offset_in_lane_vs_global (lever arm matches global geometry, worst {worst:.1e} mm)")


def test_corner_transform_vs_global():
    """Any pose inside the corner square shared by lane L and the next lane
    (in the round direction) must map to the same global point and heading
    whether it is expressed in L's frame or, after corner_transform, in the
    next lane's frame."""
    worst_p = worst_h = 0.0
    for lane in lf.SECTIONS:
        for d in lf.DIRECTIONS:
            nxt = lf.NEXT_SECTION[d][lane]
            for _ in range(100):
                x, y = rng.uniform(0, 1000), rng.uniform(2000, 3000)   # exit corner square of `lane`
                yaw = rng.uniform(-100, 100)
                nx, ny, nyaw = lf.corner_transform(x, y, yaw, d)
                g_old = lf.lane_to_global(lane, d, x, y)
                g_new = lf.lane_to_global(nxt, d, nx, ny)
                worst_p = max(worst_p, math.hypot(g_old[0] - g_new[0], g_old[1] - g_new[1]))
                h_old = lf.yaw_to_heading(yaw, lane, d)
                h_new = lf.yaw_to_heading(nyaw, nxt, d)
                worst_h = max(worst_h, _adiff(h_old, h_new))
                # and the point really is in the NEW lane's ENTRY corner square
                assert 0.0 <= nx <= 1000.0 and 0.0 <= ny <= 1000.0
    assert worst_p < 1e-6 and worst_h < 1e-9, (worst_p, worst_h)
    print(f"PASS  test_corner_transform         (same global point/heading in both lanes; "
          f"worst {worst_p:.1e} mm, {worst_h:.1e} deg; lands in the new lane's entry corner)")


if __name__ == "__main__":
    test_handedness_and_outer_side()
    test_bearing_roundtrip()
    test_offset_in_lane_vs_global()
    test_corner_transform_vs_global()
    print("\nAll lane-frame checks passed.")
