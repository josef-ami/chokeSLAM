"""
Verification for display.py, the dashboard's fixed full-loop frame.

  - Lane 1 travels UP the screen, on the right-hand strip for CCW and the
    left-hand strip for CW; lanes 2-4 fill the top, the other side and the
    bottom strips in driving order (left turns for CCW, right turns for CW).
  - A pose re-expressed at a turn by lane_frame.corner_transform lands on the
    same screen point, with the same screen heading.
  - The mock's truth overlay (global mat frame) lands exactly where the
    tracker's lane coordinates are drawn, in every lane, for every start
    section.

Run:  python3 test_display.py
"""
from __future__ import annotations

import math
import random

import display as dp
import lane_frame as lf


STRIPS = {  # slot -> the screen strip it must fill (Xmin, Xmax, Ymin, Ymax)
    "CCW": {0: (2000, 3000, 0, 3000), 1: (0, 3000, 2000, 3000), 2: (0, 1000, 0, 3000), 3: (0, 3000, 0, 1000)},
    "CW": {0: (0, 1000, 0, 3000), 1: (0, 3000, 2000, 3000), 2: (2000, 3000, 0, 3000), 3: (0, 3000, 0, 1000)},
}


def test_lanes_fill_the_loop_in_driving_order():
    for d, strips in STRIPS.items():
        for slot, (x0, x1, y0, y1) in strips.items():
            for x in (0.0, 500.0, 1000.0):
                for y in (0.0, 1500.0, 3000.0):
                    X, Y = dp.lane_to_display(slot, x, y, d)
                    assert x0 - 1e-9 <= X <= x1 + 1e-9 and y0 - 1e-9 <= Y <= y1 + 1e-9, (d, slot, x, y, X, Y)
            # travel direction: +y in the lane is the lane's display bearing
            ax, ay = dp.lane_to_display(slot, 500.0, 1000.0, d)
            bx, by = dp.lane_to_display(slot, 500.0, 1001.0, d)
            b = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
            assert abs((b - dp.lane_north_display_bearing(slot, d) + 180) % 360 - 180) < 1e-6, (d, slot, b)
        # the outer wall of lane 1 is the right edge (CCW) / left edge (CW)
        X, _ = dp.lane_to_display(0, 0.0, 1500.0, d)
        assert X == (3000.0 if d == "CCW" else 0.0)
    print("PASS  test_lanes_fill_the_loop       lane 1 travels up the right strip (CCW) / left strip (CW); lanes 2-4 "
          "fill top, far side, bottom in driving order; each lane's +y is its drawn direction of travel")


def test_turns_are_seamless():
    rng = random.Random(4)
    worst_p = worst_h = 0.0
    for d in ("CCW", "CW"):
        for slot in range(8):                              # two laps: slot wraps 0..3
            for _ in range(200):
                x, y = rng.uniform(0, 1000), rng.uniform(2000, 3000)   # in the corner square ahead
                psi = rng.uniform(-80, 80)
                nx, ny, npsi = lf.corner_transform(x, y, psi, d)
                a = dp.lane_to_display(slot, x, y, d)
                b = dp.lane_to_display(slot + 1, nx, ny, d)
                worst_p = max(worst_p, math.hypot(a[0] - b[0], a[1] - b[1]))
                ha = dp.lane_north_display_bearing(slot, d) + psi
                hb = dp.lane_north_display_bearing(slot + 1, d) + npsi
                worst_h = max(worst_h, abs((ha - hb + 180) % 360 - 180))
    assert worst_p < 1e-9 and worst_h < 1e-9, (worst_p, worst_h)
    print(f"PASS  test_turns_are_seamless       3200 poses re-expressed at a turn: same screen point "
          f"(worst {worst_p:.1e} mm) and heading (worst {worst_h:.1e} deg)")


def test_truth_overlay_matches():
    rng = random.Random(5)
    worst = 0.0
    for d in ("CCW", "CW"):
        for sec0 in lf.SECTIONS:
            g2d = dp.GlobalToDisplay(sec0, d)
            sec = sec0
            for slot in range(4):
                for _ in range(50):
                    x, y = rng.uniform(0, 1000), rng.uniform(0, 3000)
                    a = dp.lane_to_display(slot, x, y, d)
                    b = g2d.point(*lf.lane_to_global(sec, d, x, y))
                    worst = max(worst, math.hypot(a[0] - b[0], a[1] - b[1]))
                psi = rng.uniform(-30, 30)
                hb = g2d.bearing(lf.yaw_to_heading(psi, sec, d))
                ha = (dp.lane_north_display_bearing(slot, d) + psi) % 360
                assert abs((ha - hb + 180) % 360 - 180) < 1e-9
                sec = lf.NEXT_SECTION[d][sec]
    assert worst < 1e-9, worst
    print(f"PASS  test_truth_overlay_matches    mock truth (global frame) lands where the tracker's lane coordinates "
          f"are drawn: 8 start cases x 4 lanes, worst {worst:.1e} mm; headings agree")


def test_robot_frame():
    X, Y = dp.robot_to_display(100.0, 200.0, 90.0, 50.0, 0.0)      # facing right, 50 ahead
    assert abs(X - 150.0) < 1e-9 and abs(Y - 200.0) < 1e-9
    X, Y = dp.robot_to_display(100.0, 200.0, 0.0, 0.0, 30.0)       # facing up, 30 to the right
    assert abs(X - 130.0) < 1e-9 and abs(Y - 200.0) < 1e-9
    print("PASS  test_robot_frame              robot-frame (forward, right) vectors go where the robot faces")


if __name__ == "__main__":
    test_lanes_fill_the_loop_in_driving_order()
    test_turns_are_seamless()
    test_truth_overlay_matches()
    test_robot_frame()
    print("\nAll display checks passed.")
