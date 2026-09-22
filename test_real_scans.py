"""
Regression tests on REAL scans from the robot (test_data/*.json, saved with
`run_init.py --real --dump`). Each file is replayed through the same path the
robot uses (clean_and_project with the scan's recorded sign/offset).

real_2026-09-23_pillar_ahead_and_beside.json
    Practice field, lane measured ~926-934 mm wide (not the rulebook 1000),
    robot ~3.5 deg off parallel, a pillar ~143 mm dead ahead and a pillar
    ~165 mm beside the LIDAR. First run failed: the 2-deg ray read the side
    pillar (166 mm) and d(90)+d(270) = 637. The dominant-wall measurement
    (approved Sept 23) must read the walls behind it.

    OPEN: which physical side raw 270 is (LIDAR_ANGLE_SIGN) is not settled --
    the owner reports the robot faced CCW (=> island on the left => raw 270 is
    left => sign +1) AND that the side pillar was on the right (=> sign -1);
    the scan puts the pillar and the island opening on the SAME side, so both
    can't hold. Until a sign-check scan settles it, this test asserts only what
    does not depend on the sign, plus the mirror consistency of the answer.

Run:  python3 test_real_scans.py
"""
from __future__ import annotations

import json
import os

import config
config.LANE_WIDTH_MM = 930.0          # this practice field (tape: ~930, owner, Sept 23)

import direction_detect as dd
import lane_init as li
from scan_processing import clean_and_project

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, sign=None):
    with open(os.path.join(HERE, "test_data", name)) as fh:
        d = json.load(fh)
    s = d["angle_sign"] if sign is None else sign
    return clean_and_project([tuple(p) for p in d["raw"]], s, d["angle_zero_offset_deg"])


def test_2026_09_23_pillar_beside():
    name = "real_2026-09-23_pillar_ahead_and_beside.json"
    results = {}
    for sign in (+1, -1):
        pts = _load(name, sign)
        # the old 2-deg median reads the side pillar on the raw-270 side
        raw270 = 270.0 if sign == 1 else 90.0
        assert dd.side_ray_distance(pts, raw270) < 200.0
        res = li.initialise(pts)
        assert res.ok, res.reason
        d = res.direction
        walls = sorted((d.left.fit.d_ray_mm, d.right.fit.d_ray_mm))
        assert 440 < walls[0] < 470 and 460 < walls[1] < 490, walls     # 453 / 474 behind the pillar
        assert 910 < res.x.lane_sum_mm < 950, res.x.lane_sum_mm
        # the opening is on the raw-270 side (same side as the side pillar), none on the other
        pillar_side = d.left if sign == 1 else d.right
        other = d.right if sign == 1 else d.left
        assert pillar_side.opening_mm >= 700 and other.opening_mm <= 150, (pillar_side.opening_mm, other.opening_mm)
        assert abs(res.y.y_mm - 1464) < 20, res.y.y_mm                  # fan reads the wall ahead, not the pillar
        results[sign] = res
    # mirror consistency: flipping the sign flips the answer, and nothing else
    assert {results[1].direction.direction, results[-1].direction.direction} == {"CCW", "CW"}
    assert abs(results[1].x.lane_sum_mm - results[-1].x.lane_sum_mm) < 1e-6
    r = results[config.LIDAR_ANGLE_SIGN]
    print(f"PASS  test_2026_09_23_pillar_beside walls {r.direction.left.fit.d_ray_mm:.0f} / "
          f"{r.direction.right.fit.d_ray_mm:.0f} mm (2-deg ray read the pillar at ~166); "
          f"opening {max(r.direction.left.opening_mm, r.direction.right.opening_mm):.0f} mm; "
          f"y {r.y.y_mm:.0f}; with the current sign ({config.LIDAR_ANGLE_SIGN:+d}): "
          f"{r.direction.direction}, x {r.x.x_mm:.0f} mm")


if __name__ == "__main__":
    test_2026_09_23_pillar_beside()
    print("\nAll real-scan checks passed (angle sign still OPEN -- see module docstring).")
