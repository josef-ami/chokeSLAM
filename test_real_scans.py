"""
Regression tests on REAL scans from the robot (test_data/*.json, saved with
`run_init.py --real --dump`). Each file is replayed through the same path the
robot uses: clean_and_project with the CURRENT config sign/offset (the raw
angles in the files are the sensor's own).

real_2026-09-23_sign_check.json
    Robot on the field FACING THE INNER (ISLAND) WALL, ~400 mm from it (owner;
    the scan agrees: a finite wall ~400 mm ahead with rays passing both of its
    ends out to 1.2-2 m). One object (~45 mm wide) placed ~30 cm to the
    robot's RIGHT, looking the way it faces. It reads raw 272 deg, so the
    upside-down LIDAR's raw angles run counter-clockwise: LIDAR_ANGLE_SIGN = -1.
    This test pins that (the object must read ~90 = right), and that
    initialisation REFUSES this scan: facing a wall is not a lane start pose.

real_2026-09-23_pillar_ahead_and_beside.json
    Practice field, lane measured 926-934 mm (rulebook 1000; passes the
    +/-100 mm margin), robot ~3.5 deg off parallel, a pillar ~143 mm dead ahead
    and a pillar ~165 mm beside the LIDAR (on the robot's right, owner). The
    first run failed: the 2-deg ray read the side pillar. The dominant-wall
    measurement reads the walls behind it.
    Direction: the island opening is on the SAME side as the side pillar, i.e.
    the robot's RIGHT under the measured sign -> CW. CONFIRMED by the owner
    (the island was on the robot's right; the earlier "CCW" was a mislabel),
    so this is a ground-truth test: CW, x ~478 mm, y ~1464 mm.

Run:  python3 test_real_scans.py
"""
from __future__ import annotations

import json
import os

import config
config.LANE_WIDTH_MM = 1000.0         # rulebook value (agreed); the ~930 mm field passes on the margin

import direction_detect as dd
import lane_init as li
from scan_processing import clean_and_project

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, sign=None):
    """Replay with `sign` (default: the CURRENT config sign -- the raw angles in
    the file are the sensor's own, whatever sign was configured when saved)."""
    with open(os.path.join(HERE, "test_data", name)) as fh:
        d = json.load(fh)
    s = config.LIDAR_ANGLE_SIGN if sign is None else sign
    return clean_and_project([tuple(p) for p in d["raw"]], s, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)


def test_2026_09_23_sign_check():
    pts = [p for p in _load("real_2026-09-23_sign_check.json") if p.dist_mm >= 60.0]
    near = min(pts, key=lambda p: p.dist_mm)
    obj = [p for p in pts if abs(p.dist_mm - near.dist_mm) < 20
           and abs((p.angle_deg - near.angle_deg + 180) % 360 - 180) < 10]
    ang = sum(p.angle_deg for p in obj) / len(obj)
    assert 250 < near.dist_mm < 350, near.dist_mm
    assert abs(ang - 90.0) < 10.0, (ang, "the object placed on the robot's RIGHT must read ~90")
    # the robot was facing the inner wall, not along a lane: initialisation must refuse
    res = li.initialise(_load("real_2026-09-23_sign_check.json"))
    assert not res.ok and res.direction.direction is None, res.reason
    print(f"PASS  test_2026_09_23_sign_check     object on the robot's right reads {ang:.1f} deg at "
          f"{near.dist_mm:.0f} mm with LIDAR_ANGLE_SIGN = {config.LIDAR_ANGLE_SIGN:+d}; "
          f"facing the inner wall -> init refuses ({res.reason[:60]}...)")


def test_2026_09_23_pillar_beside():
    name = "real_2026-09-23_pillar_ahead_and_beside.json"
    results = {}
    for sign in (+1, -1):
        pts = _load(name, sign)
        # the old 2-deg median reads the side pillar (raw 270 -> corrected 270 with +1, 90 with -1)
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
    # with the MEASURED sign: the opening (and the side pillar) are on the robot's right -> CW (owner-confirmed)
    r = results[config.LIDAR_ANGLE_SIGN]
    assert config.LIDAR_ANGLE_SIGN == -1 and r.direction.direction == "CW", r.direction.reason
    assert abs(r.x.x_mm - 478) < 15, r.x.x_mm            # CW: x = d(270), the outer wall on the left
    print(f"PASS  test_2026_09_23_pillar_beside walls {r.direction.left.fit.d_ray_mm:.0f} / "
          f"{r.direction.right.fit.d_ray_mm:.0f} mm (2-deg ray read the pillar at ~166); "
          f"lane {r.x.lane_sum_mm:.0f} mm (rulebook 1000 +/- {config.LANE_WIDTH_TOLERANCE_MM:.0f}); "
          f"opening {max(r.direction.left.opening_mm, r.direction.right.opening_mm):.0f} mm; "
          f"y {r.y.y_mm:.0f}; measured sign {config.LIDAR_ANGLE_SIGN:+d} -> "
          f"{r.direction.direction} (confirmed), x {r.x.x_mm:.0f} mm")


if __name__ == "__main__":
    test_2026_09_23_sign_check()
    test_2026_09_23_pillar_beside()
    print("\nAll real-scan checks passed.")
