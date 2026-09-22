"""
Verification for lane_init.py: x, y and the seat verdicts produced by the
one-time initialisation, against ground truth from independently ray-cast
worlds (the caster and world builder are test_direction.py's).

Same test-world assumptions as test_direction.py (see its docstring).
Robot yaw is exactly 0 in these worlds unless a test says otherwise, because
initialisation takes yaw = 0 (agreed); test_placement_yaw_effect measures
what a small placement yaw does to x and y.

Run:  python3 test_init.py
"""
from __future__ import annotations

import math
import random
import statistics

import config
import lane_frame as lf
import lane_init as li
import scan_processing as sp
import seat_occupancy as so
from test_direction import (LANES, limitation_boxes, pillar_box, random_world, scan_at,
                            zone_poses)


def to_points(pairs):
    """(angle, range) pairs -> ScanPoints through the real input path
    (clean_and_project with sign +1 / offset 0: the caster is already clockwise)."""
    return sp.clean_and_project([(a, d, 47) for a, d in pairs], 1, 0.0)


def _run(poses, parking=False, yaw_range=0.0, seed=1, reps=1):
    rng = random.Random(seed)
    rows = []
    for _ in range(reps):
        for lane, direction, x, y in poses:
            lot = rng.uniform(1300.0, 1700.0) if parking else None
            if parking:
                x, y = 100.0, lot
            boxes, truth = random_world(lane, direction, x, y, rng, parking, lot)
            yaw = rng.uniform(-yaw_range, yaw_range)
            res = li.initialise(to_points(scan_at(lane, direction, x, y, boxes, yaw, rng=rng)))
            rows.append((lane, direction, x, y, truth, res))
    return rows


def _xy_report(label, rows, x_tol=30.0, y_tol=50.0):
    n = len(rows)
    init_ok = [r for r in rows if r[5].ok]
    dir_wrong = sum(1 for r in rows if r[5].direction.direction not in (None, r[1]))
    ex = [r[5].x.x_mm - r[2] for r in init_ok]
    ey = [r[5].y.y_mm - r[3] for r in init_ok]
    bad_x = sum(1 for e in ex if abs(e) > x_tol)
    bad_y = sum(1 for e in ey if abs(e) > y_tol)
    x_rej = sum(1 for r in rows if r[5].x is not None and not r[5].x.ok)
    print(f"      {label:30s} n={n:4d} init ok {len(init_ok):4d} ({100*len(init_ok)/n:5.1f}%)  "
          f"dir wrong {dir_wrong}  x rejected {x_rej:3d}  |x err| med {statistics.median(map(abs, ex)):4.1f} "
          f"max {max(map(abs, ex)):5.1f}  |y err| med {statistics.median(map(abs, ey)):4.1f} "
          f"max {max(map(abs, ey)):6.1f}  (>{x_tol:.0f}/{y_tol:.0f} mm: {bad_x}/{bad_y})")
    return dir_wrong, bad_x, bad_y, init_ok, ey


def test_x_y_accuracy():
    print("PASS  test_x_y_accuracy             (accepted x/y must be right; rejection is allowed)")
    tot = [0, 0, 0]
    outliers = []
    for label, poses, park, seed in (
            ("start zones", list(zone_poses()), False, 21),
            ("parking lot", [(ln, d, 100.0, 0.0) for ln in LANES for d in lf.DIRECTIONS for _ in range(20)], True, 22)):
        dw, bx, by, ok_rows, ey = _xy_report(label, _run(poses, park, seed=seed, reps=2 if not park else 1))
        tot[0] += dw
        tot[1] += bx
        tot[2] += by
        outliers += [(r[0], r[1], r[2], r[3], sorted(r[4]), round(r[5].y.y_mm - r[3]))
                     for r in ok_rows if abs(r[5].y.y_mm - r[3]) > 50.0]
    for o in outliers[:5]:
        print(f"        y outlier: lane {o[0]} {o[1]} x={o[2]:.0f} y={o[3]:.0f} seats {o[4]} err {o[5]} mm")
    assert tot[0] == 0 and tot[1] == 0, tot
    return tot[2]


def test_lever_arm():
    """With the LIDAR mounted off the reference point, x and y must describe
    the REFERENCE point. The sensor pose is derived in global coordinates
    here, independently of lane_init's formula, for both directions."""
    saved = (config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM)
    try:
        config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM = 110.0, 60.0
        worst = 0.0
        for lane in LANES:
            for direction in lf.DIRECTIONS:
                x, y = 450.0, 1400.0
                heading = lf.grid_north_bearing(lane, direction)
                gx, gy = lf.lane_to_global(lane, direction, x, y)
                hf, hl = math.radians(heading), math.radians(heading - 90.0)
                sgx = gx + 110.0 * math.sin(hf) + 60.0 * math.sin(hl)
                sgy = gy + 110.0 * math.cos(hf) + 60.0 * math.cos(hl)
                from test_direction import cast
                res = li.initialise(to_points(cast(sgx, sgy, heading, [], noise_mm=0.0, dropout=0.0)))
                assert res.ok, res.reason
                worst = max(worst, abs(res.x.x_mm - x), abs(res.y.y_mm - y))
        assert worst < 3.0, worst
        print(f"PASS  test_lever_arm                (sensor 110 mm ahead / 60 mm left: reference x,y within {worst:.1f} mm)")
    finally:
        config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM = saved


def test_pillar_abeam_rejects_x():
    """A pillar between the robot and a side wall makes d(90)+d(270) short:
    initialisation must FAIL (the direction test's precondition catches it
    first), never report an x."""
    lane, direction = "E", "CCW"
    # robot at y = 1500 beside the mid-outer seat (x = 400): pillar between it and the outer wall
    x, y = 700.0, 1500.0
    seat = [s for s in so.seats() if s.name == "mid-outer"][0]
    boxes = [pillar_box(*lf.lane_to_global(lane, direction, seat.x_mm, seat.y_mm))]
    res = li.initialise(to_points(scan_at(lane, direction, x, y, boxes, noise_mm=0.0, dropout=0.0)))
    assert not res.ok and (res.x is None or not res.x.ok), res.reason
    assert "d(90)+d(270)" in res.reason, res.reason
    print(f"PASS  test_pillar_abeam_rejects     ({res.reason[:78]}...)")


def test_pillar_ahead_does_not_fool_y():
    """P2: a pillar standing on the far seat straight ahead. The single d(0)
    ray reads the pillar; the front-wall fan must still read the wall."""
    lane, direction = "S", "CW"
    x, y = 400.0, 1300.0
    seat = [s for s in so.seats() if s.name == "far-outer"][0]      # x = 400, y = 2000: dead ahead
    boxes = [pillar_box(*lf.lane_to_global(lane, direction, seat.x_mm, seat.y_mm))]
    pairs = scan_at(lane, direction, x, y, boxes, noise_mm=0.0, dropout=0.0)
    d0 = min(d for a, d in pairs if abs((a + 180) % 360 - 180) <= 0.5)
    res = li.initialise(to_points(pairs))
    assert res.ok, res.reason
    assert abs(res.y.y_mm - y) < 5.0, res.y
    print(f"PASS  test_pillar_ahead_not_y       (d(0) alone: y = {3000 - d0:.0f}; fan: y = {res.y.y_mm:.0f}; truth {y:.0f})")


def test_parking_limitation_does_not_fool_y():
    lane, direction = "N", "CCW"
    lot = 1500.0
    boxes = limitation_boxes(lane, direction, lot)
    pairs = scan_at(lane, direction, 100.0, lot, boxes, noise_mm=0.0, dropout=0.0)
    d0 = min(d for a, d in pairs if abs((a + 180) % 360 - 180) <= 0.5)
    res = li.initialise(to_points(pairs))
    assert res.ok, res.reason
    assert abs(res.y.y_mm - lot) < 5.0 and abs(res.x.x_mm - 100.0) < 5.0, (res.x, res.y)
    print(f"PASS  test_parking_limitation_y     (d(0) alone: y = {3000 - d0:.0f}; fan: y = {res.y.y_mm:.0f}; truth {lot:.0f})")


def test_init_seats_never_wrong():
    """End to end: every seat verdict from a successful initialisation is
    checked against the world. OCCUPIED on an empty seat or EMPTY on an
    occupied one is a failure; UNKNOWN is allowed (and counted)."""
    rows = _run(list(zone_poses()), seed=31, reps=2)
    rows += _run([(ln, d, 100.0, 0.0) for ln in LANES for d in lf.DIRECTIONS for _ in range(20)],
                 parking=True, seed=32)
    fp = fn = tp = tn = unk_occ = unk_emp = 0
    for lane, direction, x, y, truth, res in rows:
        if not res.ok:
            continue
        for r in res.seats:
            occ = r.seat.index in truth
            if r.state is so.Occupancy.UNKNOWN:
                unk_occ += occ
                unk_emp += (not occ)
            elif r.state is so.Occupancy.OCCUPIED:
                tp, fp = (tp + 1, fp) if occ else (tp, fp + 1)
            else:
                tn, fn = (tn + 1, fn) if not occ else (tn, fn + 1)
    dec = tp + tn + fp + fn
    tot = dec + unk_occ + unk_emp
    print(f"PASS  test_init_seats_never_wrong   {tot} seat verdicts: present {tp}, absent {tn}, "
          f"unknown {unk_occ + unk_emp} ({100*(unk_occ+unk_emp)/tot:.1f}%; {unk_occ} of them occupied); "
          f"WRONG: false present {fp}, false absent {fn}")
    assert fp == 0 and fn == 0


def test_placement_yaw_effect():
    """Not a pass/fail on accuracy: initialisation takes yaw = 0 by agreement.
    This reports what a real placement yaw does to x, y and the seats, so the
    number is on record."""
    for yr in (1.0, 2.0, 3.0):
        rows = _run(list(zone_poses()), yaw_range=yr, seed=41)
        ok = [r for r in rows if r[5].ok]
        ex = [abs(r[5].x.x_mm - r[2]) for r in ok]
        ey = [abs(r[5].y.y_mm - r[3]) for r in ok]
        fp = fn = 0
        for lane, direction, x, y, truth, res in ok:
            for r in res.seats:
                occ = r.seat.index in truth
                fp += (r.state is so.Occupancy.OCCUPIED and not occ)
                fn += (r.state is so.Occupancy.EMPTY and occ)
        print(f"      placement yaw +/-{yr:g}: init ok {len(ok)}/{len(rows)}, |x err| max {max(ex):5.1f} mm, "
              f"|y err| max {max(ey):6.1f} mm, seat false present {fp}, false absent {fn}")


if __name__ == "__main__":
    test_x_y_accuracy()
    test_lever_arm()
    test_pillar_abeam_rejects_x()
    test_pillar_ahead_does_not_fool_y()
    test_parking_limitation_does_not_fool_y()
    test_init_seats_never_wrong()
    print("INFO  test_placement_yaw_effect     (yaw is taken as 0 at init by agreement; effect on record)")
    test_placement_yaw_effect()
    print("\nAll init checks passed.")
