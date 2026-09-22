"""
Verification for seat_occupancy.py.

Everything here ray-casts the REAL global field (3000x3000 outer square +
1000x1000 island + 50 mm pillars) and hands the detector only lane-local
numbers, so the lane<->global transform is exercised honestly rather than
assumed. Run with:  python3 test_seat_occupancy.py
"""
from __future__ import annotations

import math
import random
import statistics
from itertools import combinations

import seat_occupancy as so
from lane_frame import (LANE_WIDTH_MM as LANE_W, OUTER_SIZE_MM as OUTER,
                        SECTIONS as LANES, global_to_lane, lane_to_global)
from lane_frame import grid_north_bearing as lane_grid_north_bearing

random.seed(20260922)

ISLAND_MIN, ISLAND_MAX = 1000.0, 2000.0


# ---------------------------------------------------------------- ray caster
def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin, tmax


def cast_global(x, y, heading_bearing_deg, pillars=(), n_points=720,
                noise_mm=0.0, dropout=0.0, blind_center=180.0, blind_width=105.0):
    """Returns (robot_rel_angle_deg, range_mm) with 0 = forward, 90 = left."""
    outer = (0.0, 0.0, OUTER, OUTER)
    island = (ISLAND_MIN, ISLAND_MIN, ISLAND_MAX, ISLAND_MAX)
    boxes = [island] + [(px - 25, py - 25, px + 25, py + 25) for px, py in pillars]
    pts = []
    for i in range(n_points):
        rel = i * 360.0 / n_points
        if blind_width and abs((rel - blind_center + 180.0) % 360.0 - 180.0) <= blind_width / 2.0:
            continue
        world = math.radians((90.0 - heading_bearing_deg) + rel)
        dx, dy = math.cos(world), math.sin(world)
        best = None
        hit = _ray_box(x, y, dx, dy, *outer)
        if hit and hit[1] > 0:
            best = hit[1]
        for b in boxes:
            hit = _ray_box(x, y, dx, dy, *b)
            if hit and hit[0] > 1e-6 and (best is None or hit[0] < best):
                best = hit[0]
        if best is None:
            continue
        if dropout and random.random() < dropout:
            continue
        pts.append((rel, max(1.0, best + (random.gauss(0, noise_mm) if noise_mm else 0.0))))
    return pts


def scene(lane, direction, x_local, y_local, occupied_indices, yaw_deg=0.0,
          noise_mm=4.0, dropout=0.02):
    """Build a scan for a robot at a lane-local pose with the given seats filled."""
    rx, ry = lane_to_global(lane, direction, x_local, y_local)
    north = lane_grid_north_bearing(lane, direction)
    pillars = [lane_to_global(lane, direction, s.x_mm, s.y_mm)
               for s in so.seats() if s.index in occupied_indices]
    return cast_global(rx, ry, (north + yaw_deg) % 360.0, pillars,
                       noise_mm=noise_mm, dropout=dropout)


# --------------------------------------------------------------------- tests
def test_frame_maps_to_global():
    """The 6 lane-local seats, mapped out over 4 lanes x 2 directions, must
    reproduce exactly the 24 global seat positions read off Figure 11."""
    expected = set()
    for a in (1000, 1500, 2000):
        expected |= {(a, 400), (a, 600), (a, 2400), (a, 2600),
                     (400, a), (600, a), (2400, a), (2600, a)}
    for direction in ("CCW", "CW"):
        got = set()
        for lane in LANES:
            for s in so.seats():
                gx, gy = lane_to_global(lane, direction, s.x_mm, s.y_mm)
                got.add((round(gx), round(gy)))
                # and the inverse the dashboard's lane view depends on
                bx, by = global_to_lane(lane, direction, gx, gy)
                assert abs(bx - s.x_mm) < 1e-9 and abs(by - s.y_mm) < 1e-9, \
                    f"{lane}/{direction} {s.name}: round trip gave ({bx}, {by})"
        assert got == expected, f"{direction}: {sorted(got ^ expected)}"
    print("PASS  test_frame_maps_to_global      "
          "(24 global seats reproduced + round-trip, both directions)")


def test_bearing_conventions():
    assert abs(so.bearing_to(0, 100) - 0.0) < 1e-9        # ahead
    assert abs(so.bearing_to(100, 0) - 90.0) < 1e-9       # right
    assert abs(so.bearing_to(0, -100) - 180.0) < 1e-9     # behind
    assert abs(so.bearing_to(-100, 0) - 270.0) < 1e-9     # left
    # bearing -> lidar angle is a mirror (the sketch's 2*pi - theta)
    assert abs(so.bearing_to_lidar_angle(90.0) - 270.0) < 1e-9   # right -> 270
    assert abs(so.bearing_to_lidar_angle(270.0) - 90.0) < 1e-9   # left  -> 90
    assert abs(so.bearing_to_lidar_angle(0.0) - 0.0) < 1e-9
    print("PASS  test_bearing_conventions")


def test_no_false_positives_on_empty_field():
    """Finding 6 comparison: the cluster classifier calls 'pillar' on 45% of
    empty-field scans. This detector must call OCCUPIED on none of them."""
    bad = 0
    total = 0
    for lane in LANES:
        for direction in ("CCW", "CW"):
            for y in range(600, 2400, 100):
                for x in (300.0, 500.0, 700.0):
                    scan = scene(lane, direction, x, float(y), set())
                    for r in so.detect_seat_occupancy(scan, x, float(y)):
                        total += 1
                        if r.state is so.Occupancy.OCCUPIED:
                            bad += 1
    print(f"PASS  test_no_false_positives       "
          f"{bad} false OCCUPIED out of {total} seat decisions on an empty field")
    assert bad == 0


def test_occlusion_reports_unknown():
    """A seat hidden behind a nearer pillar must come back UNKNOWN, never
    EMPTY -- the near-left and far-left seats line up from a robot sitting on
    the left-hand side of the lane."""
    # seats 0 (near-left, y=1000) and 4 (far-left, y=2000) share x=400.
    scan = scene("S", "CCW", 400.0, 500.0, {0})   # only the NEAR one is filled
    rs = so.detect_seat_occupancy(scan, 400.0, 500.0)
    near, far = rs[0], rs[4]
    assert near.state is so.Occupancy.OCCUPIED, near
    assert far.state is so.Occupancy.UNKNOWN, far
    print(f"PASS  test_occlusion_reports_unknown "
          f"(near={near.state.value}, far={far.state.value}: '{far.reason}')")


def test_yaw_error_is_handled():
    """With the yaw argument supplied the answer must survive a few degrees of
    heading error; without it, it must not. This is the argument for making
    robot_yaw_deg mandatory in practice."""
    occ = {1, 2, 5}
    x, y = 500.0, 700.0
    for yaw in (-6.0, -3.0, 0.0, 3.0, 6.0):
        scan = scene("E", "CCW", x, y, occ, yaw_deg=yaw)
        with_yaw = so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=yaw)
        without = so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=0.0)
        ok_w = sum(1 for r in with_yaw
                   if (r.state is so.Occupancy.OCCUPIED) == (r.seat.index in occ))
        ok_n = sum(1 for r in without
                   if (r.state is so.Occupancy.OCCUPIED) == (r.seat.index in occ))
        print(f"      yaw {yaw:+5.1f} deg -> correct 6/6? with_yaw={ok_w}/6  ignoring_yaw={ok_n}/6")
        assert ok_w == 6, [(r.seat.name, r.state.value, r.reason) for r in with_yaw]
    print("PASS  test_yaw_error_is_handled")


def test_accuracy_sweep():
    """Confusion matrix over lanes, directions, poses, layouts and noise."""
    tp = fp = tn = fn = unk_true = unk_false = 0
    residuals = []
    layouts = [set()] + [set(c) for k in (1, 2, 3) for c in combinations(range(6), k)][:24]
    for lane in LANES:
        for direction in ("CCW", "CW"):
            for y in (500.0, 800.0, 1100.0, 1400.0, 1700.0):
                for x in (400.0, 500.0, 600.0):
                    for yaw in (-2.0, 0.0, 2.0):
                        occ = random.choice(layouts)
                        scan = scene(lane, direction, x, y, occ, yaw_deg=yaw)
                        for r in so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=yaw):
                            truth = r.seat.index in occ
                            if r.state is so.Occupancy.UNKNOWN:
                                unk_true += truth
                                unk_false += (not truth)
                            elif r.state is so.Occupancy.OCCUPIED:
                                if truth:
                                    tp += 1
                                    if r.residual_mm is not None:
                                        residuals.append(r.residual_mm)
                                else:
                                    fp += 1
                            else:
                                tn += (not truth)
                                fn += truth
    decided = tp + fp + tn + fn
    total = decided + unk_true + unk_false
    print("PASS  test_accuracy_sweep")
    print(f"      {total} seat decisions over 4 lanes x 2 directions x 45 poses")
    print(f"      decided {decided} ({100*decided/total:.1f}%), "
          f"unknown {unk_true+unk_false} ({100*(unk_true+unk_false)/total:.1f}%)")
    print(f"      of the decided:  TP {tp}  TN {tn}  FP {fp}  FN {fn}"
          f"   -> accuracy {100*(tp+tn)/decided:.2f}%")
    if residuals:
        print(f"      range residual (observed - expected face): "
              f"median {statistics.median(residuals):+.1f} mm, "
              f"max |{max(abs(r) for r in residuals):.1f}| mm")
    print(f"      unknowns: {unk_true} were actually occupied, {unk_false} actually empty")
    assert fp == 0, f"{fp} false positives"
    assert fn == 0, f"{fn} false negatives"


def test_drive_through_resolves_every_seat_in_time():
    """The operational question: driving up the lane and merging each scan's
    verdicts, is every seat resolved -- correctly -- BEFORE the robot reaches
    it? A seat that only resolves once you are level with it is useless for
    planning. Merge policy: the first non-UNKNOWN verdict sticks; later scans
    must not contradict it."""
    worst_lead = None
    contradictions = 0
    trials = 0
    for lane in LANES:
        for direction in ("CCW", "CW"):
            for occ in ({0, 3}, {1, 2, 5}, {4}, set(), {0, 1, 2, 3, 4, 5}):
                trials += 1
                settled: dict[int, so.Occupancy] = {}
                settled_at: dict[int, float] = {}
                for y in [float(v) for v in range(100, 2050, 50)]:
                    x = 500.0 + 60.0 * math.sin(y / 400.0)   # a wandering line
                    yaw = 2.0 * math.cos(y / 350.0)
                    scan = scene(lane, direction, x, y, occ, yaw_deg=yaw)
                    for r in so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=yaw):
                        if r.state is so.Occupancy.UNKNOWN:
                            continue
                        if r.seat.index not in settled:
                            settled[r.seat.index] = r.state
                            settled_at[r.seat.index] = y
                        elif settled[r.seat.index] is not r.state:
                            contradictions += 1
                for s in so.seats():
                    truth = (so.Occupancy.OCCUPIED if s.index in occ
                             else so.Occupancy.EMPTY)
                    assert s.index in settled, (
                        f"{lane}/{direction} {occ}: seat {s.name} never resolved")
                    assert settled[s.index] is truth, (
                        f"{lane}/{direction} {occ}: seat {s.name} resolved "
                        f"{settled[s.index].value}, truth {truth.value}")
                    lead = s.y_mm - settled_at[s.index]
                    worst_lead = lead if worst_lead is None else min(worst_lead, lead)
    print("PASS  test_drive_through_resolves_every_seat_in_time")
    print(f"      {trials} runs x 6 seats: all resolved correctly, "
          f"{contradictions} contradictions between scans")
    print(f"      smallest lead distance (resolved this far before reaching "
          f"the seat): {worst_lead:.0f} mm")


def test_coarse_sampling_never_reports_false_empty():
    """Regression. At 1 deg angular sampling a 50 mm pillar at ~1.7 m subtends
    1.7 deg, so it returns one or two points and a dropout can leave one --
    too few to certify as a pillar-shaped run. The detector must then say
    UNKNOWN, never EMPTY: a return sitting AT the seat's range is ambiguous,
    not evidence of absence.

    This is what the dashboard's lane view caught by comparing live verdicts
    against the mock's ground truth (simulation.simulate_scan sweeps 360
    points, i.e. 1 deg, where the rest of this file uses 720)."""
    false_empty = 0
    checked = 0
    for lane in LANES:
        for direction in ("CCW", "CW"):
            for occ in ({1, 4}, {0, 2, 5}, {3}):
                for y in (0.0, 150.0, 300.0, 450.0, 600.0):
                    x = 400.0
                    rx, ry = lane_to_global(lane, direction, x, y)
                    north = lane_grid_north_bearing(lane, direction)
                    pil = [lane_to_global(lane, direction, s.x_mm, s.y_mm)
                           for s in so.seats() if s.index in occ]
                    scan = cast_global(rx, ry, north, pil, n_points=360,
                                       noise_mm=4.0, dropout=0.02, blind_width=0.0)
                    p = so.DetectParams(blind_arc_width_deg=0.0)
                    for r in so.detect_seat_occupancy(scan, x, y, params=p):
                        checked += 1
                        if r.seat.index in occ and r.state is so.Occupancy.EMPTY:
                            false_empty += 1
    print(f"PASS  test_coarse_sampling          {false_empty} false EMPTY on an occupied "
          f"seat out of {checked} decisions at 1 deg sampling")
    assert false_empty == 0


def test_lidar_lever_arm():
    """With the sensor mounted well off the reference point, the answers must
    only stay right if the offset is declared. This is the check that the
    lever-arm maths has the correct sign."""
    fwd, lat = 120.0, 80.0          # sensor 120 mm ahead, 80 mm to the left
    occ = {0, 3, 5}
    lane, direction, x, y, yaw = "N", "CW", 520.0, 700.0, 3.0

    # Place the SENSOR where the scan is taken from, derived independently.
    yr = math.radians(yaw)
    sx = x + fwd * math.sin(yr) - lat * math.cos(yr)
    sy = y + fwd * math.cos(yr) + lat * math.sin(yr)
    gx, gy = lane_to_global(lane, direction, sx, sy)
    north = lane_grid_north_bearing(lane, direction)
    pillars = [lane_to_global(lane, direction, s.x_mm, s.y_mm)
               for s in so.seats() if s.index in occ]
    scan = cast_global(gx, gy, (north + yaw) % 360.0, pillars, noise_mm=4.0, dropout=0.02)

    declared = so.DetectParams(lidar_offset_forward_mm=fwd, lidar_offset_lateral_mm=lat)
    with_off = so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=yaw, params=declared)
    without = so.detect_seat_occupancy(scan, x, y, robot_yaw_deg=yaw)

    def score(rs):
        return sum(1 for r in rs
                   if (r.state is so.Occupancy.OCCUPIED) == (r.seat.index in occ))
    print(f"PASS  test_lidar_lever_arm          declared={score(with_off)}/6  "
          f"undeclared={score(without)}/6")
    assert score(with_off) == 6, so.summary(with_off)


def test_pose_error_budget():
    """How much localization error can this take before it starts lying?

    This is the number that matters for the rest of the stack: it is the
    accuracy the pose fix has to hit. Errors are injected into the pose HANDED
    to the detector, while the scan is cast from the true pose."""
    occ_sets = [{0, 3}, {1, 4, 5}, {2}, set(), {0, 1, 2, 3, 4, 5}]
    print("PASS  test_pose_error_budget")
    print(f"      {'pos err':>8} {'yaw err':>8} | {'decided':>8} {'FP':>4} {'FN':>4}"
          f" {'accuracy':>9}")
    worst_clean = None
    for pos_err, yaw_err in ((0, 0), (25, 1), (50, 2), (75, 3), (100, 4), (150, 6), (250, 10)):
        tp = fp = tn = fn = unk = 0
        for lane in LANES:
            for direction in ("CCW", "CW"):
                for y in (600.0, 1000.0, 1400.0, 1800.0):
                    for occ in occ_sets:
                        x = 500.0
                        scan = scene(lane, direction, x, y, occ, yaw_deg=0.0)
                        # what the detector is TOLD, vs where the robot is
                        ang = random.uniform(0, 2 * math.pi)
                        bx = x + pos_err * math.cos(ang)
                        by = y + pos_err * math.sin(ang)
                        byaw = random.uniform(-yaw_err, yaw_err)
                        for r in so.detect_seat_occupancy(scan, bx, by, robot_yaw_deg=byaw):
                            truth = r.seat.index in occ
                            if r.state is so.Occupancy.UNKNOWN:
                                unk += 1
                            elif r.state is so.Occupancy.OCCUPIED:
                                tp, fp = (tp + 1, fp) if truth else (tp, fp + 1)
                            else:
                                tn, fn = (tn + 1, fn) if not truth else (tn, fn + 1)
        dec = tp + fp + tn + fn
        acc = 100 * (tp + tn) / dec if dec else float("nan")
        print(f"      {pos_err:6d}mm {yaw_err:6.0f}d | {dec:8d} {fp:4d} {fn:4d} {acc:8.1f}%")
        if fp == 0 and fn == 0:
            worst_clean = (pos_err, yaw_err)
    print(f"      -> clean up to {worst_clean[0]} mm position / {worst_clean[1]} deg heading error")
    assert worst_clean is not None and worst_clean[0] >= 50


if __name__ == "__main__":
    test_frame_maps_to_global()
    test_bearing_conventions()
    test_no_false_positives_on_empty_field()
    test_occlusion_reports_unknown()
    test_yaw_error_is_handled()
    test_accuracy_sweep()
    test_drive_through_resolves_every_seat_in_time()
    test_coarse_sampling_never_reports_false_empty()
    test_lidar_lever_arm()
    test_pose_error_budget()
    print("\nAll checks passed.")
