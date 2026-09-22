"""
Verification for direction_detect.py (the CW/CCW gap test).

The worlds are built in GLOBAL coordinates from the rulebook geometry
(3000 x 3000 outer square, 1000 x 1000 island, 50 mm pillars on Fig. 11 seats,
200 x 20 mm parking-lot limitations) and ray-cast independently of the code
under test. The robot is placed facing its lane's direction of travel (rule
9.8), yaw 0 unless a test says otherwise.

Test-world assumptions (NOT code assumptions; they only shape the scenarios):
  - start poses: y 1150..1850 along the lane, x 200..800 from the outer wall;
  - 1-2 pillars per straightforward section (Fig. 8c cards), none within
    150 mm of the robot's reference point (the robot can't stand on one);
  - parking lot: 200 mm deep against the outer wall, 450 mm long
    (1.5 x a 300 mm robot), robot centred in it 100 mm from the outer wall,
    lot centre y 1300..1700; pillars in the start section only on the
    inner seats (Fig. 8e);
  - LIDAR: 720 rays/rev, 4 mm noise, 2 % dropout, 105 deg rear blind wedge;
  - placement yaw: 0, or uniform in +/- the stated range.

Run:  python3 test_direction.py
"""
from __future__ import annotations

import math
import random

import direction_detect as dd
import lane_frame as lf
import seat_occupancy as so
import mat_geometry as geo

LANES = lf.SECTIONS


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


def cast(gx, gy, heading_bearing, boxes=(), n_points=720, noise_mm=4.0, dropout=0.02,
         blind_center=180.0, blind_width=105.0, rng=random):
    """(angle_deg, range_mm) pairs, angle CLOCKWISE from forward. The ray's
    global grid bearing is heading + angle; global unit of bearing b is
    (sin b, cos b)."""
    pts = []
    for i in range(n_points):
        a = i * 360.0 / n_points
        if blind_width and abs((a - blind_center + 180.0) % 360.0 - 180.0) <= blind_width / 2.0:
            continue
        b = math.radians(heading_bearing + a)
        dx, dy = math.sin(b), math.cos(b)
        best = None
        hit = _ray_box(gx, gy, dx, dy, *geo.OUTER_BOX)
        if hit and hit[1] > 0:
            best = hit[1]
        for box in [geo.ISLAND_BOX] + list(boxes):
            hit = _ray_box(gx, gy, dx, dy, *box)
            if hit and hit[0] > 1e-6 and (best is None or hit[0] < best):
                best = hit[0]
        if best is None or (dropout and rng.random() < dropout):
            continue
        pts.append((a, max(1.0, best + (rng.gauss(0, noise_mm) if noise_mm else 0.0))))
    return pts


def pillar_box(gx, gy):
    return (gx - 25.0, gy - 25.0, gx + 25.0, gy + 25.0)


def limitation_boxes(lane, direction, y_centre, lot_len=450.0, depth=200.0, thick=20.0):
    boxes = []
    for yc in (y_centre - lot_len / 2 - thick / 2, y_centre + lot_len / 2 + thick / 2):
        a = lf.lane_to_global(lane, direction, 0.0, yc - thick / 2)
        b = lf.lane_to_global(lane, direction, depth, yc + thick / 2)
        boxes.append((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
    return boxes


def random_world(lane, direction, x, y, rng, parking=False, lot_centre=None):
    """Pillars on rulebook seats in all four lanes (1-2 each) + optional lot."""
    boxes, truth = [], set()
    for sec in LANES:
        k = rng.choice([1, 2])
        seat_pool = list(range(6))
        if sec == lane and parking:
            seat_pool = [1, 3, 5]                 # inner seats only (Fig. 8e)
        chosen = rng.sample(seat_pool, k)
        for s in so.seats():
            if s.index not in chosen:
                continue
            if sec == lane and math.hypot(s.x_mm - x, s.y_mm - y) < 150.0:
                continue                          # robot can't stand on a pillar
            gx, gy = lf.lane_to_global(sec, direction, s.x_mm, s.y_mm)
            boxes.append(pillar_box(gx, gy))
            if sec == lane:
                truth.add(s.index)
    if parking:
        boxes += limitation_boxes(lane, direction, lot_centre)
    return boxes, truth


def scan_at(lane, direction, x, y, boxes, yaw=0.0, rng=random, **kw):
    gx, gy = lf.lane_to_global(lane, direction, x, y)
    heading = (lf.grid_north_bearing(lane, direction) + yaw) % 360.0
    return cast(gx, gy, heading, boxes, rng=rng, **kw)


def zone_poses():
    for lane in LANES:
        for direction in lf.DIRECTIONS:
            for y in range(1150, 1900, 100):
                for x in (200.0, 350.0, 500.0, 650.0, 800.0):
                    yield lane, direction, x, float(y)


# --------------------------------------------------------------------- tests
def test_physical_sanity():
    """Ties the whole chain to the physical world: in the S lane driving CCW
    (facing east) the outer (south) wall is on the robot's RIGHT, so d(90)
    must be the distance to it; driving CW (facing west) it is on the LEFT."""
    for direction, outer_angle in (("CCW", 90.0), ("CW", 270.0)):
        pts = scan_at("S", direction, 300.0, 1500.0, [], noise_mm=0.0, dropout=0.0)
        d_outer = dd.side_ray_distance(pts, outer_angle)
        d_island = dd.side_ray_distance(pts, (outer_angle + 180.0) % 360.0)
        assert abs(d_outer - 300.0) < 1.0 and abs(d_island - 700.0) < 1.0, (direction, d_outer, d_island)
    print("PASS  test_physical_sanity          (CCW: outer wall at 90 = right; CW: at 270 = left)")


def test_opening_geometry():
    """Noiseless, empty field: the island-side opening must begin where the
    island wall ends (lane y = 2000, i.e. 2000 - y ahead of the sensor) and
    the outer side must show none."""
    worst_start = 0.0
    lengths = []
    for lane in LANES:
        for direction in lf.DIRECTIONS:
            for y in (1200.0, 1500.0, 1800.0):
                for x in (250.0, 500.0, 750.0):
                    pts = scan_at(lane, direction, x, y, [], noise_mm=0.0, dropout=0.0)
                    r = dd.detect_direction(pts)
                    island = r.left if direction == "CCW" else r.right
                    outer = r.right if direction == "CCW" else r.left
                    assert r.direction == direction, (lane, direction, x, y, r.reason)
                    assert outer.opening_mm == 0.0 and outer.n_through == 0
                    worst_start = max(worst_start, abs(island.opening_start_mm - (2000.0 - y)))
                    lengths.append(island.opening_mm)
    assert worst_start < 30.0, worst_start
    print(f"PASS  test_opening_geometry         opening starts at the island's end (worst {worst_start:.1f} mm); "
          f"length {min(lengths):.0f}..{max(lengths):.0f} mm; outer side 0 everywhere")


def _sweep(label, poses, yaw_range=0.0, parking=False, seed=1, reps=1, **kw):
    rng = random.Random(seed)
    ok = wrong = undet = 0
    island_open, outer_open = [], []
    poses = list(poses)
    for _ in range(reps):
        for lane, direction, x, y in poses:
            lot = rng.uniform(1300.0, 1700.0) if parking else None
            if parking:
                x, y = 100.0, lot
            boxes, _ = random_world(lane, direction, x, y, rng, parking, lot)
            yaw = rng.uniform(-yaw_range, yaw_range)
            r = dd.detect_direction(scan_at(lane, direction, x, y, boxes, yaw, rng=rng, **kw))
            island = r.left if direction == "CCW" else r.right
            outer = r.right if direction == "CCW" else r.left
            island_open.append(island.opening_mm)
            outer_open.append(outer.opening_mm)
            if r.direction is None:
                undet += 1
            elif r.direction == direction:
                ok += 1
            else:
                wrong += 1
    n = ok + wrong + undet
    island_open.sort()
    print(f"      {label:36s} n={n:4d}  correct {ok:4d} ({100*ok/n:5.1f}%)  "
          f"UNDETERMINED {undet:3d}  WRONG {wrong}   island opening median "
          f"{island_open[len(island_open)//2]:.0f} mm, outer max {max(outer_open):.0f} mm")
    return wrong


def test_direction_never_wrong():
    print("PASS  test_direction_never_wrong    (the requirement: UNDETERMINED is allowed, WRONG is not)")
    wrong = 0
    wrong += _sweep("start zones, yaw 0", zone_poses(), reps=2)
    wrong += _sweep("start zones, yaw 0, 360 rays/rev", zone_poses(), n_points=360)
    for yr in (1.0, 2.0, 3.0, 5.0, 8.0):
        wrong += _sweep(f"start zones, placement yaw +/-{yr:g}", zone_poses(), yaw_range=yr,
                        seed=int(7 + yr), reps=2)
    lot_poses = [(ln, d, 100.0, 0.0) for ln in LANES for d in lf.DIRECTIONS for _ in range(20)]
    wrong += _sweep("parking lot, yaw 0", lot_poses, parking=True, seed=3)
    wrong += _sweep("parking lot, placement yaw +/-5", lot_poses, yaw_range=5.0, parking=True, seed=4)
    assert wrong == 0, f"{wrong} WRONG directions"


def test_previously_wrong_scenarios():
    """Regression. These poses gave the WRONG direction with the first
    (parallel-wall, yaw = 0) model: a pillar just ahead on the mid-inner seat
    hides the real opening while placement yaw tilts the outer wall away from
    the model and fakes an opening there. Rebuilt exactly (lane pillars as
    found), over several noise draws. The tilt-fitted test must never be
    wrong here; the old model is re-run alongside to show the cases still
    bite, i.e. that this test is actually testing something."""
    cases = [("N", "CCW", 500.0, 1250.0, -2.0, (3, 5)),
             ("W", "CW", 500.0, 1350.0, 1.91, (1, 3)),
             ("E", "CCW", 500.0, 1250.0, -2.5, (3, 4)),
             ("N", "CCW", 500.0, 1350.0, -4.68, (3, 5)),
             ("W", "CW", 500.0, 1350.0, 3.19, (1, 3))]
    new_wrong = old_wrong = 0
    for lane, direction, x, y, yaw, seats_filled in cases:
        boxes = [pillar_box(*lf.lane_to_global(lane, direction, s.x_mm, s.y_mm))
                 for s in so.seats() if s.index in seats_filled]
        for seed in range(10):
            rng = random.Random(seed)
            pts = scan_at(lane, direction, x, y, boxes, yaw, rng=rng)
            r = dd.detect_direction(pts)
            new_wrong += (r.direction is not None and r.direction != direction)
            # the old model: walls parallel to the forward axis
            dl, dr_ = dd.side_ray_distance(pts, 270.0), dd.side_ray_distance(pts, 90.0)
            lo = dd.scan_side(pts, "left", dl).opening_mm
            ro = dd.scan_side(pts, "right", dr_).opening_mm
            old = "CCW" if (lo >= 500 and ro <= 150) else ("CW" if (ro >= 500 and lo <= 150) else None)
            old_wrong += (old is not None and old != direction)
    print(f"PASS  test_previously_wrong         tilt-fitted: {new_wrong} wrong / {len(cases)*10}; "
          f"old parallel model on the same scans: {old_wrong} wrong")
    assert new_wrong == 0
    assert old_wrong > 0, "the regression scenarios no longer reproduce the old failure"


def test_undetermined_cases():
    # nothing at all
    r = dd.detect_direction([])
    assert r.direction is None
    # a side ray missing entirely (e.g. inside a blind wedge)
    pts = scan_at("S", "CCW", 500.0, 1500.0, [], noise_mm=0.0, dropout=0.0)
    pts_no_right = [(a, d) for a, d in pts if abs(a - 90.0) > 5.0]
    assert dd.detect_direction(pts_no_right).direction is None
    # openings on BOTH sides (synthetic: mirror the island-side returns onto the outer side)
    left = [(a, d) for a, d in pts if a > 180.0]
    mirrored = left + [((360.0 - a) % 360.0, d) for a, d in left]
    r = dd.detect_direction(mirrored)
    assert r.direction is None, r.reason
    print("PASS  test_undetermined_cases       (no data / missing side ray / openings on both sides -> UNDETERMINED)")


if __name__ == "__main__":
    test_physical_sanity()
    test_opening_geometry()
    test_direction_never_wrong()
    test_previously_wrong_scenarios()
    test_undetermined_cases()
    print("\nAll direction checks passed.")
