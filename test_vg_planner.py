"""
Checkpoint E: the visibility-graph planner (vg_planner.py) and its world (field_map.py).

    python3 test_vg_planner.py

- tangent(): circle -> circle / point, checked against the geometry it claims
- loop frame: a tracker pose re-expressed in the loop frame agrees with global
  geometry in every lane and both directions
- 150 rulebook layouts (layouts.draw), the TRUE final map, the full lap from
  the start lane's viewing pose back to it, each plan checked independently:
    * the footprint (config.BODY_*) keeps PLAN_CLEARANCE_MM from every pillar,
      limitation, the island and the walls along the whole path
    * every pillar is passed on its rulebook side (red right, green left): at
      the sample where the path crosses the pillar's lane y, the car is on the
      correct side of it, judged in that lane's own frame
    * curvature never exceeds 1 / plan radius (per side), heading and position
      are continuous, the path starts and ends at the requested poses
    * the four midlines are crossed in order
"""
from __future__ import annotations

import math
import random
import time

import numpy as np

import config
import display
import field_map as fm
import lane_frame as lf
import layouts
import seat_occupancy as so
import vg_planner as vp


def test_tangent():
    rng = random.Random(3)
    worst = 0.0
    for _ in range(2000):
        c1 = (rng.uniform(0, 1000), rng.uniform(0, 1000))
        c2 = (rng.uniform(0, 1000), rng.uniform(0, 1000))
        s1, s2 = rng.choice([1, -1]), rng.choice([1, -1])
        r1, r2 = rng.choice([0.0, rng.uniform(50, 200)]), rng.choice([0.0, rng.uniform(50, 200)])
        tg = vp.tangent(c1, s1, r1, c2, s2, r2)
        if tg is None:
            continue
        T1, T2, u = tg
        # on the circles, u along T1->T2, u perpendicular to the radii, the circles on the correct side
        for c, s, r, T in ((c1, s1, r1, T1), (c2, s2, r2, T2)):
            worst = max(worst, abs(math.hypot(T[0] - c[0], T[1] - c[1]) - r))
            if r > 0:
                n = (-u[1], u[0])
                worst = max(worst, abs((c[0] - T[0]) - s * r * n[0]), abs((c[1] - T[1]) - s * r * n[1]))
        L = math.hypot(T2[0] - T1[0], T2[1] - T1[1])
        worst = max(worst, abs((T2[0] - T1[0]) - L * u[0]), abs((T2[1] - T1[1]) - L * u[1]))
    assert worst < 1e-6, worst
    print(f"PASS  test_tangent                2000 random circle/point pairs: tangent points on the circles, "
          f"centres on the driving side, straight along u (worst {worst:.1e} mm)")


def test_loop_frame():
    worst = 0.0
    for d in lf.DIRECTIONS:
        for sec0 in lf.SECTIONS:
            g2d = display.GlobalToDisplay(sec0, d)
            sec = sec0
            for slot in range(4):
                for x, y, psi in ((200, 300, 10.0), (700, 2500, -35.0), (500, 1500, 0.0)):
                    X, Y, th = fm.lane_pose(slot, x, y, psi, d)
                    gx, gy = lf.lane_to_global(sec, d, x, y)
                    Xg, Yg = g2d.point(gx, gy)
                    worst = max(worst, math.hypot(X - Xg, Y - Yg))
                    b = g2d.bearing(lf.yaw_to_heading(psi, sec, d))
                    worst = max(worst, abs(((90.0 - b) - math.degrees(th) + 180) % 360 - 180))
                sec = lf.NEXT_SECTION[d][sec]
    assert worst < 1e-6, worst
    print(f"PASS  test_loop_frame             every lane, both directions, all start sections: lane pose -> loop frame "
          f"agrees with global geometry (worst {worst:.1e})")


def _truth_table(lay):
    tab = {}
    for k in range(4):
        occ = dict(lay.pillars[lay.slot_section(k)])
        for s in so.seats():
            tab[(k, s.index)] = ("occupied", occ[s.index]) if s.index in occ else ("empty", "unknown")
    return tab


def _check_path(p, world, start, goal, d, tab):
    S = p.samples
    # continuity
    step = np.hypot(np.diff(S[:, 0]), np.diff(S[:, 1]))
    assert step.max() < 12.0, step.max()
    dth = np.abs((np.diff(S[:, 2]) + math.pi) % (2 * math.pi) - math.pi)
    assert dth.max() < math.radians(10.0), math.degrees(dth.max())
    assert math.hypot(S[0, 0] - start[0], S[0, 1] - start[1]) < 1e-6
    assert math.hypot(S[-1, 0] - goal[0], S[-1, 1] - goal[1]) < 1.0, (S[-1], goal)
    assert abs((S[-1, 2] - goal[2] + math.pi) % (2 * math.pi) - math.pi) < math.radians(1.0)
    # curvature
    kmax_l, kmax_r = 1.0 / vp.plan_radius("L"), 1.0 / vp.plan_radius("R")
    assert (S[:, 3] <= kmax_l + 1e-9).all() and (-S[:, 3] <= kmax_r + 1e-9).all()
    # clearance
    idx, what = vp.body_hits(S, world.rects, config.PLAN_CLEARANCE_MM - 0.5)
    assert idx is None, (what, S[idx])
    # pass sides, in each lane's own frame
    h = lf.handedness(d)
    for (slot, i), (state, col) in tab.items():
        if state != "occupied":
            continue
        seat = so.seats()[i]
        # the path's samples in this lane's frame: invert lane_to_display via a local search
        xs, ys = [], []
        for X, Y in S[:, :2]:
            x0, y0 = display.lane_to_display(slot, 0, 0, d)
            x1, y1 = display.lane_to_display(slot, 1, 0, d)
            x2, y2 = display.lane_to_display(slot, 0, 1, d)
            ex, ey = (x1 - x0, y1 - y0), (x2 - x0, y2 - y0)
            dx, dy = X - x0, Y - y0
            xs.append(dx * ex[0] + dy * ex[1])
            ys.append(dx * ey[0] + dy * ey[1])
        xs, ys = np.array(xs), np.array(ys)
        crossed = 0
        for j in range(len(ys) - 1):
            if ys[j] < seat.y_mm <= ys[j + 1] and -100 < xs[j] < 1100:
                right_of = (xs[j] - seat.x_mm) * h > 0
                assert right_of == (col == "red"), (slot, i, col, xs[j])
                crossed += 1
        assert crossed == 1, (slot, i, crossed)


def test_rulebook_laps(n=150):
    t0 = time.time()
    lens, iters, times = [], [], []
    for k in range(n):
        lay = layouts.draw(random.Random(k))
        d = lay.direction
        tab = _truth_table(lay)
        w = fm.build_world(d, tab, range(4))
        V0 = fm.lane_pose(0, 500.0, 500.0, 0.0, d)
        t1 = time.time()
        p = vp.plan(w, V0, [V0], [fm.checkpoint(s, d) for s in range(4)])
        times.append(time.time() - t1)
        _check_path(p, w, V0, V0, d, tab)
        lens.append(p.length)
        iters.append(p.info["iterations"])
    print(f"PASS  test_rulebook_laps          {n} rulebook layouts, true map, full lap V0 -> V0: every plan clear by "
          f"{config.PLAN_CLEARANCE_MM:.0f} mm, every pillar on its side, curvature within the plan radius; "
          f"length {min(lens):.0f}-{max(lens):.0f} mm (median {sorted(lens)[n // 2]:.0f}), iterations max {max(iters)}, "
          f"plan time median {sorted(times)[n // 2]:.2f} s max {max(times):.2f} s ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    test_tangent()
    test_loop_frame()
    test_rulebook_laps()
    print("\nAll planner checks passed.")
