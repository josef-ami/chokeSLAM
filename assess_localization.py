"""
Assessment harness for the existing localization code -- the script behind the
findings report.

Deliberately does NOT reuse the repo's own heading conventions: it re-derives
the physically correct driving heading from mat_geometry._section_axes plus
NEXT_SECTION_CCW, and ray-casts independently. Testing localization.py against
simulation.py proves nothing, because they share the same heading formula.

Run from inside the repo directory:   python3 assess_localization.py
"""
import math, random
import mat_geometry as geo
import localization as loc
import scan_processing as sp
import config

random.seed(1234)

# ---------------------------------------------------------------- geometry
def bearing_of_unit(ux, uy):
    """grid bearing (0=+Y, 90=+X, clockwise) of a unit vector"""
    return (90.0 - math.degrees(math.atan2(uy, ux))) % 360.0

def true_driving_heading(section, direction):
    """Heading the robot MUST face, derived from mat_geometry's own along-axis
    and the section-order dicts -- not from localization.driving_heading_deg."""
    (_, _), (ax, ay), _ = geo._section_axes(section)
    if direction == "CCW":
        ux, uy = ax, ay          # along increases in CCW travel
    else:
        ux, uy = -ax, -ay
    return bearing_of_unit(ux, uy)

def check_section_order():
    """Confirm that along-increasing really is the CCW order."""
    out = []
    for s in ("S", "E", "N", "W"):
        (sx, sy), (ax, ay), _ = geo._section_axes(s)
        end = (sx + ax * geo.OUTER_SIZE_MM, sy + ay * geo.OUTER_SIZE_MM)
        nxt = geo.NEXT_SECTION_CCW[s]
        nstart, _, _ = geo._section_axes(nxt)
        out.append((s, end, nxt, nstart, math.dist(end, nstart) < 1e-6))
    return out

# ------------------------------------------------------------- ray casting
def ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
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

def cast(x, y, heading_deg, pillars=(), n=720, noise=0.0, dropout=0.0,
         blind_center=None, blind_width=0.0):
    """Returns raw (robot_rel_angle_deg, dist_mm, quality) triples.
    Robot-relative angles: 0=forward, 90=left (counter-clockwise), matching
    scan_processing.angle_to_xy. heading_deg is a clockwise grid bearing."""
    outer = (0.0, 0.0, geo.OUTER_SIZE_MM, geo.OUTER_SIZE_MM)
    island = (geo.ISLAND_MIN_MM, geo.ISLAND_MIN_MM, geo.ISLAND_MAX_MM, geo.ISLAND_MAX_MM)
    boxes = [island] + [(px - 25, py - 25, px + 25, py + 25) for px, py in pillars]
    pts = []
    for i in range(n):
        rel = i * 360.0 / n
        if blind_center is not None:
            d = abs((rel - blind_center + 180.0) % 360.0 - 180.0)
            if d <= blind_width / 2.0:
                continue
        world = math.radians((90.0 - heading_deg) + rel)
        dx, dy = math.cos(world), math.sin(world)
        best = None
        hit = ray_box(x, y, dx, dy, *outer)
        if hit and hit[1] > 0:
            best = hit[1]
        for b in boxes:
            hit = ray_box(x, y, dx, dy, *b)
            if hit and hit[0] > 1e-6 and (best is None or hit[0] < best):
                best = hit[0]
        if best is None:
            continue
        if dropout and random.random() < dropout:
            continue
        pts.append((rel, max(1.0, best + (random.gauss(0, noise) if noise else 0.0)), 47))
    return pts

# ------------------------------------------------------------------ tests
def t_section_order():
    print("=== A. section order / axes consistency ===")
    for s, end, nxt, nstart, ok in check_section_order():
        print(f"  {s}: along-end {end} -> next({nxt}) start {tuple(nstart)}  contiguous={ok}")
    print()

def t_headings():
    print("=== B. driving_heading_deg vs geometry-derived travel heading ===")
    bad = 0
    for d in ("CCW", "CW"):
        for s in ("S", "E", "N", "W"):
            code = loc.driving_heading_deg(s, d)
            truth = true_driving_heading(s, d)
            flag = "OK " if abs((code - truth + 180) % 360 - 180) < 1e-6 else "MISMATCH"
            if flag.startswith("MIS"):
                bad += 1
            print(f"  {d} {s}: code={code:6.1f}  geometry={truth:6.1f}  {flag}")
    print(f"  --> {bad}/8 mismatched\n")

def t_outer_wall_side():
    print("=== C. which side is the OUTER wall while driving? ===")
    for d in ("CCW", "CW"):
        for s in ("S", "E", "N", "W"):
            h = true_driving_heading(s, d)
            # robot at mid-section, 300mm from outer wall
            x, y = geo.local_to_global(s, 1500.0, 300.0)
            pts = cast(x, y, h, n=720)
            cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(pts, 1, 0.0)))
            L = loc._find_wall_near(cl, 90.0, config.SIDE_SEARCH_WINDOW_DEG)
            R = loc._find_wall_near(cl, 270.0, config.SIDE_SEARCH_WINDOW_DEG)
            ld = L.line_distance_mm if L else float("nan")
            rd = R.line_distance_mm if R else float("nan")
            side = "LEFT" if abs(ld - 300) < abs(rd - 300) else "RIGHT"
            print(f"  {d} {s}: left={ld:7.1f} right={rd:7.1f}  -> outer wall on the {side}"
                  f"  (truth lateral_from_outer=300)")
    print()

def t_start_fix():
    print("=== D. compute_start_of_run_fix vs ground truth (noiseless) ===")
    print("    (robot placed at the TRUE driving heading derived from geometry)")
    hdr = f"  {'dir':4} {'sec':3} {'along_t':>8} {'lat_t':>6} | {'along_e':>8} {'lat_e':>7} | {'d_along':>8} {'d_lat':>7}  reason"
    print(hdr)
    for d in ("CCW", "CW"):
        for s in ("S", "E", "N", "W"):
            for along in (1200.0, 1500.0, 1800.0):
                lat = 400.0
                h = true_driving_heading(s, d)
                x, y = geo.local_to_global(s, along, lat)
                pts = cast(x, y, h, n=720)
                cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(pts, 1, 0.0)))
                fx = loc.compute_start_of_run_fix(cl, d)
                if not fx.ok:
                    print(f"  {d:4} {s:3} {along:8.0f} {lat:6.0f} |   FAILED  ------ |   ------  ------  {fx.reason[:60]}")
                    continue
                da = fx.along_mm - along
                dl = fx.lateral_from_outer_mm - lat
                print(f"  {d:4} {s:3} {along:8.0f} {lat:6.0f} | {fx.along_mm:8.1f} {fx.lateral_from_outer_mm:7.1f} |"
                      f" {da:8.1f} {dl:7.1f}")
    print()

def t_start_fix_code_heading():
    print("=== E. same test, but robot placed at localization.driving_heading_deg() ===")
    for d in ("CCW", "CW"):
        for s in ("S",):
            along, lat = 1500.0, 400.0
            h = loc.driving_heading_deg(s, d)
            x, y = geo.local_to_global(s, along, lat)
            pts = cast(x, y, h, n=720)
            cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(pts, 1, 0.0)))
            fx = loc.compute_start_of_run_fix(cl, d)
            print(f"  {d} {s} heading={h}: ok={fx.ok} along={fx.along_mm} lat={fx.lateral_from_outer_mm}"
                  f" (truth along={along} lat={lat}) {fx.reason[:50]}")
    print()

def t_broadside():
    print("=== F. compute_broadside_fix ===")
    for s in ("S", "E", "N", "W"):
        for lat in (250.0, 500.0, 750.0):
            h = geo.BROADSIDE_HEADING_DEG[s]
            x, y = geo.local_to_global(s, 1500.0, lat)
            pts = cast(x, y, h, n=720)
            cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(pts, 1, 0.0)))
            fx = loc.compute_broadside_fix(cl, h, s)
            got = fx.lateral_from_outer_mm
            print(f"  {s} lat_true={lat:6.1f} -> ok={fx.ok} lat_est={got if got is None else round(got,1)}"
                  f"  {'' if fx.ok else fx.reason[:60]}")
    print()

def t_blind_arc():
    print("=== G. start fix with the real 105deg rear blind arc present ===")
    for d in ("CCW", "CW"):
        s, along, lat = "S", 1500.0, 400.0
        h = true_driving_heading(s, d)
        x, y = geo.local_to_global(s, along, lat)
        pts = cast(x, y, h, n=720, blind_center=180.0, blind_width=105.0)
        cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(pts, 1, 0.0)))
        fx = loc.compute_start_of_run_fix(cl, d)
        print(f"  {d}: ok={fx.ok} along={fx.along_mm} lat={fx.lateral_from_outer_mm} {fx.reason[:70]}")
    print()

def t_slots():
    print("=== H. 24 slot positions: code vs rulebook Figure 11 ===")
    code = sorted((round(s.x_mm), round(s.y_mm)) for s in geo.all_slots())
    rule = []
    for a in (1000, 1500, 2000):
        rule += [(a, 400), (a, 600), (a, 2400), (a, 2600),
                 (400, a), (600, a), (2400, a), (2600, a)]
    rule = sorted(set(rule))
    print(f"  code slots ({len(code)}):")
    print("   ", code)
    print(f"  rulebook slots ({len(rule)}):")
    print("   ", rule)
    print(f"  identical: {code == rule}")
    print()

def _pass_one():
    t_section_order()
    t_headings()
    t_outer_wall_side()
    t_start_fix()
    t_start_fix_code_heading()
    t_broadside()
    t_blind_arc()
    t_slots()


import statistics
from statistics import median


def _pass_two():
    def robust_range(raw, target_deg, halfwin=2.0):
        vals = [d for a, d, q in raw if abs((a - target_deg + 180) % 360 - 180) <= halfwin]
        return statistics.median(vals) if vals else None

    print("=== I. ground truth via clustering-independent median ranges ===")
    print("   (confirms which side the outer wall is on, and the along formula)")
    print(f"  {'dir':4} {'sec':3} {'along':>6} {'lat':>5} | {'fwd(0)':>7} {'left(90)':>8} {'right(270)':>10} |"
          f" {'3000-fwd':>8} {'fwd':>6}")
    for d in ("CCW", "CW"):
        for s in ("S", "E", "N", "W"):
            for along, lat in ((1200.0, 400.0), (1500.0, 600.0)):
                h = true_driving_heading(s, d)
                x, y = geo.local_to_global(s, along, lat)
                raw = cast(x, y, h, n=1440)
                f = robust_range(raw, 0.0)
                l = robust_range(raw, 90.0)
                r = robust_range(raw, 270.0)
                print(f"  {d:4} {s:3} {along:6.0f} {lat:5.0f} | {f:7.1f} {l:8.1f} {r:10.1f} |"
                      f" {geo.OUTER_SIZE_MM-f:8.1f} {f:6.1f}")
    print()

    print("=== J. success rate of compute_start_of_run_fix across the field (noiseless) ===")
    for d in ("CCW", "CW"):
        ok = fail = 0
        lat_err, along_err = [], []
        for s in ("S", "E", "N", "W"):
            for along in range(200, 2900, 100):
                for lat in (200.0, 400.0, 500.0, 600.0, 800.0):
                    h = true_driving_heading(s, d)
                    x, y = geo.local_to_global(s, float(along), lat)
                    raw = cast(x, y, h, n=720)
                    cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(raw, 1, 0.0)))
                    fx = loc.compute_start_of_run_fix(cl, d)
                    if fx.ok:
                        ok += 1
                        lat_err.append(fx.lateral_from_outer_mm - lat)
                        along_err.append(fx.along_mm - along)
                    else:
                        fail += 1
        n = ok + fail
        print(f"  {d}: ok {ok}/{n} = {100*ok/n:.1f}%")
        if lat_err:
            print(f"      lateral error  median {statistics.median(lat_err):+8.1f} mm  "
                  f"min {min(lat_err):+8.1f} max {max(lat_err):+8.1f}")
            print(f"      along   error  median {statistics.median(along_err):+8.1f} mm  "
                  f"min {min(along_err):+8.1f} max {max(along_err):+8.1f}")
    print()

    print("=== K. success rate of compute_broadside_fix across the field (noiseless) ===")
    ok = fail = 0
    errs = []
    for s in ("S", "E", "N", "W"):
        for along in range(200, 2900, 100):
            for lat in (200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0):
                h = geo.BROADSIDE_HEADING_DEG[s]
                x, y = geo.local_to_global(s, float(along), lat)
                raw = cast(x, y, h, n=720)
                cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(raw, 1, 0.0)))
                fx = loc.compute_broadside_fix(cl, h, s)
                if fx.ok:
                    ok += 1
                    errs.append(fx.lateral_from_outer_mm - lat)
                else:
                    fail += 1
    n = ok + fail
    print(f"  ok {ok}/{n} = {100*ok/n:.1f}%")
    if errs:
        print(f"  lateral error median {statistics.median(errs):+.1f} mm, max |err| {max(abs(e) for e in errs):.1f} mm")
    print()

    print("=== L. same broadside sweep, restricted to in_safe_fix_zone ===")
    ok = fail = 0
    for s in ("S", "E", "N", "W"):
        for along in range(200, 2900, 100):
            if not (geo.SAFE_FIX_ALONG_MIN_MM <= along <= geo.SAFE_FIX_ALONG_MAX_MM):
                continue
            for lat in (200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0):
                h = geo.BROADSIDE_HEADING_DEG[s]
                x, y = geo.local_to_global(s, float(along), lat)
                raw = cast(x, y, h, n=720)
                cl = sp.classify_clusters(sp.cluster_points(sp.clean_and_project(raw, 1, 0.0)))
                fx = loc.compute_broadside_fix(cl, h, s)
                ok, fail = (ok + 1, fail) if fx.ok else (ok, fail + 1)
    print(f"  safe zone is along {geo.SAFE_FIX_ALONG_MIN_MM:.0f}..{geo.SAFE_FIX_ALONG_MAX_MM:.0f} mm")
    print(f"  ok {ok}/{ok+fail}")


if __name__ == "__main__":
    _pass_one()
    _pass_two()
