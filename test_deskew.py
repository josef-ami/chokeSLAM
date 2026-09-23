"""
Verification for deskew.py against independent geometry.

The robot drives a known curved path (global frame, heading clockwise from
north). World points are seen by a LIDAR with a lever arm (110 mm ahead,
60 mm left of the reference point), each return taken from the pose at its
own time over one 100 ms revolution. De-skewed to the frame's end, every
return must equal what the sensor sees from the END pose -- computed here
directly, not with deskew's formulas.

Run:  python3 test_deskew.py
"""
from __future__ import annotations

import math
import random

from deskew import PoseHistory, deskew
from scan_processing import ScanPoint

F, L = 110.0, 60.0          # lever arm: ahead, left


def pose(t):
    """Arc: 600 mm/s, turning right at 60 deg/s, from (0, 0) heading 10 deg."""
    h = 10.0 + 60.0 * t
    # integrate exactly: x' = v sin h, y' = v cos h with h linear in t
    w = math.radians(60.0)
    h0 = math.radians(10.0)
    x = 600.0 / w * (math.cos(h0) - math.cos(h0 + w * t))
    y = 600.0 / w * (math.sin(h0 + w * t) - math.sin(h0))
    return x, y, h


def seen(px, py, x, y, h):
    """World point -> sensor return (fwd, right) from robot pose (x, y, h)."""
    hr = math.radians(h)
    fwd, right, left = (math.sin(hr), math.cos(hr)), (math.cos(hr), -math.sin(hr)), (-math.cos(hr), math.sin(hr))
    sx, sy = x + F * fwd[0] + L * left[0], y + F * fwd[1] + L * left[1]
    dx, dy = px - sx, py - sy
    return dx * fwd[0] + dy * fwd[1], dx * right[0] + dy * right[1]


def sp(f, r):
    return ScanPoint(angle_deg=math.degrees(math.atan2(r, f)) % 360.0, dist_mm=math.hypot(f, r), quality=47,
                     fwd_mm=f, right_mm=r)


def history(t_end, keep=0.5, offset_xy=(0.0, 0.0)):
    h = PoseHistory(keep)
    t = t_end - 1.0
    while t <= t_end + 1e-9:
        x, y, hd = pose(t)
        h.append(t, x + offset_xy[0], y + offset_xy[1], hd)
        t += 0.01
    return h


def test_against_geometry():
    rng = random.Random(3)
    world = [(rng.uniform(-2000, 2000), rng.uniform(-500, 3000)) for _ in range(400)]
    t_end = 1.0
    hist = history(t_end)
    pts, times = [], []
    for k, (px, py) in enumerate(world):
        t_i = t_end - 0.1 * k / len(world)                   # spread over one revolution
        pts.append(sp(*seen(px, py, *pose(t_i))))
        times.append(t_i)
    out, st = deskew(pts, times, hist, 0.0, F, L)
    want = sorted((sp(*seen(px, py, *pose(t_end))) for px, py in world), key=lambda p: p.angle_deg)
    err = max(math.hypot(a.fwd_mm - b.fwd_mm, a.right_mm - b.right_mm) for a, b in zip(out, want))
    raw = max(math.hypot(p.fwd_mm - q.fwd_mm, p.right_mm - q.right_mm)
              for p, q in zip(pts, (sp(*seen(px, py, *pose(t_end))) for px, py in world)))
    assert st.used == 400 and st.dropped_old == 0 and err < 0.5, (st, err)
    # the odometry frame's origin is arbitrary: shifting it must change nothing
    out2, _ = deskew(pts, times, history(t_end, offset_xy=(5000.0, -300.0)), 0.0, F, L)
    assert max(abs(a.dist_mm - b.dist_mm) for a, b in zip(out, out2)) < 1e-6
    print(f"PASS  test_against_geometry         400 returns over one 100 ms revolution on a 600 mm/s, 60 deg/s arc, "
          f"lever arm 110 ahead / 60 left: before de-skew up to {raw:.0f} mm off the end-pose view, "
          f"after {err:.3f} mm (the 10 ms pose history is interpolated linearly)")


def test_identity_offset_and_limits():
    t_end = 1.0
    hist = history(t_end)
    pts = [sp(1000.0, 0.0), sp(0.0, 800.0), sp(-500.0, -500.0)]
    out, st = deskew(pts, [t_end] * 3, hist, 0.0, F, L)
    key = lambda p: p.dist_mm                                  # (0 deg may come back as 359.9999...)
    assert max(math.hypot(a.fwd_mm - b.fwd_mm, a.right_mm - b.right_mm)
               for a, b in zip(sorted(pts, key=key), sorted(out, key=key))) < 1e-9
    # LIDAR_TIME_OFFSET_S: stamps 20 ms late with offset 0.020 == stamps on time with offset 0
    x, y, h = pose(0.95)
    wp = (x + 300.0, y + 1500.0)
    p = sp(*seen(*wp, *pose(0.95)))
    a, _ = deskew([p], [0.95], hist, 0.0, F, L)
    b, _ = deskew([p], [0.97], hist, 0.020, F, L)
    assert abs(a[0].dist_mm - b[0].dist_mm) < 1e-9
    # too old -> dropped
    out, st = deskew(pts, [t_end - 0.6, t_end, t_end], hist, 0.0, F, L)
    assert st.dropped_old == 1 and st.used == 2, st
    # measured 8 ms after the newest pose: extrapolated along the arc
    wp = (500.0, 2500.0)
    p = sp(*seen(*wp, *pose(t_end + 0.008)))
    out, st = deskew([p], [t_end + 0.008], hist, 0.0, F, L)
    want = sp(*seen(*wp, *pose(t_end)))
    ext_err = math.hypot(out[0].fwd_mm - want.fwd_mm, out[0].right_mm - want.right_mm)
    raw_err = math.hypot(p.fwd_mm - want.fwd_mm, p.right_mm - want.right_mm)
    assert st.extrapolated == 1 and ext_err < 1.0, (st, ext_err)
    print(f"PASS  test_identity_offset_limits    returns at the end time are unchanged; the offset shifts stamps "
          f"exactly; returns older than the history are dropped; one measured 8 ms after the newest pose is "
          f"extrapolated ({raw_err:.1f} mm off -> {ext_err:.2f} mm)")


if __name__ == "__main__":
    test_against_geometry()
    test_identity_offset_and_limits()
    print("\nAll de-skew checks passed.")
