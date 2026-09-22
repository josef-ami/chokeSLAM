"""
Predicted (expected) LIDAR scan for a hypothesised pose, ray-cast against the
KNOWN field geometry (outer walls + island only -- pillars are not modelled,
their positions aren't known a-priori). This is deployment code, not part of
the mock-only simulation.py: the dashboard uses it in BOTH mock and real mode
to draw, for each of the 8 start-of-run candidate poses, what the LIDAR would
see IF the robot were at that pose. Comparing those predicted point clouds
against the single real start-of-run scan is how you (or your team) pick which
of the 4 legs the robot is actually on.

The rear chassis blind arc is modelled here on purpose (see
config.REAR_BLIND_ARC_*): on real hardware a ~105deg wedge centred on 180deg
robot-relative is permanently blocked by the robot's own body (see the
README's "Back reading dropped" section). Blanking that same wedge out of the
prediction means (a) the predicted cloud actually looks like a real return
from this robot rather than a full 360deg sweep, and (b) the two heading
variants at one leg (primary vs primary+90) produce DIFFERENT clouds -- the
blind wedge points a different way -- so all 8 predictions are visually
distinct, which is what makes the both-axes display useful rather than two
identical overlays.
"""
from __future__ import annotations

import math

import config
import mat_geometry as geo


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    """Slab-method ray/axis-aligned-box intersection. Returns (t_near, t_far)
    or None. Ray: (ox,oy) + t*(dx,dy). Same method as simulation._ray_box --
    duplicated rather than imported so this deployment module doesn't depend
    on the mock-only simulation harness."""
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


def _in_blind_arc(robot_rel_angle_deg: float) -> bool:
    """True if a ray at this robot-relative angle falls inside the chassis
    blind wedge (centred on config.REAR_BLIND_ARC_CENTER_DEG, total width
    config.REAR_BLIND_ARC_WIDTH_DEG)."""
    half = config.REAR_BLIND_ARC_WIDTH_DEG / 2.0
    d = abs((robot_rel_angle_deg - config.REAR_BLIND_ARC_CENTER_DEG + 180.0) % 360.0 - 180.0)
    return d <= half


def predict_scan_global(x_mm: float, y_mm: float, heading_deg: float,
                        n_points: int = 120, model_blind_arc: bool = True) -> list[tuple[float, float]]:
    """Ray-cast the KNOWN geometry (outer walls + island) from pose
    (x_mm, y_mm, heading_deg) and return the hit points in GLOBAL (mat)
    coordinates, ready to draw straight onto the dashboard mat.

    n_points: angular resolution of the predicted sweep. 120 (3deg spacing)
    is plenty to trace the walls for a visual overlay and keeps the streamed
    payload small; the real sensor runs far denser, this is only a prediction.
    model_blind_arc: when True, rays inside the rear chassis wedge are dropped
    so the prediction matches this robot's real (rear-occluded) field of view
    and the two heading variants at a leg differ. Set False for a full 360deg
    geometric prediction.

    Pillars are deliberately NOT modelled (unknown at start-of-run), so the
    predicted cloud traces only walls + island -- exactly the surfaces the
    start-of-run fix itself relies on.
    """
    outer = (0.0, 0.0, geo.OUTER_SIZE_MM, geo.OUTER_SIZE_MM)
    island = (geo.ISLAND_MIN_MM, geo.ISLAND_MIN_MM, geo.ISLAND_MAX_MM, geo.ISLAND_MAX_MM)

    pts: list[tuple[float, float]] = []
    for i in range(n_points):
        robot_rel = i * (360.0 / n_points)
        if model_blind_arc and _in_blind_arc(robot_rel):
            continue
        # heading_deg is a GRID BEARING; convert to a maths angle for ray trig.
        global_angle = math.radians((90.0 - heading_deg) + robot_rel)
        dx, dy = math.cos(global_angle), math.sin(global_angle)

        best_t = None
        hit = _ray_box(x_mm, y_mm, dx, dy, *outer)
        if hit is not None:
            _, t_far = hit
            if t_far > 0:
                best_t = t_far  # exiting through the outer wall

        hit = _ray_box(x_mm, y_mm, dx, dy, *island)
        if hit is not None:
            t_near, _ = hit
            if t_near > 1e-6 and (best_t is None or t_near < best_t):
                best_t = t_near

        if best_t is None or best_t <= 0:
            continue
        gx = x_mm + best_t * dx
        gy = y_mm + best_t * dy
        pts.append((round(gx, 1), round(gy, 1)))
    return pts
