"""
Visibility-graph path planner for the obstacle round (checkpoint E).

This is the method of the owner's path_planner.py / path_planning_method.md
(obstacles -> inflated corners -> visibility graph -> Dijkstra -> tangent-arc
smoothing), rebuilt on the loop frame (field_map.py) with the fixes the
simulation report found (B1-B10) re-applied, each as agreed (decisions #66-#83):

    B1  start heading       the path leaves the start POSE: a minimum-radius arc
                            (left or right) until the car points along the first
                            leg ("turn first"); likewise it ENDS in a pose
                            ("turn last") -- a viewing pose, the lap's start pose.
    B2  pass sides          enforced by field_map's gates (a line from the pillar
                            to beyond the forbidden-side wall); no leg may cross one.
    B3  side per lane       each pillar's side is relative to its OWN lane's travel.
    B4  walls / island      nodes outside the field margin are dropped; the island
                            is an obstacle; every leg stays inside the field.
    B5  cut-throughs        a leg is tested against each inflated rectangle's
                            INTERIOR (Liang-Barsky clip), so a diagonal through an
                            obstacle is never "clear".
    B6  body swing          the smoothed path is checked with the car's real
                            footprint (config.BODY_*) along its whole length; an
                            obstacle it touches (with PLAN_CLEARANCE_MM) is inflated
                            by PLAN_INFLATION_STEP_MM and the plan is repeated.
    B7  arcs that don't fit (decision #75, Q9) a corner is allowed only if its
                            tangent arc -- radius = lock radius x PLAN_RADIUS_FACTOR,
                            separately for left and right -- fits in half of each
                            adjacent leg (the whole leg at the start and goal legs).
                            The search sees this, so it picks a route that is
                            drivable instead of shrinking the radius below the lock.
    B8  corner pivots       none. Progress round the loop is enforced by
                            CHECKPOINTS (lines across lanes) that must be crossed in
                            order; the search is a layered Dijkstra over
                            (previous node, node, checkpoints crossed).
    B9  lap-1 corner arc    no fixed corner arc: the corner is part of the planned
                            path into the next lane's viewing pose (mission.py).
    B10 dead code           not carried over.

Output: a Path of straight and arc primitives for the REAR-AXLE midpoint, with
dense samples (X, Y, th, curvature, s) for the follower (follower.py).
Units: mm, radians, loop frame (maths angles, left turns positive).
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

import config
from field_map import OUTER, Rect, WorldMap

TWO_PI = 2.0 * math.pi


def lock_radius(side: str) -> float:
    lock = config.STEER_LOCK_LEFT_DEG if side == "L" else config.STEER_LOCK_RIGHT_DEG
    return config.WHEELBASE_MM / math.tan(math.radians(lock))


def plan_radius(side: str) -> float:
    return lock_radius(side) * config.PLAN_RADIUS_FACTOR


# --- geometry ------------------------------------------------------------------------
def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def segs_cross(p1, p2, p3, p4) -> bool:
    """Segments p1p2 and p3p4 intersect (touching counts)."""
    d1, d2 = _cross(p3, p4, p1), _cross(p3, p4, p2)
    d3, d4 = _cross(p1, p2, p3), _cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True

    def on(a, b, c):
        return min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9 and \
            min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9
    return (abs(d1) < 1e-9 and on(p3, p4, p1)) or (abs(d2) < 1e-9 and on(p3, p4, p2)) or \
        (abs(d3) < 1e-9 and on(p1, p2, p3)) or (abs(d4) < 1e-9 and on(p1, p2, p4))


def seg_hits_box(p, q, box, eps: float = 1e-6) -> bool:
    """Does segment pq pass through the INTERIOR of box (x0, y0, x1, y1)?
    Liang-Barsky clip against the box shrunk by eps (grazing an edge or a
    corner is allowed: that is exactly where visibility-graph legs run)."""
    x0, y0, x1, y1 = box[0] + eps, box[1] + eps, box[2] - eps, box[3] - eps
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for pp, qq in ((-dx, p[0] - x0), (dx, x1 - p[0]), (-dy, p[1] - y0), (dy, y1 - p[1])):
        if abs(pp) < 1e-12:
            if qq < 0:
                return False
            continue
        r = qq / pp
        if pp < 0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return t0 < t1 - 1e-9


def inside_box(pt, box, eps: float = 1e-6) -> bool:
    return box[0] + eps < pt[0] < box[2] - eps and box[1] + eps < pt[1] < box[3] - eps


def wrap_pi(a: float) -> float:
    return (a + math.pi) % TWO_PI - math.pi


# --- tangents (one routine for circle->circle, circle->point, point->circle) ----------
def tangent(c1, s1: int, r1: float, c2, s2: int, r2: float):
    """The straight leaving circle 1 (centre c1, radius r1, +1 = driven
    counter-clockwise i.e. a LEFT turn, -1 = right) and arriving tangent to
    circle 2, driving in the direction of both circles. A radius of 0 makes
    that end a point. Returns (T1, T2, u) or None.
    With n the left normal of the travel direction u, a CCW circle's centre is
    on the left: c = T + r n  ->  T = c - s r n.  So  c2 - c1 = L u + k n with
    k = s2 r2 - s1 r1, hence L = sqrt(|V|^2 - k^2) and u = V rotated by
    -atan2(k, L)."""
    vx, vy = c2[0] - c1[0], c2[1] - c1[1]
    k = s2 * r2 - s1 * r1
    D2 = vx * vx + vy * vy
    if D2 <= k * k + 1e-9:
        return None
    L = math.sqrt(D2 - k * k)
    g = math.atan2(k, L)
    c, s = math.cos(-g), math.sin(-g)
    D = math.sqrt(D2)
    ux, uy = (vx * c - vy * s) / D, (vx * s + vy * c) / D
    nx, ny = -uy, ux
    T1 = (c1[0] - s1 * r1 * nx, c1[1] - s1 * r1 * ny)
    T2 = (c2[0] - s2 * r2 * nx, c2[1] - s2 * r2 * ny)
    return T1, T2, (ux, uy)


def circle_centre(pose, s: int, r: float):
    x, y, th = pose
    return (x - s * r * math.sin(th), y + s * r * math.cos(th))


def arc_sweep(centre, s: int, a_from, a_to) -> float:
    """Angle swept driving from point a_from to a_to round `centre` in direction s."""
    a0 = math.atan2(a_from[1] - centre[1], a_from[0] - centre[0])
    a1 = math.atan2(a_to[1] - centre[1], a_to[0] - centre[0])
    d = (a1 - a0) % TWO_PI if s > 0 else (a0 - a1) % TWO_PI
    return 0.0 if d > TWO_PI - 1e-7 else d


# --- path ----------------------------------------------------------------------------------
@dataclass
class Prim:
    kind: str            # "line" | "arc"
    x: float             # start pose
    y: float
    th: float
    length: float
    curv: float = 0.0    # signed, + = left


@dataclass
class Path:
    prims: list[Prim]
    length: float
    samples: np.ndarray = None       # N x 5: X, Y, th, curvature, s
    waypoints: list = field(default_factory=list)
    info: dict = field(default_factory=dict)

    def end_pose(self) -> tuple[float, float, float]:
        X, Y, th = self.samples[-1, :3]
        return float(X), float(Y), float(th)


def _prim_point(p: Prim, u: float):
    if p.kind == "line" or abs(p.curv) < 1e-12:
        return p.x + u * math.cos(p.th), p.y + u * math.sin(p.th), p.th
    r = 1.0 / p.curv
    th = p.th + u * p.curv
    return p.x + r * (math.sin(th) - math.sin(p.th)), p.y - r * (math.cos(th) - math.cos(p.th)), th


def sample_prims(prims: list[Prim], step: float = 10.0) -> np.ndarray:
    rows, s0 = [], 0.0
    for p in prims:
        n = max(1, int(math.ceil(p.length / step)))
        for i in range(n):
            u = p.length * i / n
            x, y, th = _prim_point(p, u)
            rows.append((x, y, th, p.curv if p.kind == "arc" else 0.0, s0 + u))
        s0 += p.length
    if prims:
        x, y, th = _prim_point(prims[-1], prims[-1].length)
        rows.append((x, y, th, prims[-1].curv if prims[-1].kind == "arc" else 0.0, s0))
    return np.array(rows, dtype=float)


def build_prims(start_pose, start_arc, poly, goal_arc) -> list[Prim]:
    """start_arc / goal_arc: (s, r, sweep) or None; poly: [T0, N1, ..., Nk, T1]
    (T0 = where the start arc ends, T1 = where the goal arc begins). Interior
    corners of poly are filleted with the side's plan radius (the search has
    already checked that every fillet fits)."""
    prims: list[Prim] = []
    x, y, th = start_pose
    if start_arc and start_arc[2] > 1e-9:
        s, r, sw = start_arc
        prims.append(Prim("arc", x, y, th, r * sw, s / r))
        x, y, th = _prim_point(prims[-1], prims[-1].length)
    # drop repeated points (a zero-length first / last leg)
    pts = []
    for p in poly:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
            pts.append((float(p[0]), float(p[1])))
    cur, cur_h = (x, y), th
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        hd = math.atan2(b[1] - a[1], b[0] - a[0])
        end_off, arc = 0.0, None
        if i + 2 < len(pts):
            c = pts[i + 2]
            ho = math.atan2(c[1] - b[1], c[0] - b[0])
            phi = wrap_pi(ho - hd)
            if abs(phi) > 1e-6:
                r = plan_radius("L" if phi > 0 else "R")
                end_off = r * math.tan(abs(phi) / 2.0)
                arc = (r, phi)
        e = (b[0] - end_off * math.cos(hd), b[1] - end_off * math.sin(hd))
        Ls = math.hypot(e[0] - cur[0], e[1] - cur[1])
        if Ls > 1e-6:
            prims.append(Prim("line", cur[0], cur[1], hd, Ls))
        cur_h = hd
        if arc:
            r, phi = arc
            prims.append(Prim("arc", e[0], e[1], hd, r * abs(phi), (1.0 if phi > 0 else -1.0) / r))
            cur = (b[0] + end_off * math.cos(hd + phi), b[1] + end_off * math.sin(hd + phi))
            cur_h = hd + phi
        else:
            cur = e
    if goal_arc and goal_arc[2] > 1e-9:
        s, r, sw = goal_arc
        prims.append(Prim("arc", cur[0], cur[1], cur_h, r * sw, s / r))
    return prims


# --- swept footprint -------------------------------------------------------------------
def body_hits(samples: np.ndarray, rects: list[Rect], clearance: float,
              front: float | None = None, rear: float | None = None, half: float | None = None):
    """First sample index where the car's footprint (+clearance) touches a rect
    or leaves the field, and what it touched (a Rect key or ("wall",)).
    Separating-axis test of the oriented body rectangle against each
    axis-aligned rect, vectorised over the samples."""
    F = (config.BODY_FRONT_MM if front is None else front) + clearance
    B = (config.BODY_REAR_MM if rear is None else rear) + clearance
    H = (config.BODY_HALF_WIDTH_MM if half is None else half) + clearance
    X, Y, th = samples[:, 0], samples[:, 1], samples[:, 2]
    c, s = np.cos(th), np.sin(th)
    fx = np.stack([F * c - H * s, F * c + H * s, -B * c + H * s, -B * c - H * s], axis=1)
    fy = np.stack([F * s + H * c, F * s - H * c, -B * s - H * c, -B * s + H * c], axis=1)
    cx, cy = X[:, None] + fx, Y[:, None] + fy
    bad = np.full(len(X), False)
    what = [None] * len(X)
    wall = (cx.min(1) < 0) | (cx.max(1) > OUTER) | (cy.min(1) < 0) | (cy.max(1) > OUTER)
    for i in np.nonzero(wall)[0]:
        what[i] = ("wall",)
    bad |= wall
    mid_f, half_f = (F - B) / 2.0, (F + B) / 2.0
    ox, oy = X + mid_f * c, Y + mid_f * s          # body centre
    for r in rects:
        hx, hy = (r.x1 - r.x0) / 2.0, (r.y1 - r.y0) / 2.0
        mx, my = (r.x0 + r.x1) / 2.0, (r.y0 + r.y1) / 2.0
        sep = (cx.max(1) < r.x0) | (cx.min(1) > r.x1) | (cy.max(1) < r.y0) | (cy.min(1) > r.y1)
        dx, dy = mx - ox, my - oy
        pu = np.abs(dx * c + dy * s)
        sep |= pu > half_f + hx * np.abs(c) + hy * np.abs(s)
        pn = np.abs(-dx * s + dy * c)
        sep |= pn > H + hx * np.abs(s) + hy * np.abs(c)
        hit = ~sep
        for i in np.nonzero(hit & ~bad)[0]:
            what[i] = r.key
        bad |= hit
    idx = np.nonzero(bad)[0]
    if len(idx) == 0:
        return None, None
    return int(idx[0]), what[idx[0]]


# --- the planner ---------------------------------------------------------------------------
class PlanError(RuntimeError):
    pass


@dataclass
class _Ctx:
    rects: list
    boxes: list          # inflated boxes, same order as rects
    margin: float        # field margin
    gates: list
    checkpoints: list
    extra: list = field(default_factory=list)   # free-space nodes (corner squares)


def _box_dist(pt, r: Rect) -> float:
    """Chebyshev distance from pt to the rect (negative inside)."""
    return max(r.x0 - pt[0], pt[0] - r.x1, r.y0 - pt[1], pt[1] - r.y1)


def _leg_ok(ctx: _Ctx, p, q) -> bool:
    for b in ctx.boxes:
        if seg_hits_box(p, q, b):
            return False
    for g in ctx.gates:
        if segs_cross(p, q, g.a, g.b):
            return False
    m = ctx.margin
    for pt in (p, q):
        if not (m - 1e-6 <= pt[0] <= OUTER - m + 1e-6 and m - 1e-6 <= pt[1] <= OUTER - m + 1e-6):
            return False
    return True


def _arc_ok(ctx: _Ctx, centre, s: int, r: float, a_from, sweep: float) -> bool:
    if sweep < 1e-9:
        return True
    a0 = math.atan2(a_from[1] - centre[1], a_from[0] - centre[0])
    n = max(2, int(r * sweep / 15.0) + 1)
    pts = [(centre[0] + r * math.cos(a0 + s * sweep * i / n), centre[1] + r * math.sin(a0 + s * sweep * i / n))
           for i in range(n + 1)]
    for p, q in zip(pts, pts[1:]):
        if not _leg_ok(ctx, p, q):
            return False
    return True


def _crossing(ctx: _Ctx, p, q) -> int:
    """Index of the checkpoint the leg pq crosses, -1 none, -2 more than one."""
    hit = -1
    for k, (a, b) in enumerate(ctx.checkpoints):
        if segs_cross(p, q, a, b):
            if hit != -1:
                return -2
            hit = k
    return hit


def _search(ctx: _Ctx, start, goals, max_start_sweep=math.radians(150.0)):
    RL, RR = plan_radius("L"), plan_radius("R")
    rad = {1: RL, -1: RR}
    K = len(ctx.checkpoints)
    # nodes: inflated corners inside the field margin and outside every other inflated box
    nodes = []
    for b in ctx.boxes:
        for pt in ((b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3])):
            if not (ctx.margin <= pt[0] <= OUTER - ctx.margin and ctx.margin <= pt[1] <= OUTER - ctx.margin):
                continue
            if any(inside_box(pt, o) for o in ctx.boxes):
                continue
            nodes.append(pt)
    for pt in ctx.extra:
        if (ctx.margin <= pt[0] <= OUTER - ctx.margin and ctx.margin <= pt[1] <= OUTER - ctx.margin
                and not any(inside_box(pt, o) for o in ctx.boxes)):
            nodes.append(tuple(pt))
    n = len(nodes)
    vis = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if _leg_ok(ctx, nodes[i], nodes[j]):
                L = math.hypot(nodes[j][0] - nodes[i][0], nodes[j][1] - nodes[i][1])
                if L < 1e-6:
                    continue
                cr = _crossing(ctx, nodes[i], nodes[j])
                if cr == -2:
                    continue
                vis[i].append((j, L, cr))
                vis[j].append((i, L, cr))

    def udir(p, q):
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        return ((q[0] - p[0]) / L, (q[1] - p[1]) / L), L

    def fillet(din, dout, avail_in, avail_out):
        """(ok, cost adjustment) of the corner between unit directions din, dout."""
        phi = math.atan2(din[0] * dout[1] - din[1] * dout[0], din[0] * dout[0] + din[1] * dout[1])
        a = abs(phi)
        if a < 1e-6:
            return True, 0.0
        if a > math.radians(config.PLAN_MAX_TURN_DEG):
            return False, 0.0
        r = RL if phi > 0 else RR
        d = r * math.tan(a / 2.0)
        if d > avail_in + 1e-6 or d > avail_out + 1e-6:
            return False, 0.0
        return True, r * a - 2.0 * d

    # start: arc (left or right) then a straight to a node -- per start side u in (-1, -2)
    start_legs = {}          # (u, j) -> (T, dir, L, cost_before_leg, crossing, arc)
    for u, s in ((-1, 1), (-2, -1)):
        r = rad[s]
        c1 = circle_centre(start, s, r)
        for j, N in enumerate(nodes):
            tg = tangent(c1, s, r, N, 1, 0.0)
            if tg is None:
                continue
            T, _, d = tg
            sw = arc_sweep(c1, s, start[:2], T)
            if sw > max_start_sweep or not _arc_ok(ctx, c1, s, r, start[:2], sw):
                continue
            if not _leg_ok(ctx, T, N):
                continue
            L = math.hypot(N[0] - T[0], N[1] - T[1])
            cr = _crossing(ctx, T, N)
            if cr == -2:
                continue
            start_legs[(u, j)] = (T, d, L, r * sw, cr, (s, r, sw))

    # goal approach from a point: straight to T', arc into the goal pose
    def goal_from(P, g):
        best = None
        for s in (1, -1):
            r = rad[s]
            c2 = circle_centre(g, s, r)
            tg = tangent(P, 1, 0.0, c2, s, r)
            if tg is None:
                continue
            _, T2, u = tg
            sw = arc_sweep(c2, s, T2, g[:2])
            if sw > max_start_sweep or not _arc_ok(ctx, c2, s, r, T2, sw):
                continue
            if not _leg_ok(ctx, P, T2):
                continue
            L = math.hypot(T2[0] - P[0], T2[1] - P[1])
            cr = _crossing(ctx, P, T2)
            if cr == -2:
                continue
            cost = L + r * sw
            if best is None or cost < best[0]:
                best = (cost, T2, L, cr, (s, r, sw), u)
        return best

    goal_cache = {}
    best_total, best_end = math.inf, None
    # direct start -> goal (CSC)
    for gi, g in enumerate(goals):
        for s1 in (1, -1):
            for s2 in (1, -1):
                c1, c2 = circle_centre(start, s1, rad[s1]), circle_centre(g, s2, rad[s2])
                tg = tangent(c1, s1, rad[s1], c2, s2, rad[s2])
                if tg is None:
                    continue
                T1, T2, _ = tg
                sw1 = arc_sweep(c1, s1, start[:2], T1)
                sw2 = arc_sweep(c2, s2, T2, g[:2])
                if sw1 > max_start_sweep or sw2 > max_start_sweep:
                    continue
                cr = _crossing(ctx, T1, T2)
                if (K > 0 and not (K == 1 and cr == 0)) or (K == 0 and cr not in (-1,)):
                    continue
                if not (_arc_ok(ctx, c1, s1, rad[s1], start[:2], sw1) and _leg_ok(ctx, T1, T2)
                        and _arc_ok(ctx, c2, s2, rad[s2], T2, sw2)):
                    continue
                cost = rad[s1] * sw1 + math.hypot(T2[0] - T1[0], T2[1] - T1[1]) + rad[s2] * sw2
                if cost < best_total:
                    best_total = cost
                    best_end = ("direct", gi, (s1, rad[s1], sw1), T1, T2, (s2, rad[s2], sw2))

    # layered Dijkstra over (prev, node, layer); prev < 0 = a start side
    dist = {}
    prev = {}
    pq = []
    for (u, j), (T, d, L, c0, cr, arc) in start_legs.items():
        layer = 0
        if cr >= 0:
            if cr != 0:
                continue
            layer = 1
        st = (u, j, layer)
        cost = c0 + L
        if cost < dist.get(st, math.inf):
            dist[st] = cost
            prev[st] = None
            heapq.heappush(pq, (cost, st))
    while pq:
        cost, st = heapq.heappop(pq)
        if cost > dist.get(st, math.inf) + 1e-9 or cost >= best_total:
            continue
        u, v, layer = st
        if u < 0:
            T, din, Lin, _, _, _ = start_legs[(u, v)]
            avail_in = Lin
        else:
            din, Lin = udir(nodes[u], nodes[v])
            avail_in = Lin / 2.0
        # goal from v
        if layer <= K:
            for gi, g in enumerate(goals):
                key = (v, gi)
                if key not in goal_cache:
                    goal_cache[key] = goal_from(nodes[v], g)
                gf = goal_cache[key]
                if gf is None:
                    continue
                gcost, T2, Lg, cr, arc, dout = gf
                lay2 = layer
                if cr >= 0:
                    if cr != layer:
                        continue
                    lay2 += 1
                if lay2 != K:
                    continue
                ok, adj = fillet(din, dout, avail_in, Lg)
                if not ok:
                    continue
                tot = cost + adj + gcost
                if tot < best_total:
                    best_total = tot
                    best_end = ("graph", gi, st, T2, arc)
        for w, L, cr in vis[v]:
            if w == u:
                continue
            lay2 = layer
            if cr >= 0:
                if cr != layer:
                    continue
                lay2 += 1
            dout, _ = udir(nodes[v], nodes[w])
            ok, adj = fillet(din, dout, avail_in, L / 2.0)
            if not ok:
                continue
            st2 = (v, w, lay2)
            c2 = cost + adj + L
            if c2 < dist.get(st2, math.inf) - 1e-9:
                dist[st2] = c2
                prev[st2] = st
                heapq.heappush(pq, (c2, st2))
    if best_end is None:
        return None
    if best_end[0] == "direct":
        _, gi, a1, T1, T2, a2 = best_end
        return build_prims(start, a1, [T1, T2], a2), best_total, [T1, T2], gi
    _, gi, st, T2, garc = best_end
    chain = []
    while st is not None:
        chain.append(st)
        st = prev[st]
    chain.reverse()
    u0, j0, _ = chain[0]
    T0, _, _, _, _, sarc = start_legs[(u0, j0)]
    poly = [T0] + [nodes[s[1]] for s in chain] + [T2]
    return build_prims(start, sarc, poly, garc), best_total, poly, gi


def plan(world: WorldMap, start, goals, checkpoints=(), bumps: dict | None = None,
         inflation: float | None = None, clearance: float | None = None, max_iter: int | None = None,
         extra_nodes=None) -> Path:
    """Shortest drivable path from `start` (X, Y, th) to the cheapest of `goals`
    (poses), crossing `checkpoints` in order. Raises PlanError if none."""
    base = config.PLAN_INFLATION_MM if inflation is None else inflation
    clearance = config.PLAN_CLEARANCE_MM if clearance is None else clearance
    bumps = dict(bumps or {})
    max_iter = config.PLAN_MAX_ITER if max_iter is None else max_iter
    goals = [tuple(g) for g in goals]
    # a goal pose whose own footprint is already too close to something can never be reached
    # cleanly (inflation is shrunk around goals), so drop it before searching
    ok_goals = [g for g in goals
                if body_hits(np.array([[g[0], g[1], g[2], 0.0, 0.0]]), world.rects, clearance)[0] is None]
    if not ok_goals:
        raise PlanError("every goal pose is closer than the clearance to an obstacle or wall")
    goal_index = [goals.index(g) for g in ok_goals]
    goals = ok_goals
    last_err = "no path"
    for it in range(max_iter):
        boxes = []
        margin = base + bumps.get(("wall",), 0.0)
        for pt in [start[:2]] + [g[:2] for g in goals]:
            margin = min(margin, pt[0] - 1.0, pt[1] - 1.0, OUTER - pt[0] - 1.0, OUTER - pt[1] - 1.0)
        for r in world.rects:
            infl = base + bumps.get(r.key, 0.0)
            for pt in [start[:2]] + [g[:2] for g in goals]:
                dd = _box_dist(pt, r)
                if dd <= 0:
                    raise PlanError(f"{'start' if pt == start[:2] else 'goal'} is inside {r.key}")
                infl = min(infl, dd - 1.0)
            boxes.append((r.x0 - infl, r.y0 - infl, r.x1 + infl, r.y1 + infl))
        ctx = _Ctx(world.rects, boxes, max(margin, 0.0), world.gates, list(checkpoints),
                   list(extra_nodes if extra_nodes is not None else world.extra_nodes))
        res = _search(ctx, start, goals)
        if res is None:
            raise PlanError(f"no path (iteration {it}, bumps {bumps}); last: {last_err}")
        prims, cost, poly, gi = res
        samples = sample_prims(prims)
        path = Path(prims, float(samples[-1, 4]), samples, poly, {"goal": goal_index[gi], "iterations": it + 1,
                                                                  "bumps": dict(bumps)})
        # swept footprint: leading samples that already touch (the start pose sits close to a pillar)
        # are checked for real contact only
        idx, what = body_hits(samples, world.rects, clearance)
        if idx is not None and idx == 0:
            k = 0
            while k < len(samples):
                i2, _ = body_hits(samples[k:k + 1], world.rects, clearance)
                if i2 is None:
                    break
                k += 1
            i0, w0 = body_hits(samples[:k], world.rects, 0.0) if k else (None, None)
            if i0 is not None:
                raise PlanError(f"the start pose touches {w0}")
            idx, what = body_hits(samples[k:], world.rects, clearance)
            if idx is not None:
                idx += k
        # the same at the goal end (a goal pose close to something)
        if idx is None:
            return path
        last_err = f"footprint touches {what} at s = {samples[idx, 4]:.0f} mm"
        bumps[what] = bumps.get(what, 0.0) + config.PLAN_INFLATION_STEP_MM
    raise PlanError(f"no collision-free path after {max_iter} iterations: {last_err}")
