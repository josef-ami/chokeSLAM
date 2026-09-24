"""
The dashboard's planned path (checkpoint F2): the page's picture of a plan, and
the planner preview's job, which runs in a WORKER PROCESS.

Why a process: a plan takes tens to hundreds of ms of pure Python. On a thread
it holds the interpreter lock that long, and the tracker's loop and the
LIDAR / STM32 reader threads (which stamp arrival times) wait -- found by
test_dashboard's real-mode test (2-4 wrong entry verdicts with the preview on a
thread, 0 with it off or in a process). The worker has its own interpreter, so
the tracker never waits for the planner (the Pi 5 has four cores).

The worker imports config.py afresh, so every request carries the current
config values (the dashboard edits them live) and the worker applies them first.
"""
from __future__ import annotations

import math
import time

import config
import field_map as fm


def _r(v, nd=1):
    return None if v is None else round(float(v), nd)


PLAN_VIEW_POINTS = 800


def plan_view(source, ok, what, path, world, goals, ms=None) -> dict:
    """The page's picture of a plan: path points, goal poses, pass-side lines
    (coloured by the pillar's colour) and the forward-only lines, and the
    planner's pillars with the side it must pass them on."""
    pts = []
    if path is not None:
        S = path.samples
        step = max(1, len(S) // PLAN_VIEW_POINTS)
        pts = [[round(float(a), 1), round(float(b), 1)] for a, b in S[::step, :2]]
        pts.append([round(float(S[-1, 0]), 1), round(float(S[-1, 1]), 1)])
    gates, pillars = [], []
    if world is not None:
        colour = {(p.slot, p.seat): p.color for p in world.pillars}
        for g in world.gates:
            kind = g.key[0]
            col = colour.get((g.key[1], g.key[2])) if kind == "gate" else None
            gates.append({"a": [_r(g.a[0]), _r(g.a[1])], "b": [_r(g.b[0]), _r(g.b[1])], "kind": kind, "color": col})
        pillars = [{"X": _r(p.X), "Y": _r(p.Y), "side": p.side, "state": p.state} for p in world.pillars]
    return {"source": source, "ok": ok, "what": what, "ms": ms, "at": time.monotonic(),
            "length_mm": None if path is None else _r(path.length), "path": pts,
            "goals": [[_r(g[0]), _r(g[1]), _r(90.0 - math.degrees(g[2]), 1)] for g in (goals or [])],
            "gates": gates, "pillars": pillars}


def preview_snapshot(trk):
    """What mission.plan_preview needs, copied out of the tracker (under the lock)."""
    return (trk.direction, fm.seat_table_from_tracker(trk), fm.tracker_pose(trk), trk.slot, trk.lane_index, trk.y)


def config_values() -> dict:
    """Every config.py value (UPPER_CASE), to replay in the worker."""
    return {k: getattr(config, k) for k in dir(config) if k.isupper()}


def preview_job(cfg: dict, snap) -> dict:
    """Runs in the worker: apply the config, plan, return the page's picture."""
    from mission import plan_preview
    for k, v in cfg.items():
        setattr(config, k, v)
    t0 = time.perf_counter()
    pv = plan_preview(*snap)
    ms = round((time.perf_counter() - t0) * 1000.0)
    view = plan_view("preview", pv.ok, pv.what, pv.path, pv.world, pv.goals, ms)
    view.pop("at", None)
    return view
