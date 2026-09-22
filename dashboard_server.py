"""
Flask dashboard: the mat drawn to scale with the live (classified) LIDAR
point cloud and the estimated robot pose overlaid, streamed over
Server-Sent Events (plain Flask, no socketio/eventlet needed -- this
sandbox had no network access to install extra packages, and SSE is
plenty for a local single-viewer debug dashboard).

Run:
    python3 dashboard_server.py
then open http://<pi-ip-or-localhost>:5056/

Mode is controlled by config.MODE ("mock" or "real"). "real" mode still
needs you to wire up your actual encoder/IMU feed -- see the comment in
_real_mode_loop() below; that part could not be written for you without
your STM32 UART protocol.
"""
from __future__ import annotations

import json
import math
import threading
import time

from flask import Flask, Response, render_template

import config
import mat_geometry as geo
from localization import (PoseEstimator, candidate_start_positions,
                           compute_start_of_run_fix)
from scan_processing import process_scan

app = Flask(__name__)

_state_lock = threading.Lock()
_latest_state: dict = {"ready": False}


def _publish(state: dict):
    with _state_lock:
        _latest_state.clear()
        _latest_state.update(state)
        _latest_state["ready"] = True


def _rotate_to_global(x_mm, y_mm, pose_x, pose_y, heading_deg):
    h = math.radians(heading_deg)
    gx = pose_x + x_mm * math.cos(h) - y_mm * math.sin(h)
    gy = pose_y + x_mm * math.sin(h) + y_mm * math.cos(h)
    return gx, gy


def _clusters_to_points(clusters, pose_x, pose_y, heading_deg):
    out = []
    for c in clusters:
        for p in c.points:
            gx, gy = _rotate_to_global(p.x_mm, p.y_mm, pose_x, pose_y, heading_deg)
            out.append({"x": round(gx, 1), "y": round(gy, 1), "kind": c.kind})
    return out


def _fix_to_dict(fix):
    if fix is None:
        return None
    return {
        "ok": fix.ok, "reason": fix.reason,
        "front_mm": None if fix.front_distance_mm is None else round(fix.front_distance_mm, 1),
        "back_mm": None if fix.back_distance_mm is None else round(fix.back_distance_mm, 1),
        "lane_sum_mm": None if fix.lane_sum_mm is None else round(fix.lane_sum_mm, 1),
    }


def _start_fix_to_dict(fix):
    if fix is None:
        return None
    r = lambda v: None if v is None else round(v, 1)
    return {
        "ok": fix.ok, "reason": fix.reason,
        "front_mm": r(fix.front_distance_mm), "back_mm": r(fix.back_distance_mm), "lane_sum_mm": r(fix.lane_sum_mm),
        "left_mm": r(fix.left_distance_mm), "right_mm": r(fix.right_distance_mm),
        "section_length_sum_mm": r(fix.section_length_sum_mm),
        "along_mm": r(fix.along_mm), "along_mm_from_left": r(fix.along_mm_from_left),
        "along_mm_from_right": r(fix.along_mm_from_right),
    }


def _candidates_to_list(candidates):
    return [{"section": c.section, "heading_variant": c.heading_variant,
             "x_mm": round(c.x_mm, 1), "y_mm": round(c.y_mm, 1), "heading_deg": round(c.heading_deg, 1)}
            for c in candidates]


def _debug_dump_clusters(clusters, label=""):
    """Prints EVERY cluster (not just ones classified 'wall') with its
    robot-relative angle range and fit quality. Call this whenever
    compute_broadside_fix/compute_start_of_run_fix fails, so the next
    failure is fully visible in one shot instead of needing another
    round-trip. Reading it: nothing near the expected angle (0/90/180/270)
    at all suggests an angle-calibration problem (LIDAR_ANGLE_SIGN /
    _ZERO_OFFSET_DEG -- see config.py); something there but classified
    'pillar'/'unclassified' with a short span or high flatness suggests
    a physical occlusion (chassis, bracket, wiring) or too-close range
    (MIN_RANGE_MM) rather than a calibration issue."""
    print(f"[debug] full cluster dump{f' ({label})' if label else ''}:")
    for c in clusters:
        a0, a1 = c.points[0].angle_deg, c.points[-1].angle_deg
        dist = None if c.line_distance_mm is None else round(c.line_distance_mm, 1)
        flat = None if c.flatness_residual_mm is None else round(c.flatness_residual_mm, 1)
        print(f"  kind={c.kind:12s} angle=({a0:6.1f},{a1:6.1f}) span={c.angular_span_deg:5.1f} "
              f"n={len(c.points):3d} dist={dist} flat={flat}")


def _mock_mode_loop():
    from simulation import MockRobotSimulator

    sim = MockRobotSimulator(initial_section="S", direction="CCW")
    # MockRobotSimulator's own default along_mm (50.0) sits right next to a
    # corner -- too close for the new start-of-run left/right fix below to
    # resolve (found during testing: with a wall corner that close, the
    # near-corner wall return blends into the same merged/rejected cluster
    # as the outer wall, exactly the kind of corner-blending effect
    # mat_geometry.py already documents for the back/island reading, just
    # from the opposite direction -- see compute_start_of_run_fix()'s
    # docstring). Overridden here to a comfortably centred value so this
    # demo actually exercises the new fix successfully; your team's real
    # starting placement needs similar clearance from both corners -- you
    # said yours always is, but if you see this fix reject unexpectedly on
    # real hardware, corner proximity is the first thing to check.
    sim.along_mm = 1500.0

    # One-time start-of-run reading, per the "good idea for bootstrapping"
    # discussion: robot stationary and already placed broadside. Used for
    # TWO things now: (1) the usual lateral broadside fix, and (2) resolving
    # along-track position from the left/right (90/270 deg) rays -- see
    # localization.compute_start_of_run_fix(). Both come from the SAME scan.
    raw, heading = sim.broadside_scan()
    clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)

    start_fix = compute_start_of_run_fix(clusters)
    if start_fix.ok:
        initial_along_mm = start_fix.along_mm
        # Section identity still can't come from the LIDAR alone (see
        # localization.py's module docstring) -- "S" here is still YOUR
        # team's manual call (e.g. watching the referee place the robot),
        # same as before. start_candidates below is what the dashboard
        # shows so you can visually confirm that call against all 4
        # possibilities, one-time, at start-of-run only.
        start_candidates = candidate_start_positions(start_fix.along_mm, start_fix.lateral_from_outer_mm)
    else:
        # Couldn't resolve along-track position (occlusion, bad geometry,
        # etc.) -- fall back to 0.0 like before and flag it loudly rather
        # than silently trusting a bad number.
        print(f"[start-of-run fix] FAILED: {start_fix.reason} -- along_mm defaulting to 0.0")
        _debug_dump_clusters(clusters, label="start-of-run, mock")
        initial_along_mm = 0.0
        start_candidates = []

    estimator = PoseEstimator(initial_section="S", driving_direction="CCW", initial_along_mm=initial_along_mm)
    estimator.update_heading(heading)
    last_fix = estimator.apply_lidar_fix(clusters)

    dt = 1.0 / config.STREAM_HZ
    awaiting_fix = False
    while True:
        delta_mm, heading_deg, corner_completed = sim.step(dt)
        estimator.update_heading(heading_deg)
        estimator.update_odometry(delta_mm)

        if corner_completed:
            estimator.on_corner_completed()   # advance section / reset along_mm, no fix yet
            awaiting_fix = True

        if awaiting_fix and estimator.in_safe_fix_zone():
            # Briefly broadside for the re-anchor, then straighten back out.
            raw, broadside_heading = sim.broadside_scan()
            estimator.update_heading(broadside_heading)
            clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            last_fix = estimator.apply_lidar_fix(clusters)
            estimator.update_heading(heading_deg)  # restore driving heading
            awaiting_fix = False
        else:
            raw = sim.current_scan()
            clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)

        est = estimator.state
        true = sim.true_state()
        points = _clusters_to_points(clusters, est.x_mm, est.y_mm, est.heading_deg)

        _publish({
            "mode": "mock",
            "estimated": {
                "x_mm": round(est.x_mm, 1), "y_mm": round(est.y_mm, 1),
                "heading_deg": round(est.heading_deg, 1), "section": est.section,
                "lateral_mm": round(est.lateral_mm, 1), "along_mm": round(est.along_mm, 1),
            },
            "true": {
                "x_mm": round(true.x_mm, 1), "y_mm": round(true.y_mm, 1),
                "heading_deg": round(true.heading_deg, 1), "section": true.section,
            },
            "last_fix": _fix_to_dict(last_fix),
            "start_fix": _start_fix_to_dict(start_fix),
            "start_candidates": _candidates_to_list(start_candidates),
            "points": points,
            "pillars": [{"x_mm": p.x_mm, "y_mm": p.y_mm, "color": p.color} for p in sim.pillars],
            "t": time.time(),
        })
        time.sleep(dt)


def _real_mode_loop():
    from lidar_source import RPLidarC1Source

    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    lidar.start()

    # Give the background thread a few seconds to connect and start filling
    # the scan table, polling rather than one fixed sleep -- if it's STILL
    # empty after this, print full diagnostics instead of silently handing
    # compute_start_of_run_fix() zero points (which just produces an opaque
    # "no wall cluster found for front, back, left, right").
    start_raw = []
    for _ in range(10):
        time.sleep(0.5)
        start_raw = lidar.get_latest_scan()
        if start_raw:
            break
    if not start_raw:
        print(f"[start-of-run fix] no LIDAR points received after 5s -- lidar.status(): {lidar.status()}")

    # One-time start-of-run reading: robot stationary, already placed
    # broadside per your team's start procedure. Resolves along-track
    # position from the left/right (90/270 deg) rays -- see
    # localization.compute_start_of_run_fix() -- and builds the 4-section
    # x 2-heading candidate list the dashboard displays for you to
    # visually cross-check.
    start_clusters = process_scan(start_raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
    start_fix = compute_start_of_run_fix(start_clusters)
    if start_fix.ok:
        initial_along_mm = start_fix.along_mm
        start_candidates = candidate_start_positions(start_fix.along_mm, start_fix.lateral_from_outer_mm)
    else:
        print(f"[start-of-run fix] FAILED: {start_fix.reason} -- along_mm defaulting to 0.0")
        _debug_dump_clusters(start_clusters, label="start-of-run, real")
        initial_along_mm = 0.0
        start_candidates = []

    # --- INTEGRATION POINT --------------------------------------------
    # Plug your real starting section (still YOUR team's manual call --
    # section identity can't come from the LIDAR alone, see
    # localization.py's module docstring; start_candidates above is what
    # the dashboard shows so you can visually confirm this call) and
    # driving direction (as randomised/announced for the round) in here:
    estimator = PoseEstimator(initial_section="S", driving_direction="CCW", initial_along_mm=initial_along_mm)

    # You need to feed this estimator from your real STM32 telemetry:
    #   - call estimator.update_heading(imu_yaw_deg) whenever you get a new
    #     IMU reading (translated into this module's 0=east/90=north/CCW
    #     convention -- your IMU almost certainly reports something else,
    #     convert once at the boundary rather than changing convention here)
    #   - call estimator.update_odometry(delta_forward_mm) with the
    #     incremental distance since the last call, from your wheel encoder
    #   - call estimator.on_corner_completed() from your turn FSM the
    #     moment a corner turn finishes (no LIDAR needed for this call --
    #     it just advances section/along-track state)
    #   - then keep driving normally until estimator.in_safe_fix_zone() is
    #     True, at which point go briefly broadside and call
    #     estimator.apply_lidar_fix() with a fresh lidar.get_latest_scan()
    #     processed through process_scan() -- see mat_geometry.py's note on
    #     why a fix attempted right at the corner itself usually just fails
    # None of that STM32 UART parsing exists in this sandbox, so it isn't
    # implemented here -- wire it up where your Pi already reads the STM32.
    # --------------------------------------------------------------------

    dt = 1.0 / config.STREAM_HZ
    while True:
        raw = lidar.get_latest_scan()
        clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
        est = estimator.state
        points = _clusters_to_points(clusters, est.x_mm, est.y_mm, est.heading_deg)
        _publish({
            "mode": "real",
            "estimated": {
                "x_mm": round(est.x_mm, 1), "y_mm": round(est.y_mm, 1),
                "heading_deg": round(est.heading_deg, 1), "section": est.section,
                "lateral_mm": round(est.lateral_mm, 1), "along_mm": round(est.along_mm, 1),
            },
            "true": None,
            "last_fix": _fix_to_dict(None if not est.initialized else
                                      type("F", (), {"ok": est.last_fix_ok, "reason": est.last_fix_reason,
                                                      "front_distance_mm": None, "back_distance_mm": None,
                                                      "lane_sum_mm": None})()),
            "start_fix": _start_fix_to_dict(start_fix),
            "start_candidates": _candidates_to_list(start_candidates),
            "points": points,
            "pillars": [],
            "t": time.time(),
        })
        time.sleep(dt)


@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/api/field")
def api_field():
    slots = [{"section": s.section, "row": s.row, "col": s.col, "x_mm": s.x_mm, "y_mm": s.y_mm}
              for s in geo.all_slots()]
    return {
        "outer_size_mm": geo.OUTER_SIZE_MM,
        "lane_width_mm": geo.LANE_WIDTH_MM,
        "island_min_mm": geo.ISLAND_MIN_MM,
        "island_max_mm": geo.ISLAND_MAX_MM,
        "slots": slots,
    }


@app.route("/stream")
def stream():
    def gen():
        last_sent = None
        while True:
            with _state_lock:
                state = dict(_latest_state)
            if state.get("ready"):
                payload = json.dumps(state)
                yield f"data: {payload}\n\n"
            time.sleep(1.0 / config.STREAM_HZ)
    return Response(gen(), mimetype="text/event-stream")


def main():
    target = _mock_mode_loop if config.MODE == "mock" else _real_mode_loop
    t = threading.Thread(target=target, daemon=True)
    t.start()
    app.run(host=config.DASHBOARD_HOST, port=config.DASHBOARD_PORT, threaded=True, debug=False)


if __name__ == "__main__":
    main()
