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
import scan_processing
from scan_prediction import predict_scan_global
from scan_processing import process_scan

app = Flask(__name__)

_state_lock = threading.Lock()
_latest_state: dict = {"ready": False}

# Per-run settings, filled in when a mode loop starts, surfaced read-only in the
# dashboard's TUNING PARAMETERS panel (they aren't in config.py -- they're the
# round-specific calls made in the loop).
_RUN_PARAMS: dict = {"mode": None, "driving_direction": None,
                     "initial_section": None, "predict_n_points": 90}


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
        "forward_mm": r(fix.forward_distance_mm),
        "left_mm": r(fix.left_distance_mm), "right_mm": r(fix.right_distance_mm),
        "lane_sum_mm": r(fix.lane_sum_mm),
        "lateral_mm": r(fix.lateral_from_outer_mm), "along_mm": r(fix.along_mm),
    }


def _candidates_to_list(candidates):
    """Enrich each of the (up to 8) start-of-run candidates with its PREDICTED
    LIDAR scan -- what the sensor would see IF the robot were at that pose --
    ray-cast against walls+island with this robot's rear blind arc modelled
    (see scan_prediction). `idx` is the candidate's stable index, which the
    dashboard maps to a fixed colour so the arrow and its predicted cloud
    share one colour. Predicted points are in GLOBAL mat coordinates, ready to
    draw directly. NOTE: predictions are STATIC (they depend only on the fixed
    candidate poses), so build this list ONCE at start-of-run and reuse it in
    every published frame -- don't recompute the ray-casts at stream rate."""
    out = []
    for i, c in enumerate(candidates):
        pred = predict_scan_global(c.x_mm, c.y_mm, c.heading_deg, n_points=90, model_blind_arc=True)
        out.append({
            "idx": i, "section": c.section, "heading_variant": c.heading_variant,
            "x_mm": round(c.x_mm, 1), "y_mm": round(c.y_mm, 1), "heading_deg": round(c.heading_deg, 1),
            "predicted": [{"x": px, "y": py} for px, py in pred],
        })
    return out


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

    DRIVING_DIRECTION = "CCW"
    _RUN_PARAMS.update({"mode": "mock", "driving_direction": DRIVING_DIRECTION,
                        "initial_section": "S", "predict_n_points": 90})
    sim = MockRobotSimulator(initial_section="S", direction=DRIVING_DIRECTION)
    # MockRobotSimulator's own default along_mm (50.0) sits right next to a
    # corner, too close for the start-of-run fix to resolve cleanly. Overridden
    # to a comfortably centred value so this demo exercises the fix; your real
    # placement needs similar clearance from both corners.
    sim.along_mm = 1500.0

    # One-time start-of-run reading: robot stationary in its REAL start
    # orientation -- parallel to the walls, facing the direction of travel
    # (NOT broadside). current_scan() is the along-lane view (driving heading);
    # broadside_scan() would be the wrong orientation for this fix. The side
    # rays (90/270) resolve cross-lane position, the forward ray (0) resolves
    # along-track -- see localization.compute_start_of_run_fix().
    heading = sim.true_state().heading_deg
    raw = sim.current_scan()
    clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)

    start_fix = compute_start_of_run_fix(clusters, driving_direction=DRIVING_DIRECTION)
    if start_fix.ok:
        initial_along_mm = start_fix.along_mm
        # Which of the 4 legs still can't come from the LIDAR alone (see
        # localization.py's module docstring) -- "S" here is still YOUR team's
        # manual call. start_candidates is what the dashboard shows so you can
        # visually confirm that call against all 4 possibilities.
        start_candidates = candidate_start_positions(
            start_fix.along_mm, start_fix.lateral_from_outer_mm, driving_direction=DRIVING_DIRECTION)
    else:
        print(f"[start-of-run fix] FAILED: {start_fix.reason} -- along_mm defaulting to 0.0")
        _debug_dump_clusters(clusters, label="start-of-run, mock")
        initial_along_mm = 0.0
        start_candidates = []
    # Build the candidate payload (poses + predicted scans) ONCE -- it's static.
    start_candidates_payload = _candidates_to_list(start_candidates)

    estimator = PoseEstimator(initial_section="S", driving_direction=DRIVING_DIRECTION,
                              initial_along_mm=initial_along_mm)
    estimator.update_heading(heading)
    # Seed lateral straight from the along-lane start fix (don't run the
    # broadside apply_lidar_fix here -- the robot isn't broadside at start).
    if start_fix.ok:
        estimator.state.lateral_mm = start_fix.lateral_from_outer_mm
        estimator.state.initialized = True
        estimator._recompute_xy()
    last_fix = None

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
            "start_candidates": start_candidates_payload,
            "points": points,
            "pillars": [{"x_mm": p.x_mm, "y_mm": p.y_mm, "color": p.color} for p in sim.pillars],
            "t": time.time(),
        })
        time.sleep(dt)


def _real_mode_loop():
    from lidar_source import RPLidarC1Source

    # YOUR team's manual calls for the round (see INTEGRATION POINT below):
    # which leg the robot starts on, and the randomised driving direction.
    INITIAL_SECTION = "S"
    DRIVING_DIRECTION = "CCW"
    _RUN_PARAMS.update({"mode": "real", "driving_direction": DRIVING_DIRECTION,
                        "initial_section": INITIAL_SECTION, "predict_n_points": 90})

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

    # One-time start-of-run reading: robot stationary in its REAL start
    # orientation -- parallel to the walls, facing the direction of travel
    # (NOT broadside). The side rays (90/270) resolve cross-lane position and
    # the forward ray (0) resolves along-track -- see
    # localization.compute_start_of_run_fix() -- and this builds the 4-leg
    # x 2-heading candidate list (with predicted scans) the dashboard shows.
    start_clusters = process_scan(start_raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
    start_fix = compute_start_of_run_fix(start_clusters, driving_direction=DRIVING_DIRECTION)
    if start_fix.ok:
        initial_along_mm = start_fix.along_mm
        start_candidates = candidate_start_positions(
            start_fix.along_mm, start_fix.lateral_from_outer_mm, driving_direction=DRIVING_DIRECTION)
    else:
        print(f"[start-of-run fix] FAILED: {start_fix.reason} -- along_mm defaulting to 0.0")
        _debug_dump_clusters(start_clusters, label="start-of-run, real")
        initial_along_mm = 0.0
        start_candidates = []
    # Build the candidate payload (poses + predicted scans) ONCE -- it's static.
    start_candidates_payload = _candidates_to_list(start_candidates)

    # --- INTEGRATION POINT --------------------------------------------
    # INITIAL_SECTION / DRIVING_DIRECTION are set at the top of this function
    # -- both are still YOUR team's manual calls (section identity can't come
    # from the LIDAR alone, see localization.py's module docstring;
    # start_candidates above is what the dashboard shows so you can visually
    # confirm the section call against all 4 legs).
    estimator = PoseEstimator(initial_section=INITIAL_SECTION, driving_direction=DRIVING_DIRECTION,
                              initial_along_mm=initial_along_mm)
    # Seed lateral from the along-lane start fix (the robot isn't broadside at
    # start, so don't run the broadside apply_lidar_fix here).
    if start_fix.ok:
        estimator.state.lateral_mm = start_fix.lateral_from_outer_mm
        estimator.state.initialized = True
        estimator._recompute_xy()

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
            "start_candidates": start_candidates_payload,
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


def _tuning_groups():
    """Every tunable, read LIVE from the modules so the panel never drifts from
    the code, grouped and annotated with what each one does / breaks. `critical`
    marks the make-or-break ones you must set per robot/round."""
    sp = scan_processing
    r = _RUN_PARAMS

    def P(name, value, unit, implication, critical=False):
        return {"name": name, "value": value, "unit": unit,
                "implication": implication, "critical": critical}

    return [
        {"group": "Round setup", "note": "Set these every round, before start (they live in the mode loop, not config.py).", "params": [
            P("MODE", config.MODE, "", "real hardware vs simulator (config.py)."),
            P("DRIVING_DIRECTION", r["driving_direction"], "", "CCW/CW. Fixes which side ray is the OUTER wall (CCW=left/90, CW=right/270) and the along-track sign. Wrong => lateral & along-track flipped.", True),
            P("INITIAL_SECTION", r["initial_section"], "", "Which leg the robot starts on (S/E/N/W). Cannot be sensed (that's the 4-candidate ambiguity); affects the live tracked pose only, not the candidate markers.", True),
        ]},
        {"group": "Field geometry (mat_geometry.py)", "note": "Measure against your real mat -- every fix is referenced to these.", "params": [
            P("OUTER_SIZE_MM", geo.OUTER_SIZE_MM, "mm", "Outer wall inside length = section length. Used in along-track (CW: along=OUTER-forward). Wrong => along-track biased, candidate markers sit off the walls.", True),
            P("LANE_WIDTH_MM", geo.LANE_WIDTH_MM, "mm", "Outer-wall-to-island gap. The start fix checks left+right ~= this. Wrong => good scans rejected or bad ones accepted.", True),
            P("INNER_SIZE_MM", geo.INNER_SIZE_MM, "mm", "Island size, DERIVED = OUTER - 2*LANE. Don't set by hand."),
            P("ISLAND_MIN_MM", geo.ISLAND_MIN_MM, "mm", "Island near edge (derived)."),
            P("ISLAND_MAX_MM", geo.ISLAND_MAX_MM, "mm", "Island far edge (derived)."),
            P("SAFE_FIX_MARGIN_MM", geo.SAFE_FIX_MARGIN_MM, "mm", "How far past a corner (into the mid-edge band) before a fix is trusted. Bigger = safer but narrower usable start band."),
            P("SAFE_FIX_ALONG_MIN_MM", geo.SAFE_FIX_ALONG_MIN_MM, "mm", "Valid mid-edge zone lower bound (derived). Start the robot inside [min,max]."),
            P("SAFE_FIX_ALONG_MAX_MM", geo.SAFE_FIX_ALONG_MAX_MM, "mm", "Valid mid-edge zone upper bound (derived)."),
        ]},
        {"group": "LIDAR hardware (config.py)", "note": "", "params": [
            P("LIDAR_PORT", config.LIDAR_PORT, "", "Serial device. Wrong => no data at all. Confirm with `ls /dev/ttyUSB*`."),
            P("LIDAR_BAUDRATE", config.LIDAR_BAUDRATE, "baud", "C1 default 460800. Wrong => garbage / no scan."),
            P("LIDAR_SCAN_TIMEOUT_S", config.LIDAR_SCAN_TIMEOUT_S, "s", "How long to assemble a scan. Too low drops points; too high adds latency."),
        ]},
        {"group": "LIDAR angle calibration (config.py)", "note": "The thing that bit you -- calibrate against a known heading.", "params": [
            P("LIDAR_ANGLE_ZERO_OFFSET_DEG", config.LIDAR_ANGLE_ZERO_OFFSET_DEG, "deg", "Rotates raw angle so 0=forward, 90=left, 270=right. Wrong => the 90/270 searches look in the wrong physical direction and the fix fails/mislabels walls.", True),
            P("LIDAR_ANGLE_SIGN", config.LIDAR_ANGLE_SIGN, "+/-1", "Flip if your unit's angle runs clockwise vs the code's CCW convention. Wrong => left/right swapped.", True),
        ]},
        {"group": "Sensor lever-arm (config.py)", "note": "Your measured LIDAR mount offset from the path-planner reference point.", "params": [
            P("LIDAR_OFFSET_FORWARD_MM", config.LIDAR_OFFSET_FORWARD_MM, "mm", "Shifts the along-track (forward) read. A big mount offset biases along-track by that much."),
            P("LIDAR_OFFSET_LATERAL_MM", config.LIDAR_OFFSET_LATERAL_MM, "mm", "Shifts the cross-lane read. Likely the source of a small lane-sum gap (yours read ~915 vs 1000)."),
        ]},
        {"group": "Rear blind arc (config.py) -- predicted overlay only", "note": "Does NOT affect the fix; only shapes each candidate's predicted cloud.", "params": [
            P("REAR_BLIND_ARC_CENTER_DEG", config.REAR_BLIND_ARC_CENTER_DEG, "deg", "Robot-relative centre of the chassis-blocked wedge (180=straight back). Measure the empty gap in the debug dump."),
            P("REAR_BLIND_ARC_WIDTH_DEG", config.REAR_BLIND_ARC_WIDTH_DEG, "deg", "Total width of that wedge. Wrong => the predicted gap doesn't line up with the real one (yours was ~107deg)."),
        ]},
        {"group": "Fix sanity tolerances (config.py)", "note": "", "params": [
            P("LANE_WIDTH_TOLERANCE_MM", config.LANE_WIDTH_TOLERANCE_MM, "mm", "Noise margin on left+right ~= LANE_WIDTH. Too tight => good scans rejected; too loose => an occluded side wall gets accepted (wrong lateral).", True),
            P("SIDE_SEARCH_WINDOW_DEG", config.SIDE_SEARCH_WINDOW_DEG, "deg", "How far from 90/270 to accept a lane-wall cluster. Too tight => 'no wall for left(90)'; too wide => grabs the wrong surface near a corner."),
            P("FRONT_BACK_SEARCH_WINDOW_DEG", config.FRONT_BACK_SEARCH_WINDOW_DEG, "deg", "+/- window around 0deg used to gather the forward (along-track) range. Too tight => misses it; too wide => biases along-track with off-axis points."),
            P("BROADSIDE_HEADING_TOLERANCE_DEG", config.BROADSIDE_HEADING_TOLERANCE_DEG, "deg", "Only the post-corner broadside RE-fix (compute_broadside_fix). Not used by the start-of-run fix."),
            P("SECTION_LENGTH_TOLERANCE_MM", config.SECTION_LENGTH_TOLERANCE_MM, "mm", "DEPRECATED / unused -- belonged to the old broadside-start assumption (side rays summing to OUTER_SIZE). Ignore."),
        ]},
        {"group": "Clustering: scan -> walls/pillars (scan_processing.py)", "note": "Decide whether a surface even becomes a 'wall'. Tune only when the debug dump shows a wall missed or mislabelled.", "params": [
            P("MIN_RANGE_MM", sp.MIN_RANGE_MM, "mm", "Drop points closer than this (chassis / sensor floor). Raise if the chassis intrudes as points."),
            P("MAX_RANGE_MM", sp.MAX_RANGE_MM, "mm", "Drop implausibly far returns."),
            P("MIN_QUALITY", sp.MIN_QUALITY, "", "Drop low-quality returns. Raise above 0 if weak noisy points form phantom clusters."),
            P("MAX_ANGLE_GAP_DEG", sp.MAX_ANGLE_GAP_DEG, "deg", "Break a cluster on an angular gap bigger than this. Too small => one dropout splits a close wall (the ~9% sim rejects); too big => merges across real gaps."),
            P("MAX_CHORD_JUMP_MM", sp.MAX_CHORD_JUMP_MM, "mm", "Break a cluster on a straight-line jump bigger than this. Too small fragments grazing walls; too big merges wall into a nearby pillar."),
            P("MAX_CLUSTER_SPAN_DEG", sp.MAX_CLUSTER_SPAN_DEG, "deg", "Hard cap on one cluster's angular width (stops chaining around a corner). A CLOSE lane wall subtends a wide arc and can hit this cap -- raise (120-130) if close side walls get truncated; too high risks merging two walls at a corner.", True),
            P("WALL_MIN_ARC_LENGTH_MM", sp.WALL_MIN_ARC_LENGTH_MM, "mm", "Min arc length (range x width) to call a cluster a wall. Too high drops short/far wall segments; too low calls a big pillar a wall."),
            P("WALL_MAX_FLATNESS_RESIDUAL_MM", sp.WALL_MAX_FLATNESS_RESIDUAL_MM, "mm", "Max RMS off the fitted line to still be 'flat'. This is what rejects the forward corner-blend. Too low rejects a noisy real wall; too high lets a curved corner pass as flat and corrupts the fix.", True),
            P("PILLAR_MIN_ARC_LENGTH_MM", sp.PILLAR_MIN_ARC_LENGTH_MM, "mm", "Lower arc bound for a pillar. Obstacle detection only; irrelevant to start-of-run."),
            P("PILLAR_MAX_ARC_LENGTH_MM", sp.PILLAR_MAX_ARC_LENGTH_MM, "mm", "Upper arc bound for a pillar (50mm posts). Obstacle detection only."),
            P("MIN_POINTS_PER_PILLAR", sp.MIN_POINTS_PER_PILLAR, "", "Reject singleton-point pillar fragments. Obstacle detection only."),
        ]},
        {"group": "Overlay & server", "note": "", "params": [
            P("predict n_points", r["predict_n_points"], "", "Angular resolution of each candidate's predicted cloud (dashboard_server). Higher = denser overlay, bigger payload. Purely visual."),
            P("STREAM_HZ", config.STREAM_HZ, "Hz", "Dashboard stream/refresh rate. No effect on localization; lower if the Pi is loaded."),
            P("DASHBOARD_HOST", config.DASHBOARD_HOST, "", "Server bind address."),
            P("DASHBOARD_PORT", config.DASHBOARD_PORT, "", "Server port."),
        ]},
    ]


@app.route("/api/tuning")
def api_tuning():
    return {"groups": _tuning_groups()}


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
