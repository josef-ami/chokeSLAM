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

# Shared runtime: the live objects the mode loop owns, exposed so the editable
# TUNING panel's POST routes can act on them (re-run the fix, edit the pose,
# etc.). Set when a loop starts. `freeze_pose` lets you pause the mock's
# odometry/heading feed so hand-edited pose values aren't immediately
# overwritten by the simulator (in real mode nothing feeds the estimator here,
# so edits stick regardless).
_RT: dict = {
    "mode": None, "driving_direction": "CCW", "initial_section": "S",
    "predict_n_points": 90, "freeze_pose": False,
    "estimator": None, "sim": None, "lidar": None,
    "start_fix": None, "start_candidates_payload": [],
}


def _recompute_geo_derived():
    """Re-derive the dependent field constants after OUTER/LANE/SAFE_FIX_MARGIN
    are edited, so the derived values (island extent, safe zone) stay
    consistent instead of holding their import-time values."""
    geo.INNER_SIZE_MM = geo.OUTER_SIZE_MM - 2 * geo.LANE_WIDTH_MM
    geo.ISLAND_MIN_MM = (geo.OUTER_SIZE_MM - geo.INNER_SIZE_MM) / 2.0
    geo.ISLAND_MAX_MM = geo.ISLAND_MIN_MM + geo.INNER_SIZE_MM
    geo.SAFE_FIX_ALONG_MIN_MM = geo.ISLAND_MIN_MM + geo.SAFE_FIX_MARGIN_MM
    geo.SAFE_FIX_ALONG_MAX_MM = geo.ISLAND_MAX_MM - geo.SAFE_FIX_MARGIN_MM


def _coerce(kind, v):
    if kind == "float":
        return float(v)
    if kind == "int":
        return int(round(float(v)))
    if kind == "bool":
        return v in (True, "true", "True", "1", 1)
    return str(v)


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
    n_points = int(_RT.get("predict_n_points", 90))
    for i, c in enumerate(candidates):
        pred = predict_scan_global(c.x_mm, c.y_mm, c.heading_deg, n_points=n_points, model_blind_arc=True)
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
    _RT.update({"mode": "mock", "driving_direction": DRIVING_DIRECTION,
                "initial_section": "S", "predict_n_points": 90, "freeze_pose": False})
    sim = MockRobotSimulator(initial_section="S", direction=DRIVING_DIRECTION)
    _RT["sim"] = sim
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
    # Publish handles + fix/candidates through _RT so the editable panel's
    # routes (edit pose, re-run fix) act on the same live objects.
    _RT["estimator"] = estimator
    _RT["start_fix"] = start_fix
    _RT["start_candidates_payload"] = start_candidates_payload

    awaiting_fix = False
    while True:
        dt = 1.0 / config.STREAM_HZ          # re-read each loop so STREAM_HZ edits take effect
        delta_mm, heading_deg, corner_completed = sim.step(dt)
        if not _RT["freeze_pose"]:            # freeze lets hand-edited pose values stick
            estimator.update_heading(heading_deg)
            estimator.update_odometry(delta_mm)

        if corner_completed and not _RT["freeze_pose"]:
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
            "start_fix": _start_fix_to_dict(_RT["start_fix"]),
            "start_candidates": _RT["start_candidates_payload"],
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
    _RT.update({"mode": "real", "driving_direction": DRIVING_DIRECTION,
                "initial_section": INITIAL_SECTION, "predict_n_points": 90, "freeze_pose": False})

    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    lidar.start()
    _RT["lidar"] = lidar

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
    _RT["estimator"] = estimator
    _RT["start_fix"] = start_fix
    _RT["start_candidates_payload"] = start_candidates_payload

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

    while True:
        dt = 1.0 / config.STREAM_HZ          # re-read each loop so STREAM_HZ edits take effect
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
            "start_fix": _start_fix_to_dict(_RT["start_fix"]),
            "start_candidates": _RT["start_candidates_payload"],
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


# --- Editable parameter registry -----------------------------------------
# Every knob (tuning constants AND the live pose-estimation state) as a spec
# with a get/set closure, so one GET builds the panel and one POST applies an
# edit to the RUNNING server. GEO_CORE_NAMES trigger a field redraw + derived
# recompute on the client.
GEO_CORE_NAMES = {"OUTER_SIZE_MM", "LANE_WIDTH_MM", "SAFE_FIX_MARGIN_MM"}


def _cfg(name, kind, unit, imp, crit=False, options=None):
    return {"name": name, "kind": kind, "unit": unit, "implication": imp, "critical": crit,
            "options": options, "get": (lambda: getattr(config, name)),
            "set": (lambda v: setattr(config, name, _coerce(kind, v)))}


def _spc(name, kind, unit, imp, crit=False):
    return {"name": name, "kind": kind, "unit": unit, "implication": imp, "critical": crit,
            "options": None, "get": (lambda: getattr(scan_processing, name)),
            "set": (lambda v: setattr(scan_processing, name, _coerce(kind, v)))}


def _geo_core(name, unit, imp, crit=False):
    def _s(v):
        setattr(geo, name, _coerce("float", v))
        _recompute_geo_derived()
    return {"name": name, "kind": "float", "unit": unit, "implication": imp, "critical": crit,
            "options": None, "get": (lambda: getattr(geo, name)), "set": _s}


def _geo_ro(name, unit, imp):
    return {"name": name, "kind": "readonly", "unit": unit, "implication": imp, "critical": False,
            "options": None, "get": (lambda: getattr(geo, name)), "set": None}


def _ro(name, kind, value_fn, unit, imp):
    return {"name": name, "kind": "readonly", "unit": unit, "implication": imp, "critical": False,
            "options": None, "get": value_fn, "set": None}


def _run(name, kind, key, unit, imp, crit=False, options=None, setter=None):
    return {"name": name, "kind": kind, "unit": unit, "implication": imp, "critical": crit,
            "options": options, "get": (lambda: _RT[key]),
            "set": (setter if setter else (lambda v: _RT.__setitem__(key, _coerce(kind, v))))}


def _set_driving_direction(v):
    v = str(v).upper()
    if v not in ("CCW", "CW"):
        raise ValueError("driving direction must be CCW or CW")
    _RT["driving_direction"] = v
    est = _RT["estimator"]
    if est is not None:
        est._next_section = geo.NEXT_SECTION_CCW if v == "CCW" else geo.NEXT_SECTION_CW


def _pose(name, field, unit, imp, options=None, kind="float"):
    def _g():
        est = _RT["estimator"]
        return None if est is None else getattr(est.state, field)

    def _s(v):
        est = _RT["estimator"]
        if est is None:
            raise RuntimeError("estimator not started yet")
        if field == "section":
            v = str(v).upper()
            if v not in ("S", "E", "N", "W"):
                raise ValueError("section must be S/E/N/W")
            est.state.section = v
        elif field == "heading_deg":
            est.state.heading_deg = _coerce("float", v) % 360.0
        else:
            setattr(est.state, field, _coerce("float", v))
        est._recompute_xy()
    return {"name": name, "kind": kind, "unit": unit, "implication": imp, "critical": False,
            "options": options, "get": _g, "set": _s}


def _param_registry():
    """Ordered list of {group, note, params:[spec,...]}. Rebuilt per request so
    every value is read live."""
    return [
        {"group": "Round setup", "note": "Per-round calls (live in the loop, not config.py). MODE needs a restart.", "params": [
            _ro("MODE", "str", (lambda: config.MODE), "", "real hardware vs simulator. Switching needs a server restart (read-only here)."),
            _run("DRIVING_DIRECTION", "enum", "driving_direction", "", "CCW/CW. Fixes which side ray is the OUTER wall (CCW=left/90, CW=right/270) and the along-track sign. Wrong => lateral & along-track flipped. Click Re-run fix after changing.", True, options=["CCW", "CW"], setter=_set_driving_direction),
            _run("INITIAL_SECTION", "enum", "initial_section", "", "Which leg the robot starts on. Can't be sensed (the 4-candidate ambiguity). Informational after start; edit the live 'section' below to move the tracked marker.", True, options=["S", "E", "N", "W"]),
        ]},
        {"group": "Pose estimate (live)", "note": "The estimator's current state. In mock mode the simulator overwrites these each frame unless you set freeze_pose=true.", "params": [
            _run("freeze_pose", "bool", "freeze_pose", "", "Pause the mock's odometry/heading feed so hand-edited pose values stick (no effect in real mode -- nothing feeds it there)."),
            _pose("section", "section", "", "Which leg the live tracked pose is on. Sets the estimator's section directly.", options=["S", "E", "N", "W"], kind="enum"),
            _pose("along_mm", "along_mm", "mm", "Along-track position of the tracked pose (distance along the section from its CCW-first corner)."),
            _pose("lateral_mm", "lateral_mm", "mm", "Cross-lane position of the tracked pose (distance from the OUTER wall)."),
            _pose("heading_deg", "heading_deg", "deg", "Tracked heading (0=east,90=north). Wrapped to 0..360."),
        ]},
        {"group": "Field geometry (mat_geometry.py)", "note": "Measure against your real mat. Editing OUTER/LANE/margin re-derives island + safe zone and redraws the mat.", "params": [
            _geo_core("OUTER_SIZE_MM", "mm", "Outer wall inside length = section length. Used in along-track (CW: along=OUTER-forward). Wrong => along-track biased, markers off the walls.", True),
            _geo_core("LANE_WIDTH_MM", "mm", "Outer-to-island gap. Start fix checks left+right ~= this. Wrong => good scans rejected / bad accepted.", True),
            _geo_ro("INNER_SIZE_MM", "mm", "Island size, DERIVED = OUTER - 2*LANE."),
            _geo_ro("ISLAND_MIN_MM", "mm", "Island near edge (derived)."),
            _geo_ro("ISLAND_MAX_MM", "mm", "Island far edge (derived)."),
            _geo_core("SAFE_FIX_MARGIN_MM", "mm", "How far past a corner (into the mid-edge band) before a fix is trusted. Bigger = safer but narrower start band."),
            _geo_ro("SAFE_FIX_ALONG_MIN_MM", "mm", "Valid mid-edge zone lower bound (derived). Start inside [min,max]."),
            _geo_ro("SAFE_FIX_ALONG_MAX_MM", "mm", "Valid mid-edge zone upper bound (derived)."),
        ]},
        {"group": "LIDAR hardware (config.py)", "note": "Applied on the next scan (real mode).", "params": [
            _cfg("LIDAR_PORT", "str", "", "Serial device. Wrong => no data. Confirm with `ls /dev/ttyUSB*`. (Takes effect on reconnect/restart.)"),
            _cfg("LIDAR_BAUDRATE", "int", "baud", "C1 default 460800. (Takes effect on reconnect/restart.)"),
            _cfg("LIDAR_SCAN_TIMEOUT_S", "float", "s", "How long to assemble a scan. Too low drops points; too high adds latency."),
        ]},
        {"group": "LIDAR angle calibration (config.py)", "note": "Applied LIVE to the point cloud on the next frame -- watch the cloud rotate as you edit.", "params": [
            _cfg("LIDAR_ANGLE_ZERO_OFFSET_DEG", "float", "deg", "Rotates raw angle so 0=forward, 90=left, 270=right. Wrong => 90/270 searches look the wrong way and the fix fails/mislabels walls.", True),
            _cfg("LIDAR_ANGLE_SIGN", "int", "+/-1", "Flip if your unit's angle runs clockwise vs the code's CCW convention. Wrong => left/right swapped.", True, options=[1, -1]),
        ]},
        {"group": "Sensor lever-arm (config.py)", "note": "Your measured LIDAR mount offset from the path-planner reference point.", "params": [
            _cfg("LIDAR_OFFSET_FORWARD_MM", "float", "mm", "Shifts the along-track (forward) read."),
            _cfg("LIDAR_OFFSET_LATERAL_MM", "float", "mm", "Shifts the cross-lane read. Likely the source of a small lane-sum gap (yours read ~915 vs 1000)."),
        ]},
        {"group": "Rear blind arc (config.py) -- predicted overlay only", "note": "Does NOT affect the fix; shapes each candidate's predicted cloud. Click Re-run fix to rebuild the overlay after editing.", "params": [
            _cfg("REAR_BLIND_ARC_CENTER_DEG", "float", "deg", "Robot-relative centre of the chassis-blocked wedge (180=straight back)."),
            _cfg("REAR_BLIND_ARC_WIDTH_DEG", "float", "deg", "Total width of the wedge (yours was ~107deg)."),
        ]},
        {"group": "Fix sanity tolerances (config.py)", "note": "Applied on the next fix. Click Re-run fix to re-evaluate the start-of-run scan.", "params": [
            _cfg("LANE_WIDTH_TOLERANCE_MM", "float", "mm", "Noise margin on left+right ~= LANE_WIDTH. Too tight => good scans rejected; too loose => an occluded side wall accepted (wrong lateral).", True),
            _cfg("SIDE_SEARCH_WINDOW_DEG", "float", "deg", "How far from 90/270 to accept a lane-wall cluster. Too tight => 'no wall for left(90)'; too wide => wrong surface near a corner."),
            _cfg("FRONT_BACK_SEARCH_WINDOW_DEG", "float", "deg", "+/- window around 0deg to gather the forward (along-track) range. Too tight => misses it; too wide => biases along-track."),
            _cfg("BROADSIDE_HEADING_TOLERANCE_DEG", "float", "deg", "Only the post-corner broadside RE-fix. Not used by the start-of-run fix."),
            _cfg("SECTION_LENGTH_TOLERANCE_MM", "float", "mm", "DEPRECATED / unused (old broadside-start assumption). Ignore."),
        ]},
        {"group": "Clustering: scan -> walls/pillars (scan_processing.py)", "note": "Applied LIVE on the next scan -- the classified cloud updates as you edit.", "params": [
            _spc("MIN_RANGE_MM", "float", "mm", "Drop points closer than this (chassis / sensor floor)."),
            _spc("MAX_RANGE_MM", "float", "mm", "Drop implausibly far returns."),
            _spc("MIN_QUALITY", "int", "", "Drop low-quality returns. Raise above 0 if weak points form phantom clusters."),
            _spc("MAX_ANGLE_GAP_DEG", "float", "deg", "Break a cluster on an angular gap bigger than this. Too small => a dropout splits a close wall; too big => merges across real gaps."),
            _spc("MAX_CHORD_JUMP_MM", "float", "mm", "Break a cluster on a straight-line jump bigger than this. Too small fragments grazing walls; too big merges wall into a pillar."),
            _spc("MAX_CLUSTER_SPAN_DEG", "float", "deg", "Hard cap on one cluster's angular width. A CLOSE lane wall subtends a wide arc and can hit this -- raise (120-130) if close side walls get truncated; too high merges two walls at a corner.", True),
            _spc("WALL_MIN_ARC_LENGTH_MM", "float", "mm", "Min arc length to call a cluster a wall. Too high drops short/far segments; too low calls a big pillar a wall."),
            _spc("WALL_MAX_FLATNESS_RESIDUAL_MM", "float", "mm", "Max RMS off the fitted line to still be 'flat'. Rejects the forward corner-blend. Too low rejects a noisy wall; too high lets a curved corner pass as flat.", True),
            _spc("PILLAR_MIN_ARC_LENGTH_MM", "float", "mm", "Lower arc bound for a pillar. Obstacle detection only."),
            _spc("PILLAR_MAX_ARC_LENGTH_MM", "float", "mm", "Upper arc bound for a pillar. Obstacle detection only."),
            _spc("MIN_POINTS_PER_PILLAR", "int", "", "Reject singleton-point pillar fragments. Obstacle detection only."),
        ]},
        {"group": "Overlay & server", "note": "", "params": [
            _run("predict_n_points", "int", "predict_n_points", "", "Angular resolution of each candidate's predicted cloud. Higher = denser overlay, bigger payload. Purely visual; click Re-run fix to apply."),
            _cfg("STREAM_HZ", "int", "Hz", "Dashboard stream/refresh rate. No effect on localization; lower if the Pi is loaded."),
            _ro("DASHBOARD_HOST", "str", (lambda: config.DASHBOARD_HOST), "", "Server bind address (read-only; needs restart)."),
            _ro("DASHBOARD_PORT", "int", (lambda: config.DASHBOARD_PORT), "", "Server port (read-only; needs restart)."),
        ]},
    ]


# Snapshot of file defaults, captured once at import, for the Reset button.
_DEFAULTS = {}
for _grp in _param_registry():
    for _p in _grp["params"]:
        if _p["set"] is not None:
            try:
                _DEFAULTS[_p["name"]] = _p["get"]()
            except Exception:
                pass


def _find_spec(name):
    for grp in _param_registry():
        for p in grp["params"]:
            if p["name"] == name:
                return p
    return None


def _rerun_start_fix():
    """Re-take the start-of-run scan with the CURRENT parameters and rebuild the
    fix + candidates + predicted overlay, without restarting the server."""
    est = _RT["estimator"]
    dirn = _RT["driving_direction"]
    if _RT["mode"] == "mock" and _RT["sim"] is not None:
        sim = _RT["sim"]
        raw = sim.current_scan()
    elif _RT["lidar"] is not None:
        raw = _RT["lidar"].get_latest_scan()
    else:
        return {"ok": False, "reason": "no scan source available"}
    clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
    fix = compute_start_of_run_fix(clusters, driving_direction=dirn)
    if fix.ok:
        cands = candidate_start_positions(fix.along_mm, fix.lateral_from_outer_mm, driving_direction=dirn)
        payload = _candidates_to_list(cands)
        if est is not None:
            est.state.along_mm = fix.along_mm
            est.state.lateral_mm = fix.lateral_from_outer_mm
            est.state.initialized = True
            est._recompute_xy()
    else:
        payload = []
    with _state_lock:
        _RT["start_fix"] = fix
        _RT["start_candidates_payload"] = payload
    return {"ok": fix.ok, "reason": fix.reason,
            "along_mm": fix.along_mm, "lateral_mm": fix.lateral_from_outer_mm}


@app.route("/api/tuning")
def api_tuning():
    groups = []
    for grp in _param_registry():
        params = []
        for p in grp["params"]:
            try:
                val = p["get"]()
            except Exception:
                val = None
            params.append({"name": p["name"], "value": val, "unit": p["unit"],
                           "implication": p["implication"], "critical": p["critical"],
                           "kind": p["kind"], "options": p["options"],
                           "editable": p["set"] is not None,
                           "geo": p["name"] in GEO_CORE_NAMES})
        groups.append({"group": grp["group"], "note": grp["note"], "params": params})
    return {"groups": groups}


@app.route("/api/param", methods=["POST"])
def api_param():
    from flask import request
    body = request.get_json(force=True, silent=True) or {}
    name, value = body.get("name"), body.get("value")
    spec = _find_spec(name)
    if spec is None or spec["set"] is None:
        return {"ok": False, "error": f"unknown or read-only parameter {name!r}"}, 400
    try:
        spec["set"](value)
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}, 400
    try:
        newval = spec["get"]()
    except Exception:
        newval = None
    return {"ok": True, "name": name, "value": newval, "geo": name in GEO_CORE_NAMES}


@app.route("/api/refit", methods=["POST"])
def api_refit():
    return _rerun_start_fix()


@app.route("/api/reset", methods=["POST"])
def api_reset():
    restored = 0
    for grp in _param_registry():
        for p in grp["params"]:
            if p["set"] is not None and p["name"] in _DEFAULTS:
                try:
                    p["set"](_DEFAULTS[p["name"]])
                    restored += 1
                except Exception:
                    pass
    _recompute_geo_derived()
    return {"ok": True, "restored": restored}


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
