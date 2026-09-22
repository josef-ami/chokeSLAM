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
import lane_frame as lf
import mat_geometry as geo
import seat_occupancy as so
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
    # --- seat occupancy (the LANE view) ---------------------------------
    # `seat_params` is the live DetectParams the tuning panel edits.
    # `seat_memory` is the sticky map: once a seat has been decided
    # OCCUPIED/EMPTY that verdict is held, because a seat that was resolved
    # cleanly at 1.5 m is not re-opened just because the robot has since
    # driven past it into the rear blind wedge. Cleared on every corner, since
    # a new section means a new set of six seats.
    "seat_params": so.DetectParams(),
    "seat_sticky": True,
    "seat_memory": {},          # seat index -> {"state", "reason", "at"}
    "seat_yaw_offset_deg": 0.0,  # added to the lane-local yaw before detection
    # Which pose the seat detector is fed. "estimated" is what the robot will
    # actually have. "true" is mock-only and exists to separate two failure
    # modes that otherwise look identical on screen: a seat called wrongly
    # because the DETECTOR is wrong, versus one called wrongly because the
    # POSE handed to it is wrong. Flip between them and whichever one changes
    # the answer is the one at fault. (The detector is clean to ~50mm of
    # position error and degrades past ~75mm; the mock's estimator holds
    # lateral at a constant 500mm until a LIDAR fix succeeds, while the
    # simulated robot wanders +/-120mm, so "estimated" is routinely outside
    # that budget in mock mode.)
    "seat_pose_source": "true",
}


class _ScanPose:
    """A pose whose heading is the orientation a particular scan was captured
    at, rather than the robot's nominal driving heading. Used on the frames
    where the mock goes briefly broadside to re-anchor."""

    __slots__ = ("section", "x_mm", "y_mm", "heading_deg")

    def __init__(self, section, x_mm, y_mm, heading_deg):
        self.section, self.x_mm, self.y_mm, self.heading_deg = \
            section, x_mm, y_mm, heading_deg


def _seat_memory_reset(why: str = ""):
    _RT["seat_memory"] = {}
    if why:
        print(f"[seats] memory cleared ({why})")


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
    # heading_deg is a GRID BEARING (0=N, CW); convert to a maths angle so the
    # robot-frame point (x=forward, y=left) rotates into world x/y correctly.
    h = math.radians(90.0 - heading_deg)
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


def _rulebook_pillar_positions(section, direction, seat_indices):
    """GLOBAL (x, y) of the given seat indices in `section`, using the
    rulebook seat table from seat_occupancy.py.

    Used only by the mock demo. simulation.MockRobotSimulator picks its default
    pillars from mat_geometry.all_slots(), whose coordinates are Finding 1 --
    they disagree with rulebook Figure 11 on all 24 slots. Overriding the
    simulator's `pillars` attribute at runtime (rather than editing
    simulation.py, which is left untouched) puts the mock pillars where the
    detector is actually looking, so the lane view demonstrates something.
    """
    seats = {s.index: s for s in so.seats()}
    return [lf.lane_to_global(section, direction, seats[i].x_mm, seats[i].y_mm)
            for i in seat_indices if i in seats]


def _truth_occupied_indices(pillars, section, direction, tol_mm=60.0):
    """Which seat indices the mock's real pillars are standing on -- ground
    truth for the lane view's TRUTH column. Returns None in real mode, where
    there is nothing to compare against."""
    if not pillars:
        return []
    out = []
    for s in so.seats():
        gx, gy = lf.lane_to_global(section, direction, s.x_mm, s.y_mm)
        for p in pillars:
            if math.hypot(p.x_mm - gx, p.y_mm - gy) <= tol_mm:
                out.append(s.index)
                break
    return out


def _seat_view(clusters, est, direction, truth_indices=None, pose_source="estimated"):
    """Everything the LANE view needs: the lane-local pose, the six seats with
    their live and sticky verdicts, and the affine transform the browser uses
    to re-project the point cloud it already has.

    The lane-local pose is derived from the estimator's GLOBAL x/y (via
    lane_frame.pose_to_lane), not from its along_mm/lateral_mm, so this view is
    a pure re-projection of the point the mat view draws. Both views therefore
    agree with each other even while the localization findings are open.
    """
    p = _RT["seat_params"]
    section = est.section
    x_l, y_l, yaw = lf.pose_to_lane(section, direction, est.x_mm, est.y_mm,
                                     est.heading_deg)
    yaw = lf.wrap180(yaw + _RT["seat_yaw_offset_deg"])
    sx, sy = lf.sensor_origin(x_l, y_l, yaw,
                              p.lidar_offset_forward_mm, p.lidar_offset_lateral_mm)

    points = [pt for c in clusters for pt in c.points]
    readings = so.detect_seat_occupancy(points, x_l, y_l, robot_yaw_deg=yaw, params=p)

    memory = _RT["seat_memory"]
    seats_out = []
    counts = {"occupied": 0, "empty": 0, "unknown": 0}
    for r in readings:
        live = r.state.value
        if _RT["seat_sticky"]:
            prev = memory.get(r.seat.index)
            if r.state is not so.Occupancy.UNKNOWN:
                if prev is None or prev["state"] != live:
                    memory[r.seat.index] = {"state": live, "reason": r.reason,
                                             "at": round(y_l, 0)}
            held = memory.get(r.seat.index)
        else:
            held = {"state": live, "reason": r.reason, "at": round(y_l, 0)} \
                if r.state is not so.Occupancy.UNKNOWN else None
        shown = (held or {}).get("state", "unknown")
        counts[shown] += 1
        rnd = lambda v: None if v is None else round(v, 1)
        seats_out.append({
            "index": r.seat.index, "name": r.seat.name,
            "x_mm": r.seat.x_mm, "y_mm": r.seat.y_mm,
            "state": shown,                 # sticky (what the planner should use)
            "live_state": live,             # this scan alone
            "reason": r.reason,
            "held_at_mm": (held or {}).get("at"),
            "bearing_deg": rnd(r.predicted_bearing_deg),
            "rel_bearing_deg": rnd(r.predicted_rel_bearing_deg),
            "lidar_angle_deg": rnd(r.predicted_lidar_angle_deg),
            "half_width_deg": rnd(r.search_half_width_deg),
            "expected_centre_mm": rnd(r.expected_centre_mm),
            "expected_face_mm": rnd(r.expected_face_mm),
            "tolerance_mm": rnd(r.range_tolerance_mm),
            "observed_mm": rnd(r.observed_range_mm),
            "residual_mm": rnd(r.residual_mm),
            "n_points": r.n_points_in_window,
            "expected_hits": r.expected_hits,
            "truth": None if truth_indices is None else (r.seat.index in truth_indices),
        })

    return {
        "section": section, "direction": direction,
        "grid_north_deg": round(lf.grid_north_bearing(section, direction), 1),
        "outer_wall_side": "right" if lf.outer_wall_is_on_the_right(direction) else "left",
        "lane_length_mm": lf.OUTER_SIZE_MM, "lane_width_mm": lf.LANE_WIDTH_MM,
        "pose": {"x_mm": round(x_l, 1), "y_mm": round(y_l, 1), "yaw_deg": round(yaw, 1)},
        "sensor": {"x_mm": round(sx, 1), "y_mm": round(sy, 1)},
        "affine": [round(v, 6) for v in lf.lane_affine(section, direction)],
        "blind_arc": {"center_deg": p.blind_arc_center_deg, "width_deg": p.blind_arc_width_deg},
        "seats": seats_out,
        "counts": counts,
        "sticky": _RT["seat_sticky"],
        "pose_source": pose_source,
        "yaw_offset_deg": _RT["seat_yaw_offset_deg"],
    }


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
    # Put the mock's pillars on RULEBOOK seats (see _rulebook_pillar_positions)
    # so the lane view has something real to find. The simulator's own defaults
    # come from mat_geometry.all_slots(), which is Finding 1. Runtime override
    # only -- simulation.py is not edited.
    # A plausible draw: a few seats filled in each of the four sections (the
    # rules allow up to 7 red + 7 green across the 24 seats).
    MOCK_LAYOUT = {"S": ((0, "red"), (3, "green"), (5, "red")),
                   "E": ((1, "green"), (4, "red")),
                   "N": ((2, "red"), (3, "green"), (4, "green")),
                   "W": ((0, "green"), (5, "red"))}
    from simulation import Pillar as _Pillar
    sim.pillars = []
    for _sec, _spec in MOCK_LAYOUT.items():
        _idx = [i for i, _ in _spec]
        for (gx, gy), (_, col) in zip(
                _rulebook_pillar_positions(_sec, DRIVING_DIRECTION, _idx), _spec):
            sim.pillars.append(_Pillar(gx, gy, col))
    print(f"[seats] mock pillars placed on rulebook seats: "
          f"{ {k: [i for i, _ in v] for k, v in MOCK_LAYOUT.items()} }")

    # simulation.simulate_scan ray-casts a full, unobstructed 360 deg -- it
    # models no chassis occlusion at all. Leaving the detector's blind wedge at
    # its real-hardware default here would throw away 105 deg of a sweep that
    # actually has data in it, so it's zeroed for the mock only. On real
    # hardware this must go back to the measured wedge (see config.py), because
    # there the returns genuinely are not there.
    _RT["seat_params"].blind_arc_width_deg = 0.0
    print("[seats] blind_arc_width_deg = 0 for the mock (simulate_scan casts a "
          "full 360 deg); restore the measured wedge on real hardware")
    # NOTE the yaw is left HONEST (seat_yaw_offset_deg = 0). The simulator
    # reports a heading 180 deg from its own direction of travel -- Finding 2,
    # simulation.true_heading_deg and localization.driving_heading_deg share
    # the inverted formula -- and casts its scans at that same heading, so pose
    # and scan stay mutually consistent and the detector works correctly on
    # them. What you will SEE in the lane view is the robot driving up the lane
    # while pointing down it. That is Finding 2, drawn to scale; it is not a
    # fault in the view, and it should disappear the moment that formula is
    # corrected.
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
            # New section => a different six seats. Nothing learned about the
            # last section's seats carries over.
            _seat_memory_reset(f"corner completed, now on section {estimator.state.section}")

        # The orientation the scan below is actually taken at. On the frames
        # where the robot goes briefly broadside for its re-anchor, the scan is
        # NOT taken at the driving heading -- and anything that interprets a
        # scan against a pose has to use the heading that scan was captured at,
        # not the one the robot is nominally driving. (The lane view is what
        # surfaced this: seats with pillars on them were reading EMPTY on
        # exactly the frames a broadside fix fired.)
        if awaiting_fix and estimator.in_safe_fix_zone():
            # Briefly broadside for the re-anchor, then straighten back out.
            raw, broadside_heading = sim.broadside_scan()
            estimator.update_heading(broadside_heading)
            clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            last_fix = estimator.apply_lidar_fix(clusters)
            estimator.update_heading(heading_deg)  # restore driving heading
            awaiting_fix = False
            scan_heading = broadside_heading
        else:
            raw = sim.current_scan()
            clusters = process_scan(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            scan_heading = None   # scan matches the pose's own heading

        est = estimator.state
        true = sim.true_state()
        points = _clusters_to_points(clusters, est.x_mm, est.y_mm, est.heading_deg)
        # In mock mode the seat detector can be fed either pose -- see
        # _RT["seat_pose_source"]. `true` also carries section/x/y/heading, so
        # it drops straight into _seat_view in place of the estimator state.
        _src = _RT["seat_pose_source"]
        _seat_pose = true if _src == "true" else est
        if scan_heading is not None:
            # Same position, but the heading this scan was actually captured at.
            _seat_pose = _ScanPose(_seat_pose.section, _seat_pose.x_mm,
                                   _seat_pose.y_mm, scan_heading)
        seat_view = _seat_view(clusters, _seat_pose,
                               _RT["driving_direction"],
                               truth_indices=_truth_occupied_indices(
                                   sim.pillars, est.section, _RT["driving_direction"]),
                               pose_source=_src)

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
            "seat_view": seat_view,
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
    #   - call estimator.update_heading(imu_bearing_deg) whenever you get a new
    #     IMU reading. Headings here are GRID BEARINGS (0=north, clockwise),
    #     the same convention as a compass/IMU, so a north-referenced IMU feeds
    #     in directly. If your IMU's zero isn't grid north, add the fixed
    #     offset once at this boundary.
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
        # No ground truth in real mode -- truth_indices stays None, and the
        # lane view's TRUTH column shows a dash instead of a comparison.
        seat_view = _seat_view(clusters, est, _RT["driving_direction"],
                               truth_indices=None, pose_source="estimated")
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
            "seat_view": seat_view,
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
        # --- lane view -----------------------------------------------------
        # `slots` above is mat_geometry's table (Finding 1: it disagrees with
        # rulebook Figure 11 on all 24). `lane.seats` is the rulebook table the
        # detector actually uses. Both are sent so the mat view can draw the
        # old one faintly and the difference stays visible rather than being
        # quietly papered over.
        "lane": {
            "length_mm": lf.OUTER_SIZE_MM,
            "width_mm": lf.LANE_WIDTH_MM,
            "section_y_min_mm": min(so.SEAT_Y_MM),
            "section_y_max_mm": max(so.SEAT_Y_MM),
            "seat_x_mm": list(so.SEAT_X_MM),
            "seat_y_mm": list(so.SEAT_Y_MM),
            "seat_size_mm": so.PILLAR_SIDE_MM,
            "seat_circle_dia_mm": 85.0,
            "seats": [{"index": s.index, "name": s.name,
                        "x_mm": s.x_mm, "y_mm": s.y_mm} for s in so.seats()],
        },
        "rulebook_slots_global": [
            {"x_mm": round(gx, 1), "y_mm": round(gy, 1), "section": sec, "index": s.index}
            for sec in lf.SECTIONS for s in so.seats()
            for gx, gy in [lf.lane_to_global(sec, _RT["driving_direction"], s.x_mm, s.y_mm)]
        ],
        "convention_warnings": lf.convention_disagreements(),
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


def _seat(name, kind, unit, imp, crit=False):
    """A field of the live seat_occupancy.DetectParams instance."""
    return {"name": name, "kind": kind, "unit": unit, "implication": imp, "critical": crit,
            "options": None, "get": (lambda: getattr(_RT["seat_params"], name)),
            "set": (lambda v: setattr(_RT["seat_params"], name, _coerce(kind, v)))}


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
            _pose("heading_deg", "heading_deg", "deg", "Tracked heading as a GRID BEARING (0=north, 90=east, clockwise). Wrapped to 0..360."),
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
        {"group": "Seat occupancy (seat_occupancy.py)", "note": "Drives the LANE view. Applied LIVE on the next scan. Seat coordinates come from rulebook Figure 11, not from mat_geometry's slot table.", "params": [
            _run("seat_pose_source", "enum", "seat_pose_source", "", "Which pose the detector is fed. MOCK ONLY: 'true' uses the simulator's ground truth, 'estimated' uses the pose estimator. Flip between them to tell a detector fault from a localization fault -- whichever one changes the verdict is the one at fault. Real mode always uses 'estimated'.", True, options=["true", "estimated"]),
            _run("seat_sticky", "bool", "seat_sticky", "", "Hold each seat's first OCCUPIED/EMPTY verdict instead of re-deciding every scan. A seat resolved cleanly at 1.5m shouldn't re-open just because you've driven past it into the blind wedge. Cleared at every corner."),
            _run("seat_yaw_offset_deg", "float", "seat_yaw_offset_deg", "deg", "Added to the lane-local yaw before detection. Non-zero if your IMU's zero isn't grid north. In MOCK mode this defaults to 180 to compensate Finding 2 (simulation.py reports a heading opposite its own travel) -- set it to 0 once that's fixed.", True),
            _seat("angular_margin_deg", "float", "deg", "Search half-window around each seat's predicted bearing, ON TOP of the pillar's own subtense. Size it from heading uncertainty. Too tight => real pillars missed on a yawing car; too wide => a wall at the right range can enter the window.", True),
            _seat("range_tol_mm", "float", "mm", "Absolute range agreement. Dominated by POSITION error, not sensor noise -- measured clean to 50mm pose error, degrading past 75mm.", True),
            _seat("range_tol_frac", "float", "", "Range-proportional part of the tolerance, so a far seat is judged more leniently than a near one."),
            _seat("min_points", "int", "", "Minimum contiguous returns to accept a pillar face. 1 lets a single stray point become an obstacle."),
            _seat("max_width_factor", "float", "x", "Reject a run wider than this multiple of the pillar's predicted angular width -- that's a wall passing through the right range, not a 50mm pillar."),
            _seat("max_range_step_mm", "float", "mm", "Consecutive points further apart than this in range are not the same surface. Sets where a candidate run is broken."),
            _seat("blind_arc_center_deg", "float", "deg", "Robot-relative centre of the chassis-blocked wedge. Seats inside it report UNKNOWN, never EMPTY."),
            _seat("blind_arc_width_deg", "float", "deg", "Total width of that wedge. Measure it off a raw scan dump on your unit."),
            _seat("min_expected_hits", "float", "returns", "Before calling a seat EMPTY, require that a pillar standing there would have produced at least this many returns at the scan's own measured angular resolution. Below it the verdict is UNKNOWN. Guards against a far pillar whose one-or-two returns were lost to dropout reading as 'empty' -- and EMPTY is the verdict that gets held.", True),
            _seat("min_observable_face_mm", "float", "mm", "A seat nearer than this is inside the sensor dead zone -> UNKNOWN. Without it a seat passing beside the robot reads EMPTY and un-sets an earlier correct OCCUPIED.", True),
            _seat("seat_position_slack_mm", "float", "mm", "Slack for a sign nudged inside its 85mm circle, plus mat print tolerance."),
            _seat("lidar_offset_forward_mm", "float", "mm", "Sensor lever arm, forward of the pose reference point. Separate from config.LIDAR_OFFSET_* -- this one shifts the SEAT vectors."),
            _seat("lidar_offset_lateral_mm", "float", "mm", "Sensor lever arm, to the robot's LEFT. With a 300x200mm vehicle this is not a rounding error."),
            _seat("min_range_mm", "float", "mm", "Returns closer than this are discarded before detection."),
            _seat("max_range_mm", "float", "mm", "Returns further than this are discarded before detection."),
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


@app.route("/api/seats/reset", methods=["POST"])
def api_seats_reset():
    """Forget every held seat verdict for the current section and start
    deciding again from the next scan. Use it after moving pillars on the
    bench, or after changing a detector threshold, so you're not looking at a
    verdict formed under the old settings."""
    _seat_memory_reset("manual reset from the dashboard")
    return {"ok": True}


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
