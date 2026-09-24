"""
Checkpoint D: pillar colour identification (color_id.py, lane_tracker colour
requests, camera_sim.py). Run: python3 test_color_id.py
"""
from __future__ import annotations

import math
import random
from contextlib import contextmanager

import numpy as np

import camera_sim
import color_id
import config
# Checkpoint E set the real lever arm (LIDAR 134.6 mm ahead of the rear axle, from CAD).
# This suite's own ray-casting / worlds were written for the sensor AT the pose point,
# so it pins the lever arm to 0 (the lever arm itself is tested in test_init.test_lever_arm).
config.LIDAR_OFFSET_FORWARD_MM = 0.0
config.LIDAR_OFFSET_LATERAL_MM = 0.0
# ... and the camera offsets, which were placeholders equal to the LIDAR's (0) when these tests were written
config.CAMERA_OFFSET_FORWARD_MM = 0.0
config.CAMERA_OFFSET_LATERAL_MM = 0.0
import simulation as sim


@contextmanager
def cfg(**kw):
    old = {k: getattr(config, k) for k in kw}
    for k, v in kw.items():
        setattr(config, k, v)
    try:
        yield
    finally:
        for k, v in old.items():
            setattr(config, k, v)


def test_projection_matches_the_polynomial():
    """bearing_to_pixel (cv2.fisheye.projectPoints) vs the equidistant formula
    written out by hand: u = cx + fx * t (1 + D0 t^2 + D1 t^4 + D2 t^6 + D3 t^8)."""
    K, D = config.CAMERA_K, config.CAMERA_D
    worst = 0.0
    for deg in np.arange(-45.0, 45.01, 2.5):
        t = abs(math.radians(deg))
        r = K[0][0] * t * (1 + D[0] * t ** 2 + D[1] * t ** 4 + D[2] * t ** 6 + D[3] * t ** 8)
        u_hand = K[0][2] + math.copysign(r, deg)
        u, v = color_id.bearing_to_pixel(deg)
        worst = max(worst, abs(u - u_hand), abs(v - K[1][2]))
    assert worst < 1e-6, worst
    print(f"PASS  test_projection_matches        -45..+45 deg on the horizon: projectPoints == hand polynomial "
          f"(max diff {worst:.1e} px)")


def test_view_limits():
    """The supplied calibration: image edges near +46 / -50 deg, the polynomial
    folds at ~61 deg. A pillar past the fold is OUT OF VIEW, never given the
    folded (valid-looking) column."""
    fold = color_id.fold_deg()
    assert 61.0 < fold < 61.6, fold
    u70, _ = color_id.bearing_to_pixel(70.0)
    assert 0 <= u70 < 640, u70                                  # the trap: 70 deg "lands" in the image
    roi, why = color_id.pillar_roi(70.0, 600.0)
    assert roi is None and "lens model" in why, why
    roi, why = color_id.pillar_roi(55.0, 600.0)
    assert roi is None and "outside" in why, why
    roi, why = color_id.pillar_roi(40.0, 600.0)
    assert roi is not None, why
    roi, why = color_id.pillar_roi(0.0, 60.0)                    # close: the pillar's foot beyond the fold
    assert roi is None, roi
    print(f"PASS  test_view_limits               fold at {fold:.2f} deg; 70 deg would project to column {u70:.0f} "
          f"but is rejected; 55 deg outside the image; 40 deg in view")


def _grid(sign=+1, rotate=True, seed=7, n=400):
    """Random camera poses and pillars in front of it (range 150-1700 mm,
    bearing -48..48 deg), rendered and classified at the exact pose."""
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)
    res = {"right": 0, "wrong": 0, "none": 0, "out": 0, "ranges_none": []}
    with cfg(CAMERA_BEARING_SIGN=sign, CAMERA_ROTATE_180=rotate):
        for _ in range(n):
            cx, cy, b = rng.uniform(300, 700), rng.uniform(200, 900), rng.uniform(-20, 20)
            rngm, th = rng.uniform(150, 1700), rng.uniform(-48, 48)
            px = cx + rngm * math.sin(math.radians(b + th))
            py = cy + rngm * math.cos(math.radians(b + th))
            colour = rng.choice(["red", "green"])
            frame = camera_sim.render(cx, cy, b, [sim.Pillar(px, py, colour)], nrng)
            roi, _ = color_id.pillar_roi(th, rngm - 25.0)
            if roi is None:
                res["out"] += 1
                continue
            got, _, _ = color_id.classify(color_id.correct_frame(frame), roi)
            if got is None:
                res["none"] += 1
                res["ranges_none"].append(round(rngm))
            elif got == colour:
                res["right"] += 1
            else:
                res["wrong"] += 1
    return res


def test_classify_synthetic_grid():
    """400 random single-pillar views, all four mount conventions: never wrong."""
    for sign in (+1, -1):
        for rotate in (True, False):
            r = _grid(sign, rotate)
            assert r["wrong"] == 0, (sign, rotate, r)
            assert r["right"] >= 0.9 * (r["right"] + r["none"]), (sign, rotate, r)
            print(f"PASS  test_classify_grid sign {sign:+d} rot180 {str(rotate):5s} {r['right']} right, 0 wrong, "
                  f"{r['none']} not confident (ranges {sorted(r['ranges_none'])[:6]}), {r['out']} out of view")


def test_wrong_mount_config_is_caught():
    """Frames from a camera mounted one way, read with the other sign: the ROI
    lands on the wrong side and reads nothing (not a wrong colour)."""
    rng = random.Random(3)
    nrng = np.random.default_rng(3)
    right = none = wrong = 0
    for _ in range(100):
        th, d = rng.uniform(12, 40), rng.uniform(300, 1200)
        colour = rng.choice(["red", "green"])
        with cfg(CAMERA_BEARING_SIGN=+1):
            frame = camera_sim.render(500, 500, 0, [sim.Pillar(500 + d * math.sin(math.radians(th)),
                                                               500 + d * math.cos(math.radians(th)), colour)], nrng)
        with cfg(CAMERA_BEARING_SIGN=-1):
            roi, _ = color_id.pillar_roi(th, d - 25.0)
            got, _, _ = color_id.classify(color_id.correct_frame(frame), roi)
        if got is None:
            none += 1
        elif got == colour:
            right += 1
        else:
            wrong += 1
    assert right == 0 and wrong == 0, (right, wrong, none)
    print(f"PASS  test_wrong_mount_config        sign read the wrong way: {none}/100 not confident, 0 read "
          f"(the bench sign-check is what finds this)")


def test_tracker_end_to_end():
    """run_mock with the camera: init + tracker + entry re-checks + colour
    requests served by frames rendered from the true pose. Both directions,
    several seeds, 600 and 1000 mm/s, frames handed over up to 150 ms late."""
    tot = {"right": 0, "wrong": 0, "unknown": 0, "pending": 0, "not_a_pillar": 0}
    unknown_reasons = []
    runs = 0
    for direction in ("CCW", "CW"):
        for seed in range(1, 7):
            for speed, delay in ((600.0, 0.0), (1000.0, 0.15)):
                r = sim.run_mock(direction, seed % 4, seed=seed, laps=1, speed_mm_s=speed, camera_hz=15,
                                 camera_delay_s=delay)
                if not r["ok"]:
                    continue
                runs += 1
                c = r["colors"]
                for k in tot:
                    tot[k] += c[k]
                unknown_reasons += [d[4] for d in c["details"] if d[2] == "unknown"]
    assert tot["wrong"] == 0 and tot["not_a_pillar"] == 0, tot
    assert tot["pending"] == 0, tot
    assert tot["right"] >= 0.8 * (tot["right"] + tot["unknown"]), tot
    print(f"PASS  test_tracker_end_to_end        {runs} one-lap runs: {tot['right']} colours right, 0 wrong, "
          f"{tot['unknown']} UNKNOWN, 0 left pending")
    for why in unknown_reasons[:4]:
        print(f"        UNKNOWN because: {why[:150]}")


def test_pose_at_capture_time():
    """#62: a frame is judged from the pose at its CAPTURE time. The same late
    frame judged from the robot's pose at processing time (what the draft's
    'bearing stored on the seat' or 'pose now' would do) misses the pillar."""
    import lane_frame as lf
    rng = np.random.default_rng(5)
    seat_x, seat_y = 400.0, 1500.0
    colour = "red"
    sec, direction = "S", "CCW"
    gx, gy = lf.lane_to_global(sec, direction, seat_x, seat_y)
    pillar = sim.Pillar(gx, gy, colour)
    ok_then = ok_now = 0
    for k in range(20):
        y_cap = 700.0 + 20 * k
        x = 500.0
        # capture: camera at lane (x, y_cap), heading along the lane
        cgx, cgy = lf.lane_to_global(sec, direction, x, y_cap)
        brg = lf.yaw_to_heading(0.0, sec, direction)
        frame = color_id.correct_frame(camera_sim.render(cgx, cgy, brg, [pillar], rng))
        for pose_y, tag in ((y_cap, "then"), (y_cap + 250.0, "now")):     # 250 mm later (1 m/s, 0.25 s)
            v = color_id.seat_view(x, pose_y, 0.0, direction, seat_x, seat_y)
            roi, _ = color_id.pillar_roi(v.theta_deg, v.face_mm)
            got = color_id.classify(frame, roi)[0] if roi is not None else None
            if got == colour:
                if tag == "then":
                    ok_then += 1
                else:
                    ok_now += 1
    assert ok_then == 20 and ok_now <= 5, (ok_then, ok_now)
    print(f"PASS  test_pose_at_capture_time      20 frames: capture-time pose {ok_then}/20 right; the pose 250 mm "
          f"later {ok_now}/20")


def test_wait_until_in_view_and_lane_close():
    """#63: a PRESENT seat out of view uses no attempts and its request stays
    open; it is identified once it comes into view. Scripted: initialise
    facing along the lane with a pillar ahead, turn 90 deg on the spot (IMU
    only), offer a frame (out of view), turn back, offer a frame (identified).
    Then, from run_mock: a seat never in view closes as UNKNOWN at the lane change."""
    import lane_frame as lf
    import lane_init as li
    from lane_tracker import LaneTracker
    from scan_processing import clean_and_project
    from stm32_link import ImuSample
    direction, sec = "CCW", "S"
    seat = {s.index: s for s in __import__("seat_occupancy").seats()}[4]          # far-outer, y = 2000
    pgx, pgy = lf.lane_to_global(sec, direction, seat.x_mm, seat.y_mm)
    pillar = sim.Pillar(pgx, pgy, "green")
    x0, y0 = 500.0, 1400.0
    gx, gy = lf.lane_to_global(sec, direction, x0, y0)
    brg = lf.yaw_to_heading(0.0, sec, direction)
    raw = sim.simulate_scan(gx, gy, brg, [pillar], n_points=720, rng_noise=random.Random(1),
                            blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                            blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
    init = li.initialise(clean_and_project([(sim.to_sensor_raw(a), d, q) for a, d, q in raw],
                                           config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    assert init.ok and init.direction.direction == direction, init.reason
    assert [r.state.value for r in init.seats if r.seat.index == 4] == ["occupied"]
    trk = LaneTracker(init, ImuSample(1, 0, 0, 0.0, 0.0))
    st = trk.lanes[0].seats[4]
    assert st.color == "pending" and trk.wants_camera
    nrng = np.random.default_rng(2)

    def turn_to(heading_rel, seq0, t0):
        for k in range(1, 31):                                   # 0.3 s of samples, on the spot
            yaw = config.IMU_YAW_SIGN * heading_rel * k / 30.0 + 0.0
            trk.on_imu(ImuSample(seq0 + k, t0 + 10 * k, 0, yaw, (t0 + 10 * k) / 1000.0))
        return seq0 + 30, t0 + 300

    def frame_at(heading_rel):
        b = brg + heading_rel
        cx, cy = camera_sim.camera_global(gx, gy, b)
        return camera_sim.render(cx, cy, b, [pillar], nrng)

    # turn clockwise, AGAINST the CCW round (no lane switch): the seat ends up ~90 deg to the left
    seq, t = turn_to(90.0, 1, 0)
    trk.on_camera_frame(frame_at(90.0), t / 1000.0)
    assert st.color == "pending" and st.color_out_of_view == 1 and st.color_attempts == 0, st
    first_wait = st.color_reason
    # turn back to facing along the lane: the yaw sample is absolute, so go to 0
    for k in range(1, 31):
        yaw = config.IMU_YAW_SIGN * 90.0 * (1 - k / 30.0)
        trk.on_imu(ImuSample(seq + k, t + 10 * k, 0, yaw, (t + 10 * k) / 1000.0))
    t += 300
    trk.on_camera_frame(frame_at(0.0), t / 1000.0)
    assert st.color == "green" and st.color_attempts == 1, (st.color, st.color_reason)
    print(f"PASS  test_wait_until_in_view        turned away: out of view, no attempt used ({first_wait[:70]}...); "
          f"turned back: GREEN on attempt 1")

    closed = None
    for seed in range(1, 20):
        r = sim.run_mock("CCW", seed % 4, seed=seed, laps=1, camera_hz=15)
        if not r["ok"]:
            continue
        # (checkpoint E, decision #82: the start lane's unknown colours are asked for again when lap 1
        # comes back to it, so the closing is read from the tracker's events, not the final state)
        for e in r["tracker"].events:
            if e.kind == "color" and "UNKNOWN -- never in the camera's view" in e.detail and closed is None:
                closed = (seed, e.lane_index, e.detail, e.detail)
        if closed:
            break
    assert closed and "never in the camera's view" in closed[3], closed
    print(f"PASS  test_lane_close                seed {closed[0]} slot {closed[1]} seat {closed[2]}: never in view -> "
          f"UNKNOWN at the lane change ({closed[3][:70]}...)")


def _standing_tracker(colour="green"):
    """A tracker initialised (simulated scan) facing along lane S (CCW) at
    x 500, y 1400, with one pillar on seat 4 ahead. Returns (trk, render_fn)."""
    import lane_frame as lf
    import lane_init as li
    import seat_occupancy as so
    from lane_tracker import LaneTracker
    from scan_processing import clean_and_project
    from stm32_link import ImuSample
    direction, sec = "CCW", "S"
    seat = {x.index: x for x in so.seats()}[4]
    pillar = sim.Pillar(*lf.lane_to_global(sec, direction, seat.x_mm, seat.y_mm), colour)
    gx, gy = lf.lane_to_global(sec, direction, 500.0, 1400.0)
    brg = lf.yaw_to_heading(0.0, sec, direction)
    raw = sim.simulate_scan(gx, gy, brg, [pillar], n_points=720, rng_noise=random.Random(1),
                            blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                            blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
    init = li.initialise(clean_and_project([(sim.to_sensor_raw(a), d, q) for a, d, q in raw],
                                           config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    trk = LaneTracker(init, ImuSample(1, 0, 0, 0.0, 0.0))
    nrng = np.random.default_rng(4)
    cx, cy = camera_sim.camera_global(gx, gy, brg)
    return trk, (lambda: camera_sim.render(cx, cy, brg, [pillar], nrng))


def test_attempts_and_window():
    """A pillar the classifier can't call (a grey one: neither hue): 5 frames
    inside the window -> UNKNOWN after 5 attempts; frames 0.1 s apart -> the
    0.3 s window ends first (4 attempts). Nothing is ever guessed."""
    from stm32_link import ImuSample
    camera_sim.BGR["grey"] = (128, 128, 128)
    try:
        out = []
        for period_ms, n_frames in ((66, 6), (100, 6)):
            trk, frame = _standing_tracker("grey")
            st = trk.lanes[0].seats[4]
            seq, t = 1, 0
            for k in range(n_frames):
                for j in range(period_ms // 10 if k else 1):          # keep the pose history current
                    seq, t = seq + 1, t + 10
                    trk.on_imu(ImuSample(seq, t, 0, 0.0, t / 1000.0))
                trk.on_camera_frame(frame(), t / 1000.0)
            out.append((st.color, st.color_attempts, st.color_reason))
        assert out[0][0] == "unknown" and out[0][1] == 5 and "no confident read" in out[0][2], out[0]
        assert out[1][0] == "unknown" and out[1][1] == 4 and "window ended" in out[1][2], out[1]
    finally:
        del camera_sim.BGR["grey"]
    print(f"PASS  test_attempts_and_window       grey pillar, frames 66 ms apart: UNKNOWN after {out[0][1]} attempts; "
          f"100 ms apart: UNKNOWN when the 0.3 s window ended ({out[1][1]} attempts)")


if __name__ == "__main__":
    test_projection_matches_the_polynomial()
    test_view_limits()
    test_classify_synthetic_grid()
    test_wrong_mount_config_is_caught()
    test_pose_at_capture_time()
    test_wait_until_in_view_and_lane_close()
    test_attempts_and_window()
    test_tracker_end_to_end()
    print("All colour-ID checks passed.")
