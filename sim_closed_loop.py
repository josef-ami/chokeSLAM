"""
Closed-loop simulation of the whole obstacle round (checkpoint E, decision #83):

    rulebook layout (layouts.py)
      -> simulated car (kinematic bicycle at the rear axle, steering servo lag
         and rate limit, lock, speed loop lag, per-trial errors)
      -> the REAL chokeSLAM stack: start scan -> lane_init.initialise ->
         LaneTracker fed with $IMU lines (simulation.SimStm32) from the car's
         true motion; LIDAR revolutions (swept, timed, parking limitations
         included) while the tracker asks; camera frames (camera_sim.render)
         while it asks
      -> mission.Mission (planner + follower) -> DRIVE command -> the car.

The judge applies the rulebook to the TRUE motion:
    contact      the car's footprint below 100 mm (config.BODY_*) touches a
                 pillar, a parking limitation, the island or the outer wall.
                 Stricter than the rules (walls may be touched, 9.18; a pillar
                 may move inside its circle, 9.20) -- decision #83.
    wrong side   while fewer than 3 laps are complete: the car's centre crosses
                 a pillar's line (lane y of the pillar, App. A.5) on the wrong
                 side. (The rulebook ends the round only once the car has
                 COMPLETELY crossed the line; judging at the centre is stricter.)
    laps         a lap is complete when the whole outline has left the last
                 corner section into the start section (10.2 table, 1.2)
    finish       after 3 laps, stopped with the whole outline inside the start
                 section (App. A.2)
    time         3 minutes (9.2)
"""
from __future__ import annotations

import math
import random
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

import config
import field_map as fm
import lane_frame as lf
import lane_init as li
import mat_geometry as geo
import simulation as sim
from lane_tracker import LaneTracker
from mission import Mission
from scan_processing import clean_and_project, clean_and_project_timed
from stm32_link import ImuSample, parse_line
from vg_planner import body_hits
from field_map import Rect

NOISE = {
    "none": dict(start_xy=0.0, start_yaw=0.0, enc_scale=0.0, steer_bias=0.0, steer_noise=0.0,
                 yaw_drift=0.0, yaw_noise=0.0, lock_err=0.0, speed_err=0.0),
    # the report's "moderate" preset, adapted to closed-loop driving
    "moderate": dict(start_xy=10.0, start_yaw=2.0, enc_scale=0.02, steer_bias=1.0, steer_noise=1.0,
                     yaw_drift=0.5 / 60.0, yaw_noise=0.15, lock_err=0.05, speed_err=0.05),
    # moderate, but with the encoder calibrated to 0.5 % (e.g. by rolling a measured 2 m, CHANGES section 11)
    "moderate_cal": dict(start_xy=10.0, start_yaw=2.0, enc_scale=0.005, steer_bias=1.0, steer_noise=1.0,
                         yaw_drift=0.5 / 60.0, yaw_noise=0.15, lock_err=0.05, speed_err=0.05),
    "harsh": dict(start_xy=20.0, start_yaw=4.0, enc_scale=0.04, steer_bias=2.0, steer_noise=2.0,
                  yaw_drift=1.0 / 60.0, yaw_noise=0.3, lock_err=0.10, speed_err=0.10),
}

DT = 0.01
CTRL_EVERY = 2          # 50 Hz control
LIDAR_EVERY = 10        # 10 Hz revolutions
CAM_EVERY = 10          # 10 Hz frames
CMD_LATENCY_S = 0.02
STEER_TAU_S = 0.06
STEER_RATE_DPS = 400.0
SPEED_TAU_S = 0.15


@dataclass
class Result:
    ok: bool
    reason: str
    t: float
    laps: int
    layout: str
    detail: str = ""
    min_pillar_gap_mm: float = math.inf
    min_gap_at: tuple = None
    max_track_err_mm: float = 0.0
    plans: int = 0
    replans: int = 0
    stops_for_colour: int = 0
    colour_given_up: int = 0
    wrong_colour: int = 0
    wrong_seat: int = 0
    events: list = None
    trace: list = None          # true rear-axle path in the loop frame, every 50 ms
    paths: list = None          # every planned path's samples (loop frame)
    seats: dict = None          # final tracker seat table {slot: {seat: (state, colour, truth colour)}}


def _rect_gap(X, Y, th, r: Rect) -> float:
    """Separating distance between the car's footprint and an axis-aligned rect (0 if touching)."""
    F, B, H = config.BODY_FRONT_MM, config.BODY_REAR_MM, config.BODY_HALF_WIDTH_MM
    c, s = math.cos(th), math.sin(th)
    ox, oy = X + (F - B) / 2 * c, Y + (F - B) / 2 * s
    hf = (F + B) / 2
    body = [(ox + a * hf * c - b * H * s, oy + a * hf * s + b * H * c) for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    xs, ys = [p[0] for p in body], [p[1] for p in body]
    gaps = [r.x0 - max(xs), min(xs) - r.x1, r.y0 - max(ys), min(ys) - r.y1]
    mx, my = (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2
    hx, hy = (r.x1 - r.x0) / 2, (r.y1 - r.y0) / 2
    dx, dy = mx - ox, my - oy
    gaps.append(abs(dx * c + dy * s) - hf - hx * abs(c) - hy * abs(s))
    gaps.append(abs(-dx * s + dy * c) - H - hx * abs(s) - hy * abs(c))
    return max(0.0, max(gaps))


def run(layout, seed: int = 0, noise: str = "moderate", verbose: bool = False, camera: bool = True,
        max_time: float = 180.0) -> Result:
    rng = random.Random(seed)
    nz = NOISE[noise]
    d = layout.direction
    start_sec = layout.start_section
    # --- world (global mat frame) ---
    pillars, truth = [], {}
    for k in range(4):
        sec = layout.slot_section(k)
        for i, col in layout.pillars.get(sec, []):
            s = __import__("seat_occupancy").seats()[i]
            gx, gy = lf.lane_to_global(sec, d, s.x_mm, s.y_mm)
            pillars.append(sim.Pillar(gx, gy, col))
            truth[(k, i)] = col
    barriers = []
    for (x0, y0, x1, y1) in fm.parking_barriers(d):
        a = lf.lane_to_global(start_sec, d, x0, y0)
        b = lf.lane_to_global(start_sec, d, x1, y1)
        barriers.append((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
    judge_rects = [Rect(p.x_mm - 25, p.y_mm - 25, p.x_mm + 25, p.y_mm + 25, "pillar", ("pillar", n))
                   for n, p in enumerate(pillars)]
    judge_rects += [Rect(*b, "barrier", ("barrier", n)) for n, b in enumerate(barriers)]
    judge_rects.append(Rect(*geo.ISLAND_BOX, "island", ("island",)))

    # --- the car's true state (rear axle, global, grid bearing clockwise from +Y) ---
    sx = layout.start_x + rng.gauss(0, nz["start_xy"])
    sy = layout.start_y + rng.gauss(0, nz["start_xy"])
    yaw0 = rng.gauss(0, nz["start_yaw"])
    gx, gy = lf.lane_to_global(start_sec, d, sx, sy)
    brg = lf.yaw_to_heading(yaw0, start_sec, d)
    v = 0.0
    delta = 0.0
    steer_bias = rng.gauss(0, nz["steer_bias"])
    lock_scale = 1.0 + rng.uniform(-nz["lock_err"], nz["lock_err"])
    speed_scale = 1.0 + rng.uniform(-nz["speed_err"], nz["speed_err"])
    lock_l, lock_r = config.STEER_LOCK_LEFT_DEG * lock_scale, config.STEER_LOCK_RIGHT_DEG * lock_scale
    dist = 0.0
    hist = deque(maxlen=60)          # (t, gx, gy, brg) for the swept LIDAR
    t = 0.0
    hist.append((t, gx, gy, brg))

    def sensor(gx_, gy_, b_):
        r = math.radians(b_)
        F, L = config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM
        return gx_ + F * math.sin(r) - L * math.cos(r), gy_ + F * math.cos(r) + L * math.sin(r)

    def pose_at(tq):
        h = list(hist)
        if tq <= h[0][0]:
            return h[0][1:]
        for (t0, x0, y0, b0), (t1, x1, y1, b1) in zip(h, h[1:]):
            if t0 <= tq <= t1:
                f = (tq - t0) / max(t1 - t0, 1e-9)
                db = (b1 - b0 + 180) % 360 - 180
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), b0 + f * db
        return h[-1][1:]

    # --- initialisation from the start scan ---
    lx, ly = sensor(gx, gy, brg)
    raw = sim.simulate_scan(lx, ly, brg, pillars, barriers, n_points=720, rng_noise=rng,
                            blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                            blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
    raw = [(sim.to_sensor_raw(a), dd, q) for a, dd, q in raw]
    init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    lay_s = layout.describe()
    if not init.ok:
        return Result(False, "init_failed", 0.0, 0, lay_s, init.reason)
    if init.direction.direction != d:
        return Result(False, "init_wrong_direction", 0.0, 0, lay_s)

    stm = sim.SimStm32(rng, ticks_per_cm=config.ENCODER_TICKS_PER_CM * (1.0 + rng.uniform(-nz["enc_scale"], nz["enc_scale"])),
                       yaw_noise_deg=nz["yaw_noise"], yaw_drift_deg_s=rng.gauss(0, nz["yaw_drift"]) if nz["yaw_drift"] else 0.0)

    def sample(t_):
        line = stm.line(int(round(t_ * 1000)), dist, brg)
        smp, _ = parse_line(line)
        return ImuSample(smp.seq, smp.t_ms, smp.enc, smp.yaw_deg, t_ + 0.002)

    trk = LaneTracker(init, sample(0.0))
    logf = print if verbose else None
    mis = Mission(trk, log=logf)
    cmdq = deque()
    cmd_now = (0.0, 0.0)
    cam_rng = np.random.default_rng(seed + 7)

    # judge state
    laps = 0
    lap_dist = 0.0
    rear_in = True                     # outline rear already past the start line (starts inside the section)
    side_state = {}                    # pillar n -> previous lane y of the car's centre
    min_gap = math.inf
    min_gap_at = None
    trace, paths = [], []
    max_err = 0.0
    n = 0
    fail = None
    stopped_since = None
    import camera_sim
    import seat_occupancy as so
    seat_xy = {s.index: (s.x_mm, s.y_mm) for s in so.seats()}
    pinfo = []
    for k in range(4):
        sec = layout.slot_section(k)
        for i, col in layout.pillars.get(sec, []):
            pinfo.append((sec, seat_xy[i][0], seat_xy[i][1], col))

    while t < max_time:
        n += 1
        t = n * DT
        # actuators: command after latency, steering lag + rate limit + lock, speed lag
        while cmdq and cmdq[0][0] <= t:
            cmd_now = cmdq.popleft()[1]
        d_cmd = cmd_now[0] + steer_bias + (rng.gauss(0, nz["steer_noise"]) if nz["steer_noise"] and n % CTRL_EVERY == 0 else 0.0)
        d_cmd = max(-lock_r, min(lock_l, d_cmd))
        step = (d_cmd - delta) * min(1.0, DT / STEER_TAU_S)
        step = max(-STEER_RATE_DPS * DT, min(STEER_RATE_DPS * DT, step))
        delta += step
        v += (cmd_now[1] * speed_scale - v) * min(1.0, DT / SPEED_TAU_S)
        if cmd_now[1] == 0.0 and abs(v) < 5.0:
            v = 0.0
        ds = v * DT
        dth = ds * math.tan(math.radians(delta)) / config.WHEELBASE_MM      # + = left = bearing decreases
        bm = math.radians(brg - math.degrees(dth) / 2)
        gx += ds * math.sin(bm)
        gy += ds * math.cos(bm)
        brg = (brg - math.degrees(dth)) % 360.0
        dist += ds
        hist.append((t, gx, gy, brg))

        # sensors -> tracker
        trk.on_imu(sample(t))
        if trk.wants_lidar and n % LIDAR_EVERY == 0:
            rawt = []
            npts = 500
            for kk in range(npts):
                ra = kk * 360.0 / npts
                rel = (config.LIDAR_ANGLE_SIGN * ra + config.LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360.0
                if abs((rel - config.REAR_BLIND_ARC_CENTER_DEG + 180) % 360 - 180) <= config.REAR_BLIND_ARC_WIDTH_DEG / 2:
                    continue
                tm = t - 0.1 * (1 - kk / npts)
                px, py, pb = pose_at(tm)
                ex, ey = sensor(px, py, pb)
                for _, dd, q in sim.simulate_scan(ex, ey, (pb + rel) % 360.0, pillars, barriers, n_points=1, rng_noise=rng):
                    rawt.append((ra, dd, q, tm + 0.002 + config.LIDAR_TIME_OFFSET_S))
            pts, times = clean_and_project_timed(rawt, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
            trk.on_lidar_frame(pts, times)
        if camera and trk.wants_camera and n % CAM_EVERY == 0:
            cx, cy = camera_sim.camera_global(gx, gy, brg)
            trk.on_camera_frame(camera_sim.render(cx, cy, brg, pillars, cam_rng), t + 0.002)

        # control
        if n % CTRL_EVERY == 0:
            cmd = mis.update(t, v)
            cmdq.append((t + CMD_LATENCY_S, (0.0, 0.0) if cmd.stop else (cmd.steer_deg, cmd.speed_mm_s)))

        # --- judge ---
        th = math.radians(90.0 - brg)
        if n % 2 == 0:
            idx, what = body_hits(np.array([[gx, gy, th, 0.0, 0.0]]), judge_rects, 0.0)
            if idx is not None:
                fail = ("contact", str(what))
                break
            for r in judge_rects[:len(pillars)]:
                g_ = _rect_gap(gx, gy, th, r)
                if g_ < min_gap:
                    min_gap, min_gap_at = g_, (round(t, 2), r.key, mis.phase, trk.lane_index)
            # tracking error
            X, Y, _ = fm.tracker_pose(trk)
            tx, ty = lf.global_to_lane(start_sec, d, gx, gy)
            TX, TY = fm.lane_point(0, tx, ty, d)
            max_err = max(max_err, math.hypot(X - TX, Y - TY))
            if n % 10 == 0:
                g2d_b = (brg - lf.grid_north_bearing(start_sec, d)) % 360.0
                trace.append((round(TX, 1), round(TY, 1), round(X, 1), round(Y, 1), round(v), round(delta, 1), mis.phase,
                              round(t, 2), round(g2d_b, 2), round(trk.heading % 360.0, 2)))
            if mis.path is not None and (not paths or paths[-1] is not mis.path):
                paths.append(mis.path)
        cxg = gx + (config.OUTLINE_FRONT_MM - config.OUTLINE_REAR_MM) / 2 * math.sin(math.radians(brg))
        cyg = gy + (config.OUTLINE_FRONT_MM - config.OUTLINE_REAR_MM) / 2 * math.cos(math.radians(brg))
        if laps < 3:
            for pn, (sec, px, py, col) in enumerate(pinfo):
                lx_, ly_ = lf.global_to_lane(sec, d, cxg, cyg)
                inside = -50 <= lx_ <= 1050 and 700 <= ly_ <= 2300
                prev_y = side_state.get(pn)
                side_state[pn] = ly_ if inside else None
                if prev_y is not None and inside and prev_y < py <= ly_:
                    h = lf.handedness(d)
                    right_of = (lx_ - px) * h > 0            # the car is on the pillar's right
                    ok = right_of if col == "red" else not right_of
                    if not ok:
                        fail = ("wrong_side", f"{sec} seat ({px:.0f},{py:.0f}) {col}, car x {lx_:.0f}")
                        break
            if fail:
                break
        # laps: the outline's rearmost point crosses the start section's entry line
        rx_, ry_ = lf.global_to_lane(start_sec, d, gx - config.OUTLINE_REAR_MM * math.sin(math.radians(brg)),
                                     gy - config.OUTLINE_REAR_MM * math.cos(math.radians(brg)))
        now_in = ry_ >= 1000.0 and -50 <= rx_ <= 1050
        if now_in and not rear_in and ry_ < 1300.0 and dist - lap_dist > 4000.0:
            laps += 1                  # (backing out of the section and driving in again is not a lap)
            lap_dist = dist
        rear_in = now_in if (-50 <= rx_ <= 1050) else False
        if mis.phase == "FAILED":
            fail = ("mission", mis.events[-1].detail if mis.events else "")
            break
        if mis.phase == "DONE":
            stopped_since = stopped_since or t
            if v < 1.0 and t - stopped_since > 0.5:
                break
    res_extra = dict(plans=mis.plans, replans=mis.replans,
                     stops_for_colour=sum(1 for e in mis.events if e.kind == "stop_for_colour"),
                     colour_given_up=sum(1 for e in mis.events if e.kind == "colour_given_up"))
    # what the tracker believed vs the truth
    wrong_seat = wrong_col = 0
    for slot, rec in trk.lanes.items():
        for i, st in rec.seats.items():
            tc = truth.get((slot, i))
            if st.state == "occupied" and tc is None or st.state == "empty" and tc is not None:
                wrong_seat += 1
            if st.color in ("red", "green") and tc is not None and st.color != tc:
                wrong_col += 1
    seat_tab = {slot: {i: (st.state, st.color, truth.get((slot, i)), st.color_reason[:200]) for i, st in rec.seats.items()}
                for slot, rec in trk.lanes.items()}
    common = dict(seats=seat_tab, min_pillar_gap_mm=min_gap, min_gap_at=min_gap_at, max_track_err_mm=max_err, wrong_seat=wrong_seat,
                  wrong_colour=wrong_col, trace=trace, paths=[p.samples for p in paths], events=[(round(e.t, 2), e.kind, e.detail) for e in mis.events], **res_extra)
    if fail:
        return Result(False, fail[0], t, laps, lay_s, fail[1], **common)
    if t >= max_time:
        return Result(False, "timeout", t, laps, lay_s, mis.phase, **common)
    if laps != 3:
        return Result(False, "lap_count", t, laps, lay_s, f"{laps} laps", **common)
    # finish: whole outline inside the start section
    ok_fin = True
    for a, b in ((config.OUTLINE_FRONT_MM, config.OUTLINE_HALF_WIDTH_MM), (config.OUTLINE_FRONT_MM, -config.OUTLINE_HALF_WIDTH_MM),
                 (-config.OUTLINE_REAR_MM, config.OUTLINE_HALF_WIDTH_MM), (-config.OUTLINE_REAR_MM, -config.OUTLINE_HALF_WIDTH_MM)):
        r_ = math.radians(brg)
        px_, py_ = gx + a * math.sin(r_) + b * math.cos(r_), gy + a * math.cos(r_) - b * math.sin(r_)
        lx_, ly_ = lf.global_to_lane(start_sec, d, px_, py_)
        if not (0 <= lx_ <= 1000 and 1000 <= ly_ <= 2000):
            ok_fin = False
    if not ok_fin:
        return Result(False, "finish_outside", t, laps, lay_s, **common)
    return Result(True, "ok", t, laps, lay_s, **common)


if __name__ == "__main__":
    import argparse
    import layouts
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--noise", default="moderate")
    ap.add_argument("--direction", default=None)
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    lay = layouts.draw(random.Random(a.seed), a.direction)
    t0 = time.time()
    r = run(lay, seed=a.seed, noise=a.noise, verbose=not a.quiet)
    print(lay.describe())
    print(f"{'OK' if r.ok else 'FAIL'} {r.reason} {r.detail} t={r.t:.1f}s laps={r.laps} gap={r.min_pillar_gap_mm:.0f} "
          f"(at {r.min_gap_at}) err={r.max_track_err_mm:.0f} plans={r.plans} replans={r.replans} wall={time.time() - t0:.1f}s")
