"""
Measures config.LIDAR_TIME_OFFSET_S on the robot (decision #38): how much
later a LIDAR return reaches the Pi than an STM32 line does, each counted from
when it was measured. The entry re-check's de-skew needs it (docs/CHANGES.md
section 9.11); in simulation an error of 10 ms is harmless and 30 ms starts to
cost wrong verdicts.

PROCEDURE (--real), about 20 s:
  1. Put the robot on the mat, anywhere with walls within ~3 m, and start:
         python3 measure_lidar_delay.py --real --record delay.json
  2. Keep it STILL until told (3 s): this is the reference scan.
  3. When told, TURN it about its own centre, back and forth by about +/-30
     degrees, roughly one swing per second, for 10 s. By hand is fine. Keep it
     on the same spot as well as you can; lifting the drive wheels slightly
     is fine too -- only the IMU heading is used, not the wheels.
  4. When told, keep it still again (2 s). The result is printed:
         LIDAR_TIME_OFFSET_S = +0.0xx   -> put this value in config.py
  --replay delay.json re-analyses a saved recording.

HOW IT WORKS
  While the robot turns, each return is rotated back into the reference
  (still) robot frame with the IMU heading at the time the return was taken.
  If that time is right, the rotated returns land on the reference scan's
  walls; if it is off by d, the robot's rotation during d smears them. So for
  every candidate offset d in -100..+100 ms (1 ms steps):
      t_i  = the return's sweep time (timing.SweepClock) - d
      turn = IMU heading(t_i) - heading at the reference  (timing.LinkClock times)
      rotate the return (with the lever arm) by -turn, compare its range with
      the reference scan's range at the new angle
      cost(d) = mean of |range difference|, each capped at 50 mm (so walls seen
                only in one of the two scans don't dominate)
  The offset is the d with the smallest cost (refined between the 1 ms steps
  with a parabola). Only rotation is modelled, so the robot should turn on the
  spot; a few cm of wander adds the same error at every d and doesn't move the
  minimum.

  --sim builds a recording from a simulated robot turned by hand, with a known
  offset, bursty LIDAR delivery and USB jitter, and checks it is recovered.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time

import numpy as np

import config
from timing import LinkClock, SweepClock

GRID_S = np.arange(-0.100, 0.1005, 0.001)
CAP_MM = 50.0


# =============================================================================
# analysis
# =============================================================================
def _imu_series(imu):
    """[(seq, t_ms, enc, yaw, rx)] -> (times on the Pi clock, unwrapped clockwise heading)."""
    clk = LinkClock()
    ts, hs, h, last = [], [], 0.0, None
    for seq, t_ms, enc, yaw, rx in imu:
        if last is not None:
            h += config.IMU_YAW_SIGN * ((yaw - last + 180.0) % 360.0 - 180.0)
        last = yaw
        ts.append(clk.update(t_ms, rx))
        hs.append(h)
    return np.array(ts), np.array(hs)


def _lidar_times(lidar):
    """[(raw, d, q, ..., arrival)] in arrival order -> each return's sweep time,
    as lidar_source computes it (the clock's fit, refreshed every 250 returns)."""
    clk = SweepClock()
    times, pending = np.full(len(lidar), np.nan), []
    for k, p in enumerate(lidar):
        th = clk.add(p[0], p[-1])
        pending.append((k, th))
        if len(pending) >= 250:
            f = clk.fit()
            if f is not None:
                for kk, tth in pending:
                    times[kk] = SweepClock.time_of(tth, f)
            pending = []
    f = clk.fit()
    if f is not None:
        for kk, tth in pending:
            times[kk] = SweepClock.time_of(tth, f)
    arrival = np.array([p[-1] for p in lidar])
    return times, arrival, clk


def analyse(rec: dict) -> dict:
    sign, off = rec.get("angle_sign", config.LIDAR_ANGLE_SIGN), rec.get("angle_zero_offset_deg",
                                                                        config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
    F, L = rec.get("lever_forward_mm", config.LIDAR_OFFSET_FORWARD_MM), rec.get("lever_left_mm",
                                                                              config.LIDAR_OFFSET_LATERAL_MM)
    t_imu, h_imu = _imu_series(rec["imu"])
    t_lid, arrival, clk = _lidar_times(rec["lidar"])
    raw = np.array([[p[0], p[1], p[2]] for p in rec["lidar"]], dtype=float)
    a = np.radians((sign * raw[:, 0] + off) % 360.0)
    d, q = raw[:, 1], raw[:, 2]
    ok = np.isfinite(t_lid) & (d >= 150.0) & (d <= 6000.0) & (q >= 0)
    move_at, stop_at = rec["move_at"], rec["stop_at"]

    # reference: the last second before the robot is told to turn
    ref_sel = ok & (t_lid > move_at - 1.2) & (t_lid < move_at - 0.2)
    h_ref = float(np.median(np.interp(t_lid[ref_sel], t_imu, h_imu))) if ref_sel.any() else 0.0
    ref = np.full(360, np.nan)
    ref_ang = (np.degrees(a[ref_sel]) % 360.0).astype(int)
    for b in range(360):
        v = d[ref_sel][ref_ang == b]
        if len(v) >= 2:
            ref[b] = np.median(v)

    mov = ok & (t_lid > move_at + 0.5) & (t_lid < stop_at - 0.2)
    idx = np.flatnonzero(mov)
    if len(idx) > 30000:
        idx = idx[:: len(idx) // 30000 + 1]
    fr = F + d[idx] * np.cos(a[idx])            # return in the robot frame (fwd, right) at its own time
    rr = -L + d[idx] * np.sin(a[idx])
    tt = t_lid[idx]
    hr = math.radians(h_ref)
    costs = []
    for dt in GRID_S:
        h = np.radians(np.interp(tt - dt, t_imu, h_imu))
        X = fr * np.sin(h) + rr * np.cos(h)     # pure rotation about the reference point
        Y = fr * np.cos(h) - rr * np.sin(h)
        f2 = X * math.sin(hr) + Y * math.cos(hr) - F
        r2 = X * math.cos(hr) - Y * math.sin(hr) + L
        ang = np.degrees(np.arctan2(r2, f2)) % 360.0
        rng_ = np.hypot(f2, r2)
        x = ang - 0.5                               # reference bin b holds the median over [b, b + 1)
        b0 = np.floor(x).astype(int) % 360
        b1 = (b0 + 1) % 360
        w = x - np.floor(x)
        rv = ref[b0] * (1 - w) + ref[b1] * w
        good = np.isfinite(rv)
        costs.append(float(np.mean(np.minimum(np.abs(rng_[good] - rv[good]), CAP_MM))) if good.any() else np.nan)
    costs = np.array(costs)
    k = int(np.nanargmin(costs))
    best = float(GRID_S[k])
    if 0 < k < len(GRID_S) - 1:                  # parabola through the minimum and its neighbours
        c0, c1, c2 = costs[k - 1], costs[k], costs[k + 1]
        den = c0 - 2 * c1 + c2
        if den > 0:
            best += 0.001 * 0.5 * (c0 - c2) / den
    i10 = [int(np.argmin(np.abs(GRID_S - (GRID_S[k] + s)))) for s in (-0.010, 0.010)]
    rate = np.abs(np.diff(h_imu)) / np.maximum(np.diff(t_imu), 1e-3)
    in_move = (t_imu[1:] > move_at) & (t_imu[1:] < stop_at)
    f = clk.fit()
    delay_over_min = arrival[ok] - t_lid[ok]
    return {
        "offset_s": best,
        "at_edge": k in (0, len(GRID_S) - 1),
        "cost_min_mm": float(costs[k]), "cost_at_0_mm": float(costs[int(np.argmin(np.abs(GRID_S)))]),
        "cost_pm10ms_mm": [float(costs[i]) for i in i10],
        "returns_used": int(len(idx)), "ref_bins": int(np.isfinite(ref).sum()),
        "turn_range_deg": float(np.ptp(h_imu[(t_imu > move_at) & (t_imu < stop_at)])) if in_move.any() else 0.0,
        "turn_rate_p90_deg_s": float(np.percentile(rate[in_move], 90)) if in_move.any() else 0.0,
        "spin_hz": None if f is None else 1.0 / (f[1] * 360.0),
        "lidar_backwards_steps": clk.backwards,
        "lidar_delivery_spread_ms": [float(np.percentile(delay_over_min, p)) * 1000 for p in (50, 95, 100)],
        "curve": list(zip([round(float(g), 3) for g in GRID_S], [round(float(c), 2) for c in costs])),
    }


def report(r: dict) -> str:
    lines = ["=== LIDAR vs STM32 delay ===",
             f"turning seen by the IMU : {r['turn_range_deg']:.0f} deg peak to peak, 90th-percentile rate "
             f"{r['turn_rate_p90_deg_s']:.0f} deg/s",
             f"LIDAR spin              : {r['spin_hz']:.2f} Hz" if r["spin_hz"] else "LIDAR spin: not measured",
             f"LIDAR delivery          : returns arrive up to {r['lidar_delivery_spread_ms'][2]:.1f} ms later than the "
             f"least-delayed ones (median {r['lidar_delivery_spread_ms'][0]:.1f}, 95% {r['lidar_delivery_spread_ms'][1]:.1f}); "
             f"that spread is removed by the sweep clock",
             f"returns compared        : {r['returns_used']} against {r['ref_bins']} reference directions",
             f"mismatch                : {r['cost_min_mm']:.1f} mm at the best offset, {r['cost_at_0_mm']:.1f} mm at 0, "
             f"{r['cost_pm10ms_mm'][0]:.1f} / {r['cost_pm10ms_mm'][1]:.1f} mm at -10 / +10 ms from the best"]
    problems = []
    if r["turn_rate_p90_deg_s"] < 30.0:
        problems.append("the robot hardly turned (need a 90th-percentile rate of 30 deg/s or more) -- repeat, turning more")
    if r["at_edge"]:
        problems.append("the best offset is at the edge of the -100..+100 ms search -- check the recording")
    if min(r["cost_pm10ms_mm"]) - r["cost_min_mm"] < 1.0:
        problems.append("the minimum is shallow (less than 1 mm better than 10 ms away) -- repeat, turning faster")
    if problems:
        lines += ["NOT RELIABLE:"] + [f"  - {p}" for p in problems]
    lines.append(f"LIDAR_TIME_OFFSET_S = {r['offset_s']:+.3f}" + ("" if problems else "   -> put this value in config.py"))
    return "\n".join(lines)


# =============================================================================
# recording on the robot
# =============================================================================
def record_real(path: str | None) -> dict:
    from lidar_source import RPLidarC1Source
    from stm32_link import Stm32Link
    link = Stm32Link()
    link.start()
    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    lidar.start()
    imu, pts, last_t = [], [], 0.0

    def collect(seconds):
        nonlocal last_t
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            time.sleep(0.1)
            imu.extend([s.seq, s.t_ms, s.enc, s.yaw_deg, s.rx_time] for s in link.drain())
            new = lidar.get_points_since(last_t)
            if new:
                last_t = max(p[4] for p in new)
                pts.extend(list(p) for p in new)

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0 and (link.stats.last_sample is None
                                               or lidar.timing_status()["spin_hz"] is None):
            time.sleep(0.1)
        if link.stats.last_sample is None:
            sys.exit(f"no STM32 samples: {link.status()}")
        if lidar.timing_status()["spin_hz"] is None:
            sys.exit(f"no LIDAR sweep: {lidar.status()}")
        link.drain()
        print("Keep the robot STILL ...", flush=True)
        collect(3.0)
        move_at = time.monotonic()
        print(">>> TURN it now: back and forth about +/-30 deg, about one swing per second, on the spot <<<", flush=True)
        collect(10.0)
        stop_at = time.monotonic()
        print("STOP -- keep it still ...", flush=True)
        collect(2.0)
    finally:
        link.stop()
        lidar.stop()
    rec = {"imu": imu, "lidar": pts, "move_at": move_at, "stop_at": stop_at,
           "angle_sign": config.LIDAR_ANGLE_SIGN, "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG,
           "lever_forward_mm": config.LIDAR_OFFSET_FORWARD_MM, "lever_left_mm": config.LIDAR_OFFSET_LATERAL_MM}
    if path:
        with open(path, "w") as fh:
            json.dump(rec, fh)
    return rec


# =============================================================================
# simulated recording
# =============================================================================
def simulate_recording(true_offset_s=0.013, imu_delay_s=0.002, burst_s=0.025, spin_hz=10.2,
                       wander_mm=0.0, seed=1, lever=(0.0, 0.0), swing_deg=30.0) -> dict:
    """A robot turned by hand on the spot in lane S (x 500, y 1400, CCW), with
    pillars; the LIDAR's returns reach the Pi true_offset_s + imu_delay_s after
    they are measured at the least, in bursts every burst_s; the STM32 lines
    imu_delay_s after they are sampled at the least, plus USB jitter.
    wander_mm: the robot's centre also wanders by up to this much (unseen)."""
    import lane_frame as lf
    import simulation as sim
    rng = random.Random(seed)
    F, Lf = lever
    gx, gy = lf.lane_to_global("S", "CCW", 500.0, 1400.0)
    north = lf.grid_north_bearing("S", "CCW")
    pillars = sim.rulebook_pillars("S", "CCW", [2, 5]) + sim.rulebook_pillars("E", "CCW", [1])
    move_at, stop_at, t_end = 3.0, 13.0, 15.0

    def heading(t):
        if move_at <= t <= stop_at:
            return swing_deg * math.sin(2 * math.pi * 0.7 * (t - move_at)) * min(1.0, (t - move_at) / 0.5)
        return 0.0

    def centre(t):
        if not wander_mm:
            return gx, gy
        return gx + wander_mm * math.sin(1.3 * t), gy + wander_mm * math.cos(0.9 * t)

    pi0, stm0 = 5000.0, 123.456                     # the two clocks' arbitrary zeros
    imu, t = [], 0.0
    seq = 0
    while t <= t_end:
        seq += 1
        yaw = config.IMU_YAW_SIGN * heading(t) + 41.0 + rng.gauss(0, 0.05)
        yaw = (yaw + 180.0) % 360.0 - 180.0
        rx = pi0 + t + imu_delay_s + rng.expovariate(1 / 0.001)
        imu.append([seq, int(round((stm0 + t) * 1000)), 0, round(yaw, 2), rx])
        t += 0.01
    lidar, t, phase = [], 0.0, rng.uniform(0, 360)
    dt = 1.0 / 5000.0
    delay = true_offset_s + imu_delay_s
    while t <= t_end:
        phase = (phase + 360.0 * spin_hz * dt) % 360.0
        rel = (config.LIDAR_ANGLE_SIGN * phase + config.LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360.0
        if abs((rel - 180.0 + 180.0) % 360.0 - 180.0) > config.REAR_BLIND_ARC_WIDTH_DEG / 2.0:
            h = north + heading(t)
            hr = math.radians(h)
            cx, cy = centre(t)
            sx = cx + F * math.sin(hr) - Lf * math.cos(hr)
            sy = cy + F * math.cos(hr) + Lf * math.sin(hr)
            for _, d, q in sim.simulate_scan(sx, sy, (h + rel) % 360.0, pillars, n_points=1, rng_noise=rng):
                arr = pi0 + math.ceil(t / burst_s) * burst_s + delay
                lidar.append([phase, d, q, arr])
        t += dt
    lidar.sort(key=lambda p: p[-1])
    return {"imu": imu, "lidar": lidar, "move_at": pi0 + move_at, "stop_at": pi0 + stop_at,
            "angle_sign": config.LIDAR_ANGLE_SIGN, "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG,
            "lever_forward_mm": F, "lever_left_mm": Lf, "true_offset_s": true_offset_s}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    m = ap.add_mutually_exclusive_group(required=True)
    m.add_argument("--real", action="store_true")
    m.add_argument("--replay", metavar="FILE")
    m.add_argument("--sim", action="store_true")
    ap.add_argument("--record", metavar="FILE", help="save the recording (--real), overwriting FILE")
    ap.add_argument("--true-offset-ms", type=float, default=13.0, help="(sim)")
    ap.add_argument("--burst-ms", type=float, default=25.0, help="(sim)")
    ap.add_argument("--wander-mm", type=float, default=0.0, help="(sim)")
    ap.add_argument("--seed", type=int, default=1, help="(sim)")
    a = ap.parse_args()
    if a.real:
        rec = record_real(a.record)
    elif a.replay:
        with open(a.replay) as fh:
            rec = json.load(fh)
    else:
        rec = simulate_recording(a.true_offset_ms / 1000.0, burst_s=a.burst_ms / 1000.0,
                                 wander_mm=a.wander_mm, seed=a.seed)
    r = analyse(rec)
    print(report(r))
    if "true_offset_s" in rec:
        print(f"(simulation: true offset {rec['true_offset_s']:+.3f}, error {1000 * (r['offset_s'] - rec['true_offset_s']):+.1f} ms)")


if __name__ == "__main__":
    main()
