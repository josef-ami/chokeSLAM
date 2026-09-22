"""
Run the one-time initialisation once and print everything it decided, and why.

    python3 run_init.py --real [--dump scan.json]    # real RPLidar on config.LIDAR_PORT
    python3 run_init.py --replay scan.json            # a scan saved earlier with --dump
    python3 run_init.py --sim --lane S --direction CCW --x 500 --y 1500 \
                        [--yaw 0] [--pillars 1,4] [--parking 1500]

--real / --replay take raw sensor angles and apply config.LIDAR_ANGLE_SIGN /
LIDAR_ANGLE_ZERO_OFFSET_DEG, exactly like the robot. --sim builds a world with
simulation.py; --x is from the outer wall, --y along travel, --pillars are
seat indices (0..5, see seat_occupancy.seats()), --parking puts the two
limitations around that lane y (and should come with --x 100).

This is the review tool for checkpoint A (before the dashboard exists): run
it on the robot at a few real start poses and compare with a tape measure.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import config
import lane_init as li
import seat_occupancy as so
from scan_processing import clean_and_project


def _raw_from_real(wait_s: float = 5.0):
    from lidar_source import RPLidarC1Source
    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    lidar.start()
    raw = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        time.sleep(0.5)
        raw = lidar.get_latest_scan()
        if len(raw) >= 300:          # a full revolution's worth of 1-deg buckets, near enough
            break
    status = lidar.status()
    lidar.stop()
    if not raw:
        sys.exit(f"no LIDAR points after {wait_s:.0f} s -- lidar.status(): {status}")
    return raw


def _raw_from_sim(a):
    import simulation as sim
    pillars = sim.rulebook_pillars(a.lane, a.direction,
                                   [int(i) for i in a.pillars.split(",")] if a.pillars else [])
    boxes = []
    if a.parking is not None:
        boxes = sim.parking_lot_boxes(a.lane, a.direction, a.parking - 235.0, a.parking + 235.0)
    robot = sim.MockRobotSimulator(a.lane, a.direction, a.x, a.y, a.yaw, pillars=pillars,
                                   extra_boxes=boxes, n_points=720)
    return robot.current_scan()


def report(res: li.InitResult) -> str:
    d = res.direction
    lines = ["", "=== INITIALISATION ===", f"result    : {'OK' if res.ok else 'FAILED'} -- {res.reason}", ""]
    lines.append(f"direction : {d.direction or 'UNDETERMINED'}")
    lines.append(f"            {d.reason}")
    for side in (d.left, d.right):
        fit = side.fit
        fit_s = (f"fit {fit.angle_deg:+.2f} deg, {fit.n_inliers} inliers, rms {fit.rms_mm:.1f} mm"
                 if fit is not None and fit.ok else "not fitted")
        span = ("" if side.opening_start_mm is None else
                f", from {side.opening_start_mm:.0f} to {side.opening_end_mm:.0f} mm ahead")
        d_side = "n/a" if side.d_side_mm is None else f"{side.d_side_mm:.0f}"
        lines.append(f"  {side.side:5s}: d={d_side} mm  {fit_s}; wall {side.n_wall}, "
                     f"through {side.n_through}, blocked {side.n_blocked} -> opening {side.opening_mm:.0f} mm{span}")
    def mm(v):
        return "n/a" if v is None else f"{v:.0f}"
    if res.x is not None:
        x = res.x
        lines.append("")
        lines.append(f"x         : {'n/a' if x.x_mm is None else f'{x.x_mm:.0f} mm from the outer wall'}"
                     f"   (d90 {mm(x.d90_mm)}, d270 {mm(x.d270_mm)}, sum {mm(x.lane_sum_mm)}) {x.reason}")
    if res.y is not None:
        y = res.y
        lines.append(f"y         : {'n/a' if y.y_mm is None else f'{y.y_mm:.0f} mm from the wall behind'}"
                     f"   (front {mm(y.front_mm)} mm, median of {y.n_front} of {y.n_fan} fan returns) {y.reason}")
    lines.append(f"yaw       : {res.yaw_deg:.1f} deg (taken as 0 by agreement)")
    if res.seats:
        lines.append("")
        lines.append("seats (x from outer wall, y along travel):")
        lines.append(so.summary(res.seats))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--real", action="store_true")
    src.add_argument("--replay", metavar="FILE")
    src.add_argument("--sim", action="store_true")
    ap.add_argument("--dump", metavar="FILE", help="save the raw scan used (with --real or --sim)")
    ap.add_argument("--lane", default="S", choices=["S", "E", "N", "W"])
    ap.add_argument("--direction", default="CCW", choices=["CCW", "CW"])
    ap.add_argument("--x", type=float, default=500.0)
    ap.add_argument("--y", type=float, default=1500.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pillars", default="")
    ap.add_argument("--parking", type=float, default=None)
    a = ap.parse_args()

    if a.real:
        raw = _raw_from_real()
    elif a.replay:
        with open(a.replay) as fh:
            raw = [tuple(p) for p in json.load(fh)["raw"]]
    else:
        raw = _raw_from_sim(a)
    if a.dump:
        with open(a.dump, "w") as fh:
            json.dump({"raw": raw, "angle_sign": config.LIDAR_ANGLE_SIGN,
                       "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG}, fh)
        print(f"raw scan saved to {a.dump} ({len(raw)} points)")

    pts = clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
    print(report(li.initialise(pts)))


if __name__ == "__main__":
    main()
