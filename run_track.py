"""
Checkpoint-B review tool: initialisation + IMU lane tracking, printed as it
happens (turns, lane switches, entry re-checks, the tracked pose).

    python3 run_track.py --bench [--log imu.log]
        STM32 only, no LIDAR. Prints the raw yaw and encoder, the heading the
        tracker would compute (IMU_YAW_SIGN applied) and the distance
        (ENCODER_TICKS_PER_CM applied). Use it to check both calibrations:
          - turn the robot CLOCKWISE by hand (seen from above): "heading"
            must INCREASE. If it decreases, flip config.IMU_YAW_SIGN.
          - roll the robot a measured distance straight: "distance" must match.

    python3 run_track.py --real [--log imu.log] [--dump scan.json]
        The real thing: the STM32 link is opened, the LIDAR start scan is taken
        (robot standing still at its start), initialisation runs, then tracking
        from the STM32 feed, with the LIDAR entry-corner re-checks. Push or drive
        the robot round the track once "=== TRACKING" shows (moving earlier, but
        after the scan, is still counted); Ctrl-C to stop and print the seats.
        --log saves every raw STM32 line, --dump the start scan together with
        the seq of the STM32 sample current at the scan (both files are
        overwritten, so one --log / --dump pair is always one run).

    python3 run_track.py --replay-scan scan.json --replay-imu imu.log
        Offline replay of a real run recorded with --dump / --log, starting at
        the sample current at the scan (the LIDAR re-checks can't be replayed:
        only the start scan is saved).

    python3 run_track.py --sim [--direction CCW] [--slot 0] [--y 1400] [--laps 3]
                         [--seed 1] [--placement-yaw 0] [--drop 0] [--enc-err 0] [--speed 600]
                         [--no-sweep] [--no-deskew] [--stamp-error-ms 0]
        The simulated end-to-end run, with ground truth. LIDAR frames are real
        revolutions of a spinning sensor on the moving robot, de-skewed.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import config
import lane_init as li
from lane_tracker import LaneTracker
from run_init import report as init_report
from scan_processing import clean_and_project, clean_and_project_timed

STATE_NAMES = {"occupied": "PRESENT", "empty": "absent", "unknown": "unknown"}


def _seat_table(trk: LaneTracker, truth=None, sections=None) -> str:
    import seat_occupancy as so
    names = {s.index: s.name for s in so.seats()}
    lines = []
    for slot in sorted(trk.lanes):
        rec = trk.lanes[slot]
        head = f"  lane slot {slot} ({'start lane' if slot == 0 else f'lane {slot + 1} of the lap'})"
        if sections:
            head += f", section {sections[slot]}"
        head += (f": source {rec.source}, frames used {rec.frames_used}, "
                 f"skipped by the align gate {rec.frames_skipped_align}")
        lines.append(head)
        for i in sorted(rec.seats):
            st = rec.seats[i]
            t = ""
            if truth is not None:
                occ = i in truth[sections[slot]]
                ok = st.state == "unknown" or (st.state == "occupied") == occ
                t = f"  truth {'pillar' if occ else 'clear '}{'' if ok else '  <-- WRONG'}"
            at = "" if st.at_y_mm is None else f" (y {st.at_y_mm:.0f})"
            lines.append(f"      [{i}] {names[i]:10s} {STATE_NAMES[st.state]:8s}{at}{t}")
    return "\n".join(lines)


def _print_event(e):
    print(f"  t={e.t_ms / 1000:7.2f}s  {e.kind:15s} {e.detail}")


def run_sim(a):
    import simulation as sim
    r = sim.run_mock(a.direction, start_slot=a.slot, y_start=a.y, laps=a.laps, seed=a.seed,
                     placement_yaw_deg=a.placement_yaw, drop_prob=a.drop, enc_scale_err=a.enc_err,
                     speed_mm_s=a.speed, lidar_sweep=not a.no_sweep, deskew=not a.no_deskew,
                     lidar_stamp_error_s=a.stamp_error_ms / 1000.0)
    print(init_report(r["init"]))
    if not r["ok"]:
        return
    trk = r["tracker"]
    print(f"\n=== TRACKING (simulated, {a.laps} laps, {r['distance_mm'] / 1000:.1f} m) ===")
    for e in trk.events:
        _print_event(e)
    sections = {k: sim.LoopPath(a.direction).section(a.slot + k) for k in range(4)}
    print("\nseats per lane (truth from the simulated world):")
    print(_seat_table(trk, r["truth_seats"], sections))
    print(f"\nturns detected {r['turns']} (expected {r['expected_turns']}); tracked vs true position error: "
          f"median {r['pos_err_median_mm']:.1f} mm, worst {r['pos_err_max_mm']:.1f} mm, at the end "
          f"{r['pos_err_end_mm']:.1f} mm; heading error worst {r['head_err_max_deg']:.2f} deg; "
          f"initialisation error x {r['init_err_mm'][0]:+.1f} y {r['init_err_mm'][1]:+.1f} mm, "
          f"start heading error {r['psi0_err_deg']:+.2f} deg")


def _open_link(a):
    """Open the STM32 link and wait (up to 3 s) until samples arrive."""
    from stm32_link import Stm32Link
    link = Stm32Link()
    link.start(log_path=a.log)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0 and link.is_alive():
        if link.stats.last_sample is not None:
            return link
        time.sleep(0.05)
    st = link.status()
    link.stop()
    sys.exit(f"no STM32 samples on {link.port} within 3 s -- link.status(): {st}")


def _take_latest(link, timeout_s: float = 1.0):
    """The newest sample on the link NOW. Everything older is taken off and
    discarded, so every sample after this one comes out of the next
    link.drain(): none is fed twice, none after it is lost. Exits if no sample
    arrives within timeout_s."""
    t0 = time.monotonic()
    while True:
        batch = link.drain()
        if batch:
            return batch[-1]
        if time.monotonic() - t0 > timeout_s or not link.is_alive():
            st = link.status()
            link.stop()
            sys.exit(f"STM32 samples stopped on {link.port} -- link.status(): {st}")
        time.sleep(0.01)


def run_bench(a):
    link = _open_link(a)
    first = _take_latest(link)
    heading, last = 0.0, first
    print(f"STM32 on {link.port}. Turn the robot CLOCKWISE by hand: heading must INCREASE. "
          f"Roll it a measured distance: distance must match. Ctrl-C to stop.")
    try:
        while True:
            for s in link.drain():
                heading += config.IMU_YAW_SIGN * ((s.yaw_deg - last.yaw_deg + 180.0) % 360.0 - 180.0)
                last = s
            st = link.status()
            dist = (last.enc - first.enc) * 10.0 / config.ENCODER_TICKS_PER_CM
            print(f"\r  raw yaw {last.yaw_deg:8.2f}  heading {heading:+8.2f} deg (sign {config.IMU_YAW_SIGN:+d})   "
                  f"enc {last.enc:8d}  distance {dist:8.1f} mm ({config.ENCODER_TICKS_PER_CM} ticks/cm)   "
                  f"{st['rate_hz']:5.1f} Hz  gaps {st['seq_gaps']}  bad {st['lines_bad']}   ", end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()
    finally:
        link.stop()


def run_real(a):
    from lidar_source import RPLidarC1Source
    # The STM32 link is opened BEFORE the start scan is taken, so that the
    # tracker's starting sample is the one current at the moment of the scan
    # (the pose initialisation measures). Every sample after it is integrated,
    # including any that arrive while initialisation is still computing -- if
    # the robot starts moving then, that motion is not lost. (Opening the link
    # only after initialisation, as first written, silently dropped whatever
    # motion happened in between: see docs/CHANGES.md, run_track.py.)
    link = _open_link(a)
    lidar = RPLidarC1Source(config.LIDAR_PORT, config.LIDAR_BAUDRATE, config.LIDAR_SCAN_TIMEOUT_S)
    try:
        lidar.start()
        raw = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and len(raw) < 300:
            time.sleep(0.5)
            raw = lidar.get_latest_scan()
        first = _take_latest(link)
    except BaseException:
        lidar.stop()
        link.stop()
        raise
    if a.dump:
        with open(a.dump, "w") as fh:
            json.dump({"raw": raw, "angle_sign": config.LIDAR_ANGLE_SIGN,
                       "angle_zero_offset_deg": config.LIDAR_ANGLE_ZERO_OFFSET_DEG,
                       "imu_seq_at_scan": first.seq}, fh)
    init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    print(init_report(init))
    if not init.ok:
        lidar.stop()
        link.stop()
        return
    trk = LaneTracker(init, first)
    print("\n=== TRACKING (Ctrl-C to stop) ===")
    shown, last_print = 0, 0.0
    try:
        while True:
            # The LIDAR frame is read BEFORE the STM32 samples are taken, so the pose history the
            # de-skew uses already reaches the frame's newest returns.
            raw4 = lidar.get_latest_scan_timed() if trk.wants_lidar else None
            for s in link.drain():
                trk.on_imu(s)
            if raw4 and trk.wants_lidar:
                # returns with their measurement times -> de-skewed to the current pose (decision #38)
                pts, times = clean_and_project_timed(raw4, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)
                trk.on_lidar_frame(pts, times)
            for e in trk.events[shown:]:
                print()
                _print_event(e)
            shown = len(trk.events)
            if time.monotonic() - last_print > 0.5:
                st = link.status()
                print(f"\r  lane {trk.lane_index} (slot {trk.slot}, lap {trk.lap})  x {trk.x:6.0f}  y {trk.y:6.0f}  "
                      f"psi {trk.psi:+6.1f}  {'RE-CHECK' if trk.wants_lidar else '        '}  "
                      f"link {st['rate_hz']:5.1f} Hz gaps {st['seq_gaps']} {'STALE' if st['stale'] else ''}   ",
                      end="", flush=True)
                last_print = time.monotonic()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        link.stop()
        lidar.stop()
    print("\nseats per lane:")
    print(_seat_table(trk))


def run_replay(a):
    from stm32_link import read_log
    with open(a.replay_scan) as fh:
        dump = json.load(fh)
    raw = [tuple(p) for p in dump["raw"]]
    init = li.initialise(clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG))
    print(init_report(init))
    if not init.ok:
        return
    samples = read_log(a.replay_imu)
    if not samples:
        sys.exit(f"no valid $IMU lines in {a.replay_imu}")
    # Start from the sample that was current when the scan was taken (saved in
    # the dump by --real); a dump without it starts from the log's first line.
    k0 = 0
    if "imu_seq_at_scan" in dump:
        k0 = next((k for k, s in enumerate(samples) if s.seq == dump["imu_seq_at_scan"]), None)
        if k0 is None:
            sys.exit(f"the scan's IMU sample (seq {dump['imu_seq_at_scan']}) is not in {a.replay_imu}")
    trk = LaneTracker(init, samples[k0])
    for s in samples[k0 + 1:]:
        trk.on_imu(s)
    print(f"\n=== TRACKING (replayed {len(samples) - k0} samples from seq {samples[k0].seq}) ===")
    for e in trk.events:
        _print_event(e)
    print(f"\nfinal: lane {trk.lane_index} (slot {trk.slot}, lap {trk.lap}) x {trk.x:.0f} y {trk.y:.0f} "
          f"psi {trk.psi:+.1f}; distance {trk.distance_mm / 1000:.2f} m")
    print("\nseats per lane (the entry re-checks can't be replayed -- only the start scan is saved):")
    print(_seat_table(trk))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")
    mode.add_argument("--bench", action="store_true")
    mode.add_argument("--replay-scan", metavar="SCAN_JSON")
    ap.add_argument("--replay-imu", metavar="IMU_LOG")
    ap.add_argument("--log", metavar="FILE", help="save every raw STM32 line, overwriting FILE (--real / --bench)")
    ap.add_argument("--dump", metavar="FILE", help="save the start scan, overwriting FILE (--real)")
    ap.add_argument("--direction", default="CCW", choices=["CCW", "CW"])
    ap.add_argument("--slot", type=int, default=0, help="start lane: 0=S, then in driving order")
    ap.add_argument("--y", type=float, default=1400.0)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--placement-yaw", type=float, default=0.0)
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of STM32 lines lost (sim)")
    ap.add_argument("--enc-err", type=float, default=0.0, help="encoder calibration error, e.g. 0.02 (sim)")
    ap.add_argument("--speed", type=float, default=600.0, help="mm/s (sim)")
    ap.add_argument("--no-sweep", action="store_true", help="instantaneous LIDAR frames instead of real revolutions (sim)")
    ap.add_argument("--no-deskew", action="store_true", help="hand the re-check frames over without times (sim)")
    ap.add_argument("--stamp-error-ms", type=float, default=0.0,
                    help="error in the measured LIDAR_TIME_OFFSET_S, ms (sim)")
    a = ap.parse_args()
    if a.sim:
        run_sim(a)
    elif a.bench:
        run_bench(a)
    elif a.real:
        run_real(a)
    else:
        if not a.replay_imu:
            ap.error("--replay-scan needs --replay-imu")
        run_replay(a)


if __name__ == "__main__":
    main()
