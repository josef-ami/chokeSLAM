"""
Verification for run_track.py's HARDWARE paths -- --bench, --real, and --replay
of what --real saved -- with the two hardware libraries replaced by stubs:

  - pyserial's `serial.Serial` reads a real pseudo-terminal that a feeder
    thread writes simulated $IMU lines into, in real time at 100 Hz
    (simulation.SimStm32: chip yaw sign/zero, noise, drift, whole ticks);
  - lidar_source.RPLidarC1Source returns simulated scans (sensor-raw angles,
    upside-down mount). get_latest_scan_timed() -- what the entry re-check
    reads -- returns one real 100 ms revolution ending when it is called, each
    ray cast from the robot's true pose at the (real, time.monotonic()) time
    it was measured, with that time: the de-skew then runs on real clocks, the
    STM32 side timed through the real reader thread and LinkClock.

Everything else is the real code: run_track, stm32_link (thread, assembler,
parser, log), lane_init, lane_tracker, seat_occupancy.

Each scenario runs in its own child process (the stubs are installed into
sys.modules there), ~10 s each, real time:

  bench        robot still 1 s, turned 90 deg clockwise, rolled 500 mm:
               the bench display must read heading +90 and distance 500.
  real         robot still until tracking starts, then 1 lap at 1000 mm/s.
  real-moving  the robot starts driving the moment the start scan has been
               taken, and initialisation is made to take 0.8 s to compute (the
               robot is ~800 mm down the lane before the tracker exists): that
               motion must not be lost.
  replay       the scan + IMU log saved by `real-moving` (--dump / --log),
               replayed offline: must reproduce the live run's turns exactly.

Run:  python3 test_run_track.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


# =============================================================================
# child: one scenario, stubs installed, prints "RESULT <json>" as its last line
# =============================================================================
def child(mode: str, outdir: str):
    import _thread
    import fcntl

    import numpy as np
    import math
    import random
    import select
    import struct
    import termios
    import threading
    import time
    import tty
    import types

    sys.path.insert(0, HERE)
    import config
    import lane_frame as lf
    import simulation as sim

    master, slave = os.openpty()
    tty.setraw(slave)
    slave_path = os.ttyname(slave)
    opened = threading.Event()
    tracking = threading.Event()
    box = {"lidar_calls": 0, "lag": []}

    class Serial:                                         # pyserial stand-in
        def __init__(self, port, baudrate, timeout):
            assert port == config.IMU_PORT, port
            self.fd = os.open(slave_path, os.O_RDONLY | os.O_NOCTTY)
            self.timeout = timeout
            opened.set()

        @property
        def in_waiting(self):
            return struct.unpack("i", fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0"))[0]

        def read(self, n):
            r, _, _ = select.select([self.fd], [], [], self.timeout)
            return os.read(self.fd, n) if r else b""

    m = types.ModuleType("serial")
    m.Serial = Serial
    sys.modules["serial"] = m

    direction, slot, y0 = "CCW", 2, 1300.0
    rng = random.Random(11)
    path = sim.LoopPath(direction)
    s0 = path.s_at_lane_y(slot, y0)
    gx, gy, brg = path.pose(s0)
    sec0 = path.section(slot)
    x0, _ = lf.global_to_lane(sec0, direction, gx, gy)
    pillars, seat_truth = sim.random_lane_pillars(direction, rng, avoid=(sec0, x0, y0))
    pose = {"g": (gx, gy, brg)}
    track = [(time.monotonic(), s0)]                     # (Pi time, arc length) as the feeder moves the robot

    class RPLidarC1Source:                                # rplidarc1 stand-in
        def __init__(self, port, baudrate, timeout):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def get_latest_scan(self):
            g = pose["g"]
            raw = sim.simulate_scan(g[0], g[1], g[2], pillars, n_points=720, rng_noise=rng,
                                    blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                                    blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
            box["lidar_calls"] += 1
            box["scan_pose"] = g
            return [(sim.to_sensor_raw(a), d, q) for a, d, q in raw]

        def get_latest_scan_timed(self):
            now = time.monotonic()
            tr = list(track)
            ts, ss = [t for t, _ in tr], [x for _, x in tr]
            out, n, rev = [], 720, 0.1
            for k in range(n):
                raw_angle = k * 360.0 / n
                rel = (config.LIDAR_ANGLE_SIGN * raw_angle + config.LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360.0
                if abs((rel - config.REAR_BLIND_ARC_CENTER_DEG + 180.0) % 360.0 - 180.0) <= \
                        config.REAR_BLIND_ARC_WIDTH_DEG / 2.0:
                    continue
                t_k = now - rev * (1.0 - k / n)
                x, y, b = path.pose(float(np.interp(t_k, ts, ss)))
                for _, d, q in sim.simulate_scan(x, y, (b + rel) % 360.0, pillars, n_points=1, rng_noise=rng):
                    out.append((raw_angle, d, q, t_k))
            box["lidar_calls"] += 1
            box["scan_pose"] = path.pose(float(np.interp(now, ts, ss)))
            return out

    m = types.ModuleType("lidar_source")
    m.RPLidarC1Source = RPLidarC1Source
    sys.modules["lidar_source"] = m

    import lane_init
    import lane_tracker
    import run_track

    orig_init = lane_tracker.LaneTracker.__init__

    def tracker_init(self, *a, **k):
        orig_init(self, *a, **k)
        box["trk"] = self
        box["moved_before_tracker_mm"] = math.hypot(pose["g"][0] - gx, pose["g"][1] - gy)
        tracking.set()

    lane_tracker.LaneTracker.__init__ = tracker_init
    orig_frame = lane_tracker.LaneTracker.on_lidar_frame

    def tracker_frame(self, points, times=None):
        # the pose the tracker judges the frame from (its lane pose at the frame's end, from the
        # history) vs the true pose at the frame's end
        g = box["scan_pose"]
        lp = self._hist.lane_pose_at(min(max(times) - config.LIDAR_TIME_OFFSET_S, self.t_now))
        if lp is not None:
            k, x, y, psi = lp
            sec = path.section(slot + k)
            tx, ty = lf.lane_to_global(sec, direction, x, y)
            th = lf.yaw_to_heading(psi, sec, direction)
            box["lag"].append((math.hypot(tx - g[0], ty - g[1]), abs((g[2] - th + 180) % 360 - 180)))
        box["timed_frames"] = box.get("timed_frames", 0) + (times is not None)
        r = orig_frame(self, points, times)
        if self.last_deskew is not None and times is not None:
            box["max_shift"] = max(box.get("max_shift", 0.0), self.last_deskew.max_shift_mm)
        return r

    lane_tracker.LaneTracker.on_lidar_frame = tracker_frame

    def write(stm, t_ms, dist, b):
        os.write(master, (stm.line(t_ms, dist, b) + "\n").encode())

    def feeder_bench():
        stm = sim.SimStm32(random.Random(3), yaw_noise_deg=0.0, yaw_drift_deg_s=0.0)
        opened.wait(10)
        t_ms, b = 0, 0.0
        for phase in ("still", "turn", "roll", "idle"):
            for k in range(100):
                t_ms += 10
                if phase == "turn":
                    b = 90.0 * (k + 1) / 100
                d = {"still": 0.0, "turn": 0.0, "roll": 5.0 * (k + 1), "idle": 500.0}[phase]
                write(stm, t_ms, d, b)
                time.sleep(0.01)
        time.sleep(0.3)
        _thread.interrupt_main()

    def feeder_real(move_after_scan: bool, speed=1000.0):
        stm = sim.SimStm32(random.Random(3))
        opened.wait(10)
        t_ms, dist, s = 0, 0.0, s0
        px, py = gx, gy
        t0 = time.monotonic()

        def pace():
            time.sleep(max(0.0, t0 + t_ms / 1000.0 - time.monotonic()))

        while not (box["lidar_calls"] >= 1 if move_after_scan else tracking.is_set()):
            t_ms += 10
            write(stm, t_ms, dist, brg)
            pace()
        while s < s0 + path.length + 300:
            t_ms += 10
            s += speed * 0.01
            g = path.pose(s)
            dist += math.hypot(g[0] - px, g[1] - py)
            px, py = g[0], g[1]
            pose["g"] = g
            write(stm, t_ms, dist, g[2])
            track.append((t0 + t_ms / 1000.0, s))     # the scheduled time: what the STM32's t_ms says
            pace()
        time.sleep(0.5)
        _thread.interrupt_main()

    if mode == "bench":
        threading.Thread(target=feeder_bench, daemon=True).start()
        sys.argv = ["run_track.py", "--bench"]
    else:
        if mode == "real-moving":
            orig_initialise = lane_init.initialise

            def slow_initialise(points):
                r = orig_initialise(points)
                time.sleep(0.8)
                return r

            lane_init.initialise = slow_initialise
        threading.Thread(target=feeder_real, args=(mode == "real-moving",), daemon=True).start()
        sys.argv = ["run_track.py", "--real", "--log", f"{outdir}/imu.log", "--dump", f"{outdir}/scan.json"]
    run_track.main()

    out = {"mode": mode}
    if mode != "bench":
        trk = box["trk"]
        wrong = decided = unknown = 0
        for k, rec in trk.lanes.items():
            truth = seat_truth[path.section(slot + k)]
            for i, st in rec.seats.items():
                if st.state == "unknown":
                    unknown += 1
                    continue
                decided += 1
                wrong += (st.state == "occupied") != (i in truth)
        out.update(turns=[e.detail for e in trk.events if e.kind == "turn"],
                   wrong=wrong, decided=decided, unknown=unknown,
                   lidar_frames=len(box["lag"]), timed_frames=box.get("timed_frames", 0),
                   max_shift_mm=box.get("max_shift", 0.0),
                   lag_mm_max=max(d for d, _ in box["lag"]), lag_deg_max=max(h for _, h in box["lag"]),
                   moved_before_tracker_mm=box["moved_before_tracker_mm"])
    print("RESULT " + json.dumps(out))


# =============================================================================
# parent: run the scenarios, check them
# =============================================================================
def _run(args, timeout=120):
    p = subprocess.run([sys.executable] + args, cwd=HERE, capture_output=True, text=True, timeout=timeout)
    return p.returncode, p.stdout.replace("\r", "\n"), p.stderr


def _result(stdout):
    lines = [ln for ln in stdout.splitlines() if ln.startswith("RESULT ")]
    assert lines, stdout[-2000:]
    return json.loads(lines[-1][7:])


def test_bench():
    rc, out, err = _run([__file__, "--child", "bench", "."])
    assert rc == 0, err[-2000:]
    last = [ln for ln in out.splitlines() if "heading" in ln and "distance" in ln][-1]
    heading = float(last.split("heading")[1].split("deg")[0])
    dist = float(last.split("distance")[1].split("mm")[0])
    assert abs(heading - 90.0) < 0.5 and abs(dist - 500.0) < 1.0, last
    print(f"PASS  test_bench                    turned 90 deg clockwise, rolled 500 mm -> display "
          f"heading {heading:+.2f} deg, distance {dist:.1f} mm")


def test_real_and_replay():
    with tempfile.TemporaryDirectory() as d:
        for mode in ("real", "real-moving"):          # both write d/imu.log: --log overwrites, never appends
            rc, out, err = _run([__file__, "--child", mode, d])
            assert rc == 0, err[-2000:]
            r = _result(out)
            assert len(r["turns"]) == 4, r["turns"]
            assert r["wrong"] == 0 and r["decided"] >= 15, r
            # Gross-error guard (the bug of section 9.10 item 1 showed 94-200 mm). The degree bound is loose
            # because under heavy CPU load the feeder thread falls behind its own schedule, which smears this
            # harness's truth timeline (seen up to ~3.5 deg), not the tracker's.
            assert r["lag_mm_max"] < 60 and r["lag_deg_max"] < 5.0, r
            assert r["timed_frames"] == r["lidar_frames"] > 0, r      # every re-check frame was de-skewed
            if mode == "real-moving":
                assert r["moved_before_tracker_mm"] > 500, r
            print(f"PASS  {'test_' + mode:30s}4/4 turns; entry re-checks + init: {r['decided']} decided, "
                  f"{r['unknown']} unknown, 0 wrong; {r['lidar_frames']} real-sweep LIDAR frames, all de-skewed "
                  f"(returns moved up to {r['max_shift_mm']:.0f} mm); the pose each frame was judged from (the "
                  f"tracked pose at the frame's end) was "
                  f"within {r['lag_mm_max']:.0f} mm / {r['lag_deg_max']:.1f} deg of the truth"
                  + (f"; the robot was {r['moved_before_tracker_mm']:.0f} mm down the lane when the "
                     f"tracker was created" if mode == "real-moving" else ""))
        live = r["turns"]
        rc, out, err = _run(["run_track.py", "--replay-scan", f"{d}/scan.json", "--replay-imu", f"{d}/imu.log"])
        assert rc == 0, err[-2000:]
        replay = [ln.split("turn", 1)[1].strip() for ln in out.splitlines() if " turn " in ln]
        assert replay == live, (replay, live)
        dump = json.load(open(f"{d}/scan.json"))
        print(f"PASS  test_replay                   the saved scan + IMU log (from seq {dump['imu_seq_at_scan']}, "
              f"the sample current at the scan) replay the live run's {len(live)} turns identically")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "--child":
        child(sys.argv[2], sys.argv[3])
        sys.exit(0)
    test_bench()
    test_real_and_replay()
    print("\nAll run_track hardware-path checks passed.")
