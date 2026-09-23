"""
Verification for measure_lidar_delay.py: simulated recordings of the robot
turned by hand on the spot (+/-30 deg at 0.7 Hz, 10 s), with a KNOWN LIDAR vs
STM32 offset, LIDAR returns delivered in bursts or one by one, STM32 lines with
USB jitter, the two clocks on unrelated zeros, a lever arm, and the robot's
centre wandering unseen. The offset must be recovered; a recording where the
robot hardly turned must be reported as not reliable.

Run:  python3 test_lidar_delay.py
"""
from __future__ import annotations

import measure_lidar_delay as m


def test_offset_recovered():
    cases = [("offset +13 ms, bursts every 25 ms", {"true_offset_s": 0.013}),
             ("offset -20 ms", {"true_offset_s": -0.020}),
             ("offset +30 ms", {"true_offset_s": 0.030}),
             ("offset +13 ms, bursts every 50 ms", {"true_offset_s": 0.013, "burst_s": 0.050}),
             ("offset +13 ms, returns one by one", {"true_offset_s": 0.013, "burst_s": 0.0002}),
             ("offset +13 ms, lever arm 110 ahead / 60 left", {"true_offset_s": 0.013, "lever": (110.0, 60.0)}),
             ("offset +13 ms, centre wanders 20 mm unseen", {"true_offset_s": 0.013, "wander_mm": 20.0})]
    worst = 0.0
    for name, kw in cases:
        r = m.analyse(m.simulate_recording(**kw))
        err = (r["offset_s"] - kw["true_offset_s"]) * 1000.0
        worst = max(worst, abs(err))
        assert abs(err) < 3.0 and "NOT RELIABLE" not in m.report(r), (name, err, m.report(r))
        print(f"      {name:46s}: measured {r['offset_s'] * 1000:+6.1f} ms (error {err:+.1f} ms); mismatch "
              f"{r['cost_min_mm']:.1f} mm at the best offset vs {r['cost_at_0_mm']:.1f} mm at 0")
    print(f"PASS  test_offset_recovered         {len(cases)} simulated recordings: offset recovered within "
          f"{worst:.1f} ms (the de-skew tolerates ~10 ms)")


def test_no_turn_is_flagged():
    r = m.analyse(m.simulate_recording(true_offset_s=0.013, swing_deg=1.0))
    rep = m.report(r)
    assert "NOT RELIABLE" in rep and "hardly turned" in rep, rep
    print("PASS  test_no_turn_is_flagged       a recording where the robot turned only +/-1 deg is reported NOT RELIABLE")




# =============================================================================
# --real's recording path, end to end in real time with stand-in hardware
# =============================================================================
def _child_record(out_path):
    """Runs measure_lidar_delay.record_real() against a pseudo-terminal STM32
    and a stand-in rplidarc1, both driven by the same simulated hand-turned
    robot on the real clock, and writes the truth next to the recording."""
    import asyncio
    import fcntl
    import json
    import math
    import os
    import select
    import struct
    import sys
    import termios
    import threading
    import time
    import tty
    import types

    import config
    import lane_frame as lf
    import simulation as sim

    T0 = time.monotonic()
    phase_of = {"move_at": None, "stop_at": None}

    def heading(t):
        mv, sp = phase_of["move_at"], phase_of["stop_at"]
        if mv is None or t < mv:
            return 0.0
        t_end = t if sp is None else min(t, sp)
        return 30.0 * math.sin(2 * math.pi * 0.7 * (t_end - mv)) * min(1.0, (t_end - mv) / 0.5)

    gx, gy = lf.lane_to_global("S", "CCW", 500.0, 1400.0)
    north = lf.grid_north_bearing("S", "CCW")
    pillars = sim.rulebook_pillars("S", "CCW", [2, 5])
    truth_lidar, truth_imu = {}, {}

    # --- STM32 over a pseudo-terminal
    master, slave = os.openpty()
    tty.setraw(slave)
    slave_path = os.ttyname(slave)

    class Serial:
        def __init__(self, port, baudrate, timeout):
            self.fd = os.open(slave_path, os.O_RDONLY | os.O_NOCTTY)
            self.timeout = timeout

        @property
        def in_waiting(self):
            return struct.unpack("i", fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0"))[0]

        def read(self, n):
            r, _, _ = select.select([self.fd], [], [], self.timeout)
            return os.read(self.fd, n) if r else b""

    sm = types.ModuleType("serial")
    sm.Serial = Serial
    sys.modules["serial"] = sm
    stop = threading.Event()

    def feeder():
        seq, t_next = 0, time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            seq += 1
            t_ms = int(round((now - T0) * 1000)) + 5000
            truth_imu[seq] = (t_ms, now)
            yaw = (config.IMU_YAW_SIGN * heading(now) + 20.0 + 180.0) % 360.0 - 180.0
            os.write(master, f"$IMU,{seq},{t_ms},0,{yaw:.2f}\n".encode())
            t_next += 0.01
            time.sleep(max(0.0, t_next - time.monotonic()))

    threading.Thread(target=feeder, daemon=True).start()

    # --- stand-in rplidarc1: 5000 returns/s, 10.2 Hz, bursts every 25 ms
    class RPLidar:
        def __init__(self, port, baudrate, timeout=None):
            self.output_queue = asyncio.Queue()
            self.stop_event = threading.Event()

        async def simple_scan(self):
            sent_until, phase, k = time.monotonic(), 0.0, 0
            while not self.stop_event.is_set():
                await asyncio.sleep(0.025)
                now = time.monotonic()
                n = int((now - sent_until) * 5000)
                for i in range(n):
                    t_meas = sent_until + (i + 1) / 5000.0
                    phase = (phase + 360.0 * 10.2 / 5000.0) % 360.0
                    rel = (config.LIDAR_ANGLE_SIGN * phase + config.LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360.0
                    for _, d, q in sim.simulate_scan(gx, gy, (north + heading(t_meas) + rel) % 360.0, pillars,
                                                     n_points=1, dropout_prob=0.0):
                        k += 1
                        d = round(d) + k * 1e-6                  # unique tag -> truth lookup
                        truth_lidar[d] = t_meas
                        self.output_queue.put_nowait({"a_deg": phase, "d_mm": d, "q": q})
                sent_until += n / 5000.0

        def reset(self):
            pass

    rm = types.ModuleType("rplidarc1")
    rm.RPLidar = RPLidar
    sys.modules["rplidarc1"] = rm

    import measure_lidar_delay as mld
    real_print = print

    def hooked_print(*a, **kw):
        msg = " ".join(str(x) for x in a)
        if "TURN it now" in msg:
            phase_of["move_at"] = time.monotonic()
        if msg.startswith("STOP"):
            phase_of["stop_at"] = time.monotonic()
        real_print(*a, **kw)

    mld.print = hooked_print
    rec = mld.record_real(out_path)
    stop.set()
    # the truth: each side's actual delay, as the analysis' clocks see it
    t_lid, _, _ = mld._lidar_times(rec["lidar"])
    lid = [t - truth_lidar[p[1]] for p, t in zip(rec["lidar"], t_lid) if p[1] in truth_lidar and t == t]
    from timing import LinkClock
    clk = LinkClock()
    imu = [clk.update(t_ms, rx) - truth_imu[seq][1] for seq, t_ms, enc, yaw, rx in rec["imu"] if seq in truth_imu]
    import numpy as np
    true_offset = float(np.median(lid) - np.median(imu))
    r = mld.analyse(rec)
    real_print(mld.report(r))
    real_print("RESULT " + json.dumps({"true_offset_s": true_offset, "offset_s": r["offset_s"],
                                       "reliable": "NOT RELIABLE" not in mld.report(r),
                                       "n_imu": len(rec["imu"]), "n_lidar": len(rec["lidar"])}))


def test_record_real_end_to_end():
    import json
    import os
    import subprocess
    import sys
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "delay.json")
        p = subprocess.run([sys.executable, __file__, "--child-record", path], capture_output=True, text=True,
                           timeout=120, cwd=os.path.dirname(os.path.abspath(__file__)))
        assert p.returncode == 0, p.stderr[-3000:]
        line = [ln for ln in p.stdout.splitlines() if ln.startswith("RESULT ")][-1]
        r = json.loads(line[7:])
        assert os.path.getsize(path) > 0
        rep = m.analyse(json.load(open(path)))                  # --replay of the saved file gives the same
        assert abs(rep["offset_s"] - r["offset_s"]) < 1e-9
    err = (r["offset_s"] - r["true_offset_s"]) * 1000.0
    assert r["reliable"] and abs(err) < 3.0, (r, p.stdout[-2000:])
    print(f"PASS  test_record_real_end_to_end   --real's recording path, real time, stand-in STM32 (pseudo-terminal) "
          f"and rplidarc1 (bursts): {r['n_imu']} STM32 lines, {r['n_lidar']} returns; measured offset "
          f"{r['offset_s'] * 1000:+.1f} ms, actual {r['true_offset_s'] * 1000:+.1f} ms (error {err:+.1f} ms); "
          f"the saved recording replays to the same value")


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 3 and sys.argv[1] == "--child-record":
        _child_record(sys.argv[2])
        sys.exit(0)
    test_offset_recovered()
    test_no_turn_is_flagged()
    test_record_real_end_to_end()
    print("\nAll LIDAR-delay checks passed.")
