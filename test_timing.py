"""
Verification for timing.py (the time base the de-skew relies on) and for
lidar_source.py's timestamps.

  - LinkClock: STM32 lines arriving with USB jitter -> the mapped times keep
    only the smallest delivery delay; an STM32 restart is followed.
  - SweepClock: a LIDAR stream delivered point by point, or in bursts, with a
    spin rate that changes -> each return's time comes out as its
    measurement time + the smallest delivery delay, to well under 1 ms;
    arrival stamps alone would be off by up to a whole burst.
  - lidar_source.RPLidarC1Source end to end, with a stand-in for the
    rplidarc1 package (its assumed API: RPLidar(port, baud, timeout=...),
    simple_scan(), output_queue of {"a_deg", "d_mm", "q"}, stop_event,
    reset()) that delivers a simulated stream in bursts, in real time.

Run:  python3 test_timing.py
"""
from __future__ import annotations

import asyncio
import math
import random
import sys
import threading
import time
import types

import numpy as np

from timing import LinkClock, SweepClock


def test_link_clock():
    rng = random.Random(1)
    clk = LinkClock()
    stm_zero, pi_zero, min_delay = 12.345, 1000.0, 0.0015
    errs = []
    for k in range(1000):                                   # 10 s at 100 Hz
        t_ms = int(stm_zero * 1000) + 10 * k
        true_pi = pi_zero + (t_ms / 1000.0 - stm_zero)      # when it was sampled, on the Pi clock
        rx = true_pi + min_delay + rng.expovariate(1 / 0.002)   # USB + thread jitter, mean 2 ms
        mapped = clk.update(t_ms, rx)
        if k >= 100:
            errs.append(mapped - true_pi - min_delay)
    errs = np.array(errs)
    assert np.max(np.abs(errs)) < 0.0005, (errs.min(), errs.max())
    # restart: t_ms starts again from 0 with a new relation to the Pi clock
    for k in range(200):
        t_ms = 10 * k
        true_pi = 2000.0 + t_ms / 1000.0
        mapped = clk.update(t_ms, true_pi + min_delay + rng.expovariate(1 / 0.002))
    assert abs(mapped - true_pi - min_delay) < 0.0005, mapped - true_pi
    assert clk.update(5, 0.0) == 0.005                      # no arrival time: STM32 clock alone
    print(f"PASS  test_link_clock               arrival jitter mean 2 ms: mapped time within "
          f"{np.max(np.abs(errs)) * 1000:.2f} ms of sample time + the smallest delay; follows an STM32 restart")


def _stream(duration_s, spin_hz, rate_hz, delivery, rng, spin_change=None):
    """Simulated returns: (raw angle, true measurement time, arrival time).
    delivery: ("trickle", latency_s, jitter_s) or ("burst", latency_s, period_s).
    spin_change: None, ("ramp", hz_at_end) or ("step", hz_after_half_time)."""
    out, t, phase = [], 0.0, rng.uniform(0, 360)
    dt = 1.0 / rate_hz
    while t < duration_s:
        if spin_change is None:
            hz = spin_hz
        elif spin_change[0] == "ramp":
            hz = spin_hz + (spin_change[1] - spin_hz) * t / duration_s
        else:
            hz = spin_hz if t < duration_s / 2 else spin_change[1]
        phase = (phase + 360.0 * hz * dt) % 360.0
        if delivery[0] == "trickle":
            arr = t + delivery[1] + rng.uniform(0, delivery[2])
        else:
            arr = math.ceil(t / delivery[2]) * delivery[2] + delivery[1]
        out.append((phase, t, arr))
        t += dt
    out.sort(key=lambda p: p[2])                            # the Pi sees them in arrival order
    return out


def test_sweep_clock():
    rng = random.Random(2)
    cases = [("trickle, 3 ms + 0-2 ms jitter", ("trickle", 0.003, 0.002), 10.0, None, 0.003),
             ("bursts every 30 ms, +5 ms", ("burst", 0.005, 0.030), 10.0, None, 0.005),
             ("bursts every 50 ms, spin drifting 10->10.3 Hz", ("burst", 0.005, 0.050), 10.0, ("ramp", 10.3), 0.005)]
    for name, delivery, hz, hz2, min_delay in cases:
        clk = SweepClock()
        errs, arr_errs = [], []
        for k, (a, t_true, arr) in enumerate(_stream(4.0, hz, 5000.0, delivery, rng, hz2)):
            th = clk.add(a, arr)
            if k > 5000 and k % 7 == 0:                      # after 1 s; sample the error
                f = clk.fit()
                errs.append(SweepClock.time_of(th, f) - t_true - min_delay)
                arr_errs.append(arr - t_true - min_delay)
        errs, arr_errs = np.abs(errs), np.abs(arr_errs)
        assert np.percentile(errs, 99) < 0.001, (name, np.percentile(errs, 99))
        print(f"PASS  test_sweep_clock              {name:38s}: sweep time within "
              f"{np.percentile(errs, 99) * 1000:.2f} ms (99%) of measurement + smallest delay; "
              f"arrival stamps alone up to {arr_errs.max() * 1000:.1f} ms")
    # INFO: an abrupt 10% change of spin rate (not expected from a speed-controlled
    # motor except at start-up) is followed within one window (1 s)
    clk = SweepClock()
    during, after = [], []
    for k, (a, t_true, arr) in enumerate(_stream(4.0, 10.0, 5000.0, ("burst", 0.005, 0.030), rng, ("step", 11.0))):
        th = clk.add(a, arr)
        if k > 5000 and k % 7 == 0:
            e = abs(SweepClock.time_of(th, clk.fit()) - t_true - 0.005)
            (during if 2.0 <= t_true < 3.0 else after if t_true >= 3.0 else []).append(e)
    print(f"INFO  test_sweep_clock              abrupt spin step 10 -> 11 Hz: error up to {max(during) * 1000:.1f} ms "
          f"during the next 1 s, then within {np.percentile(after, 99) * 1000:.2f} ms (99%)")
    clk = SweepClock()
    for a, t, arr in _stream(0.05, 10.0, 5000.0, ("trickle", 0.003, 0.0), rng):
        clk.add(a, arr)
    assert clk.fit() is None                                # half a revolution: no fit yet
    print("PASS  test_sweep_clock_needs_data   no fit before about one revolution of data")


def _fake_rplidarc1(spin_hz, burst_s, latency_s, truth):
    """Stand-in for the rplidarc1 package, real time, bursts."""
    class RPLidar:
        def __init__(self, port, baudrate, timeout=None):
            self.output_queue = asyncio.Queue()
            self.stop_event = threading.Event()

        async def simple_scan(self):
            t0 = time.monotonic()
            phase, sent_until = 0.0, t0
            while not self.stop_event.is_set():
                await asyncio.sleep(burst_s)
                now = time.monotonic() - latency_s          # everything measured up to here goes out now
                n = int((now - sent_until) * 5000)
                for i in range(n):
                    t_meas = sent_until + (i + 1) / 5000.0
                    phase_i = (phase + 360.0 * spin_hz * (i + 1) / 5000.0) % 360.0
                    dist = 1000.0 + len(truth) * 1e-4           # unique, so each return can be looked up
                    truth[(round(phase_i, 6), dist)] = t_meas
                    self.output_queue.put_nowait({"a_deg": phase_i, "d_mm": dist, "q": 47})
                phase = (phase + 360.0 * spin_hz * n / 5000.0) % 360.0
                sent_until += n / 5000.0

        def reset(self):
            pass

    m = types.ModuleType("rplidarc1")
    m.RPLidar = RPLidar
    return m


def test_lidar_source_timestamps():
    truth = {}
    sys.modules["rplidarc1"] = _fake_rplidarc1(spin_hz=10.0, burst_s=0.04, latency_s=0.004, truth=truth)
    from lidar_source import RPLidarC1Source
    src = RPLidarC1Source("/dev/fake", 460800, 0.2)
    src.start()
    time.sleep(2.0)
    timed = src.get_latest_scan_timed()
    plain = src.get_latest_scan()
    since = src.get_points_since(time.monotonic() - 0.5)
    ts = src.timing_status()
    src.stop()
    errs = np.array([t - truth[(round(a, 6), d)] for a, d, q, t in timed])
    arrival_errs = np.array([p[4] - truth[(round(p[0], 6), p[1])] for p in since])
    # What the de-skew needs is that every return carries the SAME delay: the
    # constant part (here the stand-in's latency plus event-loop overshoot) is
    # what LIDAR_TIME_OFFSET_S calibrates out on the robot.
    const = float(np.median(errs))
    spread = np.percentile(np.abs(errs - const), 99)
    assert len(timed) == len(plain) >= 300, (len(timed), len(plain))
    assert spread < 0.001 and 0.0 < const < 0.020, (spread, const)
    assert abs(ts["spin_hz"] - 10.0) < 0.05 and len(since) > 1000, ts
    print(f"PASS  test_lidar_source_timestamps  stand-in rplidarc1, 10 Hz, bursts every 40 ms, real time: "
          f"{len(timed)} buckets; every return's time = measurement + {const * 1000:.1f} ms "
          f"(constant) within {spread * 1000:.2f} ms (99%); arrival stamps alone spread "
          f"{np.ptp(arrival_errs) * 1000:.0f} ms; spin measured {ts['spin_hz']:.2f} Hz")


if __name__ == "__main__":
    test_link_clock()
    test_sweep_clock()
    test_lidar_source_timestamps()
    print("\nAll timing checks passed.")
