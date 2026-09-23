"""
One time base for the LIDAR returns and the STM32 samples, so the entry
re-check can de-skew a LIDAR frame with the tracked pose history (decision
#38, docs/CHANGES.md section 9.11).

Everything is expressed on the Pi's time.monotonic() clock. Two sources of
error are removed here; the one constant that remains (how much later a
LIDAR return reaches the Pi than an STM32 line does, measured from when each
was taken) is config.LIDAR_TIME_OFFSET_S, measured on the robot with
measure_lidar_delay.py.

LinkClock -- STM32 samples.
    Each $IMU line carries t_ms, the STM32's own clock when it was sampled,
    and rx_time, the Pi's clock when it arrived. Arrival jitters (USB polling,
    the reader thread); t_ms doesn't. So:
        pi_time(t_ms) = t_ms / 1000 + offset,
        offset        = min over the last window of (rx_time - t_ms / 1000)
    i.e. the offset is set by the least-delayed line in the window: the
    jitter drops out, only the smallest (constant) delivery delay stays in.
    The window slides, so the two crystals' slow drift is followed. If t_ms
    goes backwards (STM32 restart) the window starts over.

SweepClock -- LIDAR returns.
    A return's arrival time on the Pi is only an upper bound on when it was
    measured: the driver may hand points over in bursts. But the sensor spins
    at a steady rate, so a return's measurement time follows from its angle:
    the raw angle increases through each revolution (RPLIDAR convention), so
    the angle unwrapped across revolutions, theta, grows linearly with time
    (each return is unwrapped to the nearest turn of the previous one, so the
    360 -> 0 wrap and returns delivered slightly out of order are both right):
        t(theta) = t0 + theta * sec_per_deg
    sec_per_deg: least-squares slope over the last window, fitted through
    the least-delayed returns (the lowest 5% under a first fit, twice).
    t0: set by the least-delayed return, min over the window of
    (arrival - theta * sec_per_deg). With points trickling in one by one this
    is just the arrival time; with bursts it is still the measurement time
    plus the smallest delivery delay.
"""
from __future__ import annotations

from collections import deque

import numpy as np


class LinkClock:
    def __init__(self, window_s: float = 5.0):
        self.window_s = window_s
        self._w: deque = deque()          # (t_ms, rx_time - t_ms / 1000)
        self.offset: float | None = None

    def update(self, t_ms: int, rx_time: float) -> float:
        """Add one sample; returns its time on the Pi clock. A sample with no
        arrival time (rx_time <= 0, e.g. built by hand in a test) is timed by
        the STM32 clock alone."""
        if rx_time <= 0.0:
            return t_ms / 1000.0
        if self._w and t_ms < self._w[-1][0]:
            self._w.clear()                               # STM32 restarted: new clock
        d = rx_time - t_ms / 1000.0
        self._w.append((t_ms, d))
        while self._w and t_ms - self._w[0][0] > self.window_s * 1000.0:
            self._w.popleft()
        self.offset = min(x for _, x in self._w)
        return t_ms / 1000.0 + self.offset


class SweepClock:
    MIN_POINTS = 50
    MIN_HZ, MAX_HZ = 2.0, 30.0

    def __init__(self, window_s: float = 1.0):
        self.window_s = window_s
        self._w: deque = deque()          # (theta, arrival)
        self._last_theta: float | None = None
        self.backwards = 0                # raw angle stepped back by < 180: out-of-order or reversed sweep
        self._fit: tuple[float, float] | None = None
        self._n_added = 0                 # total returns added (the fit is cached per value)
        self._fit_n = -1

    def add(self, raw_angle_deg: float, arrival: float) -> float:
        """Add one return as it arrives; returns its unwrapped angle theta."""
        a = raw_angle_deg % 360.0
        if self._last_theta is None:
            theta = a
        else:
            # the nearest unwrapping to the previous return: correct across the
            # 360 -> 0 wrap, and for a return delivered slightly out of order
            step = (a - self._last_theta + 180.0) % 360.0 - 180.0
            if step < 0:
                self.backwards += 1
            theta = self._last_theta + step
        self._last_theta = theta
        self._w.append((theta, arrival))
        while self._w and arrival - self._w[0][1] > self.window_s:
            self._w.popleft()
        self._n_added += 1
        return theta

    def fit(self) -> tuple[float, float] | None:
        """(t0, sec_per_deg), or None while there isn't at least about one
        revolution of data or the rate is implausible."""
        if self._fit_n == self._n_added:
            return self._fit
        if len(self._w) < self.MIN_POINTS:
            return None
        w = np.asarray(self._w, dtype=float)
        th, t = w[:, 0], w[:, 1]
        if th.max() - th.min() < 300.0:
            return None
        th0 = th.min()                                    # fit in a local origin for precision
        # The slope is fitted through the least-delayed returns: a first fit
        # through all of them, then twice more through the 5% that sit lowest
        # (arrived earliest for their angle) under the previous fit. With
        # burst delivery each burst contributes its last-measured returns to
        # that 5%, all with the same small spread of delays, so the fit runs
        # parallel to the delivery envelope; a fit through all returns would
        # be tilted by the bursts' sawtooth at the window's two ends.
        x = th - th0
        slope, icpt = np.polyfit(x, t, 1)
        for _ in range(2):
            r = t - slope * x
            low = r <= np.percentile(r, 5.0)
            if low.sum() < 10 or np.ptp(x[low]) < 180.0:
                break
            slope, icpt = np.polyfit(x[low], t[low], 1)
        if not slope > 0:
            return None
        hz = 1.0 / (slope * 360.0)
        if not (self.MIN_HZ <= hz <= self.MAX_HZ):
            return None
        t0 = float(np.min(t - slope * (th - th0))) - slope * th0
        self._fit, self._fit_n = (t0, float(slope)), self._n_added
        return self._fit

    @staticmethod
    def time_of(theta: float, fit: tuple[float, float]) -> float:
        t0, slope = fit
        return t0 + theta * slope

    def spin_hz(self) -> float | None:
        f = self.fit()
        return None if f is None else 1.0 / (f[1] * 360.0)
