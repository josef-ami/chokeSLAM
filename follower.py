"""
Pure-pursuit path follower on the tracker's pose (checkpoint E, decision #67).

The Pi closes the loop: every control tick it takes the tracker's pose
(rear-axle midpoint, loop frame, field_map.tracker_pose), finds where it is
on the planned Path, aims at a point one lookahead distance further along,
and turns that into a road-wheel angle (+ = LEFT, as the DRIVE frame wants)
and a speed. The STM32 only executes (servo + speed loop).

    lookahead  Ld = clamp(v * PP_LOOKAHEAD_S, PP_LOOKAHEAD_MIN_MM, PP_LOOKAHEAD_MAX_MM)
    alpha      angle from the car's heading to the lookahead point
    curvature  k = 2 sin(alpha) / Ld          (pure pursuit, rear axle)
    steer      delta = atan(WHEELBASE_MM * k), clamped to the lock (left / right)
    speed      min(cruise, sqrt(LAT_ACCEL * R) on arcs ahead, sqrt(2 DECEL * remaining))

FOLLOWER_MODE = "rwf" (the default, found necessary in simulation -- see
docs/CHANGES.md E) replaces the pure-pursuit steering law with REAR-WHEEL
FEEDBACK (Paden et al. 2016, the standard tracking law for a car referenced at
its rear axle). Pure pursuit steers at a point one lookahead ahead and so cuts
every arc inward by roughly Ld^2 / (2 R) -- tens of mm on this car's tight
S-bends, against a planned clearance of 20 mm. Rear-wheel feedback feeds the
path's own curvature forward and corrects the lateral error e (+ = left of the
path) and heading error eh:
    k = k_ref cos(eh) / (1 - k_ref e) - K_H eh - K_E e sin(eh)/eh
with K_E = 1 / RWF_LENGTH_MM^2 and K_H = 2 RWF_DAMPING / RWF_LENGTH_MM, i.e. a
lateral error dies out over about RWF_LENGTH_MM of travel. The reference is
taken at the nearest point; only the curvature fed forward is taken
RWF_PREVIEW_S ahead, to cover the servo lag and the link latency.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import config


@dataclass
class DriveCmd:
    steer_deg: float        # road-wheel angle, + = LEFT
    speed_mm_s: float       # + = forward; 0 with stop=True = stand still
    stop: bool = False


def speed_profile(samples: np.ndarray, cruise: float, v_end: float = 0.0) -> np.ndarray:
    """Speed limit per sample: cruise, arcs (lateral acceleration), braking to v_end at the end."""
    k = np.abs(samples[:, 3])
    v = np.full(len(samples), float(cruise))
    with np.errstate(divide="ignore"):
        v_arc = np.sqrt(config.LAT_ACCEL_MM_S2 / np.maximum(k, 1e-9))
    v = np.minimum(v, v_arc)
    # look ahead: a sample's limit also respects braking into any slower sample ahead
    s = samples[:, 4]
    rem = s[-1] - s
    v = np.minimum(v, np.sqrt(v_end ** 2 + 2.0 * config.DECEL_MM_S2 * rem))
    for i in range(len(v) - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * config.DECEL_MM_S2 * ds))
    return np.maximum(v, 0.0)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    def __init__(self, path, cruise_mm_s: float, stop_at_end: bool = True):
        self.path = path
        self.S = path.samples
        self.vmax = speed_profile(self.S, cruise_mm_s, 0.0 if stop_at_end else cruise_mm_s)
        self.stop_at_end = stop_at_end
        self.i = 0                         # progress index (monotonic)
        self.done = False
        self.cross_track_mm = 0.0

    def _nearest(self, X: float, Y: float) -> int:
        lo, hi = self.i, min(len(self.S), self.i + 80)     # search a window ahead (10 mm samples)
        d = np.hypot(self.S[lo:hi, 0] - X, self.S[lo:hi, 1] - Y)
        j = lo + int(np.argmin(d))
        self.cross_track_mm = float(d[j - lo])
        return j

    def remaining_mm(self) -> float:
        return float(self.S[-1, 4] - self.S[self.i, 4])

    def update(self, X: float, Y: float, th: float, v_meas: float) -> DriveCmd:
        if self.done:
            return DriveCmd(0.0, 0.0, stop=True)
        self.i = self._nearest(X, Y)
        # finished: at (or past) the end
        ex, ey, eth = self.S[-1, :3]
        along = (X - ex) * math.cos(eth) + (Y - ey) * math.sin(eth)
        if self.i >= len(self.S) - 2 or (self.remaining_mm() < 30.0 and along > -15.0):
            if self.stop_at_end:
                self.done = True
                return DriveCmd(0.0, 0.0, stop=True)
        v_lim = float(self.vmax[self.i])
        v_ref = max(v_lim, config.SPEED_MIN_MM_S) if self.remaining_mm() > 40.0 else v_lim
        Ld = min(max(max(v_meas, v_ref) * config.PP_LOOKAHEAD_S, config.PP_LOOKAHEAD_MIN_MM),
                 config.PP_LOOKAHEAD_MAX_MM)
        s_t = self.S[self.i, 4] + Ld
        j = int(np.searchsorted(self.S[:, 4], s_t))
        if j >= len(self.S):
            # beyond the end: extend the final heading
            extra = s_t - self.S[-1, 4]
            tx, ty = ex + extra * math.cos(eth), ey + extra * math.sin(eth)
        else:
            tx, ty = self.S[j, 0], self.S[j, 1]
        if config.FOLLOWER_MODE == "rwf":
            j = min(int(np.searchsorted(self.S[:, 4], self.S[self.i, 4] + max(v_meas, v_ref) * config.RWF_PREVIEW_S)),
                    len(self.S) - 1)
            pk = self.S[j, 3]                                              # curvature: preview (lag)
            px, py, pth = self.S[self.i, :3]                               # errors: the nearest point
            e = -(X - px) * math.sin(pth) + (Y - py) * math.cos(pth)       # + = car left of the path
            eh = _wrap(th - pth)
            KE = 1.0 / config.RWF_LENGTH_MM ** 2
            KH = 2.0 * config.RWF_DAMPING / config.RWF_LENGTH_MM
            sinc = math.sin(eh) / eh if abs(eh) > 1e-6 else 1.0
            den = 1.0 - pk * e
            k = (pk * math.cos(eh) / den if abs(den) > 0.2 else pk) - KH * eh - KE * e * sinc
            delta = math.degrees(math.atan(config.WHEELBASE_MM * k))
            delta = max(-config.STEER_LOCK_RIGHT_DEG, min(config.STEER_LOCK_LEFT_DEG, delta))
            return DriveCmd(delta, v_ref)
        dx, dy = tx - X, ty - Y
        alpha = math.atan2(dy, dx) - th
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        L = max(math.hypot(dx, dy), 1.0)
        k = 2.0 * math.sin(alpha) / L
        delta = math.degrees(math.atan(config.WHEELBASE_MM * k))
        delta = max(-config.STEER_LOCK_RIGHT_DEG, min(config.STEER_LOCK_LEFT_DEG, delta))
        return DriveCmd(delta, v_ref)
