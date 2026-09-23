"""
De-skew of a LIDAR frame for the entry re-check (decision #38, P9 in
docs/CHANGES.md section 9.11).

A frame holds the latest return in each angle bucket, i.e. returns measured
over the last revolution (~100 ms). While the robot moves, each return was
taken from a slightly different pose; the seat check assumes one pose. So
each return is moved to where it would have been seen from the pose at the
frame's reference time: the END of the frame (its newest return) for the seat
check -- so the check sees the world from where the robot was when the frame
was taken, however late the frame is processed -- or the tracker's newest pose
for display:

    for a return measured at t_i (lidar_source's sweep time - LIDAR_TIME_OFFSET_S):
        pose_i   = the odometry pose at t_i, interpolated in the pose history
        p_robot  = the return in the robot frame at t_i   (sensor + lever arm)
        p_odo    = p_robot placed with pose_i              (odometry frame)
        p_ref    = p_odo seen from the reference pose      (robot frame, now)
        return'  = p_ref - lever arm                       (sensor frame, now)

Only RELATIVE motion over the frame's ~0.1 s enters, taken from the same
integration the tracker uses, so the odometry frame can be any fixed frame:
the tracker keeps it in the start lane's orientation, continuous across lane
switches (a lane switch changes the lane coordinates, not this frame).

Returns measured before the oldest pose in the history (DESKEW_HISTORY_S) are
dropped: a bucket that hasn't been refreshed for that long (no return there
any more) holds a stale range from somewhere else. Returns measured after the
reference time (after the newest STM32 sample -- typically less than one
10 ms STM32 period) are placed with the pose extrapolated at the velocity of
the last 30 ms, at most EXTRAPOLATE_MAX_S ahead.

Frames: robot frame (fwd, right); odometry frame (X, Y) with heading h
clockwise from +Y:  X = ox + fwd sin h + right cos h,  Y = oy + fwd cos h - right sin h.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from scan_processing import ScanPoint


class PoseHistory:
    """(t, ox, oy, heading_deg) with t on the Pi clock, oldest first, and with
    each entry the lane pose at that time (lane_index, x, y, psi) so a past
    lane pose can be looked up (lane_pose_at)."""

    def __init__(self, keep_s: float):
        self.keep_s = keep_s
        self._d: deque = deque()

    def append(self, t: float, ox: float, oy: float, heading_deg: float,
               lane: tuple[int, float, float, float] | None = None) -> None:
        if self._d and t < self._d[-1][0]:
            self._d.clear()                  # time went backwards (clock re-based): start over
        self._d.append((t, ox, oy, heading_deg, lane))
        while len(self._d) > 2 and t - self._d[0][0] > self.keep_s:
            self._d.popleft()

    def arrays(self):
        a = np.asarray([e[:4] for e in self._d], dtype=float)
        return a[:, 0], a[:, 1], a[:, 2], a[:, 3]

    def lane_pose_at(self, t: float) -> tuple[int, float, float, float] | None:
        """(lane_index, x, y, psi) at time t, interpolated between the two
        entries around t. None if t is before the history, or if a lane switch
        falls between those two entries (the pose is then in neither lane's
        frame cleanly), or if no lane poses were stored."""
        d = self._d
        if not d or t < d[0][0]:
            return None
        if t >= d[-1][0]:
            return d[-1][4]
        ts = [e[0] for e in d]
        k = int(np.searchsorted(ts, t, side="right"))
        a, b = d[k - 1], d[k]
        if a[4] is None or b[4] is None or a[4][0] != b[4][0]:
            return None
        w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        la, lb = a[4], b[4]
        dpsi = (lb[3] - la[3] + 180.0) % 360.0 - 180.0
        return (la[0], la[1] + w * (lb[1] - la[1]), la[2] + w * (lb[2] - la[2]),
                (la[3] + w * dpsi + 180.0) % 360.0 - 180.0)

    def __len__(self):
        return len(self._d)

    @property
    def span(self) -> tuple[float, float] | None:
        return (self._d[0][0], self._d[-1][0]) if self._d else None


EXTRAPOLATE_MAX_S = 0.05
VELOCITY_WINDOW_S = 0.03
# One revolution of the C1 (10 Hz) with margin. The seat check uses a frame only
# if the pose history reaches back this far before the frame's end: a frame
# processed so late that part of its own revolution has fallen out of the
# history would lose a sector of returns, and a seat window with the pillar's
# returns missing but the wall's beside it kept reads as a false EMPTY.
FRAME_SPAN_S = 0.15


@dataclass
class DeskewStats:
    used: int = 0
    dropped_old: int = 0          # measured before the history starts
    extrapolated: int = 0         # measured after the reference time
    max_shift_mm: float = 0.0     # largest distance a return was moved


def deskew(points: list[ScanPoint], times: list[float], history: PoseHistory,
           lidar_time_offset_s: float, fwd_off_mm: float, left_off_mm: float,
           t_ref: float | None = None) -> tuple[list[ScanPoint], DeskewStats]:
    """t_ref: the time whose pose the returns are moved to (within the
    history); None = the newest pose."""
    st = DeskewStats()
    if not points or len(history) < 1:
        return [], st
    ht, hx, hy, hh = history.arrays()
    t_new = ht[-1]
    if t_ref is None or t_ref >= t_new:
        t_ref, oxe, oye, he = t_new, hx[-1], hy[-1], math.radians(hh[-1])
    else:
        oxe, oye = float(np.interp(t_ref, ht, hx)), float(np.interp(t_ref, ht, hy))
        he = math.radians(float(np.interp(t_ref, ht, hh)))
    t = np.asarray(times, dtype=float) - lidar_time_offset_s
    keep = t >= ht[0]
    st.dropped_old = int((~keep).sum())
    t = t[keep]
    pts = [p for p, k in zip(points, keep) if k]
    if not pts:
        return [], st
    ox = np.interp(t, ht, hx)
    oy = np.interp(t, ht, hy)
    hd = np.interp(t, ht, hh)                        # heading is unwrapped: plain interpolation is right
    new = t > t_new                                  # measured after the newest pose: extrapolate
    st.extrapolated = int(new.sum())
    j = min(max(0, int(np.searchsorted(ht, t_new - VELOCITY_WINDOW_S, side="right")) - 1), len(ht) - 2)
    dt = t_new - ht[j] if len(ht) >= 2 else 0.0
    if st.extrapolated and dt > 0:
        te = np.minimum(t[new], t_new + EXTRAPOLATE_MAX_S) - t_new
        ox[new] = hx[-1] + te * (hx[-1] - hx[j]) / dt
        oy[new] = hy[-1] + te * (hy[-1] - hy[j]) / dt
        hd[new] = hh[-1] + te * (hh[-1] - hh[j]) / dt
    h = np.radians(hd)
    f = np.array([p.fwd_mm for p in pts]) + fwd_off_mm
    r = np.array([p.right_mm for p in pts]) - left_off_mm
    X = ox + f * np.sin(h) + r * np.cos(h)
    Y = oy + f * np.cos(h) - r * np.sin(h)
    dX, dY = X - oxe, Y - oye
    f2 = dX * math.sin(he) + dY * math.cos(he) - fwd_off_mm
    r2 = dX * math.cos(he) - dY * math.sin(he) + left_off_mm
    out = []
    for p, fw, rt in zip(pts, f2, r2):
        out.append(ScanPoint(angle_deg=math.degrees(math.atan2(rt, fw)) % 360.0, dist_mm=math.hypot(fw, rt),
                             quality=p.quality, fwd_mm=float(fw), right_mm=float(rt)))
    shift = np.hypot(f2 - (f - fwd_off_mm), r2 - (r + left_off_mm))
    st.used, st.max_shift_mm = len(out), float(shift.max())
    out.sort(key=lambda p: p.angle_deg)
    return out, st
