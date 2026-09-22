"""
Round direction (CW / CCW) from the one-time start scan, by finding which
side has a ~1000 mm gap in the wall running along the direction of travel.

WHY THERE IS A GAP ON EXACTLY ONE SIDE
--------------------------------------
The robot starts in a straightforward section, facing the round direction
(rule 9.8), with one wall on each side:
  - the OUTER wall is the boundary of the whole field. It runs unbroken from
    the wall behind (y = 0) to the wall ahead (y = 3000). Nothing can be seen
    THROUGH it.
  - the ISLAND wall is only 1000 mm long (y = 1000 .. 2000, Fig. 2/11). Past
    its far end (y = 2000 .. 3000) there is no wall: that is the opening into
    the next lane, 1000 mm long, ending at the wall ahead.
So looking forward along each side, one side shows an opening of ~1000 mm and
the other shows none. The island is on the robot's LEFT when driving CCW and on
its RIGHT when driving CW, so:
        gap on the LEFT  (270 deg side)  ->  CCW
        gap on the RIGHT ( 90 deg side)  ->  CW

THE PROCEDURE (clockwise angles: 0 = forward, 90 = right, 270 = left;
robot frame (f, s): f = r cos a ahead, s = r sin a to the right)
------------------------------------------------------------------------
1. Side distances. d_right = median range of the returns within
   +/- SIDE_RAY_HALF_WINDOW_DEG of 90; d_left = the same around 270.
   They must add up to the lane width (1000 +/- LANE_WIDTH_TOLERANCE_MM),
   else UNDETERMINED: one of them isn't reaching its wall (e.g. a pillar
   abeam), so there is no wall line to test against.

2. Wall-tilt fit (approved after the placement-yaw finding, see "WHY THE
   TILT FIT" below). For each side, take the returns within
   +/- GAP_FIT_HALF_DEG of that side's 90/270 whose s lies within
   GAP_FIT_BAND_MM of +d_right (right) / -d_left (left). Total-least-squares
   line through them, then drop returns more than GAP_FIT_INLIER_MM off the
   line and refit (up to 3 times). A side is "fitted" with at least
   GAP_FIT_MIN_POINTS inliers.
     - both sides fitted: their directions must agree within
       GAP_FIT_AGREE_DEG (the walls are parallel), else UNDETERMINED;
       the lane direction is their mean.
     - one side fitted: the lane direction is that side's.
     - neither: UNDETERMINED.
   Each side wall is then the line along the lane direction through that
   side's fitted inliers (or, if only the other side fitted, through the
   point (0, +/-d_side)).
   The fitted tilt is used ONLY here. x, y and the seat check still take the
   robot's yaw as exactly 0 (agreed).

3. For every return in that side's FORWARD quadrant (right: 0..90,
   left: 270..360) whose ray meets the wall line ahead:
       r_line = range at which the ray meets the side-wall line
       f      = how far along the lane (from the robot) it meets it
   classify:
       r >  r_line + GAP_MARGIN_MM   -> PASSED THROUGH the wall line at f
       |r - r_line| <= GAP_MARGIN_MM -> wall present at f
       r <  r_line - GAP_MARGIN_MM   -> blocked by something nearer (pillar,
                                        parking-lot limitation): no information

4. opening(side) = max(f) - min(f) over that side's PASSED-THROUGH returns
   (0 if there are none).

5. Decision:
       opening(left)  >= GAP_OPEN_MIN_MM and opening(right) <= GAP_CLOSED_MAX_MM -> CCW
       opening(right) >= GAP_OPEN_MIN_MM and opening(left)  <= GAP_CLOSED_MAX_MM -> CW
       anything else -> UNDETERMINED (direction = None). Never a guess.

WHY "PASSED THROUGH" AND NOT "WHERE THE WALL ENDS"
--------------------------------------------------
A pillar, or the magenta parking-lot limitation, can make the outer wall
LOOK like it ends early (its far part is hidden behind the obstacle). But
nothing can make a ray pass THROUGH the outer wall -- it is the field
boundary. Occlusion can only shrink the measured opening on the island side
(-> UNDETERMINED at worst).

WHY THE TILT FIT
----------------
The first version modelled each wall as a line parallel to the robot's
forward axis (yaw = 0). If the robot is placed a few degrees off parallel,
one real wall tilts AWAY from that model; far ahead its returns come back
beyond the model line and look like "passed through", faking an opening on
the OUTER side. With a pillar just ahead hiding the real opening at the same
time, that gave the WRONG direction (1-3 per 1,600 simulated starts at
2-5 deg placement yaw). Fitting the walls' actual tilt removes the fake
opening. The fit uses raw returns, not scan_processing's clusters, so the
cluster finder's corner-merge problem (P1) doesn't apply.

WHY THE OPENING READS ~900 mm, NOT 1000
---------------------------------------
Past the island's end the ray continues into the next lane and stops at the
wall ahead's extension. Close to the wall ahead it passes the wall line by
less than GAP_MARGIN_MM, so the last ~100 mm of the opening are classified as
"wall present". Hence the >= 500 threshold rather than ~1000.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Iterable

import config
from mat_geometry import LANE_WIDTH_MM


@dataclass
class WallFit:
    ok: bool
    n_inliers: int = 0
    angle_deg: float | None = None      # line direction in the robot frame, atan2(ds, df), in (-90, 90]
    rms_mm: float | None = None
    centroid: tuple[float, float] | None = None   # (f, s) of the inliers


@dataclass
class SideScan:
    side: str                          # "left" (270 deg) or "right" (90 deg)
    d_side_mm: float | None            # median range at 90 / 270
    fit: WallFit | None = None
    n_wall: int = 0                    # returns that hit the wall line
    n_through: int = 0                 # returns that passed through it
    n_blocked: int = 0                 # returns stopped short of it
    through_f_mm: list[float] = field(default_factory=list)   # f of each passed-through return
    opening_mm: float = 0.0            # max(f) - min(f) of the passed-through returns
    opening_start_mm: float | None = None   # min f  (where the opening begins, ahead of the robot)
    opening_end_mm: float | None = None     # max f


@dataclass
class DirectionResult:
    direction: str | None              # "CCW", "CW", or None = UNDETERMINED
    reason: str
    left: SideScan
    right: SideScan
    wall_angle_deg: float | None = None   # lane direction in the robot frame (info only:
                                          # = -placement yaw; NOT used outside this test)


def _angle_of(p) -> float:
    return float(p.angle_deg) if hasattr(p, "angle_deg") else float(tuple(p)[0])


def _range_of(p) -> float:
    return float(p.dist_mm) if hasattr(p, "dist_mm") else float(tuple(p)[1])


def _adiff(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def side_ray_distance(points: Iterable, angle_deg: float,
                      half_window_deg: float | None = None) -> float | None:
    """Median range of the returns within +/- half_window of angle_deg
    (clockwise robot angle). None if there are no returns there."""
    hw = config.SIDE_RAY_HALF_WINDOW_DEG if half_window_deg is None else half_window_deg
    vals = [_range_of(p) for p in points if abs(_adiff(_angle_of(p), angle_deg)) <= hw]
    return statistics.median(vals) if vals else None


def _tls(pts: list[tuple[float, float]]):
    """Total-least-squares line through (f, s) points -> (angle_deg, centroid, rms)."""
    n = len(pts)
    mf = sum(p[0] for p in pts) / n
    ms = sum(p[1] for p in pts) / n
    sff = sum((p[0] - mf) ** 2 for p in pts)
    sss = sum((p[1] - ms) ** 2 for p in pts)
    sfs = sum((p[0] - mf) * (p[1] - ms) for p in pts)
    th = 0.5 * math.atan2(2.0 * sfs, sff - sss)      # direction of the line
    nf, ns = -math.sin(th), math.cos(th)             # its unit normal
    res = [(p[0] - mf) * nf + (p[1] - ms) * ns for p in pts]
    rms = math.sqrt(sum(r * r for r in res) / n)
    ang = math.degrees(th)
    if ang <= -90.0:
        ang += 180.0
    elif ang > 90.0:
        ang -= 180.0
    return ang, (mf, ms), rms, (nf, ns), res


def fit_side_wall(points: Iterable, side: str, d_side_mm: float | None) -> WallFit:
    """Step 2 for one side. See module docstring."""
    if d_side_mm is None:
        return WallFit(ok=False)
    centre = 90.0 if side == "right" else 270.0
    s_ref = d_side_mm if side == "right" else -d_side_mm
    cand = []
    for p in points:
        a = _angle_of(p)
        if abs(_adiff(a, centre)) > config.GAP_FIT_HALF_DEG:
            continue
        r = _range_of(p)
        f, s = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        if abs(s - s_ref) <= config.GAP_FIT_BAND_MM:
            cand.append((f, s))
    pts = cand
    if len(pts) < config.GAP_FIT_MIN_POINTS:
        return WallFit(ok=False, n_inliers=len(pts))
    for _ in range(3):
        ang, cen, rms, _, res = _tls(pts)
        keep = [q for q, r in zip(pts, res) if abs(r) <= config.GAP_FIT_INLIER_MM]
        if len(keep) == len(pts):
            break
        if len(keep) < config.GAP_FIT_MIN_POINTS:
            return WallFit(ok=False, n_inliers=len(keep))
        pts = keep
    ang, cen, rms, _, _ = _tls(pts)
    return WallFit(ok=True, n_inliers=len(pts), angle_deg=ang, rms_mm=rms, centroid=cen)


def scan_side(points: Iterable, side: str, d_side_mm: float | None,
              lane_angle_deg: float = 0.0, through_point: tuple[float, float] | None = None,
              margin_mm: float | None = None,
              min_angle_deg: float | None = None) -> SideScan:
    """Steps 3-4 for one side. The side wall is the line with direction
    lane_angle_deg (robot frame, atan2(ds, df)) through `through_point`
    (default (0, +/-d_side)). lane_angle_deg = 0 is the parallel model."""
    margin = config.GAP_MARGIN_MM if margin_mm is None else margin_mm
    tmin = config.GAP_MIN_ANGLE_FROM_FWD_DEG if min_angle_deg is None else min_angle_deg
    out = SideScan(side=side, d_side_mm=d_side_mm)
    if d_side_mm is None or d_side_mm <= 0:
        return out
    if through_point is None:
        through_point = (0.0, d_side_mm if side == "right" else -d_side_mm)
    th = math.radians(lane_angle_deg)
    tf, ts = math.cos(th), math.sin(th)          # lane direction (unit), forward-pointing
    nf, ns = -ts, tf                             # a normal to it
    dist = through_point[0] * nf + through_point[1] * ns
    if dist < 0:                                 # make the normal point from the robot to the wall
        nf, ns, dist = -nf, -ns, -dist
    for p in points:
        a = _angle_of(p) % 360.0
        if side == "right":
            if not (0.0 < a < 90.0):
                continue
            t = a
        elif side == "left":
            if not (270.0 < a < 360.0):
                continue
            t = 360.0 - a
        else:
            raise ValueError(side)
        if t < tmin:
            continue
        uf, us = math.cos(math.radians(a)), math.sin(math.radians(a))
        c = uf * nf + us * ns
        if c <= 1e-6:
            continue                              # parallel to / away from the wall line
        r_line = dist / c
        f = r_line * (uf * tf + us * ts)          # along-lane distance of the crossing
        if f <= 0:
            continue                              # crossing behind the robot: not "forward"
        r = _range_of(p)
        if r > r_line + margin:
            out.n_through += 1
            out.through_f_mm.append(f)
        elif r < r_line - margin:
            out.n_blocked += 1
        else:
            out.n_wall += 1
    if out.through_f_mm:
        out.opening_start_mm = min(out.through_f_mm)
        out.opening_end_mm = max(out.through_f_mm)
        out.opening_mm = out.opening_end_mm - out.opening_start_mm
    return out


def detect_direction(points: Iterable) -> DirectionResult:
    """The whole procedure. `points` are mount-corrected, clockwise
    (scan_processing.clean_and_project output, or (angle, range) pairs)."""
    pts = list(points)
    d_right = side_ray_distance(pts, 90.0)
    d_left = side_ray_distance(pts, 270.0)
    empty_l, empty_r = SideScan("left", d_left), SideScan("right", d_right)

    missing = [n for n, d in (("left (270)", d_left), ("right (90)", d_right)) if d is None]
    if missing:
        return DirectionResult(None, f"UNDETERMINED: no returns at {', '.join(missing)} -- "
                                     f"can't place that side wall", empty_l, empty_r)
    # Precondition (added at checkpoint A, pending approval): both side rays
    # must be hitting the two lane walls, i.e. add up to the lane width -- the
    # same check x uses. A pillar abeam would otherwise become the "wall" and
    # every ray past it would count as passing through.
    lane_sum = d_left + d_right
    if abs(lane_sum - LANE_WIDTH_MM) > config.LANE_WIDTH_TOLERANCE_MM:
        return DirectionResult(None, f"UNDETERMINED: d(90)+d(270) = {lane_sum:.0f} mm, expected "
                                     f"{LANE_WIDTH_MM:.0f} +/- {config.LANE_WIDTH_TOLERANCE_MM:.0f} -- "
                                     f"a side ray isn't reaching its wall (pillar abeam?)", empty_l, empty_r)

    fit_l = fit_side_wall(pts, "left", d_left)
    fit_r = fit_side_wall(pts, "right", d_right)
    empty_l.fit, empty_r.fit = fit_l, fit_r
    if fit_l.ok and fit_r.ok:
        if abs(fit_l.angle_deg - fit_r.angle_deg) > config.GAP_FIT_AGREE_DEG:
            return DirectionResult(None, f"UNDETERMINED: side walls fit at {fit_l.angle_deg:.1f} and "
                                         f"{fit_r.angle_deg:.1f} deg -- they must be parallel "
                                         f"(within {config.GAP_FIT_AGREE_DEG:.1f})", empty_l, empty_r)
        lane_angle = 0.5 * (fit_l.angle_deg + fit_r.angle_deg)
    elif fit_l.ok or fit_r.ok:
        lane_angle = (fit_l if fit_l.ok else fit_r).angle_deg
    else:
        return DirectionResult(None, "UNDETERMINED: neither side wall could be fitted "
                                     f"(inliers left {fit_l.n_inliers}, right {fit_r.n_inliers})",
                               empty_l, empty_r)

    left = scan_side(pts, "left", d_left, lane_angle, fit_l.centroid if fit_l.ok else None)
    right = scan_side(pts, "right", d_right, lane_angle, fit_r.centroid if fit_r.ok else None)
    left.fit, right.fit = fit_l, fit_r

    lo, ro = left.opening_mm, right.opening_mm
    open_min, closed_max = config.GAP_OPEN_MIN_MM, config.GAP_CLOSED_MAX_MM
    tilt = f"walls at {lane_angle:+.1f} deg"
    if lo >= open_min and ro <= closed_max:
        return DirectionResult("CCW", f"gap on the LEFT: {lo:.0f} mm open (>= {open_min:.0f}); "
                                      f"right {ro:.0f} mm (<= {closed_max:.0f}); {tilt} "
                                      f"-> island on the left -> CCW", left, right, lane_angle)
    if ro >= open_min and lo <= closed_max:
        return DirectionResult("CW", f"gap on the RIGHT: {ro:.0f} mm open (>= {open_min:.0f}); "
                                     f"left {lo:.0f} mm (<= {closed_max:.0f}); {tilt} "
                                     f"-> island on the right -> CW", left, right, lane_angle)
    return DirectionResult(None, f"UNDETERMINED: left opening {lo:.0f} mm, right opening {ro:.0f} mm "
                                 f"({tilt}) -- need one side >= {open_min:.0f} and the other "
                                 f"<= {closed_max:.0f}", left, right, lane_angle)
