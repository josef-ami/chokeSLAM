"""
One-time initialisation from a single LIDAR snapshot, robot stationary at its
start pose (start zone or parking lot), facing the round direction (rule 9.8).
This is the ONLY place the LIDAR fixes the robot's position. After it, motion
is tracked by the STM32 feed (BNO08x heading + drive-motor hall encoder).

Pipeline (each step must succeed for the next to run):

  1. DIRECTION   direction_detect.detect_direction() -- which side has the
                 ~1000 mm opening in its side wall. Left -> CCW, right -> CW.

  2. x           distance from the OUTER wall:
                     CCW (outer wall on the right): x = d(90)
                     CW  (outer wall on the left) : x = d(270)
                 d(90) / d(270) = where the 90 / 270 ray meets that side's wall
                 line, measured by direction_detect.measure_side_wall (the
                 farthest well-supported straight line within +/-30 deg of
                 abeam -- sees past a pillar beside the LIDAR; approved Sept 23,
                 replacing the 2-deg ray median). Sanity check:
                 d(90) + d(270) = config.LANE_WIDTH_MM within
                 LANE_WIDTH_TOLERANCE_MM, otherwise x is REJECTED, not trusted.

  3. y           along the lane from the wall behind: y = 3000 - front, where
                 `front` is the distance to the wall ahead measured over a FAN:
                     for returns with |angle| <= FRONT_FAN_HALF_DEG:
                         f = r * cos(angle)                (forward distance)
                     front = median of f over the returns with
                             f >= max(f) - FRONT_BAND_MM
                 i.e. the farthest consistent thing straight ahead that is
                 still inside the field. A pillar or the parking-lot limitation
                 ahead is nearer and is ignored, where the single d(0) ray
                 would read it (problem P2 in docs/CHANGES.md).

  4. SEATS       seat_occupancy.detect_seat_occupancy() at (x, y), yaw 0,
                 with the round direction (for the bearing formula).
                 Present / absent / unknown per seat.

Yaw is taken as exactly 0 throughout (agreed): LIDAR 0 deg is the lane's grid
north. Lever arm (config.LIDAR_OFFSET_*): x and y are reported for the pose
reference point, not the sensor:
     x_ref = x_sensor + h * LATERAL      (h = -1 CCW, +1 CW; LATERAL + = left)
     y_ref = y_sensor - FORWARD          (FORWARD + = sensor ahead of ref)
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Iterable

import config
import lane_frame as lf
import seat_occupancy as so
from direction_detect import DirectionResult, WallFit, detect_direction, measure_side_wall

LANE_LENGTH_MM = so.LANE_LENGTH_MM     # 3000, rule 13.1


@dataclass
class XReading:
    ok: bool
    reason: str = ""
    d90_mm: float | None = None        # 90 ray -> right-hand wall line
    d270_mm: float | None = None       # 270 ray -> left-hand wall line
    lane_sum_mm: float | None = None   # d90 + d270, expected config.LANE_WIDTH_MM
    x_sensor_mm: float | None = None   # distance sensor -> outer wall
    x_mm: float | None = None          # reference point, after the lever arm


@dataclass
class YReading:
    ok: bool
    reason: str = ""
    front_mm: float | None = None      # sensor -> wall ahead (fan median)
    f_max_mm: float | None = None      # farthest forward distance in the fan
    n_front: int = 0                   # returns used for the median
    n_fan: int = 0                     # returns in the fan
    y_sensor_mm: float | None = None
    y_mm: float | None = None          # reference point, after the lever arm


@dataclass
class InitResult:
    ok: bool
    reason: str
    direction: DirectionResult
    x: XReading | None = None
    y: YReading | None = None
    yaw_deg: float = 0.0               # agreed: exactly 0 at initialisation
    seats: list = field(default_factory=list)   # list[so.SeatReading]


def detect_params_from_config(base: so.DetectParams | None = None) -> so.DetectParams:
    """DetectParams with the lever arm and blind wedge taken from config.py, so
    the seat check and x/y can't disagree about where the sensor is."""
    p = so.DetectParams() if base is None else base
    p.lidar_offset_forward_mm = config.LIDAR_OFFSET_FORWARD_MM
    p.lidar_offset_lateral_mm = config.LIDAR_OFFSET_LATERAL_MM
    p.blind_arc_center_deg = config.REAR_BLIND_ARC_CENTER_DEG
    p.blind_arc_width_deg = config.REAR_BLIND_ARC_WIDTH_DEG
    return p


def measure_x(points: Iterable, direction: str,
              walls: tuple[WallFit, WallFit] | None = None) -> XReading:
    """walls = (left, right) WallFits already measured by the direction test
    (reused so both steps see exactly the same walls); measured here if None."""
    if walls is None:
        pts = list(points)
        walls = (measure_side_wall(pts, "left"), measure_side_wall(pts, "right"))
    left, right = walls
    d270 = left.d_ray_mm if left.ok else None
    d90 = right.d_ray_mm if right.ok else None
    r = XReading(ok=False, d90_mm=d90, d270_mm=d270)
    if d90 is None or d270 is None:
        r.reason = "no wall at " + ", ".join(f"{n} ({w.reason})" for n, w in (("90", right), ("270", left)) if not w.ok)
        return r
    r.lane_sum_mm = d90 + d270
    if abs(r.lane_sum_mm - config.LANE_WIDTH_MM) > config.LANE_WIDTH_TOLERANCE_MM:
        r.reason = (f"d(90)+d(270) = {r.lane_sum_mm:.0f} mm, expected {config.LANE_WIDTH_MM:.0f} "
                    f"+/- {config.LANE_WIDTH_TOLERANCE_MM:.0f} (config.LANE_WIDTH_MM) -- x rejected")
        return r
    r.x_sensor_mm = d90 if direction == "CCW" else d270
    r.x_mm = r.x_sensor_mm + lf.handedness(direction) * config.LIDAR_OFFSET_LATERAL_MM
    r.ok = True
    return r


def measure_y(points: Iterable) -> YReading:
    fan = []
    for p in points:
        a = float(p.angle_deg) if hasattr(p, "angle_deg") else float(tuple(p)[0])
        rng = float(p.dist_mm) if hasattr(p, "dist_mm") else float(tuple(p)[1])
        off = abs((a + 180.0) % 360.0 - 180.0)          # angle away from dead-ahead
        if off <= config.FRONT_FAN_HALF_DEG:
            fan.append(rng * math.cos(math.radians(off)))
    r = YReading(ok=False, n_fan=len(fan))
    if not fan:
        r.reason = f"no returns within +/-{config.FRONT_FAN_HALF_DEG:.0f} deg of ahead"
        return r
    f_max = max(fan)
    front = [f for f in fan if f >= f_max - config.FRONT_BAND_MM]
    r.f_max_mm = f_max
    r.n_front = len(front)
    r.front_mm = statistics.median(front)
    r.y_sensor_mm = LANE_LENGTH_MM - r.front_mm
    r.y_mm = r.y_sensor_mm - config.LIDAR_OFFSET_FORWARD_MM
    if not (0.0 < r.y_mm < LANE_LENGTH_MM):
        r.reason = f"y = {r.y_mm:.0f} mm is outside the lane (0..{LANE_LENGTH_MM:.0f})"
        return r
    r.ok = True
    return r


def initialise(points: Iterable, params: so.DetectParams | None = None) -> InitResult:
    """Run the whole pipeline on one snapshot. `points` are mount-corrected,
    clockwise ScanPoints (scan_processing.clean_and_project output)."""
    pts = list(points)
    d = detect_direction(pts)
    if d.direction is None:
        return InitResult(ok=False, reason=f"direction: {d.reason}", direction=d)
    xr = measure_x(pts, d.direction, walls=(d.left.fit, d.right.fit))
    yr = measure_y(pts)
    if not xr.ok or not yr.ok:
        why = "; ".join(f"{n}: {r.reason}" for n, r in (("x", xr), ("y", yr)) if not r.ok)
        return InitResult(ok=False, reason=why, direction=d, x=xr, y=yr)
    p = detect_params_from_config(params)
    seats = so.detect_seat_occupancy(pts, xr.x_mm, yr.y_mm, d.direction,
                                     robot_yaw_deg=0.0, params=p)
    return InitResult(ok=True, reason=f"{d.direction}, x={xr.x_mm:.0f}, y={yr.y_mm:.0f}",
                      direction=d, x=xr, y=yr, yaw_deg=0.0, seats=seats)
