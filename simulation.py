"""
A simulated world + robot, used only when config.MODE == "mock" and by
run_init.py --sim. Ray-casts a synthetic LIDAR scan against the rulebook field
geometry so the pipeline can run with no hardware attached.

This is a test harness, not part of the deployed robot -- on the real robot,
lidar_source.RPLidarC1Source (and, from checkpoint B, the STM32 feed) replace it.

Conventions (Sept 2026):
  - scans are (angle_deg, dist_mm, quality) with CLOCKWISE robot angles
    (0 = forward, 90 = right, 270 = left), i.e. what the real sensor gives
    with LIDAR_ANGLE_SIGN = +1, OFFSET = 0;
  - the robot's pose is kept in the LANE frame (section, direction, x from the
    outer wall, y along travel, yaw from grid north) and converted to the
    global mat frame only to ray-cast;
  - the robot faces the way it drives: heading = the lane's grid north + yaw.
    (Fixes P3: the old simulator's heading formula pointed it 180 deg away
    from its own direction of travel.)
  - the rear chassis blind wedge (config.REAR_BLIND_ARC_*) is modelled, so a
    simulated scan has the same hole as a real one.

Checkpoint A scope: a static world and a stationary robot (for initialisation).
Motion, turns and the simulated STM32 feed are added at checkpoint B.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

import config
import lane_frame as lf
import mat_geometry as geo
import seat_occupancy as so


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    """Slab-method ray/axis-aligned-box intersection. Returns (t_near, t_far)
    or None. Ray: (ox,oy) + t*(dx,dy)."""
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin, tmax


PILLAR_HALF_MM = 25.0  # 50x50mm pillar footprint (rule 13.19)


@dataclass
class Pillar:
    x_mm: float        # GLOBAL mat frame
    y_mm: float
    color: str = "unknown"


def simulate_scan(x_mm: float, y_mm: float, heading_deg: float, pillars: list[Pillar],
                  extra_boxes: list[tuple[float, float, float, float]] | None = None,
                  n_points: int = 360, noise_std_mm: float = 4.0, dropout_prob: float = 0.02,
                  quality: int = 47, blind_center_deg: float | None = None,
                  blind_width_deg: float = 0.0) -> list[tuple[float, float, int]]:
    """Raw (angle_deg, dist_mm, quality) points from a sensor at GLOBAL
    (x_mm, y_mm) facing GLOBAL grid bearing heading_deg.

    Robot-relative angle a is CLOCKWISE from forward, so the ray's global
    bearing is heading + a and its maths angle is 90 - (heading + a).
    extra_boxes: additional axis-aligned obstacles (x0, y0, x1, y1), e.g. the
    parking-lot limitations. Rays inside the blind wedge return nothing.
    """
    points = []
    boxes = [geo.ISLAND_BOX] + [
        (p.x_mm - PILLAR_HALF_MM, p.y_mm - PILLAR_HALF_MM, p.x_mm + PILLAR_HALF_MM, p.y_mm + PILLAR_HALF_MM)
        for p in pillars
    ] + list(extra_boxes or [])

    for i in range(n_points):
        rel = i * (360.0 / n_points)
        if blind_center_deg is not None and blind_width_deg > 0:
            if abs((rel - blind_center_deg + 180.0) % 360.0 - 180.0) <= blind_width_deg / 2.0:
                continue
        world = math.radians(90.0 - (heading_deg + rel))
        dx, dy = math.cos(world), math.sin(world)

        best_t = None
        hit = _ray_box(x_mm, y_mm, dx, dy, *geo.OUTER_BOX)
        if hit is not None and hit[1] > 0:
            best_t = hit[1]            # exiting through the outer wall
        for box in boxes:
            hit = _ray_box(x_mm, y_mm, dx, dy, *box)
            if hit is None:
                continue
            t_near, _ = hit
            if t_near > 1e-6 and (best_t is None or t_near < best_t):
                best_t = t_near

        if best_t is None:
            continue
        if random.random() < dropout_prob:
            continue
        dist = max(1.0, best_t + random.gauss(0.0, noise_std_mm))
        points.append((rel, dist, quality))
    return points


def rulebook_pillars(section: str, direction: str, seat_indices, color: str = "unknown") -> list[Pillar]:
    """Pillars on the given seat indices (seat_occupancy numbering) of one lane."""
    by_index = {s.index: s for s in so.seats()}
    out = []
    for i in seat_indices:
        s = by_index[i]
        gx, gy = lf.lane_to_global(section, direction, s.x_mm, s.y_mm)
        out.append(Pillar(gx, gy, color))
    return out


def parking_lot_boxes(section: str, direction: str, y_first_mm: float, y_second_mm: float,
                      depth_mm: float = 200.0, thickness_mm: float = 20.0):
    """The two magenta limitations (200 x 20 x 100 mm, rule 13.25) standing
    against the OUTER wall, centred at lane y = y_first / y_second, sticking
    depth_mm into the lane. Returned as global axis-aligned boxes."""
    boxes = []
    for yc in (y_first_mm, y_second_mm):
        a = lf.lane_to_global(section, direction, 0.0, yc - thickness_mm / 2)
        b = lf.lane_to_global(section, direction, depth_mm, yc + thickness_mm / 2)
        boxes.append((min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))
    return boxes


@dataclass
class SimTrueState:
    section: str
    direction: str
    x_mm: float          # lane frame, from the outer wall
    y_mm: float          # lane frame, along travel
    yaw_deg: float       # clockwise from the lane's grid north
    heading_deg: float   # global grid bearing
    gx_mm: float         # global
    gy_mm: float


class MockRobotSimulator:
    """A world (field + pillars + optional parking lot) and a robot pose in it.
    Checkpoint A: stationary robot; current_scan() is the start scan."""

    def __init__(self, section: str = "S", direction: str = "CCW",
                 x_mm: float = 500.0, y_mm: float = 1500.0, yaw_deg: float = 0.0,
                 pillars: list[Pillar] | None = None,
                 extra_boxes: list[tuple[float, float, float, float]] | None = None,
                 n_points: int = 360, noise_std_mm: float = 4.0, dropout_prob: float = 0.02):
        lf.handedness(direction)          # validates direction
        self.section = section
        self.direction = direction
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.yaw_deg = yaw_deg
        self.pillars = pillars if pillars is not None else self._default_pillars()
        self.extra_boxes = list(extra_boxes or [])
        self.n_points = n_points
        self.noise_std_mm = noise_std_mm
        self.dropout_prob = dropout_prob

    def _default_pillars(self) -> list[Pillar]:
        """A plausible draw on rulebook seats (Fig. 8c cards allow 1-2 per section)."""
        layout = {"S": (1, 4), "E": (0,), "N": (3, 4), "W": (5,)}
        out = []
        for sec, idx in layout.items():
            out += rulebook_pillars(sec, self.direction, idx)
        return out

    def true_state(self) -> SimTrueState:
        gx, gy = lf.lane_to_global(self.section, self.direction, self.x_mm, self.y_mm)
        heading = lf.yaw_to_heading(self.yaw_deg, self.section, self.direction)
        return SimTrueState(self.section, self.direction, self.x_mm, self.y_mm,
                            self.yaw_deg, heading, gx, gy)

    def sensor_global(self) -> tuple[float, float]:
        """Global position of the LIDAR (applies config.LIDAR_OFFSET_*)."""
        sx, sy = lf.offset_in_lane(self.x_mm, self.y_mm, self.yaw_deg, self.direction,
                                   config.LIDAR_OFFSET_FORWARD_MM, config.LIDAR_OFFSET_LATERAL_MM)
        return lf.lane_to_global(self.section, self.direction, sx, sy)

    def current_scan(self):
        ts = self.true_state()
        sx, sy = self.sensor_global()
        return simulate_scan(sx, sy, ts.heading_deg, self.pillars, self.extra_boxes,
                             n_points=self.n_points, noise_std_mm=self.noise_std_mm,
                             dropout_prob=self.dropout_prob,
                             blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                             blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
