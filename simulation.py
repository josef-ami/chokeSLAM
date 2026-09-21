"""
A simulated robot + world, used only when config.MODE == "mock". Lets you
run and watch the whole pipeline (classification, broadside fix, pose
estimation, dashboard) with no hardware attached, by ray-casting a synthetic
LIDAR scan against the field geometry and driving a simulated car around the
loop with noisy odometry.

This is a test harness, not part of the deployed robot -- on the real robot,
lidar_source.RPLidarC1Source + your real encoder/IMU feed replace this file
entirely.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

import mat_geometry as geo


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


PILLAR_HALF_MM = 25.0  # 50x50mm pillar footprint


@dataclass
class Pillar:
    x_mm: float
    y_mm: float
    color: str


def simulate_scan(x_mm: float, y_mm: float, heading_deg: float, pillars: list[Pillar],
                   n_points: int = 360, noise_std_mm: float = 4.0, dropout_prob: float = 0.02,
                   quality: int = 47) -> list[tuple[float, float, int]]:
    """Raw (angle_deg, dist_mm, quality) points, in the SAME convention
    scan_processing.angle_to_xy expects with angle_sign=+1, offset=0 (i.e.
    robot-relative angle measured the same direction as the global heading
    convention). Real hardware will need LIDAR_ANGLE_SIGN / _OFFSET
    calibrated in config.py -- the simulator sidesteps that by construction.
    """
    points = []
    outer = (0.0, 0.0, geo.OUTER_SIZE_MM, geo.OUTER_SIZE_MM)
    island = (geo.ISLAND_MIN_MM, geo.ISLAND_MIN_MM, geo.ISLAND_MAX_MM, geo.ISLAND_MAX_MM)
    boxes = [island] + [
        (p.x_mm - PILLAR_HALF_MM, p.y_mm - PILLAR_HALF_MM, p.x_mm + PILLAR_HALF_MM, p.y_mm + PILLAR_HALF_MM)
        for p in pillars
    ]

    for i in range(n_points):
        robot_rel_angle = i * (360.0 / n_points)
        global_angle = math.radians(heading_deg + robot_rel_angle)
        dx, dy = math.cos(global_angle), math.sin(global_angle)

        best_t = None
        hit = _ray_box(x_mm, y_mm, dx, dy, *outer)
        if hit is not None:
            _, t_far = hit
            if t_far > 0:
                best_t = t_far   # exiting through the outer wall

        for box in boxes:
            hit = _ray_box(x_mm, y_mm, dx, dy, *box)
            if hit is None:
                continue
            t_near, t_far = hit
            if t_near > 1e-6 and (best_t is None or t_near < best_t):
                best_t = t_near

        if best_t is None:
            continue
        if random.random() < dropout_prob:
            continue
        dist = max(1.0, best_t + random.gauss(0.0, noise_std_mm))
        points.append((robot_rel_angle, dist, quality))
    return points


@dataclass
class SimTrueState:
    section: geo.Section
    along_mm: float
    lateral_mm: float
    heading_deg: float
    x_mm: float
    y_mm: float


class MockRobotSimulator:
    """Drives a simulated car around the loop at constant speed, with a
    slow lateral wander (to exercise off-centre tracking), instant 90-degree
    corner turns, and configurable odometry/IMU noise so you can see the
    pose estimate drift between corners and snap back at each fix."""

    def __init__(self, initial_section: geo.Section = "S", direction: str = "CCW",
                 speed_mm_s: float = 300.0, encoder_noise_frac: float = 0.03,
                 heading_noise_deg: float = 1.5, pillars: list[Pillar] | None = None):
        self.direction = direction
        self._next_section = geo.NEXT_SECTION_CCW if direction == "CCW" else geo.NEXT_SECTION_CW
        self.section: geo.Section = initial_section
        self.along_mm = 50.0
        self.lateral_mm = geo.LANE_WIDTH_MM / 2
        self.speed = speed_mm_s
        self.encoder_noise_frac = encoder_noise_frac
        self.heading_noise_deg = heading_noise_deg
        self._t = 0.0
        self.pillars = pillars if pillars is not None else self._default_pillars()

    def _default_pillars(self) -> list[Pillar]:
        slots = geo.all_slots()
        chosen = [slots[2], slots[9], slots[15], slots[20]]
        colors = ["red", "green", "red", "green"]
        return [Pillar(s.x_mm, s.y_mm, c) for s, c in zip(chosen, colors)]

    def true_heading_deg(self) -> float:
        # driving heading = broadside heading rotated -90 (so the section's
        # "forward" runs along its length, matching the along-axis in
        # mat_geometry._section_axes for CCW travel; CW just reverses it).
        broadside = geo.BROADSIDE_HEADING_DEG[self.section]
        return (broadside - 90.0) % 360.0 if self.direction == "CCW" else (broadside + 90.0) % 360.0

    def true_state(self) -> SimTrueState:
        x, y = geo.local_to_global(self.section, self.along_mm, self.lateral_mm)
        return SimTrueState(self.section, self.along_mm, self.lateral_mm, self.true_heading_deg(), x, y)

    def step(self, dt_s: float):
        """Advances true state by dt_s seconds. Returns
        (noisy_encoder_delta_mm, noisy_heading_deg, corner_completed: bool)."""
        self._t += dt_s
        true_delta = self.speed * dt_s
        self.lateral_mm = geo.LANE_WIDTH_MM / 2 + 120.0 * math.sin(self._t * 0.35)
        self.lateral_mm = min(max(self.lateral_mm, 80.0), geo.LANE_WIDTH_MM - 80.0)

        corner_completed = False
        self.along_mm += true_delta
        if self.along_mm >= geo.OUTER_SIZE_MM:
            self.along_mm -= geo.OUTER_SIZE_MM
            self.section = self._next_section[self.section]
            corner_completed = True

        noisy_delta = true_delta * (1.0 + random.gauss(0.0, self.encoder_noise_frac))
        noisy_heading = (self.true_heading_deg() + random.gauss(0.0, self.heading_noise_deg)) % 360.0
        return noisy_delta, noisy_heading, corner_completed

    def current_scan(self):
        ts = self.true_state()
        return simulate_scan(ts.x_mm, ts.y_mm, ts.heading_deg, self.pillars)

    def broadside_scan(self):
        """A scan as seen with the robot broadside in its CURRENT section
        (front at the outer wall). Used for the one-time start-of-run fix
        and again right after each corner_completed=True step, matching the
        brief broadside moment simulated in step() -- current_scan() should
        be used everywhere else, since it reflects the actual driving
        heading."""
        x, y = geo.local_to_global(self.section, self.along_mm, self.lateral_mm)
        heading = geo.BROADSIDE_HEADING_DEG[self.section]
        return simulate_scan(x, y, heading, self.pillars), heading
