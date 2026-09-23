"""
A simulated world + robot, used only when config.MODE == "mock" and by
run_init.py --sim. Ray-casts a synthetic LIDAR scan against the rulebook field
geometry so the pipeline can run with no hardware attached.

This is a test harness, not part of the deployed robot -- on the real robot,
lidar_source.RPLidarC1Source (and, from checkpoint B, the STM32 feed) replace it.

Conventions (Sept 2026):
  - simulate_scan() returns CLOCKWISE robot angles (0 = forward, 90 = right,
    270 = left); MockRobotSimulator.current_scan() converts them to the raw
    angles the real sensor would report under config.LIDAR_ANGLE_SIGN /
    _ZERO_OFFSET_DEG (to_sensor_raw), so a simulated scan goes through the same
    calibration step as a real one;
  - the robot's pose is kept in the LANE frame (section, direction, x from the
    outer wall, y along travel, yaw from grid north) and converted to the
    global mat frame only to ray-cast;
  - the robot faces the way it drives: heading = the lane's grid north + yaw.
    (Fixes P3: the old simulator's heading formula pointed it 180 deg away
    from its own direction of travel.)
  - the rear chassis blind wedge (config.REAR_BLIND_ARC_*) is modelled, so a
    simulated scan has the same hole as a real one.

Checkpoint A: a static world and a stationary robot (for initialisation).
Checkpoint B adds LoopPath (a robot driving laps), SimStm32 (the same $IMU
lines the real STM32 sends, from the true motion) and run_mock() (init ->
tracking -> turns -> entry re-checks, end to end, with ground truth).
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
                  blind_width_deg: float = 0.0, rng_noise=None) -> list[tuple[float, float, int]]:
    """Raw (angle_deg, dist_mm, quality) points from a sensor at GLOBAL
    (x_mm, y_mm) facing GLOBAL grid bearing heading_deg.

    Robot-relative angle a is CLOCKWISE from forward, so the ray's global
    bearing is heading + a and its maths angle is 90 - (heading + a).
    extra_boxes: additional axis-aligned obstacles (x0, y0, x1, y1), e.g. the
    parking-lot limitations. Rays inside the blind wedge return nothing.
    rng_noise: a random.Random for reproducible noise/dropout (default: module random).
    """
    rnd = rng_noise if rng_noise is not None else random
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
        if rnd.random() < dropout_prob:
            continue
        dist = max(1.0, best_t + rnd.gauss(0.0, noise_std_mm))
        points.append((rel, dist, quality))
    return points


def to_sensor_raw(clockwise_angle_deg: float) -> float:
    """Clockwise robot angle -> the raw angle the sensor reports, i.e. the
    inverse of scan_processing.clean_and_project's
        corrected = SIGN * raw + OFFSET   ->   raw = SIGN * (corrected - OFFSET)."""
    return (config.LIDAR_ANGLE_SIGN * (clockwise_angle_deg - config.LIDAR_ANGLE_ZERO_OFFSET_DEG)) % 360.0


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
        """The scan as the REAL sensor would report it: raw angles in the sensor's
        own convention (config.LIDAR_ANGLE_SIGN / _ZERO_OFFSET_DEG), so that
        clean_and_project() with the same config turns them back into clockwise
        robot angles -- exactly the path a real scan takes."""
        ts = self.true_state()
        sx, sy = self.sensor_global()
        pts = simulate_scan(sx, sy, ts.heading_deg, self.pillars, self.extra_boxes,
                            n_points=self.n_points, noise_std_mm=self.noise_std_mm,
                            dropout_prob=self.dropout_prob,
                            blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                            blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
        return [(to_sensor_raw(a), d, q) for a, d, q in pts]


# =============================================================================
# Checkpoint B: motion, simulated STM32, end-to-end mock run
# =============================================================================
class LoopPath:
    """The robot's true path: a rounded square around the island at `offset`
    mm from the outer wall (500 = the lane centreline), corner radius `radius`,
    driven in `direction`, with a gentle sinusoidal weave of `wander_mm` on the
    straights (tapered to zero at their ends). GLOBAL frame. Arc length s = 0
    is the start of the straight in lane S; one quarter of the loop per lane,
    visiting S, E, N, W (CCW) or S, W, N, E (CW)."""

    def __init__(self, direction: str, offset: float = 500.0, radius: float = 400.0,
                 wander_mm: float = 50.0, wander_len: float = 1300.0, phase: float = 0.0):
        lf.handedness(direction)
        self.direction = direction
        c, C, R = offset, geo.OUTER_SIZE_MM - offset, radius
        self.R, self.c = R, c
        self.L = (C - c) - 2 * R
        self.quarter = self.L + math.pi * R / 2
        self.length = 4 * self.quarter
        self.A, self.lam, self.phase = wander_mm, wander_len, phase
        # CCW segments: (line start, line dir, arc centre, arc start angle[maths deg])
        self._segs = [((c + R, c), (1, 0), (C - R, c + R), -90.0),
                      ((C, c + R), (0, 1), (C - R, C - R), 0.0),
                      ((C - R, C), (-1, 0), (c + R, C - R), 90.0),
                      ((c, C - R), (0, -1), (c + R, c + R), 180.0)]

    def _ccw_point(self, s: float) -> tuple[float, float]:
        s %= self.length
        k = int(s // self.quarter)
        u = s - k * self.quarter
        (lx, ly), (dx, dy), (ax, ay), a0 = self._segs[k]
        if u <= self.L:
            w = self.A * math.sin(2 * math.pi * u / self.lam + self.phase) * math.sin(math.pi * u / self.L)
            return lx + dx * u - dy * w, ly + dy * u + dx * w      # left normal = (-dy, dx)
        a = math.radians(a0 + math.degrees((u - self.L) / self.R))
        return ax + self.R * math.cos(a), ay + self.R * math.sin(a)

    def point(self, s: float) -> tuple[float, float]:
        x, y = self._ccw_point(s)
        return (geo.OUTER_SIZE_MM - x, y) if self.direction == "CW" else (x, y)

    def pose(self, s: float) -> tuple[float, float, float]:
        """(gx, gy, grid bearing of travel)."""
        x0, y0 = self.point(s - 0.5)
        x1, y1 = self.point(s + 0.5)
        gx, gy = self.point(s)
        return gx, gy, math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0

    def s_at_lane_y(self, slot: int, y_lane: float) -> float:
        """Arc length where the path is at lane y (along travel) on the straight
        of the `slot`-th lane after lane S."""
        return slot * self.quarter + (y_lane - (self.c + self.R))

    def section(self, slot: int) -> str:
        order = ["S", "E", "N", "W"] if self.direction == "CCW" else ["S", "W", "N", "E"]
        return order[slot % 4]


class SimStm32:
    """Turns true motion into the agreed '$IMU,<seq>,<t_ms>,<enc>,<yaw>' lines.
    enc: cumulative distance x the TRUE ticks per cm (optionally mis-scaled vs
    config, to show the effect of a calibration error), rounded to whole ticks.
    yaw: IMU_YAW_SIGN x bearing + an arbitrary chip zero + noise + drift, wrapped
    to (-180, 180] like the chip's output."""

    def __init__(self, rng: random.Random, ticks_per_cm: float | None = None,
                 yaw_sign: int | None = None, yaw_zero_deg: float = 37.0,
                 yaw_noise_deg: float = 0.15, yaw_drift_deg_s: float = 0.01,
                 drop_prob: float = 0.0):
        self.rng = rng
        self.tpc = ticks_per_cm if ticks_per_cm is not None else config.ENCODER_TICKS_PER_CM
        self.sign = yaw_sign if yaw_sign is not None else config.IMU_YAW_SIGN
        self.zero, self.noise, self.drift, self.drop = yaw_zero_deg, yaw_noise_deg, yaw_drift_deg_s, drop_prob
        self.seq = 0

    def line(self, t_ms: int, distance_mm: float, bearing_deg: float) -> str | None:
        self.seq += 1
        if self.drop and self.rng.random() < self.drop:
            return None                                   # a lost line (the seq gap shows it)
        enc = int(round(distance_mm / 10.0 * self.tpc))
        yaw = self.sign * bearing_deg + self.zero + self.drift * t_ms / 1000.0 + self.rng.gauss(0, self.noise)
        yaw = (yaw + 180.0) % 360.0 - 180.0
        return f"$IMU,{self.seq},{t_ms},{enc},{yaw:.2f}"


def random_lane_pillars(direction: str, rng: random.Random, avoid=None) -> tuple[list[Pillar], dict]:
    """1-2 pillars per lane on rulebook seats (Fig. 8c). Returns (pillars,
    {section: set(seat indices)}). avoid = (section, x, y): no pillar within
    150 mm of that lane pose (the robot's start)."""
    pillars, truth = [], {}
    for sec in lf.SECTIONS:
        idx = set(rng.sample(range(6), rng.choice([1, 2])))
        if avoid and avoid[0] == sec:
            idx = {i for i in idx if math.hypot(so.seats()[i].x_mm - avoid[1], so.seats()[i].y_mm - avoid[2]) >= 150.0}
        truth[sec] = idx
        pillars += rulebook_pillars(sec, direction, sorted(idx))
    return pillars, truth


def make_world(direction: str, start_slot: int, y_start: float, rng: random.Random,
               placement_yaw_deg: float = 0.0, wander_mm: float = 50.0, radius: float = 400.0):
    """The simulated world of run_mock and the dashboard's live mock: the
    robot's path (weave phase chosen so the start pose has the requested
    placement yaw -- a hand-placed start, not whatever the weave happens to
    give), its start (arc length s0 in lane `start_slot` at lane y y_start),
    and 1-2 random pillars per lane. Returns (path, start_section, s0,
    pillars, truth) with truth = {section: set(seat indices)}."""
    best = None
    for k in range(360):
        ph = math.radians(k)
        pth = LoopPath(direction, radius=radius, wander_mm=wander_mm, phase=ph)
        sec = pth.section(start_slot)
        _, _, b = pth.pose(pth.s_at_lane_y(start_slot, y_start))
        err = abs(lf.heading_to_yaw(b, sec, direction) - placement_yaw_deg)
        if best is None or err < best[0]:
            best = (err, ph)
    path = LoopPath(direction, radius=radius, wander_mm=wander_mm, phase=best[1])
    start_sec = path.section(start_slot)
    s0 = path.s_at_lane_y(start_slot, y_start)
    gx, gy, _ = path.pose(s0)
    x0, y0 = lf.global_to_lane(start_sec, direction, gx, gy)
    pillars, truth = random_lane_pillars(direction, rng, avoid=(start_sec, x0, y0))
    return path, start_sec, s0, pillars, truth


def cast_revolution(pose_at, t_end: float, rev_s: float, pillars: list[Pillar], rng: random.Random,
                    n: int = 720) -> list[tuple[float, float, int, float]]:
    """One revolution of the spinning LIDAR ending at time t_end, each ray cast
    from pose_at(t) = (gx, gy, grid bearing) at the time t it was measured.
    Ray k is taken at the fraction k / n of the revolution in the sensor's own
    raw order (raw angle increasing with time). Returns raw (raw angle, dist,
    quality, t_measured); rays inside the rear blind wedge return nothing."""
    raw = []
    for k in range(n):
        raw_angle = k * 360.0 / n
        rel = (config.LIDAR_ANGLE_SIGN * raw_angle + config.LIDAR_ANGLE_ZERO_OFFSET_DEG) % 360.0
        if abs((rel - config.REAR_BLIND_ARC_CENTER_DEG + 180.0) % 360.0 - 180.0) <= config.REAR_BLIND_ARC_WIDTH_DEG / 2.0:
            continue
        t = t_end - rev_s * (1.0 - k / n)
        x, y, b = pose_at(t)
        for _, d, q in simulate_scan(x, y, (b + rel) % 360.0, pillars, n_points=1, rng_noise=rng):
            raw.append((raw_angle, d, q, t))
    return raw


def run_mock(direction: str = "CCW", start_slot: int = 0, y_start: float = 1400.0,
             laps: int = 3, speed_mm_s: float = 600.0, seed: int = 1,
             enc_scale_err: float = 0.0, drop_prob: float = 0.0, wander_mm: float = 50.0,
             radius: float = 400.0, lidar_hz: float = 10.0, placement_yaw_deg: float = 0.0,
             yaw_drift_deg_s: float = 0.01, lidar_sweep: bool = True, deskew: bool = True,
             lidar_stamp_error_s: float = 0.0, imu_latency_s: float = 0.002, lidar_delay_s: float = 0.0,
             verbose: bool = False) -> dict:
    """Initialise from a simulated start scan, then drive `laps` laps feeding
    the tracker simulated STM32 lines (100 Hz) and, while it asks for them,
    LIDAR frames (lidar_hz). Returns ground-truth comparisons.

    lidar_sweep: True (default) = each re-check frame is ONE REVOLUTION of a
    spinning sensor (1 / lidar_hz s), handed over at its end, and each ray is
    cast from the pose the robot had when that ray was measured -- what a real
    LIDAR on a moving robot delivers. False = every frame is cast
    instantaneously from the pose at the moment it is handed over (the model
    the tests used before P9 was found). The sensor's raw angle is taken to increase with
    time through a revolution, so the clockwise robot angle runs in the
    direction of config.LIDAR_ANGLE_SIGN.

    Timing (for the de-skew, decision #38): the STM32 lines reach the Pi
    imu_latency_s after they are sampled; every LIDAR return is stamped as
    lidar_source would stamp it, i.e. measurement time + imu_latency_s +
    config.LIDAR_TIME_OFFSET_S (so a correctly measured offset lines the two
    up exactly), + lidar_stamp_error_s (an error in that measured offset).
    deskew=False hands the frames over without times (no de-skew).
    lidar_delay_s: each frame reaches the tracker this long after it was
    taken (the tracker has integrated that much more STM32 data by then), as
    when the Pi is busy -- the frame must still be judged from where it was
    taken."""
    import lane_init as li
    from lane_tracker import LaneTracker
    from scan_processing import clean_and_project, clean_and_project_timed
    from stm32_link import ImuSample
    from stm32_link import parse_line

    rng = random.Random(seed)
    path, start_sec, s0, pillars, truth = make_world(direction, start_slot, y_start, rng, placement_yaw_deg,
                                                     wander_mm, radius)
    gx, gy, brg = path.pose(s0)
    x0, y0 = lf.global_to_lane(start_sec, direction, gx, gy)

    def scan_points(gx, gy, brg):
        raw = simulate_scan(gx, gy, brg, pillars, n_points=720, rng_noise=rng,
                            blind_center_deg=config.REAR_BLIND_ARC_CENTER_DEG,
                            blind_width_deg=config.REAR_BLIND_ARC_WIDTH_DEG)
        raw = [(to_sensor_raw(a), d, q) for a, d, q in raw]
        return clean_and_project(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)

    def stamp(t_meas):
        return t_meas + imu_latency_s + config.LIDAR_TIME_OFFSET_S + lidar_stamp_error_s

    def swept_scan_points(s_end, t_end):
        """One revolution ending at arc length s_end, time t_end; ray k is
        measured at the fraction (k / n) of the revolution in the sensor's own
        raw order. Returns (points, times)."""
        raw = cast_revolution(lambda t: path.pose(s_end - speed_mm_s * (t_end - t)), t_end, 1.0 / lidar_hz,
                              pillars, rng)
        raw = [(a, d, q, stamp(t)) for a, d, q, t in raw]
        return clean_and_project_timed(raw, config.LIDAR_ANGLE_SIGN, config.LIDAR_ANGLE_ZERO_OFFSET_DEG)

    def snapshot_scan_points(gx, gy, brg, t_end):
        pts = scan_points(gx, gy, brg)
        return pts, [stamp(t_end)] * len(pts)

    def sample(line, t_ms):
        smp, _ = parse_line(line)
        return ImuSample(smp.seq, smp.t_ms, smp.enc, smp.yaw_deg, t_ms / 1000.0 + imu_latency_s)

    init = li.initialise(scan_points(gx, gy, brg))
    out = {"direction": direction, "start_section": start_sec, "truth_start": (x0, y0),
           "placement_yaw_deg": lf.heading_to_yaw(brg, start_sec, direction),
           "init": init, "truth_seats": truth}
    if not init.ok:
        out["ok"] = False
        return out

    stm = SimStm32(rng, ticks_per_cm=config.ENCODER_TICKS_PER_CM * (1.0 + enc_scale_err), drop_prob=drop_prob,
                   yaw_drift_deg_s=yaw_drift_deg_s)
    first = sample(stm.line(0, 0.0, brg), 0)
    trk = LaneTracker(init, first)
    out["psi0_err_deg"] = trk.psi0 - out["placement_yaw_deg"]
    t_ms, dist, s = 0, 0.0, s0
    px, py = gx, gy
    s_end = s0 + laps * path.length
    lidar_every = int(round(1000.0 / lidar_hz / 10.0))
    errs, herrs, n_step = [], [], 0
    from collections import deque
    late, delay_steps = deque(), int(round(lidar_delay_s / 0.01))
    out["init_err_mm"] = (init.x.x_mm - x0, init.y.y_mm - y0)
    while s < s_end:
        t_ms += 10
        n_step += 1
        s += speed_mm_s * 0.01
        gx, gy, brg = path.pose(s)
        dist += math.hypot(gx - px, gy - py)
        px, py = gx, gy
        line = stm.line(t_ms, dist, brg)
        if line is not None:
            trk.on_imu(sample(line, t_ms))
        if trk.wants_lidar and n_step % lidar_every == 0:
            pts, times = (swept_scan_points(s, t_ms / 1000.0) if lidar_sweep
                          else snapshot_scan_points(gx, gy, brg, t_ms / 1000.0))
            late.append((n_step + delay_steps, pts, times if deskew else None))
        while late and late[0][0] <= n_step:
            _, pts, times = late.popleft()
            trk.on_lidar_frame(pts, times)
        # tracked pose -> global, via the lane the tracker believes it is in
        sec = path.section(start_slot + trk.lane_index)
        tx, ty = lf.lane_to_global(sec, direction, trk.x, trk.y)
        errs.append(math.hypot(tx - gx, ty - gy))
        th = lf.yaw_to_heading(trk.psi, sec, direction)
        herrs.append(abs((th - brg + 180.0) % 360.0 - 180.0))
    turns = [e for e in trk.events if e.kind == "turn"]
    # entry re-check verdicts vs truth
    seat_cmp = {}
    for slot, rec in trk.lanes.items():
        sec = path.section(start_slot + slot)
        right = wrong = unknown = 0
        for i, st in rec.seats.items():
            occ = i in truth[sec]
            if st.state == "unknown":
                unknown += 1
            elif (st.state == "occupied") == occ:
                right += 1
            else:
                wrong += 1
        seat_cmp[slot] = {"section": sec, "source": rec.source, "right": right, "wrong": wrong,
                          "unknown": unknown, "frames": rec.frames_used}
    errs_sorted = sorted(errs)
    out.update({
        "ok": True, "tracker": trk, "turns": len(turns), "expected_turns": 4 * laps,
        "pos_err_median_mm": errs_sorted[len(errs) // 2], "pos_err_max_mm": errs_sorted[-1],
        "pos_err_end_mm": errs[-1], "head_err_max_deg": max(herrs),
        "seats": seat_cmp, "distance_mm": dist,
        "glitches": sum(1 for e in trk.events if e.kind in ("glitch", "reset")),
    })
    return out
