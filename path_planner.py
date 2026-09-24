"""
path_planner.py

Pipeline: occupancy grid -> square obstacle extraction -> visibility graph
-> Dijkstra shortest path -> arc-smoothed trajectory -> generator of
drive commands for an STM32 low-level controller.

Design:
    - Obstacles are treated as axis-aligned squares/rectangles (grid blobs),
      each optionally tagged with a required pass-side ("left", "right", "either").
    - Visibility graph nodes = start, goal, and the (side-filtered) inflated
      corners of each obstacle.
    - Dijkstra finds the shortest collision-free polyline.
    - Each polyline corner is replaced with a tangent circular arc of a
      configured minimum turning radius (Dubins-style straight-arc-straight
      smoothing), so the result is actually drivable.
    - A generator yields (steering_angle_rad, speed, distance) triples that
      a caller streams down to the STM32 (e.g. over UART), one segment at
      a time. Distance-based, not time-based, so it's robust to speed
      variation -- the STM32 side should consume these using encoder
      distance traveled, not a timer.

No external dependencies beyond the standard library.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Generator, Iterable, Literal, Optional


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

@dataclass
class RobotConfig:
    half_width_m: float = 0.09        # robot half-width (footprint clearance)
    safety_margin_m: float = 0.03     # extra clearance beyond half-width
    min_turn_radius_m: float = 0.30   # minimum turning radius (chassis-dependent)
    wheelbase_m: float = 0.14         # for Ackermann steering-angle conversion
    cruise_speed_mps: float = 0.6     # default straight-line speed
    arc_speed_mps: float = 0.35       # default speed while holding an arc

    @property
    def inflation_m(self) -> float:
        return self.half_width_m + self.safety_margin_m


@dataclass
class FieldConfig:
    """WRO Future Engineers playfield geometry, from the official technical
    drawing (3000mm x 3000mm outer square, 800mm x 800mm center island,
    rounded outer corners). All values in meters, field-frame coordinates
    with origin at the field's bottom-left corner, +x right, +y up.

    The corner ROUNDING RADIUS used for path planning is the outer wall's
    rounded-corner radius -- this is what actually constrains how tight the
    robot's corner turn can be. Read off the technical drawing as best as
    can be determined from the dimension lines; VERIFY against the official
    WRO rules PDF for your season before trusting it on hardware, since a
    misread here changes every corner arc in both plan modes.
    """
    outer_size_m: float = 3.000        # overall outer square side
    island_size_m: float = 0.800       # center island square side
    outer_corner_radius_m: float = 0.300   # outer wall rounded-corner radius (verify!)
    lane_width_m: float = 1.000        # marked corridor width along each edge

    @property
    def center(self) -> Point:
        return (self.outer_size_m / 2.0, self.outer_size_m / 2.0)

    @property
    def island_half(self) -> float:
        return self.island_size_m / 2.0

    def leg_midline_y(self) -> float:
        """Rough y-coordinate (for a bottom-edge leg) of the driving lane's
        centerline, i.e. halfway between the island's edge and the outer
        wall. Rotate/reflect for the other 3 edges."""
        island_edge = self.center[1] - self.island_half
        return island_edge / 2.0

    def corner_pivot(self, corner_index: int) -> Point:
        """Approximate field-frame pivot point for corner `corner_index`
        (0=bottom-right, 1=top-right, 2=top-left, 3=bottom-left, going
        counterclockwise), i.e. roughly where the driving lane's centerline
        rounds the outer corner. Used as the nominal corner location for
        lap-1 fixed corner_arc() calls, and as a boundary waypoint anchor
        for lap-2+ plan_lap().
        """
        half = self.outer_size_m / 2.0
        inset = self.leg_midline_y()  # lane centerline distance from wall
        offsets = [
            (half - inset, -half + inset),   # 0: bottom-right region (relative to center)
            (half - inset, half - inset),    # 1: top-right
            (-half + inset, half - inset),   # 2: top-left
            (-half + inset, -half + inset),  # 3: bottom-left
        ]
        dx, dy = offsets[corner_index % 4]
        cx, cy = self.center
        return (cx + dx, cy + dy)


PassSide = Literal["left", "right", "either"]


@dataclass
class Obstacle:
    """An axis-aligned square/rectangular obstacle in world coordinates."""
    cx: float
    cy: float
    half_size: float          # half of the obstacle's side length (raw, uninflated)
    pass_side: PassSide = "either"   # which side the path must keep to
    id: str = ""

    def inflated_corners(self, inflation: float) -> list[tuple[float, float]]:
        """Return the 4 corners of the obstacle after Minkowski-style inflation.

        Uses a simple square inflation (push corners out along the diagonal
        offset by `inflation` in x and y independently) rather than a true
        rounded-rectangle Minkowski sum -- slightly conservative, much
        cheaper, and adequate at small-arena scale.
        """
        h = self.half_size + inflation
        return [
            (self.cx - h, self.cy - h),  # bottom-left
            (self.cx + h, self.cy - h),  # bottom-right
            (self.cx + h, self.cy + h),  # top-right
            (self.cx - h, self.cy + h),  # top-left
        ]

    def contains_point(self, x: float, y: float, inflation: float) -> bool:
        h = self.half_size + inflation
        return (self.cx - h) <= x <= (self.cx + h) and (self.cy - h) <= y <= (self.cy + h)


Point = tuple[float, float]


# --------------------------------------------------------------------------
# Step 1: occupancy grid -> obstacle list
# --------------------------------------------------------------------------

def extract_square_obstacles(
    grid: list[list[int]],
    cell_size_m: float,
    origin: Point = (0.0, 0.0),
    occupied_value: int = 1,
) -> list[Obstacle]:
    """Extract axis-aligned square obstacles from a binary occupancy grid.

    Groups connected occupied cells (4-connectivity) into blobs, then
    represents each blob by its bounding square in world coordinates.
    Good enough for sparse, well-separated obstacles like competition
    pillars/blocks; for dense clutter you'd want a smarter decomposition.

    grid[row][col], row 0 = y=origin[1], col 0 = x=origin[0], increasing
    row/col increases y/x.
    """
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    visited = [[False] * cols for _ in range(rows)]
    obstacles: list[Obstacle] = []
    blob_id = 0

    def neighbors(r: int, c: int):
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                yield nr, nc

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == occupied_value and not visited[r][c]:
                # BFS flood-fill this blob
                stack = [(r, c)]
                visited[r][c] = True
                cells = []
                while stack:
                    cr, cc = stack.pop()
                    cells.append((cr, cc))
                    for nr, nc in neighbors(cr, cc):
                        if grid[nr][nc] == occupied_value and not visited[nr][nc]:
                            visited[nr][nc] = True
                            stack.append((nr, nc))

                min_r = min(cell[0] for cell in cells)
                max_r = max(cell[0] for cell in cells)
                min_c = min(cell[1] for cell in cells)
                max_c = max(cell[1] for cell in cells)

                world_min_x = origin[0] + min_c * cell_size_m
                world_max_x = origin[0] + (max_c + 1) * cell_size_m
                world_min_y = origin[1] + min_r * cell_size_m
                world_max_y = origin[1] + (max_r + 1) * cell_size_m

                cx = (world_min_x + world_max_x) / 2.0
                cy = (world_min_y + world_max_y) / 2.0
                half_size = max(world_max_x - world_min_x, world_max_y - world_min_y) / 2.0

                blob_id += 1
                obstacles.append(Obstacle(cx=cx, cy=cy, half_size=half_size, id=f"obs{blob_id}"))

    return obstacles


# --------------------------------------------------------------------------
# Step 2: geometry helpers for visibility checks
# --------------------------------------------------------------------------

def _segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    """True if segment p1-p2 properly intersects segment p3-p4."""
    def cross(o: Point, a: Point, b: Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def _segment_crosses_square(p1: Point, p2: Point, corners: list[Point]) -> bool:
    """True if segment p1-p2 crosses any edge of the (convex, 4-corner) square,
    or if the segment's midpoint lies strictly inside it (covers the
    fully-contained-inside case)."""
    n = len(corners)
    for i in range(n):
        a = corners[i]
        b = corners[(i + 1) % n]
        if _segments_intersect(p1, p2, a, b):
            return True

    # Also reject if either endpoint sits inside the square (shouldn't
    # normally happen given how nodes are generated, but cheap to guard).
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    for pt in (p1, p2):
        if min_x < pt[0] < max_x and min_y < pt[1] < max_y:
            return True

    return False


def _line_clear(p1: Point, p2: Point, obstacles_corners: list[list[Point]]) -> bool:
    for corners in obstacles_corners:
        if _segment_crosses_square(p1, p2, corners):
            return False
    return True


def _filter_corners_by_side(
    corners: list[Point],
    obstacle: Obstacle,
    travel_direction: Point,
) -> list[Point]:
    """Keep only the corners on the mandated side of travel.

    travel_direction: rough unit vector of overall travel (e.g. (1, 0) for
    a leg running along +x). "Left"/"right" are relative to this direction
    (left = travel_direction rotated +90deg).
    """
    if obstacle.pass_side == "either":
        return corners

    dx, dy = travel_direction
    # left-normal of travel direction
    left_normal = (-dy, dx)

    def side_value(pt: Point) -> float:
        rel = (pt[0] - obstacle.cx, pt[1] - obstacle.cy)
        return rel[0] * left_normal[0] + rel[1] * left_normal[1]

    if obstacle.pass_side == "left":
        kept = [c for c in corners if side_value(c) >= 0]
    else:  # "right"
        kept = [c for c in corners if side_value(c) <= 0]

    # Guard against degenerate filtering (e.g. obstacle dead-center on the
    # travel line) -- fall back to all corners rather than leaving the
    # obstacle unroutable.
    return kept if kept else corners


# --------------------------------------------------------------------------
# Step 3: build visibility graph + Dijkstra
# --------------------------------------------------------------------------

@dataclass
class VisibilityGraph:
    nodes: list[Point]
    edges: dict[int, list[tuple[int, float]]] = field(default_factory=dict)

    def add_edge(self, i: int, j: int, w: float) -> None:
        self.edges.setdefault(i, []).append((j, w))
        self.edges.setdefault(j, []).append((i, w))


def build_visibility_graph(
    start: Point,
    goal: Point,
    obstacles: Iterable[Obstacle],
    inflation: float,
) -> VisibilityGraph:
    travel_direction = _normalize((goal[0] - start[0], goal[1] - start[1]))

    all_corners_per_obstacle: list[list[Point]] = []
    for obs in obstacles:
        raw_corners = obs.inflated_corners(inflation)
        kept = _filter_corners_by_side(raw_corners, obs, travel_direction)
        all_corners_per_obstacle.append(kept)

    # Full corner set (unfiltered) is what we test visibility against --
    # a path must not cut through ANY obstacle, filtered side or not.
    full_corners_per_obstacle = [obs.inflated_corners(inflation) for obs in obstacles]

    nodes: list[Point] = [start]
    for kept in all_corners_per_obstacle:
        nodes.extend(kept)
    nodes.append(goal)

    graph = VisibilityGraph(nodes=nodes)
    n = len(nodes)
    for i in range(n):
        for j in range(i + 1, n):
            p1, p2 = nodes[i], nodes[j]
            if _line_clear(p1, p2, full_corners_per_obstacle):
                dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                graph.add_edge(i, j, dist)

    return graph


def dijkstra(graph: VisibilityGraph, start_idx: int, goal_idx: int) -> list[int]:
    """Return the list of node indices for the shortest path, or [] if none."""
    dist = {start_idx: 0.0}
    prev: dict[int, int] = {}
    visited = set()
    pq: list[tuple[float, int]] = [(0.0, start_idx)]

    while pq:
        d, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == goal_idx:
            break
        for v, w in graph.edges.get(u, []):
            nd = d + w
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    if goal_idx not in dist:
        return []

    path = [goal_idx]
    while path[-1] != start_idx:
        path.append(prev[path[-1]])
    path.reverse()
    return path


# --------------------------------------------------------------------------
# Step 4: arc smoothing (Dubins-style straight-arc-straight per corner)
# --------------------------------------------------------------------------

def _normalize(v: Point) -> Point:
    mag = math.hypot(v[0], v[1])
    if mag < 1e-9:
        return (0.0, 0.0)
    return (v[0] / mag, v[1] / mag)


@dataclass
class PathSegment:
    kind: Literal["straight", "arc"]
    length_m: float
    # signed curvature: 0 for straight, +1/r for left turn, -1/r for right turn
    curvature: float = 0.0


def smooth_polyline_to_segments(points: list[Point], min_radius: float) -> list[PathSegment]:
    """Convert a polyline (list of waypoints) into straight/arc segments,
    rounding each interior corner with a tangent arc of radius >= min_radius
    (Dubins-style straight-arc-straight smoothing, applied corner by corner).

    Walks the polyline once, trimming a tangent-offset distance `d` off the
    end of each leg and the start of the next, replacing the trimmed corner
    with an arc. Leftover straight run on each leg is emitted as a
    'straight' segment.

    If a corner's required tangent offset would overrun an adjacent leg's
    length (legs too short for the turn), the radius for that corner is
    shrunk just enough to fit -- a simplification; for tightly packed
    obstacles where this matters a lot, prefer a full Dubins-path solver
    instead.
    """
    n = len(points)
    if n < 2:
        return []
    if n == 2:
        length = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
        return [PathSegment("straight", length, 0.0)]

    result: list[PathSegment] = []
    leg_start_offset = 0.0  # distance already consumed at the START of current leg

    for i in range(n - 1):
        p, q = points[i], points[i + 1]
        leg_vec = (q[0] - p[0], q[1] - p[1])
        leg_len = math.hypot(*leg_vec)
        leg_dir = _normalize(leg_vec)

        # tangent offset consumed at the END of this leg, for the corner at q
        # (0 if q is the final goal point)
        end_offset = 0.0
        curvature = 0.0
        arc_len = 0.0

        if i < n - 2:
            p_next = points[i + 2]
            v_out = _normalize((p_next[0] - q[0], p_next[1] - q[1]))
            heading_in = math.atan2(leg_dir[1], leg_dir[0])
            heading_out = math.atan2(v_out[1], v_out[0])
            turn = _wrap_angle(heading_out - heading_in)

            if abs(turn) > 1e-6:
                r = min_radius
                d = r * math.tan(abs(turn) / 2.0)
                next_leg_len = math.hypot(p_next[0] - q[0], p_next[1] - q[1])
                available_this_leg = max(leg_len - leg_start_offset, 0.0)
                max_d = min(available_this_leg, next_leg_len)
                if d > max_d:
                    if max_d < 1e-6:
                        # No room at all for an arc at this corner (waypoints
                        # coincide or a prior corner already consumed the
                        # whole leg). Skip the arc entirely rather than
                        # dividing by a near-zero radius; the corner is
                        # approximated as a sharp vertex here. This should
                        # be rare -- if it shows up often on real stitched
                        # (plan_lap) paths, drop near-duplicate waypoints
                        # before smoothing instead of relying on this guard.
                        d = 0.0
                        r = min_radius  # unused (arc_len forced to 0 below)
                    else:
                        d = max_d
                        r = d / math.tan(abs(turn) / 2.0)
                end_offset = d
                if d > 1e-9:
                    curvature = (1.0 / r) if turn > 0 else (-1.0 / r)
                    arc_len = r * abs(turn)
                else:
                    curvature = 0.0
                    arc_len = 0.0

        straight_len = leg_len - leg_start_offset - end_offset
        if straight_len > 1e-9:
            result.append(PathSegment("straight", straight_len, 0.0))
        if arc_len > 1e-9:
            result.append(PathSegment("arc", arc_len, curvature))

        leg_start_offset = end_offset  # next leg starts having already consumed `end_offset`... 
        # NOTE: end_offset here was measured on the OUTGOING leg's tangent
        # point, so it correctly carries forward as next leg's start offset.

    return result


def _wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# --------------------------------------------------------------------------
# Step 5: drive command generator
# --------------------------------------------------------------------------

@dataclass
class DriveCommand:
    steering_angle_rad: float   # 0 = straight, + = left, - = right (Ackermann convention)
    speed_mps: float
    distance_m: float           # how far to hold this command (encoder-tracked)


def curvature_to_steering_angle(curvature: float, wheelbase_m: float) -> float:
    """Ackermann approximation: steering_angle = atan(wheelbase * curvature)."""
    if abs(curvature) < 1e-9:
        return 0.0
    radius = 1.0 / curvature
    return math.atan(wheelbase_m / radius)


def segments_to_drive_commands(
    segments: list[PathSegment],
    robot: RobotConfig,
) -> Generator[DriveCommand, None, None]:
    """Generator: yields one DriveCommand per path segment, ready to stream
    to the STM32. Consume this in order; each command should be held until
    `distance_m` has been traveled (per wheel encoders), then advance to
    the next yielded command.
    """
    for seg in segments:
        if seg.kind == "straight":
            yield DriveCommand(
                steering_angle_rad=0.0,
                speed_mps=robot.cruise_speed_mps,
                distance_m=seg.length_m,
            )
        else:  # arc
            steering = curvature_to_steering_angle(seg.curvature, robot.wheelbase_m)
            yield DriveCommand(
                steering_angle_rad=steering,
                speed_mps=robot.arc_speed_mps,
                distance_m=seg.length_m,
            )


# --------------------------------------------------------------------------
# Corner turn (lap 1: fixed geometry, no search -- the corner's location
# and required heading change are known even when leg obstacles aren't)
# --------------------------------------------------------------------------

def corner_arc(
    turn_angle_rad: float,
    robot: RobotConfig,
    radius_m: Optional[float] = None,
) -> list[PathSegment]:
    """Build the fixed-geometry turn maneuver for a track corner.

    turn_angle_rad: signed heading change, + = left turn, - = right turn.
    For the WRO field's 90-degree corners this is +-pi/2, but the function
    takes any angle so the same code path covers non-square corners too.

    radius_m: turn radius to use. Defaults to robot.min_turn_radius_m, but
    you may want to pass FieldConfig.outer_corner_radius_m instead (or
    whichever is LARGER of the two -- see note below) since the corner is
    also physically bounded by the field's rounded-wall geometry, not just
    the chassis's minimum turning radius.

    Returns a single-arc segment list (no straight runs) -- splice this
    directly between the tail of one leg's segments and the head of the
    next leg's segments in the chained lap-1 generator.
    """
    r = radius_m if radius_m is not None else robot.min_turn_radius_m
    if r < robot.min_turn_radius_m - 1e-9:
        raise ValueError(
            f"Requested corner radius {r:.3f}m is tighter than the robot's "
            f"minimum turning radius {robot.min_turn_radius_m:.3f}m -- not drivable."
        )

    arc_len = r * abs(turn_angle_rad)
    curvature = (1.0 / r) if turn_angle_rad > 0 else (-1.0 / r)
    return [PathSegment("arc", arc_len, curvature)]


def drive_lap1(
    leg_grids: Iterable[list[list[int]]],
    cell_size_m: float,
    leg_starts: list[Point],
    leg_goals: list[Point],
    turn_angles_rad: list[float],
    robot: RobotConfig,
    grid_origins: Optional[list[Point]] = None,
    pass_sides_per_leg: Optional[list[Optional[dict[str, PassSide]]]] = None,
    corner_radius_m: Optional[float] = None,
) -> Generator[DriveCommand, None, None]:
    """Lap 1 driver: plan+drive one leg at a time (each leg's obstacles are
    only known once you reach it), with a fixed corner_arc() maneuver
    spliced in after each leg. This is a generator so the caller can pull
    leg N+1's grid from the live obstacle-logging process only once leg N
    has actually finished executing -- nothing beyond the current leg needs
    to be known in advance.

    leg_starts/leg_goals: per-leg start/goal points, in a SHARED world
    frame (e.g. field-frame coordinates) so consecutive legs line up.
    turn_angles_rad: the corner turn after each leg (len == number of legs;
    last entry can be 0.0 / omitted-effect if lap ends without a final turn
    before stopping).
    """
    leg_grids = list(leg_grids)
    n_legs = len(leg_grids)
    grid_origins = grid_origins or [(0.0, 0.0)] * n_legs
    pass_sides_per_leg = pass_sides_per_leg or [None] * n_legs

    for i in range(n_legs):
        obstacles = extract_square_obstacles(leg_grids[i], cell_size_m, origin=grid_origins[i])
        pass_sides = pass_sides_per_leg[i]
        if pass_sides:
            for obs in obstacles:
                if obs.id in pass_sides:
                    obs.pass_side = pass_sides[obs.id]

        graph = build_visibility_graph(leg_starts[i], leg_goals[i], obstacles, robot.inflation_m)
        path_indices = dijkstra(graph, 0, len(graph.nodes) - 1)
        if not path_indices:
            raise RuntimeError(f"No collision-free path found for leg {i}.")

        waypoints = [graph.nodes[j] for j in path_indices]
        leg_segments = smooth_polyline_to_segments(waypoints, robot.min_turn_radius_m)
        yield from segments_to_drive_commands(leg_segments, robot)

        if i < len(turn_angles_rad) and abs(turn_angles_rad[i]) > 1e-9:
            corner_segments = corner_arc(turn_angles_rad[i], robot, radius_m=corner_radius_m)
            yield from segments_to_drive_commands(corner_segments, robot)


# --------------------------------------------------------------------------
# Lap 2+: full lap known -- single stitched visibility graph across all
# legs and corners together, planned in one shot.
# --------------------------------------------------------------------------

def plan_lap(
    leg_obstacles: list[list[Obstacle]],
    leg_waypoints: list[tuple[Point, Point]],
    robot: RobotConfig,
    corner_waypoints: Optional[list[Point]] = None,
) -> Generator[DriveCommand, None, None]:
    """Plan an entire lap at once, now that all legs' obstacles are known
    (logged during lap 1 by the separate memory/mapping process).

    leg_obstacles: obstacles for each leg, in world/field-frame coordinates
        (already extracted -- reuse extract_square_obstacles() per leg on
        your stored map, or build Obstacle objects directly from memory).
    leg_waypoints: (start, goal) point pairs per leg, SAME frame as
        leg_obstacles. leg N's goal should equal leg N+1's start (or be
        connected via corner_waypoints below) for a continuous lap.
    corner_waypoints: optional explicit pivot point per corner (e.g. from
        FieldConfig.corner_pivot()) inserted between consecutive legs'
        waypoint pairs. If omitted, legs are assumed to already chain
        start-to-goal with no separate corner point needed.

    Unlike lap 1's leg-by-leg planning, this builds ONE visibility graph
    spanning every leg's obstacles plus the corner points, then runs a
    single Dijkstra + smoothing pass over the whole lap. This lets the
    global search trade off exit position on one leg against a better
    approach into the next leg's obstacles -- something leg-by-leg planning
    (lap 1) cannot do, since each leg there is solved independently.
    """
    if not leg_waypoints:
        raise ValueError("plan_lap requires at least one leg.")

    start = leg_waypoints[0][0]
    goal = leg_waypoints[-1][1]

    all_obstacles: list[Obstacle] = []
    for obs_list in leg_obstacles:
        all_obstacles.extend(obs_list)

    graph = build_visibility_graph(start, goal, all_obstacles, robot.inflation_m)

    # Splice corner waypoints in as mandatory pass-through nodes by forcing
    # edges through them: simplest robust approach is to add them as extra
    # nodes and re-run visibility checks against the same obstacle set, then
    # solve start -> corner0 -> corner1 -> ... -> goal as concatenated
    # shortest-path legs over the SAME graph (guarantees the path threads
    # through each fixed corner pivot while still avoiding all obstacles).
    if corner_waypoints:
        full_corners_per_obstacle = [obs.inflated_corners(robot.inflation_m) for obs in all_obstacles]
        via_points = [start] + list(corner_waypoints) + [goal]

        combined_indices: list[int] = []
        for seg_i in range(len(via_points) - 1):
            seg_start, seg_goal = via_points[seg_i], via_points[seg_i + 1]
            seg_graph = build_visibility_graph(seg_start, seg_goal, all_obstacles, robot.inflation_m)
            seg_path = dijkstra(seg_graph, 0, len(seg_graph.nodes) - 1)
            if not seg_path:
                raise RuntimeError(f"No collision-free path found between waypoint {seg_i} and {seg_i+1}.")
            seg_waypoints = [seg_graph.nodes[j] for j in seg_path]
            # avoid duplicating the shared junction point between segments
            if combined_indices and seg_waypoints:
                seg_waypoints = seg_waypoints[1:]
            combined_indices.extend(range(len(seg_waypoints)))  # placeholder, replaced below
            if seg_i == 0:
                waypoints_accum = seg_waypoints
            else:
                waypoints_accum = waypoints_accum + seg_waypoints
        waypoints = waypoints_accum
    else:
        path_indices = dijkstra(graph, 0, len(graph.nodes) - 1)
        if not path_indices:
            raise RuntimeError("No collision-free path found for this lap.")
        waypoints = [graph.nodes[i] for i in path_indices]

    segments = smooth_polyline_to_segments(waypoints, robot.min_turn_radius_m)
    return segments_to_drive_commands(segments, robot)


# --------------------------------------------------------------------------
# Top-level entry point
# --------------------------------------------------------------------------

def plan_leg(
    grid: list[list[int]],
    cell_size_m: float,
    start: Point,
    goal: Point,
    robot: RobotConfig,
    grid_origin: Point = (0.0, 0.0),
    pass_sides: Optional[dict[str, PassSide]] = None,
) -> Generator[DriveCommand, None, None]:
    """Full pipeline: occupancy grid -> drive command generator.

    pass_sides: optional {obstacle_id: "left"/"right"/"either"} override,
    e.g. from color classification (red -> "right", green -> "left").
    Obstacle ids are assigned in blob-discovery order as "obs1", "obs2", ...
    -- inspect via extract_square_obstacles() directly if you need to map
    detections to ids before calling plan_leg.
    """
    obstacles = extract_square_obstacles(grid, cell_size_m, origin=grid_origin)

    if pass_sides:
        for obs in obstacles:
            if obs.id in pass_sides:
                obs.pass_side = pass_sides[obs.id]

    graph = build_visibility_graph(start, goal, obstacles, robot.inflation_m)

    start_idx = 0
    goal_idx = len(graph.nodes) - 1
    path_indices = dijkstra(graph, start_idx, goal_idx)

    if not path_indices:
        raise RuntimeError("No collision-free path found for this leg.")

    waypoints = [graph.nodes[i] for i in path_indices]
    segments = smooth_polyline_to_segments(waypoints, robot.min_turn_radius_m)

    return segments_to_drive_commands(segments, robot)


# --------------------------------------------------------------------------
# Example usage / smoke test
# --------------------------------------------------------------------------

if __name__ == "__main__":
    # 20x6 grid, 0.5m cells -> 10m x 3m leg area
    W, H = 20, 6
    grid = [[0] * W for _ in range(H)]

    # obstacle 1 (red, pass right) roughly at x=3, y=0.5 -> cell approx
    grid[1][5] = 1
    grid[1][6] = 1
    grid[2][5] = 1
    grid[2][6] = 1

    # obstacle 2 (green, pass left) roughly at x=7, y=-0.5
    grid[3][13] = 1
    grid[3][14] = 1
    grid[4][13] = 1
    grid[4][14] = 1

    robot = RobotConfig()

    start = (0.0, 1.5)   # world coords, grid origin at (0,0), y up
    goal = (10.0, 1.5)

    # Need obstacle ids to assign pass_side -- discover them first.
    obs_preview = extract_square_obstacles(grid, cell_size_m=0.5)
    for o in obs_preview:
        print(f"{o.id}: center=({o.cx:.2f},{o.cy:.2f}) half_size={o.half_size:.2f}")

    pass_sides = {"obs1": "right", "obs2": "left"}

    cmd_gen = plan_leg(
        grid=grid,
        cell_size_m=0.5,
        start=start,
        goal=goal,
        robot=robot,
        pass_sides=pass_sides,
    )

    print("\nDrive commands:")
    total_dist = 0.0
    for cmd in cmd_gen:
        total_dist += cmd.distance_m
        print(
            f"  steer={math.degrees(cmd.steering_angle_rad):+6.1f} deg  "
            f"speed={cmd.speed_mps:.2f} m/s  dist={cmd.distance_m:.3f} m"
        )
    print(f"\nTotal path length: {total_dist:.3f} m "
          f"(straight-line was {math.hypot(goal[0]-start[0], goal[1]-start[1]):.3f} m)")
