"""
Turns a raw 360-degree LIDAR scan into classified clusters: long, flat
clusters are walls; short ones are candidate pillars. See the design
discussion this came out of -- the key points encoded here:

  - The wall/obstacle threshold is in ARC LENGTH (range x angular width),
    not point count, so it stays correct at any range.
  - A wall cluster's perpendicular distance comes from a total-least-squares
    line fit over all its points, not a single ray -- this both denoises the
    reading and gives a flatness residual you can use to sanity-check that
    it really is a flat wall and not, e.g., two pillars that happened to
    cluster together.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Tunables -- these depend on your LIDAR's noise characteristics and the
# field's real dimensions; the defaults are reasonable starting points.
# ---------------------------------------------------------------------------
MAX_CHORD_JUMP_MM = 70.0        # break a cluster when consecutive points' straight-line (x,y)
                                 # separation exceeds this. Euclidean chord distance, not raw
                                 # range difference -- a flat wall viewed at a shallow/grazing
                                 # angle has rapidly changing RANGE between equal-angle samples
                                 # even though it's physically continuous, which fragments
                                 # clustering if you threshold on range difference directly.
MAX_ANGLE_GAP_DEG = 2.5         # break a cluster when there's an angular gap bigger than this (occlusion)
MAX_CLUSTER_SPAN_DEG = 110.0    # hard cap on a single cluster's angular extent. Sequential/chord-distance
                                 # clustering can "chain" through a slowly, continuously changing sequence
                                 # of points (classically: a wall viewed at an ever more oblique angle,
                                 # smoothly bending around a real corner into the next wall) with no single
                                 # step big enough to trip the gap/chord thresholds, even though the result
                                 # spans two physically different, non-collinear surfaces. This is most
                                 # visible near a section's corners; see the near-corner note in README.
MIN_QUALITY = 0                 # drop points below this quality value (set >0 if your unit reports noisy low-quality returns)
MIN_RANGE_MM = 60.0             # drop points closer than the sensor's reliable minimum range
MAX_RANGE_MM = 6000.0           # drop obviously-bad far returns

WALL_MIN_ARC_LENGTH_MM = 220.0  # a real wall segment is metres long; a 50mm pillar
                                 # subtends far less arc than this at any sane range
WALL_MAX_FLATNESS_RESIDUAL_MM = 18.0  # RMS perpendicular distance from the fitted
                                       # line; keeps a wall+pillar merged-cluster
                                       # (or two walls merged across a nearby corner)
                                       # from being accepted as "flat"
PILLAR_MIN_ARC_LENGTH_MM = 20.0       # below this, a cluster is more likely a grazing-angle
                                       # fragment off a wall than a real 50mm pillar
PILLAR_MAX_ARC_LENGTH_MM = 160.0      # candidate-pillar upper bound (pillar cross-section is 50mm)
MIN_POINTS_PER_PILLAR = 2             # reject singleton-point fragments


@dataclass
class ScanPoint:
    angle_deg: float   # robot-relative, sensor's own convention (see lidar_source.py)
    dist_mm: float
    quality: int
    x_mm: float = 0.0  # robot-relative cartesian, +X = robot forward, +Y = robot left
    y_mm: float = 0.0


@dataclass
class Cluster:
    points: list[ScanPoint]
    angular_span_deg: float
    arc_length_mm: float
    mean_range_mm: float
    kind: str                         # "wall" | "pillar" | "unclassified"
    line_normal: tuple[float, float] | None = None   # unit normal, robot frame
    line_distance_mm: float | None = None             # perpendicular distance from robot origin to the fitted line
    flatness_residual_mm: float | None = None
    centroid: tuple[float, float] = field(default=(0.0, 0.0))


def angle_to_xy(angle_deg: float, dist_mm: float, angle_sign: int = 1, angle_zero_offset_deg: float = 0.0):
    """Robot-relative angle -> robot-relative cartesian.
    +X = robot forward, +Y = robot left, matching BROADSIDE_HEADING_DEG's
    "front/back" language used elsewhere.

    angle_sign / angle_zero_offset_deg absorb your specific mount: if your
    LIDAR's angle increases clockwise and its own zero doesn't line up with
    the chassis forward direction, calibrate these two numbers once and
    every downstream angle is correct. See config.py.
    """
    a = math.radians(angle_sign * angle_deg + angle_zero_offset_deg)
    x = dist_mm * math.cos(a)
    y = dist_mm * math.sin(a)
    return x, y


def clean_and_project(raw_points, angle_sign: int, angle_zero_offset_deg: float) -> list[ScanPoint]:
    pts = []
    for angle_deg, dist_mm, quality in raw_points:
        if dist_mm < MIN_RANGE_MM or dist_mm > MAX_RANGE_MM:
            continue
        if quality < MIN_QUALITY:
            continue
        x, y = angle_to_xy(angle_deg, dist_mm, angle_sign, angle_zero_offset_deg)
        pts.append(ScanPoint(angle_deg=angle_deg % 360.0, dist_mm=dist_mm, quality=quality, x_mm=x, y_mm=y))
    pts.sort(key=lambda p: p.angle_deg)
    return pts


def _fit_line_tls(points: list[ScanPoint]) -> tuple[tuple[float, float], float, float]:
    """Total-least-squares line fit through robot-relative (x,y) points.
    Returns (unit_normal, distance_from_origin, rms_residual)."""
    xy = np.array([[p.x_mm, p.y_mm] for p in points])
    centroid = xy.mean(axis=0)
    centered = xy - centroid
    # Smallest singular vector of the centered points = the line's normal direction.
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    distance = float(np.dot(centroid, normal))
    if distance < 0:
        # keep the normal pointing away from the robot (outward), makes distance positive
        normal = -normal
        distance = -distance
    residuals = centered @ normal
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return (float(normal[0]), float(normal[1])), distance, rms


def cluster_points(points: list[ScanPoint]) -> list[Cluster]:
    """Sequential angular clustering around the full circle.

    Rather than clustering points 0->359 and then trying to stitch the ends
    back together, this first finds the single LARGEST angular gap in the
    whole scan and starts there. That gap is almost always a genuine
    occlusion/empty region (e.g. the far side of a pillar) -- starting the
    linear pass there means a real, continuous feature (like the wall
    straight ahead, which usually straddles the 0/360 seam for a
    front-facing sensor) is never spuriously cut in half just because it
    happened to sit across index 0.
    """
    if not points:
        return []
    n = len(points)
    gaps = [((points[(i + 1) % n].angle_deg - points[i].angle_deg) % 360.0) for i in range(n)]
    cut_idx = max(range(n), key=lambda i: gaps[i])
    ordered = points[cut_idx + 1:] + points[:cut_idx + 1]

    groups: list[list[ScanPoint]] = [[ordered[0]]]
    for p in ordered[1:]:
        prev = groups[-1][-1]
        head = groups[-1][0]
        angle_gap = (p.angle_deg - prev.angle_deg) % 360.0
        chord = math.hypot(p.x_mm - prev.x_mm, p.y_mm - prev.y_mm)
        running_span = (p.angle_deg - head.angle_deg) % 360.0
        if angle_gap <= MAX_ANGLE_GAP_DEG and chord <= MAX_CHORD_JUMP_MM and running_span <= MAX_CLUSTER_SPAN_DEG:
            groups[-1].append(p)
        else:
            groups.append([p])

    clusters = []
    for g in groups:
        span = (g[-1].angle_deg - g[0].angle_deg) % 360.0
        mean_range = sum(p.dist_mm for p in g) / len(g)
        arc_length = mean_range * math.radians(max(span, 0.01))
        c = Cluster(points=g, angular_span_deg=span, arc_length_mm=arc_length,
                    mean_range_mm=mean_range, kind="unclassified")
        c.centroid = (sum(p.x_mm for p in g) / len(g), sum(p.y_mm for p in g) / len(g))
        if len(g) >= 3:
            normal, dist, rms = _fit_line_tls(g)
            c.line_normal, c.line_distance_mm, c.flatness_residual_mm = normal, dist, rms
        clusters.append(c)
    return clusters


def classify_clusters(clusters: list[Cluster]) -> list[Cluster]:
    for c in clusters:
        if (c.arc_length_mm >= WALL_MIN_ARC_LENGTH_MM
                and c.flatness_residual_mm is not None
                and c.flatness_residual_mm <= WALL_MAX_FLATNESS_RESIDUAL_MM):
            c.kind = "wall"
        elif (PILLAR_MIN_ARC_LENGTH_MM <= c.arc_length_mm <= PILLAR_MAX_ARC_LENGTH_MM
                and len(c.points) >= MIN_POINTS_PER_PILLAR):
            c.kind = "pillar"
        else:
            c.kind = "unclassified"   # e.g. a wall segment partly occluded, a grazing-angle
                                       # fragment, or two walls merged across a nearby corner
                                       # -- don't guess
    return clusters


def process_scan(raw_points, angle_sign: int = 1, angle_zero_offset_deg: float = 0.0) -> list[Cluster]:
    pts = clean_and_project(raw_points, angle_sign, angle_zero_offset_deg)
    clusters = cluster_points(pts)
    return classify_clusters(clusters)
