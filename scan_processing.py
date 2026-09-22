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

ANGLE CONVENTION (changed Sept 2026, see docs/CHANGES.md): every robot-frame
angle in this codebase is CLOCKWISE from the robot's forward axis:
0 = forward, 90 = right, 180 = back, 270 = left. The matching robot-frame
cartesian axes are fwd_mm (+ ahead) and right_mm (+ to the right), so that
angle = atan2(right, fwd). The sensor's raw angles are mapped into this frame
once, by LIDAR_ANGLE_SIGN / LIDAR_ANGLE_ZERO_OFFSET_DEG (config.py).

What this module is used for now: clean_and_project() (range filtering + the
one place mount calibration is applied) feeds initialisation and the seat
check; the clustering/classification only colours the point cloud on the
dashboard. Initialisation does NOT use the clusters (it uses raw rays).
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
                                 # visible near a section's corners.
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
    angle_deg: float   # CORRECTED robot-frame angle, CLOCKWISE (0=forward, 90=right,
                        # 180=back, 270=left), angle_sign/angle_zero_offset_deg already
                        # applied. NOT the sensor's raw angle -- see clean_and_project.
    dist_mm: float
    quality: int
    fwd_mm: float = 0.0    # robot-relative cartesian: + ahead of the sensor
    right_mm: float = 0.0  # robot-relative cartesian: + to the sensor's right


@dataclass
class Cluster:
    points: list[ScanPoint]
    angular_span_deg: float
    arc_length_mm: float
    mean_range_mm: float
    kind: str                         # "wall" | "pillar" | "unclassified"
    line_normal: tuple[float, float] | None = None   # unit normal, robot frame (fwd, right)
    line_distance_mm: float | None = None             # perpendicular distance from robot origin to the fitted line
    flatness_residual_mm: float | None = None
    centroid: tuple[float, float] = field(default=(0.0, 0.0))


def angle_to_xy(angle_deg: float, dist_mm: float, angle_sign: int = 1, angle_zero_offset_deg: float = 0.0):
    """Raw sensor angle -> robot-relative cartesian (fwd_mm, right_mm).

    The corrected angle a = angle_sign * raw + angle_zero_offset_deg is
    CLOCKWISE from forward, so fwd = d*cos(a), right = d*sin(a)
    (a = 90 -> straight right, a = 270 -> straight left).

    angle_sign / angle_zero_offset_deg absorb your specific mount: if your
    unit's raw angle runs counter-clockwise, or its zero doesn't line up with
    the chassis forward direction, calibrate these two numbers once and every
    downstream angle is correct. See config.py.
    """
    a = math.radians(angle_sign * angle_deg + angle_zero_offset_deg)
    fwd = dist_mm * math.cos(a)
    right = dist_mm * math.sin(a)
    return fwd, right


def clean_and_project(raw_points, angle_sign: int, angle_zero_offset_deg: float) -> list[ScanPoint]:
    """Range/quality filtering + mount calibration, in one place.

    ScanPoint.angle_deg holds the CORRECTED angle (angle_sign /
    angle_zero_offset_deg applied), the same angle fwd_mm/right_mm are
    computed from. Everything downstream (initialisation's 0/90/270 rays,
    the gap test, the seat check) compares against ScanPoint.angle_deg
    directly, so it must already be in the robot frame: CLOCKWISE,
    0 = forward, 90 = right, 180 = back, 270 = left.

    (History: this once stored the raw angle while computing x/y from the
    corrected one -- harmless at sign=+1/offset=0, wrong the moment a real
    calibration was set. Fixed before the Sept 2026 changes; kept.)"""
    pts = []
    for angle_deg, dist_mm, quality in raw_points:
        if dist_mm < MIN_RANGE_MM or dist_mm > MAX_RANGE_MM:
            continue
        if quality < MIN_QUALITY:
            continue
        fwd, right = angle_to_xy(angle_deg, dist_mm, angle_sign, angle_zero_offset_deg)
        corrected_angle = (angle_sign * angle_deg + angle_zero_offset_deg) % 360.0
        pts.append(ScanPoint(angle_deg=corrected_angle, dist_mm=dist_mm, quality=quality,
                             fwd_mm=fwd, right_mm=right))
    pts.sort(key=lambda p: p.angle_deg)
    return pts


def _fit_line_tls(points: list[ScanPoint]) -> tuple[tuple[float, float], float, float]:
    """Total-least-squares line fit through robot-relative (fwd, right) points.
    Returns (unit_normal, distance_from_origin, rms_residual)."""
    xy = np.array([[p.fwd_mm, p.right_mm] for p in points])
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
        chord = math.hypot(p.fwd_mm - prev.fwd_mm, p.right_mm - prev.right_mm)
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
        c.centroid = (sum(p.fwd_mm for p in g) / len(g), sum(p.right_mm for p in g) / len(g))
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
