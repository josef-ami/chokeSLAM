"""
Pillar colour identification with the OV5647 fisheye camera (checkpoint D,
docs/CHANGES.md section 15, decisions #53-#65).

The camera answers ONE question: for a seat the LIDAR has already called
PRESENT, is the pillar RED or GREEN? It never detects, locates or confirms a
pillar -- that stays with seat_occupancy.py. This module is the geometry and
the pixel test; lane_tracker.py owns the requests (one per PRESENT seat) and
feeds it frames.

PER FRAME, PER PENDING SEAT (lane_tracker.LaneTracker.on_camera_frame)
    pose     the lane pose at the frame's capture time (#62), from the tracker's
             pose history -- never the pose "now", never the pose stored on the
             seat's LIDAR verdict: the robot keeps moving between frames
    camera   the lens position = pose + lever arm (CAMERA_OFFSET_*, #53)
    seat     its fixed lane position (seat_occupancy.seats()); bearing from the
             camera, clockwise, minus the pose's psi -> theta, the horizontal
             angle off the optical axis (+ = right, x CAMERA_BEARING_SIGN)
    range    near face = distance to the seat centre - 25 mm
    box      the pillar's near face, 50 mm wide (theta +/- atan(25 / range)) and
             0..100 mm tall, seen from CAMERA_HEIGHT_MM (#65), projected with
             cv2.fisheye.projectPoints (#58) and widened by
             COLOR_ID_ROI_MARGIN_FACTOR about its centre
    in view  every corner of the box is less than FOLD_DEG off the optical axis
             (beyond it the calibrated polynomial folds back and returns wrong
             pixels that look valid) and the box centre is inside the image
    colour   HSV fractions inside the box (classify)

FRAME CONVENTION
    Every frame is corrected in exactly one place, correct_frame (the camera is
    mounted upside-down, #54). Camera coordinates: x right, y down, z forward
    (OpenCV). The optical axis is horizontal (tilt 0).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

import config
import lane_frame as lf

PILLAR_HALF_MM = 25.0          # rulebook 13.19: 50 x 50 mm footprint
PILLAR_HEIGHT_MM = 100.0       # rulebook 13.19: 100 mm tall

RED, GREEN, UNKNOWN = "red", "green", "unknown"


def camera_model():
    """(K, D) as numpy arrays, read from config on every call (tuning-safe)."""
    return (np.asarray(config.CAMERA_K, dtype=np.float64),
            np.asarray(config.CAMERA_D, dtype=np.float64).reshape(4, 1))


def fold_deg(D=None) -> float:
    """Off-axis angle at which the equidistant polynomial
    r(t) = fx * t * (1 + D0 t^2 + D1 t^4 + D2 t^6 + D3 t^8) stops increasing.
    Beyond it the model maps a wider angle to a SMALLER radius: pixels that
    look valid but are wrong. Found numerically (0.01 deg steps)."""
    d = np.asarray(config.CAMERA_D if D is None else D, dtype=np.float64).ravel()
    t = np.radians(np.arange(0.0, 90.0, 0.01))
    r = t * (1 + d[0] * t ** 2 + d[1] * t ** 4 + d[2] * t ** 6 + d[3] * t ** 8)
    falling = np.nonzero(np.diff(r) <= 0)[0]
    return float(np.degrees(t[falling[0]])) if len(falling) else 90.0


def correct_frame(raw):
    """THE one place the mount orientation is known about (#54). Everything
    downstream takes a corrected frame."""
    return cv2.rotate(raw, cv2.ROTATE_180) if config.CAMERA_ROTATE_180 else raw


def project(points_cam) -> np.ndarray:
    """Camera-frame points (N x 3: x right, y down, z forward) -> pixels (N x 2),
    via cv2.fisheye.projectPoints (one source of truth for the lens model)."""
    K, D = camera_model()
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 1, 3)
    img, _ = cv2.fisheye.projectPoints(pts, np.zeros(3), np.zeros(3), K, D)
    return img.reshape(-1, 2)


def off_axis_deg(points_cam) -> np.ndarray:
    p = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    return np.degrees(np.arctan2(np.hypot(p[:, 0], p[:, 1]), p[:, 2]))


def bearing_to_pixel(theta_deg: float, range_mm: float = 1000.0, height_mm: float | None = None):
    """A point theta_deg right of the optical axis (horizontal plane, before
    CAMERA_BEARING_SIGN), range_mm away horizontally, height_mm above the floor
    (None = at lens height, the draft's horizon ray) -> (u, v)."""
    s = config.CAMERA_BEARING_SIGN
    t = math.radians(theta_deg)
    dy = 0.0 if height_mm is None else config.CAMERA_HEIGHT_MM - height_mm
    return tuple(project([[s * range_mm * math.sin(t), dy, range_mm * math.cos(t)]])[0])


@dataclass
class SeatView:
    theta_deg: float        # horizontal angle off the optical axis, + = right of the robot
    centre_mm: float        # camera -> seat centre, horizontal
    face_mm: float          # camera -> pillar's near face
    cam_x: float            # camera position in the lane frame
    cam_y: float


def seat_view(x: float, y: float, psi: float, direction: str, seat_x: float, seat_y: float) -> SeatView:
    """Where a seat is seen from the camera with the robot at lane pose (x, y, psi)."""
    cx, cy = lf.offset_in_lane(x, y, psi, direction, config.CAMERA_OFFSET_FORWARD_MM,
                               config.CAMERA_OFFSET_LATERAL_MM)
    dx, dy = seat_x - cx, seat_y - cy
    brg = lf.bearing_of(dx, dy, direction)
    theta = (brg - psi + 180.0) % 360.0 - 180.0
    c = math.hypot(dx, dy)
    return SeatView(theta, c, c - PILLAR_HALF_MM, cx, cy)


@dataclass
class Roi:
    u0: int
    u1: int                 # exclusive
    v0: int
    v1: int                 # exclusive
    u_centre: float
    v_centre: float

    @property
    def size(self) -> tuple[int, int]:
        return self.u1 - self.u0, self.v1 - self.v0


def pillar_roi(theta_deg: float, face_mm: float) -> tuple[Roi | None, str]:
    """The pillar's near face (50 x 100 mm) as a pixel box, or (None, why)."""
    if face_mm <= 1.0:
        return None, f"pillar face {face_mm:.0f} mm away: too close to project"
    s = config.CAMERA_BEARING_SIGN
    H = config.CAMERA_HEIGHT_MM
    a = math.atan2(PILLAR_HALF_MM, face_mm)
    corners = []
    for phi in (math.radians(theta_deg) - a, math.radians(theta_deg) + a):
        for h in (0.0, PILLAR_HEIGHT_MM):
            corners.append([s * face_mm * math.sin(phi), H - h, face_mm * math.cos(phi)])
    fold = fold_deg()
    worst = float(off_axis_deg(corners).max())
    if worst >= fold:
        return None, (f"out of view: part of the pillar is {worst:.0f} deg off the optical axis "
                      f"(lens model valid below {fold:.1f} deg)")
    px = project(corners)
    u_lo, u_hi = px[:, 0].min(), px[:, 0].max()
    v_lo, v_hi = px[:, 1].min(), px[:, 1].max()
    uc, vc = (u_lo + u_hi) / 2.0, (v_lo + v_hi) / 2.0
    W, Hh = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
    if not (0.0 <= uc < W and 0.0 <= vc < Hh):
        return None, f"out of view: pillar centre at pixel ({uc:.0f}, {vc:.0f}), outside the {W}x{Hh} image"
    m = config.COLOR_ID_ROI_MARGIN_FACTOR
    hw, hh = max((u_hi - u_lo) * m / 2.0, 1.0), max((v_hi - v_lo) * m / 2.0, 1.0)
    u0, u1 = max(0, int(math.floor(uc - hw))), min(W, int(math.ceil(uc + hw)))
    v0, v1 = max(0, int(math.floor(vc - hh))), min(Hh, int(math.ceil(vc + hh)))
    if u1 <= u0 or v1 <= v0:
        return None, "out of view: the box is empty after clipping to the image"
    return Roi(u0, u1, v0, v1, uc, vc), ""


def _hue_mask(h, ranges):
    m = np.zeros(h.shape, dtype=bool)
    for lo, hi in ranges:
        m |= (h >= lo) & (h <= hi)
    return m


def classify(frame_bgr, roi: Roi) -> tuple[str | None, float, float]:
    """(RED | GREEN | None, red fraction, green fraction) inside the box of a
    CORRECTED frame. None = not confident (15.4): the winning colour must cover
    COLOR_ID_MIN_FRACTION of the box and be COLOR_ID_MARGIN_RATIO times the other."""
    patch = frame_bgr[roi.v0:roi.v1, roi.u0:roi.u1]
    if patch.size == 0:
        return None, 0.0, 0.0
    hsv = cv2.cvtColor(np.ascontiguousarray(patch), cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    vivid = (s >= config.COLOR_MIN_SAT) & (v >= config.COLOR_MIN_VAL)
    n = float(h.size)
    rf = float(np.count_nonzero(vivid & _hue_mask(h, config.COLOR_RED_HUE))) / n
    gf = float(np.count_nonzero(vivid & _hue_mask(h, config.COLOR_GREEN_HUE))) / n
    best, other, name = (rf, gf, RED) if rf >= gf else (gf, rf, GREEN)
    if best >= config.COLOR_ID_MIN_FRACTION and best >= config.COLOR_ID_MARGIN_RATIO * other and best > 0:
        return name, rf, gf
    return None, rf, gf
