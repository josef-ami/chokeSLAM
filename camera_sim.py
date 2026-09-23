"""
Simulated OV5647 fisheye frames (checkpoint D): for test_color_id.py and the
dashboard's mock mode. NOT used on the robot.

Each pixel of the 640x480 image is turned into a ray through the calibrated
lens (cv2.fisheye.undistortPoints -- the inverse direction to the one
color_id.py uses, so the two are not the same code path) and cast into a 3D
world:
    pillars   50 x 50 x 100 mm boxes on the floor, RED or GREEN
    floor     the white mat inside the field; black outside it or on the island
    above     anything that doesn't hit the floor or a pillar: black (walls)
The frame is then made the way the real camera would deliver it: mirrored if
CAMERA_BEARING_SIGN is -1 (the camera's x axis points to the robot's left, so
the calibration's principal point stays where it is), rotated 180 deg if CAMERA_ROTATE_180 (upside-down
mount), plus pixel noise. Pixels beyond the lens model's fold angle (the image
corners) are black: the calibration says nothing valid about them.

Global frame as simulation.py: bearing b clockwise from +y, direction
(sin b, cos b); z up, floor at z = 0.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

import color_id
import config
import mat_geometry as geo

BGR = {"red": (55, 39, 238), "green": (44, 214, 68)}
MAT_BGR = (235, 235, 235)
WALL_BGR = (25, 25, 25)

_rays_cache: dict = {}


def _pixel_rays():
    """Unit rays (H x W x 3, camera frame x right, y down, z forward) and a
    mask of pixels inside the lens model's valid angle."""
    key = (config.CAMERA_WIDTH, config.CAMERA_HEIGHT, tuple(map(tuple, config.CAMERA_K)), tuple(config.CAMERA_D))
    if key in _rays_cache:
        return _rays_cache[key]
    W, H = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
    K, D = color_id.camera_model()
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    pix = np.stack([u, v], axis=-1).reshape(-1, 1, 2)
    norm = cv2.fisheye.undistortPoints(pix, K, D).reshape(H, W, 2)
    rays = np.concatenate([norm, np.ones((H, W, 1))], axis=-1)
    rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
    # valid only where the distorted radius is below the fold (the polynomial is invertible there)
    d = np.asarray(config.CAMERA_D, dtype=np.float64)
    tf = math.radians(color_id.fold_deg())
    r_fold = tf * (1 + d[0] * tf ** 2 + d[1] * tf ** 4 + d[2] * tf ** 6 + d[3] * tf ** 8)
    xd = (u - K[0, 2]) / K[0, 0]
    yd = (v - K[1, 2]) / K[1, 1]
    valid = np.hypot(xd, yd) < 0.995 * r_fold
    _rays_cache[key] = (rays, valid)
    return rays, valid


def render(cam_gx: float, cam_gy: float, heading_deg: float, pillars, rng: np.random.Generator | None = None,
           noise: float = 6.0) -> np.ndarray:
    """The RAW frame (as delivered by the camera) from a lens at global
    (cam_gx, cam_gy), CAMERA_HEIGHT_MM above the floor, facing heading_deg.
    pillars: objects with x_mm, y_mm, color ("red" / "green")."""
    rays, valid = _pixel_rays()
    rays = rays.astype(np.float32, copy=False)
    Hc = float(config.CAMERA_HEIGHT_MM)
    b = math.radians(heading_deg)
    f = np.array([math.sin(b), math.cos(b)], dtype=np.float32)
    r = np.array([math.cos(b), -math.sin(b)], dtype=np.float32)
    if config.CAMERA_BEARING_SIGN < 0:
        r = -r            # a mirrored camera: the robot's right appears on the image's left
    # world ray = x * r + y * down + z * f  (down = -z); only the xy part depends on the heading
    wx = rays[..., 0] * r[0] + rays[..., 2] * f[0]
    wy = rays[..., 0] * r[1] + rays[..., 2] * f[1]
    wz = -rays[..., 1]
    img = np.empty(rays.shape[:2] + (3,), dtype=np.float32)
    img[:] = WALL_BGR

    # floor
    down = wz < -1e-6
    tfl = np.full(wz.shape, np.inf, dtype=np.float32)
    tfl[down] = Hc / -wz[down]
    x0, y0, x1, y1 = geo.OUTER_BOX
    ix0, iy0, ix1, iy1 = geo.ISLAND_BOX
    with np.errstate(invalid="ignore"):
        fx, fy = cam_gx + tfl * wx, cam_gy + tfl * wy
        on_mat = down & (fx > x0) & (fx < x1) & (fy > y0) & (fy < y1) & \
            ~((fx > ix0) & (fx < ix1) & (fy > iy0) & (fy < iy1))
    img[on_mat] = MAT_BGR
    best = tfl

    # pillars: slab test, only inside each pillar's projected pixel box (+ margin)
    W, H = config.CAMERA_WIDTH, config.CAMERA_HEIGHT
    cosb, sinb = math.cos(b), math.sin(b)
    order = sorted(pillars, key=lambda p: -math.hypot(p.x_mm - cam_gx, p.y_mm - cam_gy))
    for p in order:
        col = BGR.get(getattr(p, "color", ""), None)
        if col is None:
            continue
        corners = []
        for ex in (-25.0, 25.0):
            for ey in (-25.0, 25.0):
                for ez in (0.0, 100.0):
                    dx, dy = p.x_mm + ex - cam_gx, p.y_mm + ey - cam_gy
                    fwd = dx * sinb + dy * cosb
                    right = dx * cosb - dy * sinb
                    if config.CAMERA_BEARING_SIGN < 0:
                        right = -right
                    corners.append([right, Hc - ez, fwd])
        corners = np.asarray(corners)
        if (corners[:, 2] <= 1.0).any() or color_id.off_axis_deg(corners).max() >= color_id.fold_deg():
            continue                                    # behind, beside, or where the lens model folds
        px = color_id.project(corners)
        u0, u1 = int(max(0, px[:, 0].min() - 3)), int(min(W, px[:, 0].max() + 4))
        v0, v1 = int(max(0, px[:, 1].min() - 3)), int(min(H, px[:, 1].max() + 4))
        if u1 <= u0 or v1 <= v0:
            continue
        sl = (slice(v0, v1), slice(u0, u1))
        dxs, dys, dzs = wx[sl], wy[sl], wz[sl]
        o = (cam_gx, cam_gy, Hc)
        lo = (p.x_mm - 25.0, p.y_mm - 25.0, 0.0)
        hi = (p.x_mm + 25.0, p.y_mm + 25.0, 100.0)
        tn = np.full(dxs.shape, -np.inf, dtype=np.float32)
        tx = np.full(dxs.shape, np.inf, dtype=np.float32)
        for d, oo, a, c in ((dxs, o[0], lo[0], hi[0]), (dys, o[1], lo[1], hi[1]), (dzs, o[2], lo[2], hi[2])):
            dd = np.where(np.abs(d) < 1e-9, np.float32(1e-9), d)
            t1, t2 = (a - oo) / dd, (c - oo) / dd
            tn = np.maximum(tn, np.minimum(t1, t2))
            tx = np.minimum(tx, np.maximum(t1, t2))
        hit = (tx >= tn) & (tn > 1e-6) & (tn < best[sl])
        img[sl][hit] = col
        best[sl][hit] = tn[hit]

    img[~valid] = 0.0
    if rng is not None and noise > 0:
        img += rng.normal(0.0, noise, img.shape).astype(np.float32)
    out = np.clip(img, 0, 255).astype(np.uint8)
    if config.CAMERA_ROTATE_180:
        out = cv2.rotate(out, cv2.ROTATE_180)
    return out


def camera_global(gx: float, gy: float, heading_deg: float) -> tuple[float, float]:
    """Lens position for a robot pose reference point at global (gx, gy)."""
    b = math.radians(heading_deg)
    F, L = config.CAMERA_OFFSET_FORWARD_MM, config.CAMERA_OFFSET_LATERAL_MM
    return (gx + F * math.sin(b) - L * math.cos(b), gy + F * math.cos(b) + L * math.sin(b))
