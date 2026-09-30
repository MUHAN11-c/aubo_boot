"""Synthetic bag clouds and masks for peach2_core unit tests."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class SyntheticBag:
    points: np.ndarray
    bottom: np.ndarray
    neck: np.ndarray
    axis: np.ndarray
    d_max: float


def _basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, ref)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(axis, e1)


def tilted_axis(tilt_deg: float, azimuth_deg: float = 30.0) -> np.ndarray:
    t, a = np.radians(tilt_deg), np.radians(azimuth_deg)
    return np.array([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])


def bag_cloud(rng: np.random.Generator, bottom=(0.55, 0.10, 0.80), tilt_deg: float = 12.0,
              r_bottom: float = 0.040, r_top: float = 0.018, body_len: float = 0.13,
              r_neck: float = 0.008, neck_len: float = 0.03, n: int = 6000,
              camera=(0.0, 0.0, 0.85), noise_m: float = 0.0015,
              flying_ratio: float = 0.03, full_surface: bool = False) -> SyntheticBag:
    """
    Frustum body (wide bottom, narrow top) + neck cylinder, seen from one camera.

    Truth: bottom = centre of the bottom cap, neck = frustum/neck junction on the axis.
    Noise is applied along the camera ray (depth noise); flying pixels are pushed behind
    the surface along their ray.
    """
    axis = tilted_axis(tilt_deg)
    e1, e2 = _basis(axis)
    b = np.asarray(bottom, dtype=float)
    cam = np.asarray(camera, dtype=float)
    length = body_len + neck_len
    samples = []
    while sum(s.shape[0] for s in samples) < n:
        m = 4 * n
        t = rng.uniform(0.0, length, m)
        r = np.where(t < body_len, r_bottom + (r_top - r_bottom) * t / body_len, r_neck)
        keep = rng.uniform(0.0, r_bottom, m) < r  # area-proportional along t
        t, r = t[keep], r[keep]
        phi = rng.uniform(0.0, 2 * np.pi, t.size)
        radial = np.outer(np.cos(phi), e1) + np.outer(np.sin(phi), e2)
        p = b + np.outer(t, axis) + r[:, None] * radial
        if not full_surface:
            vis = np.einsum('ij,ij->i', radial, cam - p) > 0.0
            p = p[vis]
        samples.append(p)
    p = np.concatenate(samples)[:n]
    ray = p - cam
    ray /= np.linalg.norm(ray, axis=1)[:, None]
    p = p + ray * rng.normal(0.0, noise_m, (p.shape[0], 1))
    n_fly = int(flying_ratio * n)
    if n_fly:
        idx = rng.choice(p.shape[0], n_fly, replace=False)
        p[idx] = p[idx] + ray[idx] * rng.uniform(0.02, 0.30, (n_fly, 1))
    return SyntheticBag(points=p, bottom=b, neck=b + body_len * axis, axis=axis,
                        d_max=2 * r_bottom)


def bag_mask(h: int = 240, w: int = 320, bottom_px=(160.0, 200.0), axis_deg: float = -80.0,
             body_len: float = 130.0, r_bottom: float = 40.0, r_top: float = 18.0,
             r_neck: float = 8.0, neck_len: float = 30.0, tail_len: float = 0.0,
             r_tail: float = 14.0) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    2D bag silhouette. axis_deg is the image angle of bottom->neck (v grows downward).

    Returns (mask, bottom_px_truth, neck_px_truth, tie_px_truth).
    """
    a = np.array([np.cos(np.radians(axis_deg)), np.sin(np.radians(axis_deg))])
    n = np.array([-a[1], a[0]])
    vv, uu = np.mgrid[0:h, 0:w]
    rel = np.stack([uu - bottom_px[0], vv - bottom_px[1]], axis=-1).astype(float)
    t = rel @ a
    s = rel @ n
    total = body_len + neck_len + tail_len
    half = np.where(t < body_len, r_bottom + (r_top - r_bottom) * t / body_len,
                    np.where(t < body_len + neck_len, r_neck, r_tail))
    mask = (t >= 0.0) & (t <= total) & (np.abs(s) <= half)
    b = np.asarray(bottom_px, dtype=float)
    return mask, b, b + body_len * a, b + total * a
