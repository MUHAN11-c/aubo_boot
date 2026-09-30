"""Analytic ray-cast depth scenes for scene_core tests (no ROS, no rendering library)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from peach2_scene.scene_core import SceneParams

W, H = 320, 240
K = np.array([[250.0, 0.0, 160.0], [0.0, 250.0, 120.0], [0.0, 0.0, 1.0]])

# Camera at base (0, 0, 0.8) looking along base +X (optical z = base x, x = -y, y = -z).
T_BASE_CAMERA = np.eye(4)
T_BASE_CAMERA[:3, :3] = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BASE_CAMERA[:3, 3] = [0.0, 0.0, 0.8]

LABEL_OTHER, LABEL_BRANCH, LABEL_LEAF = 0, 1, 2


def params(**overrides) -> SceneParams:
    values = {'voxel_size_m': 0.03, 'min_points_per_voxel': 5, 'workspace_radius_m': 2.0,
              'self_margin_m': 0.03, 'target_margin_m': 0.05, 'target_axial_margin_m': 0.05,
              'target_neck_overshoot_m': 0.0, 'hard_min_thickness_m': 0.012,
              'linearity_min': 0.5, 'flat_thickness_max_m': 0.010,
              'soft_max_extent_m': 0.25, 'branch_vote_min': 0.2, 'leaf_vote_min': 0.6,
              'leaf_mask_max_width_m': 0.05, 'fit_min_fill_ratio': 0.25, 'max_objects': 400,
              'pixel_stride': 1, 'min_confidence': 0.5, 'min_depth_m': 0.1, 'max_depth_m': 3.0}
    values.update(overrides)
    return SceneParams(**values)


@dataclass
class Cylinder:
    a: np.ndarray
    b: np.ndarray
    radius: float
    label: int = LABEL_OTHER


@dataclass
class Rect:
    center: np.ndarray
    e1: np.ndarray
    e2: np.ndarray
    h1: float
    h2: float
    label: int = LABEL_LEAF


@dataclass
class Box:
    center: np.ndarray
    size: np.ndarray
    label: int = LABEL_OTHER


@dataclass
class Scene:
    items: list = field(default_factory=list)


def _rays() -> np.ndarray:
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    d = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], axis=-1)
    return d.reshape(-1, 3)


def _to_cam(p: np.ndarray) -> np.ndarray:
    R, t = T_BASE_CAMERA[:3, :3], T_BASE_CAMERA[:3, 3]
    return (np.asarray(p, dtype=np.float64) - t) @ R


def _dir_to_cam(d: np.ndarray) -> np.ndarray:
    return np.asarray(d, dtype=np.float64) @ T_BASE_CAMERA[:3, :3]


def _hit_cylinder(d: np.ndarray, c: Cylinder) -> np.ndarray:
    a, b = _to_cam(c.a), _to_cam(c.b)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    w = -a
    dp = d - (d @ u)[:, None] * u
    wp = w - (w @ u) * u
    A = np.einsum('ij,ij->i', dp, dp)
    B = 2.0 * dp @ wp
    C = wp @ wp - c.radius ** 2
    disc = B * B - 4.0 * A * C
    t = np.full(d.shape[0], np.inf)
    ok = (disc >= 0.0) & (A > 1e-12)
    tt = (-B[ok] - np.sqrt(disc[ok])) / (2.0 * A[ok])
    s = (w + tt[:, None] * d[ok]) @ u
    good = (tt > 0.0) & (s >= 0.0) & (s <= L)
    idx = np.nonzero(ok)[0][good]
    t[idx] = tt[good]
    return t


def _hit_rect(d: np.ndarray, r: Rect) -> np.ndarray:
    c = _to_cam(r.center)
    e1, e2 = _dir_to_cam(r.e1), _dir_to_cam(r.e2)
    n = np.cross(e1, e2)
    dn = d @ n
    t = np.full(d.shape[0], np.inf)
    ok = np.abs(dn) > 1e-9
    tt = (c @ n) / dn[ok]
    p = tt[:, None] * d[ok] - c
    good = (tt > 0.0) & (np.abs(p @ e1) <= r.h1) & (np.abs(p @ e2) <= r.h2)
    idx = np.nonzero(ok)[0][good]
    t[idx] = tt[good]
    return t


def _hit_box(d: np.ndarray, bx: Box) -> np.ndarray:
    # Box axes are base axes; slab test in base coordinates.
    R = T_BASE_CAMERA[:3, :3]
    o = T_BASE_CAMERA[:3, 3] - np.asarray(bx.center, dtype=np.float64)
    db = d @ R.T
    half = 0.5 * np.asarray(bx.size, dtype=np.float64)
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (-half - o) / db
        t2 = (half - o) / db
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    t = np.full(d.shape[0], np.inf)
    good = (tmax >= tmin) & (tmin > 0.0)
    t[good] = tmin[good]
    return t


def render(scene: Scene, noise_m: float = 0.001, n_flying: int = 0,
           flying_depth=(0.9, 1.2), seed: int = 0):
    """Return (depth_m (H, W), branch_mask, leaf_mask, flying_points_base (n, 3))."""
    rng = np.random.default_rng(seed)
    d = _rays()
    best = np.full(d.shape[0], np.inf)
    label = np.full(d.shape[0], -1)
    for item in scene.items:
        if isinstance(item, Cylinder):
            t = _hit_cylinder(d, item)
        elif isinstance(item, Rect):
            t = _hit_rect(d, item)
        else:
            t = _hit_box(d, item)
        closer = t < best
        best[closer] = t[closer]
        label[closer] = item.label
    depth = np.where(np.isfinite(best), best, 0.0)
    hit = depth > 0.0
    depth[hit] += rng.normal(0.0, noise_m, int(hit.sum()))
    flying = np.zeros((0, 3))
    if n_flying:
        empty = np.nonzero(~hit)[0]
        pick = rng.choice(empty, size=n_flying, replace=False)
        z = rng.uniform(flying_depth[0], flying_depth[1], n_flying)
        depth[pick] = z
        cam = d[pick] * z[:, None]
        flying = cam @ T_BASE_CAMERA[:3, :3].T + T_BASE_CAMERA[:3, 3]
    depth = depth.reshape(H, W)
    label = label.reshape(H, W)
    return depth, label == LABEL_BRANCH, label == LABEL_LEAF, flying


def box_surface(center, size, spacing: float = 0.005) -> np.ndarray:
    """Dense surface samples of an axis-aligned box in base_link (robot self geometry)."""
    c = np.asarray(center, dtype=np.float64)
    h = 0.5 * np.asarray(size, dtype=np.float64)
    pts = []
    for axis in range(3):
        o = [k for k in range(3) if k != axis]
        g0 = np.arange(-h[o[0]], h[o[0]] + 1e-9, spacing)
        g1 = np.arange(-h[o[1]], h[o[1]] + 1e-9, spacing)
        a, b = np.meshgrid(g0, g1)
        for sign in (-1.0, 1.0):
            p = np.zeros((a.size, 3))
            p[:, axis] = sign * h[axis]
            p[:, o[0]] = a.ravel()
            p[:, o[1]] = b.ravel()
            pts.append(p)
    return np.vstack(pts) + c


def _tilted(yaw_deg: float):
    """Leaf plane axes: e1 horizontal (tilted about base z), e2 vertical; normal ~ -x."""
    a = np.deg2rad(yaw_deg)
    e1 = np.array([np.sin(a), np.cos(a), 0.0])
    e2 = np.array([0.0, 0.0, 1.0])
    return e1, e2


BRANCH = Cylinder(np.array([0.75, -0.40, 1.06]), np.array([0.75, 0.40, 1.00]), 0.02, LABEL_BRANCH)
TWIG = Cylinder(np.array([0.66, -0.12, 0.52]), np.array([0.66, -0.12, 0.66]), 0.003, LABEL_BRANCH)
BAG_BOTTOM = np.array([0.70, 0.22, 0.60])
BAG_NECK = np.array([0.70, 0.22, 0.74])
BAG = Cylinder(BAG_BOTTOM, BAG_NECK, 0.04, LABEL_OTHER)
BAG_D95 = 0.08
TOOL_CENTER = np.array([0.30, 0.0, 0.70])
TOOL_SIZE = np.array([0.08, 0.06, 0.05])
TOOL = Box(TOOL_CENTER, TOOL_SIZE, LABEL_OTHER)
SQUARE_LEAVES = [
    Rect(np.array([0.62, -0.34, 0.70]), *_tilted(20.0), 0.025, 0.025),
    Rect(np.array([0.62, -0.26, 0.56]), *_tilted(-15.0), 0.025, 0.025),
]
LONG_LEAF = Rect(np.array([0.60, -0.30, 0.88]), *_tilted(10.0), 0.04, 0.015)


def orchard_scene() -> Scene:
    """Thick branch, thin twig, target bag, two square leaves, one elongated leaf, tool."""
    return Scene([BRANCH, TWIG, BAG, *SQUARE_LEAVES, LONG_LEAF, TOOL])
