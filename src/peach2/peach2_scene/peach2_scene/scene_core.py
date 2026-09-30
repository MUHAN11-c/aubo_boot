"""
Layered scene snapshot core (zero ROS): depth frame(s) -> L2 hard objects + L3 soft voxels.

Input depth is metres in the camera optical frame; everything after `prepare_frame` is in
base_link. L1 (URDF fixed geometry) is already in the PlanningScene and is removed here by the
self filter together with the arm and tool. L4 targets are hollowed out of the snapshot and left
to peach2_manipulation (it owns the target capsules and the stage-local ACM).

Pipeline (order is semantics):
  1. back-project (peach2_core.depth.mask_points, one call per 2D label) -> base_link;
  2. workspace radius crop, self filter and target-neighbourhood filter on points;
  3. voxelise; voxels with fewer than `min_points_per_voxel` points are flying points;
  4. voxel-centre filters widened by the voxel half diagonal, so a whole cube never touches the
     robot or a target neighbourhood;
  5. hard/soft classification (2D branch/leaf votes, geometric PCA fallback);
  6. 26-connected clustering of hard voxels, each cluster covered by capsules/boxes that are
     split until they are tight and clear of the robot and every target neighbourhood;
  7. cap: keep the `max_objects` objects nearest to the TCP.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import itertools
import math
from typing import Sequence

import numpy as np
from peach2_core.depth import mask_points
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

LABEL_NONE = 0
LABEL_BRANCH = 1
LABEL_LEAF = 2

KIND_BOX = 'box'
KIND_CAPSULE = 'capsule'

CLASS_HARD_GEOMETRY = 'hard_geometry'
CLASS_HARD_BRANCH = 'hard_branch'
CLASS_SOFT_LEAF = 'soft_leaf'
CLASS_SOFT_THIN = 'soft_thin'
CLASS_SOFT_FLAT = 'soft_flat'

_SQRT3 = math.sqrt(3.0)
# Neighbourhood PCA below this many points is not trusted: the voxel stays hard.
_MIN_PCA_POINTS = 10
_OFFSETS = np.array(list(itertools.product((-1, 0, 1), repeat=3)), dtype=np.int64)


# ------------------------------------------------------------------------------ data types
@dataclass(frozen=True)
class Capsule:
    """Segment p0-p1 swept by a sphere of `radius` [m]."""

    p0: np.ndarray
    p1: np.ndarray
    radius: float


@dataclass
class SelfGeometry:
    """Robot collision geometry at the frame stamp, base_link: surface samples and/or capsules."""

    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    capsules: list[Capsule] = field(default_factory=list)
    _tree: cKDTree | None = field(default=None, init=False, repr=False, compare=False)

    def kdtree(self) -> cKDTree | None:
        """KD-tree over `points`, built once (full robot samples are ~1e5..1e6 points)."""
        if self._tree is None and np.asarray(self.points).reshape(-1, 3).shape[0]:
            self._tree = cKDTree(np.asarray(self.points, dtype=np.float64).reshape(-1, 3))
        return self._tree


@dataclass(frozen=True)
class TargetCapsule:
    """L4 target neighbourhood source: bag bottom -> neck in base_link, d95 [m]."""

    target_id: str
    bottom: np.ndarray
    neck: np.ndarray
    d95_m: float


@dataclass
class FrameInput:
    """One depth frame with everything sampled at its image stamp."""

    depth_m: np.ndarray                     # (H, W) metres, 0 / non-finite = invalid
    K: np.ndarray                           # (3, 3) intrinsics of the depth image
    T_base_camera: np.ndarray               # (4, 4) base_link <- camera optical frame
    robot: SelfGeometry
    branch_mask: np.ndarray | None = None   # (H, W) bool, registered to the depth image
    leaf_mask: np.ndarray | None = None
    confidence: np.ndarray | None = None    # (H, W) mono8 or 0..1


@dataclass
class FramePoints:
    """Filtered base_link points of one frame, kept so a snapshot can be rebuilt later."""

    points: np.ndarray       # (N, 3) base_link
    labels: np.ndarray       # (N,) uint8 LABEL_*
    masked: bool             # labels carry information (frame had 2D masks)
    robot: SelfGeometry
    n_input: int
    n_workspace_removed: int
    n_self_removed: int


@dataclass(frozen=True)
class SceneParams:
    voxel_size_m: float
    min_points_per_voxel: int
    workspace_radius_m: float
    self_margin_m: float
    target_margin_m: float
    target_axial_margin_m: float
    # Tool mouth passes the neck by L_blade (max over tools) to put the blade on the neck;
    # the twig there is a mandatory pass-through region, so it is hollowed too.
    target_neck_overshoot_m: float
    hard_min_thickness_m: float
    linearity_min: float
    flat_thickness_max_m: float
    soft_max_extent_m: float
    branch_vote_min: float
    leaf_vote_min: float
    leaf_mask_max_width_m: float
    fit_min_fill_ratio: float
    max_objects: int
    pixel_stride: int
    min_confidence: float
    min_depth_m: float
    max_depth_m: float


@dataclass
class HardObject:
    """
    L2 obstacle in base_link.

    box: `axes` columns are the box axes, `half_extents` the half sizes along them.
    capsule: axis is `axes[:, 2]`, `half_extents = (radius, radius, half_length)`.
    """

    kind: str
    center: np.ndarray
    axes: np.ndarray
    half_extents: np.ndarray
    n_voxels: int = 0
    distance_to_tcp_m: float = math.inf

    @property
    def radius(self) -> float:
        return float(self.half_extents[0])

    @property
    def p0(self) -> np.ndarray:
        return self.center - self.axes[:, 2] * self.half_extents[2]

    @property
    def p1(self) -> np.ndarray:
        return self.center + self.axes[:, 2] * self.half_extents[2]

    def volume(self) -> float:
        if self.kind == KIND_BOX:
            return float(8.0 * np.prod(self.half_extents))
        r, h = self.radius, float(self.half_extents[2])
        return math.pi * r * r * 2.0 * h + 4.0 / 3.0 * math.pi * r ** 3


@dataclass
class SceneResult:
    hard_objects: list[HardObject]
    soft_voxel_centers: np.ndarray          # (K, 3) L3, not written to the collision world
    truncated: bool
    stats: dict[str, int]

    @property
    def n_soft_voxels(self) -> int:
        return int(self.soft_voxel_centers.shape[0])


# ------------------------------------------------------------------------------ geometry
def point_segment_distance(points: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(N,) distance from points to segment a-b (a sphere when a == b)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    a = np.asarray(a, dtype=np.float64).reshape(3)
    ab = np.asarray(b, dtype=np.float64).reshape(3) - a
    denom = float(ab @ ab)
    if denom < 1e-16:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip((pts - a) @ ab / denom, 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def object_distance(obj: HardObject, points: np.ndarray) -> np.ndarray:
    """(N,) distance to the object surface; 0 inside a box, negative inside a capsule."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if obj.kind == KIND_CAPSULE:
        return point_segment_distance(pts, obj.p0, obj.p1) - obj.radius
    local = (pts - obj.center) @ obj.axes
    outside = np.maximum(np.abs(local) - obj.half_extents, 0.0)
    return np.linalg.norm(outside, axis=1)


def _segment_samples(a: np.ndarray, b: np.ndarray, spacing: float) -> np.ndarray:
    n = max(int(math.ceil(float(np.linalg.norm(b - a)) / spacing)), 1)
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return a[None, :] + t * (b - a)[None, :]


def self_distance(points: np.ndarray, robot: SelfGeometry,
                  upper_bound: float = math.inf) -> np.ndarray:
    """(N,) distance to the robot samples / capsules; beyond upper_bound may be inf."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    d = np.full(pts.shape[0], math.inf)
    if pts.shape[0] == 0:
        return d
    tree = robot.kdtree()
    if tree is not None:
        d, _ = tree.query(pts, k=1, distance_upper_bound=upper_bound)
    for cap in robot.capsules:
        d = np.minimum(d, point_segment_distance(pts, cap.p0, cap.p1) - cap.radius)
    return d


def _target_segment(t: TargetCapsule, axial_extension: float,
                    neck_extra: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    bottom = np.asarray(t.bottom, dtype=np.float64)
    neck = np.asarray(t.neck, dtype=np.float64)
    axis = neck - bottom
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return bottom, neck
    unit = axis / norm
    return bottom - unit * axial_extension, neck + unit * (axial_extension + neck_extra)


def target_keep_mask(points: np.ndarray, targets: Sequence[TargetCapsule],
                     radial_margin: float, axial_margin: float,
                     neck_extra: float = 0.0) -> np.ndarray:
    """Return True = keep; drops points within d95/2 + radial_margin of the extended axis."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    keep = np.ones(pts.shape[0], dtype=bool)
    for t in targets:
        a, b = _target_segment(t, axial_margin, neck_extra)
        keep &= point_segment_distance(pts, a, b) > 0.5 * t.d95_m + radial_margin
    return keep


# ------------------------------------------------------------------------------ voxels
def voxelize(points: np.ndarray, voxel_size_m: float,
             min_points: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """
    Grid points into cubes of `voxel_size_m`.

    Returns (keys (V,3) int64, inverse (N,) voxel index or -1, counts (V,), n_sparse) where voxels
    with fewer than `min_points` points are dropped (their points get -1) and counted in n_sparse.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return (np.zeros((0, 3), dtype=np.int64), np.zeros(0, dtype=np.int64),
                np.zeros(0, dtype=np.int64), 0)
    idx = np.floor(pts / voxel_size_m).astype(np.int64)
    keys, inverse, counts = np.unique(idx, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    dense = counts >= int(min_points)
    remap = np.full(keys.shape[0], -1, dtype=np.int64)
    remap[dense] = np.arange(int(np.count_nonzero(dense)))
    return keys[dense], remap[inverse], counts[dense], int(np.count_nonzero(~dense))


def voxel_centers(keys: np.ndarray, voxel_size_m: float) -> np.ndarray:
    return (np.asarray(keys, dtype=np.float64) + 0.5) * voxel_size_m


def neighbour_table(keys: np.ndarray) -> np.ndarray:
    """(V, 27) index of each 26-neighbour (and self) among `keys`, -1 when unoccupied."""
    keys = np.asarray(keys, dtype=np.int64).reshape(-1, 3)
    n = keys.shape[0]
    table = np.full((n, _OFFSETS.shape[0]), -1, dtype=np.int64)
    if n == 0:
        return table
    lo = keys.min(axis=0) - 1
    span = keys.max(axis=0) - lo + 2

    def linear(k: np.ndarray) -> np.ndarray:
        s = k - lo
        return (s[:, 0] * span[1] + s[:, 1]) * span[2] + s[:, 2]

    lin = linear(keys)
    order = np.argsort(lin)
    sorted_lin = lin[order]
    for j, off in enumerate(_OFFSETS):
        q = linear(keys + off)
        pos = np.clip(np.searchsorted(sorted_lin, q), 0, n - 1)
        hit = sorted_lin[pos] == q
        table[hit, j] = order[pos[hit]]
    return table


def _neighbourhood_features(points: np.ndarray, inverse: np.ndarray, n_vox: int,
                            table: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per voxel: point count, eigenvalues (desc), principal axis of the 3x3x3 neighbourhood."""
    sel = inverse >= 0
    p = points[sel]
    v = inverse[sel]
    cnt = np.bincount(v, minlength=n_vox).astype(np.float64)
    s1 = np.stack([np.bincount(v, weights=p[:, i], minlength=n_vox) for i in range(3)], axis=1)
    s2 = np.stack([np.bincount(v, weights=p[:, i] * p[:, j], minlength=n_vox)
                   for i in range(3) for j in range(3)], axis=1)
    pad = np.where(table < 0, n_vox, table)
    cnt_n = np.append(cnt, 0.0)[pad].sum(axis=1)
    s1_n = np.vstack([s1, np.zeros((1, 3))])[pad].sum(axis=1)
    s2_n = np.vstack([s2, np.zeros((1, 9))])[pad].sum(axis=1).reshape(-1, 3, 3)
    safe = np.maximum(cnt_n, 1.0)
    mean = s1_n / safe[:, None]
    cov = s2_n / safe[:, None, None] - mean[:, :, None] * mean[:, None, :]
    evals, evecs = np.linalg.eigh(cov)
    evals = np.clip(evals[:, ::-1], 0.0, None)
    return cnt_n, evals, evecs[:, :, 2]


# ------------------------------------------------------------------------------ classification
def classify_voxels(points: np.ndarray, labels: np.ndarray, masked: np.ndarray,
                    inverse: np.ndarray, keys: np.ndarray, table: np.ndarray,
                    params: SceneParams) -> tuple[np.ndarray, np.ndarray]:
    """
    Classify voxels hard/soft. Returns (hard (V,) bool, reason (V,) str CLASS_*).

    2D votes (only points from frames that had masks count): branch fraction >= branch_vote_min
    -> hard unless the neighbourhood is measurably thin (linear, width < hard_min_thickness_m:
    a twig is L3 per §6.3 even inside the branch mask); else leaf fraction >= leaf_vote_min ->
    soft unless the neighbourhood is linear and wider than leaf_mask_max_width_m (no peach leaf
    is; guards mislabelled bark). Voxels without
    a decisive vote use geometry on the 3x3x3 neighbourhood: linear -> hard iff the across-axis
    width >= hard_min_thickness_m; non-linear -> soft only if flat (a leaf-like plane), otherwise
    hard. Flat-soft clusters larger than soft_max_extent_m are promoted back to hard (a large flat
    patch is a trunk, post or ground, not a leaf).
    """
    n_vox = keys.shape[0]
    reason = np.full(n_vox, CLASS_HARD_GEOMETRY, dtype=object)
    hard = np.ones(n_vox, dtype=bool)
    if n_vox == 0:
        return hard, reason
    cnt_n, evals, _ = _neighbourhood_features(points, inverse, n_vox, table)
    l1, l2, l3 = evals[:, 0], evals[:, 1], evals[:, 2]
    linearity = np.where(l1 > 0.0, (l1 - l2) / np.maximum(l1, 1e-30), 0.0)
    # Uniform spread of full width w has variance w^2 / 12.
    width = 2.0 * np.sqrt(3.0 * l2)
    thickness = 2.0 * np.sqrt(3.0 * l3)
    linear = linearity >= params.linearity_min
    trusted = cnt_n >= _MIN_PCA_POINTS
    thin = trusted & linear & (width < params.hard_min_thickness_m)
    flat = trusted & ~linear & (thickness <= params.flat_thickness_max_m)
    hard[thin] = False
    reason[thin] = CLASS_SOFT_THIN
    hard[flat] = False
    reason[flat] = CLASS_SOFT_FLAT

    sel = (inverse >= 0) & masked
    if np.any(sel):
        v = inverse[sel]
        lab = labels[sel]
        n_m = np.bincount(v, minlength=n_vox).astype(np.float64)
        n_b = np.bincount(v[lab == LABEL_BRANCH], minlength=n_vox).astype(np.float64)
        n_l = np.bincount(v[lab == LABEL_LEAF], minlength=n_vox).astype(np.float64)
        voted = n_m >= params.min_points_per_voxel
        frac_b = np.where(voted, n_b / np.maximum(n_m, 1.0), 0.0)
        frac_l = np.where(voted, n_l / np.maximum(n_m, 1.0), 0.0)
        branch = voted & (frac_b >= params.branch_vote_min)
        leaf = voted & ~branch & (frac_l >= params.leaf_vote_min)
        wide_linear = linear & (width >= params.leaf_mask_max_width_m)
        leaf_soft = leaf & ~wide_linear
        thick_branch = branch & ~thin
        hard[thick_branch] = True
        reason[thick_branch] = CLASS_HARD_BRANCH
        hard[leaf_soft] = False
        reason[leaf_soft] = CLASS_SOFT_LEAF
        hard[leaf & wide_linear] = True
        reason[leaf & wide_linear] = CLASS_HARD_GEOMETRY

    flat_soft = reason == CLASS_SOFT_FLAT
    if np.any(flat_soft):
        idx = np.nonzero(flat_soft)[0]
        comp = _components(idx, table)
        centers = voxel_centers(keys[idx], params.voxel_size_m)
        for c in np.unique(comp):
            members = idx[comp == c]
            if _extent(centers[comp == c], params.voxel_size_m) > params.soft_max_extent_m:
                hard[members] = True
                reason[members] = CLASS_HARD_GEOMETRY
    return hard, reason


def _extent(centers: np.ndarray, voxel: float) -> float:
    if centers.shape[0] < 2:
        return voxel
    c = centers - centers.mean(axis=0)
    _, vecs = np.linalg.eigh(c.T @ c)
    t = c @ vecs[:, 2]
    return float(t.max() - t.min()) + voxel


def _components(subset: np.ndarray, table: np.ndarray) -> np.ndarray:
    """26-connected component id for each voxel index in `subset` (edges only inside subset)."""
    n = subset.shape[0]
    if n == 0:
        return np.zeros(0, dtype=np.int64)
    local = np.full(table.shape[0] + 1, -1, dtype=np.int64)
    local[subset] = np.arange(n)
    nb = table[subset]
    nb_local = local[np.where(nb < 0, table.shape[0], nb)]
    rows = np.repeat(np.arange(n), nb.shape[1])
    cols = nb_local.reshape(-1)
    ok = cols >= 0
    graph = coo_matrix((np.ones(int(ok.sum())), (rows[ok], cols[ok])), shape=(n, n))
    _, comp = connected_components(graph, directed=False)
    return comp


# ------------------------------------------------------------------------------ fitting
def _principal_axes(centers: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean = centers.mean(axis=0)
    c = centers - mean
    _, vecs = np.linalg.eigh(c.T @ c)
    axes = vecs[:, ::-1].copy()
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] = -axes[:, 2]
    return mean, axes


def _box(centers: np.ndarray, voxel: float, axes: np.ndarray) -> HardObject:
    proj = centers @ axes
    # Projection half-width of an axis-aligned cube of side `voxel` onto each unit axis.
    pad = 0.5 * voxel * np.abs(axes).sum(axis=0)
    lo = proj.min(axis=0) - pad
    hi = proj.max(axis=0) + pad
    return HardObject(KIND_BOX, axes @ (0.5 * (lo + hi)), axes, 0.5 * (hi - lo),
                      n_voxels=int(centers.shape[0]))


def _capsule(centers: np.ndarray, voxel: float, mean: np.ndarray,
             axes: np.ndarray) -> HardObject:
    rel = centers - mean
    t = rel @ axes[:, 0]
    radial = float(np.linalg.norm(rel - t[:, None] * axes[:, 0][None, :], axis=1).max())
    r = radial + 0.5 * _SQRT3 * voxel
    t0, t1 = float(t.min()), float(t.max())
    cap_axes = axes[:, [1, 2, 0]]
    return HardObject(KIND_CAPSULE, mean + axes[:, 0] * 0.5 * (t0 + t1), cap_axes,
                      np.array([r, r, 0.5 * (t1 - t0)]), n_voxels=int(centers.shape[0]))


def enclosing_object(centers: np.ndarray, voxel: float) -> HardObject:
    """Smallest of {PCA box, axis-aligned box, PCA capsule} that covers every voxel cube."""
    centers = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    aabb = _box(centers, voxel, np.eye(3))
    if centers.shape[0] == 1:
        return aabb
    mean, axes = _principal_axes(centers)
    candidates = [aabb, _box(centers, voxel, axes), _capsule(centers, voxel, mean, axes)]
    return min(candidates, key=lambda o: o.volume())


def _split(centers: np.ndarray) -> list[np.ndarray]:
    _, axes = _principal_axes(centers)
    for k in range(3):
        t = centers @ axes[:, k]
        if float(t.max() - t.min()) > 1e-9:
            order = np.argsort(t, kind='stable')
            half = centers.shape[0] // 2
            return [centers[order[:half]], centers[order[half:]]]
    return [centers[i:i + 1] for i in range(centers.shape[0])]


class _Clearance:
    """Checks that a fitted object stays clear of the robot and every target neighbourhood."""

    def __init__(self, robot: SelfGeometry, targets: Sequence[TargetCapsule],
                 params: SceneParams) -> None:
        self._spacing = 0.25 * params.voxel_size_m
        self._self_margin = params.self_margin_m
        self._robot_pts = np.asarray(robot.points, dtype=np.float64).reshape(-1, 3)
        self._robot_tree = robot.kdtree()
        self._robot_caps = [(_segment_samples(np.asarray(c.p0, float), np.asarray(c.p1, float),
                                              self._spacing), c.radius) for c in robot.capsules]
        self._targets = []
        for t in targets:
            a, b = _target_segment(t, params.target_axial_margin_m,
                                   params.target_neck_overshoot_m)
            self._targets.append((_segment_samples(a, b, self._spacing),
                                  0.5 * t.d95_m + params.target_margin_m))

    def ok(self, obj: HardObject) -> bool:
        slack = 0.5 * self._spacing
        if obj.kind == KIND_CAPSULE:
            extent = obj.radius + float(obj.half_extents[2])
        else:
            extent = float(np.linalg.norm(obj.half_extents))
        reach = extent + self._self_margin + slack
        if self._robot_tree is not None:
            near = self._robot_tree.query_ball_point(obj.center, reach)
            if near and np.min(object_distance(obj, self._robot_pts[near])) <= self._self_margin:
                return False
        for samples, radius in self._robot_caps:
            if np.min(object_distance(obj, samples)) - radius - slack <= self._self_margin:
                return False
        for samples, radius in self._targets:
            if np.min(object_distance(obj, samples)) - slack <= radius:
                return False
        return True


def fit_cluster(centers: np.ndarray, voxel: float, min_fill_ratio: float,
                clearance: _Clearance | None = None) -> list[HardObject]:
    """
    Cover one cluster of voxel centres with few objects.

    An enclosing object is accepted when its fill ratio (occupied voxel volume / object volume)
    is at least min_fill_ratio and it clears robot and targets; otherwise the cluster is split in
    two along its principal axis. A single voxel becomes its own cube, which is valid by
    construction (the voxel-centre filters already used the half diagonal).
    """
    out: list[HardObject] = []
    stack = [np.asarray(centers, dtype=np.float64).reshape(-1, 3)]
    cube = voxel ** 3
    while stack:
        c = stack.pop()
        obj = enclosing_object(c, voxel)
        if c.shape[0] == 1:
            out.append(obj)
            continue
        fill = c.shape[0] * cube / obj.volume()
        if fill >= min_fill_ratio and (clearance is None or clearance.ok(obj)):
            out.append(obj)
        else:
            stack.extend(_split(c))
    return out


# ------------------------------------------------------------------------------ pipeline
def prepare_frame(frame: FrameInput, params: SceneParams) -> FramePoints:
    """Back-project one frame to base_link, crop to the workspace and remove robot points."""
    depth = np.asarray(frame.depth_m, dtype=np.float64)
    depth = np.where((depth >= params.min_depth_m) & (depth <= params.max_depth_m), depth, 0.0)
    shape = depth.shape[:2]
    masked = frame.branch_mask is not None or frame.leaf_mask is not None
    groups: list[tuple[np.ndarray, int]]
    if masked:
        branch = (np.zeros(shape, dtype=bool) if frame.branch_mask is None
                  else np.asarray(frame.branch_mask, dtype=bool))
        leaf = (np.zeros(shape, dtype=bool) if frame.leaf_mask is None
                else np.asarray(frame.leaf_mask, dtype=bool))
        if branch.shape != shape or leaf.shape != shape:
            raise ValueError('branch/leaf mask shape differs from depth')
        leaf = leaf & ~branch
        groups = [(branch, LABEL_BRANCH), (leaf, LABEL_LEAF), (~(branch | leaf), LABEL_NONE)]
    else:
        groups = [(np.ones(shape, dtype=bool), LABEL_NONE)]
    chunks, labs = [], []
    for mask, label in groups:
        p = mask_points(mask, depth, frame.K, confidence=frame.confidence,
                        min_conf=params.min_confidence, stride=params.pixel_stride)
        chunks.append(p)
        labs.append(np.full(p.shape[0], label, dtype=np.uint8))
    cam = np.vstack(chunks)
    labels = np.concatenate(labs)
    T = np.asarray(frame.T_base_camera, dtype=np.float64).reshape(4, 4)
    base = cam @ T[:3, :3].T + T[:3, 3]
    n_input = base.shape[0]
    inside = np.linalg.norm(base, axis=1) <= params.workspace_radius_m
    base, labels = base[inside], labels[inside]
    far = self_distance(base, frame.robot, params.self_margin_m + 1e-6) > params.self_margin_m
    return FramePoints(points=base[far], labels=labels[far], masked=masked, robot=frame.robot,
                       n_input=n_input, n_workspace_removed=int(n_input - base.shape[0]),
                       n_self_removed=int(np.count_nonzero(~far)))


def build_scene(frames: Sequence[FramePoints], targets: Sequence[TargetCapsule],
                tcp_position: np.ndarray | None, params: SceneParams) -> SceneResult:
    """
    Merge prepared frames of one batch into L2 hard objects and L3 soft voxels.

    Voxel-level self filtering and object clearance use the robot geometry of the last frame
    (the arm pose the planner will start from). tcp_position None ranks the cap by distance to
    the base origin.
    """
    v = params.voxel_size_m
    half_diag = 0.5 * _SQRT3 * v
    stats: dict[str, int] = {
        'frames': len(frames),
        'points_input': sum(f.n_input for f in frames),
        'points_workspace_removed': sum(f.n_workspace_removed for f in frames),
        'points_self_removed': sum(f.n_self_removed for f in frames),
    }
    empty = SceneResult([], np.zeros((0, 3)), False, stats)
    if not frames:
        return empty
    points = np.vstack([f.points for f in frames])
    labels = np.concatenate([f.labels for f in frames])
    masked = np.concatenate([np.full(f.points.shape[0], f.masked) for f in frames])
    robot = frames[-1].robot

    keep = target_keep_mask(points, targets, params.target_margin_m, params.target_axial_margin_m,
                            params.target_neck_overshoot_m)
    stats['points_target_removed'] = int(np.count_nonzero(~keep))
    points, labels, masked = points[keep], labels[keep], masked[keep]

    keys, inverse, _, n_sparse = voxelize(points, v, params.min_points_per_voxel)
    stats['voxels_sparse_removed'] = n_sparse
    centers = voxel_centers(keys, v)
    in_ws = np.linalg.norm(centers, axis=1) <= params.workspace_radius_m
    clear_self = self_distance(centers, robot, params.self_margin_m + half_diag + 1e-6) > \
        params.self_margin_m + half_diag
    clear_target = target_keep_mask(centers, targets, params.target_margin_m + half_diag,
                                    params.target_axial_margin_m + half_diag,
                                    params.target_neck_overshoot_m)
    stats['voxels_self_removed'] = int(np.count_nonzero(in_ws & ~clear_self))
    stats['voxels_target_removed'] = int(np.count_nonzero(in_ws & clear_self & ~clear_target))
    vox_keep = in_ws & clear_self & clear_target
    remap = np.full(keys.shape[0] + 1, -1, dtype=np.int64)
    remap[np.nonzero(vox_keep)[0]] = np.arange(int(np.count_nonzero(vox_keep)))
    inverse = remap[np.where(inverse < 0, keys.shape[0], inverse)]
    keys, centers = keys[vox_keep], centers[vox_keep]
    stats['voxels_kept'] = int(keys.shape[0])

    table = neighbour_table(keys)
    hard, reason = classify_voxels(points, labels, masked, inverse, keys, table, params)
    stats['voxels_hard'] = int(np.count_nonzero(hard))
    stats['voxels_soft'] = int(np.count_nonzero(~hard))
    for cls in (CLASS_HARD_BRANCH, CLASS_SOFT_LEAF, CLASS_SOFT_THIN, CLASS_SOFT_FLAT):
        stats[f'voxels_{cls}'] = int(np.count_nonzero(reason == cls))

    hard_idx = np.nonzero(hard)[0]
    comp = _components(hard_idx, table)
    stats['clusters'] = int(np.unique(comp).size)
    clearance = _Clearance(robot, targets, params)
    objects: list[HardObject] = []
    for c in np.unique(comp):
        objects.extend(fit_cluster(centers[hard_idx[comp == c]], v, params.fit_min_fill_ratio,
                                   clearance))
    stats['objects_fitted'] = len(objects)
    ref = (np.zeros(3) if tcp_position is None
           else np.asarray(tcp_position, dtype=np.float64).reshape(3))
    for obj in objects:
        obj.distance_to_tcp_m = float(max(object_distance(obj, ref[None, :])[0], 0.0))
    objects.sort(key=lambda o: o.distance_to_tcp_m)
    truncated = len(objects) > params.max_objects
    objects = objects[:params.max_objects]
    stats['objects_kept'] = len(objects)
    return SceneResult(objects, centers[~hard], truncated, stats)


def build_snapshot(frame: FrameInput, targets: Sequence[TargetCapsule],
                   tcp_position: np.ndarray | None, params: SceneParams) -> SceneResult:
    """Single-frame convenience: prepare_frame + build_scene."""
    return build_scene([prepare_frame(frame, params)], targets, tcp_position, params)
