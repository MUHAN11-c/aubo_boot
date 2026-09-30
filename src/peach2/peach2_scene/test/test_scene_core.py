import itertools

import numpy as np
from peach2_scene import scene_core as sc
import pytest
import synthetic_scene as ss

V = 0.03
HALF_DIAG = 0.5 * np.sqrt(3.0) * V
TCP = np.array([0.35, 0.0, 0.70])
TARGET = sc.TargetCapsule('target_1', ss.BAG_BOTTOM, ss.BAG_NECK, ss.BAG_D95)
ROBOT = sc.SelfGeometry(points=ss.box_surface(ss.TOOL_CENTER, ss.TOOL_SIZE))


def _cube_corners(centers: np.ndarray, voxel: float) -> np.ndarray:
    offs = np.array(list(itertools.product((-0.5, 0.5), repeat=3))) * voxel
    return (centers[:, None, :] + offs[None, :, :]).reshape(-1, 3)


def _inside_any(objects, points: np.ndarray, tol: float = 0.0) -> np.ndarray:
    inside = np.zeros(points.shape[0], dtype=bool)
    for obj in objects:
        inside |= sc.object_distance(obj, points) <= tol
    return inside


def _segment(a, b, n=60) -> np.ndarray:
    t = np.linspace(0.0, 1.0, n)[:, None]
    return np.asarray(a)[None, :] + t * (np.asarray(b) - np.asarray(a))[None, :]


def _rect_points(r: ss.Rect, n=15) -> np.ndarray:
    g = np.linspace(-1.0, 1.0, n)
    a, b = np.meshgrid(g * r.h1, g * r.h2)
    return r.center + a.reshape(-1, 1) * r.e1 + b.reshape(-1, 1) * r.e2


def _snapshot(masks: bool, targets=(TARGET,), robot=ROBOT, n_flying=40, **overrides):
    depth, branch, leaf, flying = ss.render(ss.orchard_scene(), n_flying=n_flying)
    frame = sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, robot,
                          branch if masks else None, leaf if masks else None)
    return sc.build_snapshot(frame, list(targets), TCP, ss.params(**overrides)), flying


@pytest.fixture(scope='module')
def masked():
    return _snapshot(masks=True)


@pytest.fixture(scope='module')
def geometric():
    return _snapshot(masks=False)


# ----------------------------------------------------------------------------- primitives
def test_point_segment_distance():
    pts = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    d = sc.point_segment_distance(pts, np.zeros(3), np.array([2.0, 0.0, 0.0]))
    assert np.allclose(d, [1.0, 1.0, 1.0])
    assert np.allclose(sc.point_segment_distance(pts, np.zeros(3), np.zeros(3)),
                       np.linalg.norm(pts, axis=1))


def test_voxelize_drops_sparse_voxels():
    dense = np.full((5, 3), 0.01) + np.linspace(0.0, 0.001, 5)[:, None]
    sparse = np.full((4, 3), 0.1)
    keys, inverse, counts, n_sparse = sc.voxelize(np.vstack([dense, sparse]), V, 5)
    assert keys.shape == (1, 3) and counts.tolist() == [5] and n_sparse == 1
    assert np.all(inverse[:5] == 0) and np.all(inverse[5:] == -1)
    keys, inverse, _, n_sparse = sc.voxelize(np.zeros((0, 3)), V, 5)
    assert keys.shape == (0, 3) and n_sparse == 0


def test_neighbour_table_matches_bruteforce():
    rng = np.random.default_rng(3)
    keys = np.unique(rng.integers(-4, 4, size=(120, 3)), axis=0)
    table = sc.neighbour_table(keys)
    lookup = {tuple(k): i for i, k in enumerate(keys)}
    for i, k in enumerate(keys):
        expect = sorted(lookup[tuple(k + o)] for o in itertools.product((-1, 0, 1), repeat=3)
                        if tuple(k + o) in lookup)
        assert sorted(int(x) for x in table[i] if x >= 0) == expect


@pytest.mark.parametrize('shape', ['line', 'diagonal', 'blob', 'single'])
def test_enclosing_object_covers_every_cube(shape):
    rng = np.random.default_rng(1)
    if shape == 'line':
        keys = np.column_stack([np.arange(10), np.zeros(10), np.zeros(10)])
    elif shape == 'diagonal':
        keys = np.column_stack([np.arange(8), np.arange(8), np.arange(8) // 2])
    elif shape == 'blob':
        keys = np.unique(rng.integers(0, 4, size=(30, 3)), axis=0)
    else:
        keys = np.zeros((1, 3))
    centers = sc.voxel_centers(keys, V)
    obj = sc.enclosing_object(centers, V)
    assert np.all(sc.object_distance(obj, _cube_corners(centers, V)) <= 1e-9)
    assert obj.volume() >= keys.shape[0] * V ** 3 * (1.0 - 1e-9)


def test_fit_cluster_splits_loose_l_shape():
    arm1 = np.column_stack([np.arange(12), np.zeros(12), np.zeros(12)])
    arm2 = np.column_stack([np.zeros(11), np.arange(1, 12), np.zeros(11)])
    centers = sc.voxel_centers(np.vstack([arm1, arm2]), V)
    objects = sc.fit_cluster(centers, V, 0.25)
    assert len(objects) >= 2
    assert np.all(_inside_any(objects, _cube_corners(centers, V), 1e-9))
    for obj in objects:
        assert obj.n_voxels * V ** 3 / obj.volume() >= 0.25 or obj.n_voxels == 1


def test_fit_cluster_splits_around_target_neighbourhood():
    # Inverted-V cluster whose notch holds a target: one enclosing object would swallow it.
    n = 10
    left = np.column_stack([np.arange(n), np.arange(n), np.zeros(n)])
    right = np.column_stack([np.arange(n, 2 * n), np.arange(n - 1, -1, -1), np.zeros(n)])
    centers = sc.voxel_centers(np.vstack([left, right]), V)
    notch = sc.voxel_centers(np.array([[n, 2, 0]]), V)[0]
    target = sc.TargetCapsule('t', notch - [0, 0, 0.02], notch + [0, 0, 0.02], 0.02)
    params = ss.params(target_margin_m=0.0, target_axial_margin_m=0.0)
    keep = sc.target_keep_mask(centers, [target], HALF_DIAG, HALF_DIAG)
    centers = centers[keep]
    assert len(sc.fit_cluster(centers, V, 0.05)) == 1
    clearance = sc._Clearance(sc.SelfGeometry(), [target], params)
    objects = sc.fit_cluster(centers, V, 0.05, clearance)
    assert len(objects) >= 2
    axis = _segment(target.bottom, target.neck)
    for obj in objects:
        assert np.min(sc.object_distance(obj, axis)) > 0.5 * target.d95_m
    assert np.all(_inside_any(objects, _cube_corners(centers, V), 1e-9))


# ----------------------------------------------------------------------------- orchard scene
def test_thick_branch_is_hard(masked, geometric):
    axis = _segment(ss.BRANCH.a, ss.BRANCH.b, 120)
    visible = (np.abs(axis[:, 1]) < 0.36) & (np.abs(axis[:, 1] - ss.BAG_BOTTOM[1]) > 0.15)
    for result, _ in (masked, geometric):
        assert np.all(_inside_any(result.hard_objects, axis[visible]))
    assert masked[0].stats['voxels_hard_branch'] > 0


def test_leaves_are_soft_with_masks(masked):
    result, _ = masked
    for leaf in [*ss.SQUARE_LEAVES, ss.LONG_LEAF]:
        pts = _rect_points(leaf)
        assert not np.any(_inside_any(result.hard_objects, pts))
        near = np.linalg.norm(result.soft_voxel_centers[:, None, :] - leaf.center, axis=2)
        assert np.min(near) < 0.05
    assert result.stats['voxels_soft_leaf'] > 0
    assert result.n_soft_voxels == result.stats['voxels_soft']


def test_geometric_fallback_is_conservative(geometric):
    result, _ = geometric
    square = np.vstack([_rect_points(leaf) for leaf in ss.SQUARE_LEAVES])
    # Leaf-like planes are mostly soft; truncated edge neighbourhoods may read as linear ribbons.
    assert np.mean(_inside_any(result.hard_objects, square)) < 0.4
    assert result.stats['voxels_soft_flat'] > 0
    # An elongated leaf is a 3 cm wide ribbon: geometry alone cannot tell it from a branch.
    assert np.any(_inside_any(result.hard_objects, _rect_points(ss.LONG_LEAF)))


def test_twig_is_soft(masked, geometric):
    axis = _segment(ss.TWIG.a, ss.TWIG.b, 30)
    for result, _ in (masked, geometric):
        assert not np.any(_inside_any(result.hard_objects, axis))
        assert result.stats['voxels_soft_thin'] > 0


def test_target_neighbourhood_is_hollowed(masked, geometric):
    axis = _segment(ss.BAG_BOTTOM - 0.05 * np.array([0, 0, 1]),
                    ss.BAG_NECK + 0.05 * np.array([0, 0, 1]))
    radius = 0.5 * ss.BAG_D95 + 0.05
    for result, _ in (masked, geometric):
        assert result.stats['points_target_removed'] > 0
        for obj in result.hard_objects:
            assert np.min(sc.object_distance(obj, axis)) > radius
        if result.n_soft_voxels:
            d = np.min(np.linalg.norm(result.soft_voxel_centers[:, None, :] - axis[None], axis=2),
                       axis=1)
            assert np.all(d > radius)
    control, _ = _snapshot(masks=False, targets=())
    bag_axis = _segment(ss.BAG_BOTTOM, ss.BAG_NECK, 20) - np.array([0.03, 0.0, 0.0])
    assert np.any(_inside_any(control.hard_objects, bag_axis))


def test_self_points_are_filtered(masked):
    result, _ = masked
    tool = ROBOT.points
    assert result.stats['points_self_removed'] > 0
    for obj in result.hard_objects:
        assert np.min(sc.object_distance(obj, tool)) > 0.03
    if result.n_soft_voxels:
        d = sc.self_distance(result.soft_voxel_centers, ROBOT)
        assert np.all(d > 0.03 + HALF_DIAG)
    control, _ = _snapshot(masks=True, robot=sc.SelfGeometry())
    near_tool = np.vstack([c.center for c in control.hard_objects] +
                          [control.soft_voxel_centers])
    assert np.min(sc.self_distance(near_tool, ROBOT)) < 0.03 + HALF_DIAG


def test_flying_points_do_not_become_voxels(masked):
    result, flying = masked
    assert flying.shape[0] == 40
    assert result.stats['voxels_sparse_removed'] >= 30
    assert not np.any(_inside_any(result.hard_objects, flying))
    if result.n_soft_voxels:
        d = np.min(np.linalg.norm(result.soft_voxel_centers[:, None, :] - flying[None], axis=2),
                   axis=0)
        assert np.all(d > HALF_DIAG)
    strict, _ = _snapshot(masks=True, min_points_per_voxel=1)
    assert strict.stats['voxels_sparse_removed'] == 0
    assert (strict.stats['voxels_kept'] - result.stats['voxels_kept']) >= 30


def test_cap_keeps_objects_nearest_to_tcp():
    posts = [ss.Cylinder(np.array([0.7, y, 0.62]), np.array([0.7, y, 0.95]), 0.03)
             for y in (-0.3, -0.15, 0.0, 0.15, 0.3)]
    depth, _, _, _ = ss.render(ss.Scene(posts))
    frame = sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, sc.SelfGeometry())
    tcp = np.array([0.45, 0.35, 0.8])
    full = sc.build_snapshot(frame, [], tcp, ss.params())
    assert not full.truncated and len(full.hard_objects) >= 5
    dist = [o.distance_to_tcp_m for o in full.hard_objects]
    assert dist == sorted(dist)
    capped = sc.build_snapshot(frame, [], tcp, ss.params(max_objects=2))
    assert capped.truncated and len(capped.hard_objects) == 2
    kept = [o.distance_to_tcp_m for o in capped.hard_objects]
    assert kept == pytest.approx(dist[:2])
    assert max(kept) <= min(dist[2:])
    nearest = capped.hard_objects[0]
    assert abs(nearest.center[1] - 0.3) < 0.05


def test_multi_frame_batch_and_empty():
    depth, branch, leaf, _ = ss.render(ss.orchard_scene())
    p = ss.params()
    f1 = sc.prepare_frame(sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, ROBOT, branch, leaf), p)
    f2 = sc.prepare_frame(sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, ROBOT), p)
    merged = sc.build_scene([f1, f2], [TARGET], TCP, p)
    assert merged.stats['frames'] == 2
    assert merged.stats['points_input'] == f1.n_input + f2.n_input
    assert merged.hard_objects
    empty = sc.build_scene([], [TARGET], TCP, p)
    assert empty.hard_objects == [] and empty.n_soft_voxels == 0


def test_depth_window_drops_near_and_far_pixels():
    depth, _, _, _ = ss.render(ss.orchard_scene())
    frame = sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, sc.SelfGeometry())
    full = sc.prepare_frame(frame, ss.params())
    near = sc.prepare_frame(frame, ss.params(min_depth_m=0.5))
    assert near.n_input < full.n_input
    # The tool box face at x=0.26 m is the only surface nearer than 0.5 m.
    assert np.all(full.points[:, 0] > 0.25)
    assert np.all(near.points[:, 0] > 0.5 - 1e-6)
    far = sc.prepare_frame(frame, ss.params(max_depth_m=0.7))
    assert np.all(far.points[:, 0] <= 0.7 + 1e-6)


def test_mask_shape_mismatch_rejected():
    depth = np.zeros((ss.H, ss.W))
    frame = sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, sc.SelfGeometry(),
                          branch_mask=np.zeros((10, 10), dtype=bool))
    with pytest.raises(ValueError):
        sc.prepare_frame(frame, ss.params())


def test_neck_overshoot_applies_to_points_voxels_and_fitted_objects():
    # Hanging branch 0.18 m above the neck: outside the plain hollow, inside the overshoot one.
    hang = ss.Cylinder(np.array([0.72, 0.02, 0.92]), np.array([0.72, 0.42, 0.92]), 0.02,
                       ss.LABEL_BRANCH)
    depth, _, _, _ = ss.render(ss.Scene([ss.BAG, hang]))
    frame = sc.FrameInput(depth, ss.K, ss.T_BASE_CAMERA, sc.SelfGeometry())
    axis_up = (ss.BAG_NECK - ss.BAG_BOTTOM) / np.linalg.norm(ss.BAG_NECK - ss.BAG_BOTTOM)
    hollow_r = 0.5 * ss.BAG_D95 + 0.05
    plain = sc.build_snapshot(frame, [TARGET], TCP, ss.params())
    over = sc.build_snapshot(frame, [TARGET], TCP, ss.params(target_neck_overshoot_m=0.09))
    above_neck = _segment(ss.BAG_NECK + 0.10 * axis_up, ss.BAG_NECK + 0.18 * axis_up, 10)
    assert np.any(_inside_any(plain.hard_objects, above_neck, tol=hollow_r))
    assert over.stats['points_target_removed'] > plain.stats['points_target_removed']
    assert over.stats['voxels_kept'] < plain.stats['voxels_kept']
    extended = _segment(ss.BAG_BOTTOM - 0.05 * axis_up, ss.BAG_NECK + 0.14 * axis_up, 80)
    for obj in over.hard_objects:
        assert np.min(sc.object_distance(obj, extended)) > hollow_r - 1e-6
    assert over.hard_objects, 'the branch away from the bag must stay hard'


def test_neck_overshoot_hollows_only_past_the_neck():
    bottom = np.array([0.5, 0.0, 0.5])
    neck = np.array([0.5, 0.0, 0.6])
    target = sc.TargetCapsule('t', bottom, neck, 0.06)
    # hollow radius = d95/2 + margin = 0.08, so 0.10 past the extended end is kept
    above_neck = np.array([[0.5, 0.0, 0.6 + 0.05 + 0.10]])
    below_bottom = np.array([[0.5, 0.0, 0.5 - 0.05 - 0.10]])
    pts = np.vstack([above_neck, below_bottom])
    plain = sc.target_keep_mask(pts, [target], 0.05, 0.05)
    over = sc.target_keep_mask(pts, [target], 0.05, 0.05, neck_extra=0.09)
    assert plain.tolist() == [True, True]
    assert over.tolist() == [False, True]
