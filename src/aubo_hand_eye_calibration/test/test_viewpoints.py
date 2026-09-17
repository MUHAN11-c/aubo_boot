"""Zero-ROS tests for camera-agnostic viewpoint generation."""

from aubo_hand_eye_calibration.board import Checkerboard
from aubo_hand_eye_calibration.transforms import inverse, make_transform
from aubo_hand_eye_calibration.viewpoints import (
    board_frame,
    board_outline,
    candidate_viewpoints,
    distances_for_fill,
    FovMargin,
    look_at_transform,
    select_diverse,
)
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

BOARD = Checkerboard(columns=11, rows=8, square_size_m=0.020)
K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)
SIZE = (640, 480)


def random_transform(rng):
    return make_transform(
        Rotation.random(random_state=int(rng.integers(0, 2**30))).as_matrix(),
        rng.uniform(-0.2, 0.2, 3))


def scene():
    """板固定在 base 前方 0.5m, 法向朝 -x; X0 为典型眼在手上外参."""
    base_from_target = make_transform(
        Rotation.from_euler('y', 90.0, degrees=True).as_matrix(),
        [0.50, 0.0, 0.10])
    wrist_from_camera = make_transform(
        Rotation.from_quat([0.5, 0.5, 0.5, -0.5]).as_matrix(),
        [0.045, 0.108, 0.002])
    return base_from_target, wrist_from_camera


def test_look_at_builds_valid_optical_pose():
    eye = np.array([0.1, -0.2, 0.3])
    target = np.array([0.5, 0.0, 0.1])
    pose = look_at_transform(eye, target, np.array([0.0, 0.0, -1.0]))
    rotation = pose[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert abs(np.linalg.det(rotation) - 1.0) < 1e-12
    # z 轴指向 target
    assert np.allclose(
        rotation[:, 2], (target - eye) / np.linalg.norm(target - eye),
        atol=1e-12)
    assert np.allclose(pose[:3, 3], eye)


def test_look_at_rejects_degenerate_inputs():
    with pytest.raises(ValueError, match='重合'):
        look_at_transform([0, 0, 0], [0, 0, 0], [0, 0, -1])
    with pytest.raises(ValueError, match='平行'):
        look_at_transform([0, 0, 0], [1, 0, 0], [1, 0, 0])


def test_fov_margin_accepts_interior_rejects_boundary():
    margin = FovMargin(K, D, SIZE, board_outline(BOARD), 0.08)
    # 板心对到光轴, 0.4m 处投影宽 ≈ 600*0.26/0.4 = 390px, 在余量内
    # (board_outline 在 target 系偏心, 平移把板心 [0.11,0.07] 对到原点)
    inside = make_transform(np.eye(3), [-0.11, -0.07, 0.40])
    assert margin(inside)
    assert margin.margin_px(inside) > 0.0
    # 板贴近到投影超出画面
    outside = make_transform(np.eye(3), [-0.11, -0.07, 0.10])
    assert not margin(outside)
    # 板在相机后方 (z 为负)
    behind = make_transform(np.eye(3), [-0.11, -0.07, -0.40])
    assert margin.margin_px(behind) == float('-inf')


def test_distances_for_fill_are_camera_agnostic():
    distances = distances_for_fill(K, BOARD, SIZE, (0.4, 0.7))
    # fx 600 / 宽 640 / 板物理宽 (10+2)*0.02=0.24m:
    # fill 0.4 -> 600*0.24/(0.4*640) = 0.5625; fill 0.7 -> 0.3214
    assert distances[0] == pytest.approx(0.5625, abs=1e-3)
    assert distances[1] == pytest.approx(0.3214, abs=1e-3)
    # 焦距减半 -> 距离减半 (占比不变, 适配任意相机)
    k_half = K.copy()
    k_half[0, 0] = 300.0
    assert distances_for_fill(
        k_half, BOARD, SIZE, (0.4,))[0] == pytest.approx(
        distances[0] / 2, abs=1e-4)


def test_candidate_viewpoints_pass_fov_and_map_to_wrist():
    base_from_target, wrist_from_camera = scene()
    candidates = candidate_viewpoints(
        BOARD, base_from_target, wrist_from_camera, K, D, SIZE)
    assert len(candidates) >= 12
    margin = FovMargin(K, D, SIZE, board_outline(BOARD), 0.08)
    for candidate in candidates:
        # 腕部位姿经 X0 映射回相机位姿, 链路闭合
        base_from_camera = (
            candidate.base_from_wrist @ wrist_from_camera)
        assert np.allclose(
            base_from_camera, candidate.base_from_camera_optical, atol=1e-9)
        camera_from_target = inverse(base_from_camera) @ base_from_target
        # 逐候选复核 FOV 掩码 (视点位静态保证)
        assert margin.margin_px(camera_from_target) >= 0.0
        assert candidate.distance_m > 0.0
        assert 0.0 < candidate.fill_fraction < 1.0


def test_candidates_shrink_with_larger_margin():
    base_from_target, wrist_from_camera = scene()
    loose = candidate_viewpoints(
        BOARD, base_from_target, wrist_from_camera, K, D, SIZE,
        margin_fraction=0.02)
    tight = candidate_viewpoints(
        BOARD, base_from_target, wrist_from_camera, K, D, SIZE,
        margin_fraction=0.25)
    assert len(tight) < len(loose)


def test_select_diverse_meets_span_and_is_deterministic():
    base_from_target, wrist_from_camera = scene()
    candidates = candidate_viewpoints(
        BOARD, base_from_target, wrist_from_camera, K, D, SIZE,
        polar_degrees=(0.0, 30.0, 45.0))
    first, span_first = select_diverse(candidates, 8, 30.0)
    second, span_second = select_diverse(candidates, 8, 30.0)
    assert span_first >= 30.0
    assert span_first == span_second
    assert [c.metadata() for c in first] == [c.metadata() for c in second]
    assert len(first) == 8


def test_board_frame_reports_center_and_normal():
    base_from_target, _ = scene()
    center, normal, axis_x, axis_y = board_frame(BOARD, base_from_target)
    # R_y(90) @ [0.10, 0.07, 0] + t: 板心 [0.5, 0.07, 0.0], 法向 = R 第三列
    assert np.allclose(center, [0.50, 0.07, 0.00], atol=1e-9)
    assert np.allclose(normal, [1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(np.cross(axis_x, axis_y), normal, atol=1e-9)
    outline = board_outline(BOARD)
    assert outline.shape == (4, 3)
    assert outline[:, 2].max() == 0.0
