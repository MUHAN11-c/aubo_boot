"""
Zero-ROS synthetic tests for joint intrinsics + hand-eye solving.

观测场景由 viewpoints 模块自己生成 (环绕固定棋盘格的 FOV 合格视点),
两个纯核模块在测试内闭环: 位姿多样性真实, 内参可观测度接近现场。
"""

from aubo_hand_eye_calibration.board import Checkerboard
from aubo_hand_eye_calibration.joint_calib import (
    JointObservation,
    solve_joint_intrinsics_hand_eye,
)
from aubo_hand_eye_calibration.transforms import (
    inverse,
    make_transform,
    transform_error,
)
from aubo_hand_eye_calibration.viewpoints import (
    candidate_viewpoints,
    select_diverse,
)
import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

BOARD = Checkerboard(columns=11, rows=8, square_size_m=0.020)
SIZE = (640, 480)
K_TRUE = np.array([[520.0, 0.0, 318.0], [0.0, 512.0, 245.0], [0.0, 0.0, 1.0]])
D_TRUE = np.array([-0.25, 0.08, 0.0006, -0.0003, 0.0])


def project(object_points, camera_from_target, k, d):
    rotation = Rotation.from_matrix(camera_from_target[:3, :3]).as_rotvec()
    projected, _ = cv2.projectPoints(
        object_points.astype(np.float32), rotation,
        camera_from_target[:3, 3], k, d)
    return projected.reshape(-1, 2)


def synthetic_joint_observations(view_count=15, noise_px=0.1, seed=11):
    """真值 X/B + viewpoints 环绕视点 -> (观测, X, B)."""
    rng = np.random.default_rng(seed)
    x_true = make_transform(
        Rotation.from_quat([0.5, 0.5, 0.5, -0.5]).as_matrix(),
        [0.045, 0.108, 0.002])
    b_true = make_transform(
        Rotation.from_euler('y', 90.0, degrees=True).as_matrix(),
        [0.55, 0.02, 0.15])
    viewpoints, _ = select_diverse(
        candidate_viewpoints(
            BOARD, b_true, x_true, K_TRUE, D_TRUE, SIZE,
            polar_degrees=(0.0, 20.0, 40.0)),
        view_count, 30.0)
    observations = []
    for index, viewpoint in enumerate(viewpoints[:view_count]):
        camera_from_target = (
            inverse(viewpoint.base_from_camera_optical) @ b_true)
        corners = project(
            BOARD.object_points, camera_from_target, K_TRUE, D_TRUE)
        corners = corners + rng.normal(0.0, noise_px, corners.shape)
        observations.append(JointObservation(
            viewpoint.base_from_wrist, corners, f'view_{index:02d}'))
    return observations, x_true, b_true


def test_joint_recovery_from_synthetic_views():
    observations, x_true, b_true = synthetic_joint_observations()
    result = solve_joint_intrinsics_hand_eye(
        observations, BOARD, SIZE, K0=None, D0=None)
    k = result.camera_matrix
    assert abs(k[0, 0] - K_TRUE[0, 0]) / K_TRUE[0, 0] < 0.005
    assert abs(k[1, 1] - K_TRUE[1, 1]) / K_TRUE[1, 1] < 0.005
    assert abs(k[0, 2] - K_TRUE[0, 2]) < 2.0
    assert abs(k[1, 2] - K_TRUE[1, 2]) < 2.0
    assert abs(result.distortion[0] - D_TRUE[0]) < 0.03
    translation_m, rotation_deg = transform_error(
        x_true, result.gripper_from_camera)
    assert translation_m < 1e-3, translation_m
    assert rotation_deg < 0.5, rotation_deg
    assert result.passed, result.failures
    assert result.reprojection_rms_px < 0.30
    assert result.polish['converged']


def test_joint_recovers_when_seed_intrinsics_are_wrong():
    observations, x_true, _ = synthetic_joint_observations(seed=23)
    k0 = K_TRUE.copy()
    k0[0, 0] *= 1.05
    k0[1, 1] *= 0.95
    result = solve_joint_intrinsics_hand_eye(
        observations, BOARD, SIZE, K0=k0, D0=np.zeros(5))
    k = result.camera_matrix
    assert abs(k[0, 0] - K_TRUE[0, 0]) / K_TRUE[0, 0] < 0.005
    translation_m, _ = transform_error(
        x_true, result.gripper_from_camera)
    assert translation_m < 1e-3
    assert result.passed, result.failures


def test_joint_flags_low_rotation_span():
    # 所有腕部姿态几乎同姿态 (仅平移变化) => 旋转跨度不足触发质量门
    rng = np.random.default_rng(5)
    x_true = make_transform(
        Rotation.from_quat([0.5, 0.5, 0.5, -0.5]).as_matrix(),
        [0.045, 0.108, 0.002])
    b_true = make_transform(
        Rotation.from_euler('y', 90.0, degrees=True).as_matrix(),
        [0.55, 0.02, 0.15])
    flattened = []
    for index in range(12):
        base_from_gripper = make_transform(
            Rotation.from_rotvec(rng.normal(0.0, 0.002, 3)).as_matrix(),
            [0.30 + 0.01 * index, 0.05, 0.35])
        camera_from_target = (
            inverse(x_true) @ inverse(base_from_gripper) @ b_true)
        corners = project(
            BOARD.object_points, camera_from_target, K_TRUE, D_TRUE)
        flattened.append(JointObservation(
            base_from_gripper, corners, f'flat_{index:02d}'))
    result = solve_joint_intrinsics_hand_eye(
        flattened, BOARD, SIZE, min_rotation_span_deg=20.0)
    assert not result.passed
    assert any('rotation span' in failure for failure in result.failures)


def test_joint_exposes_corrupted_view():
    observations, x_true, _ = synthetic_joint_observations(seed=13)
    # 视点 3 的角点整体错位 6px
    observations[3] = JointObservation(
        observations[3].base_from_gripper,
        observations[3].corners + np.array([6.0, -4.0]),
        observations[3].sample_id)
    result = solve_joint_intrinsics_hand_eye(
        observations, BOARD, SIZE, joint_max_reprojection_rms_px=1.0)
    per_view = {view['sample_id']: view for view in result.per_view}
    assert not per_view['view_03']['accepted']
    good = [view['rms_px'] for view in result.per_view
            if view['accepted']]
    assert max(good) < 0.5
    # 坏视点拉高总 RMS, 门应不通过或明显暴露
    assert (not result.passed) or result.reprojection_rms_px > 0.3


def test_joint_requires_minimum_views():
    observations, _, _ = synthetic_joint_observations(view_count=4)
    with pytest.raises(ValueError, match='at least 6 views'):
        solve_joint_intrinsics_hand_eye(observations, BOARD, SIZE)
