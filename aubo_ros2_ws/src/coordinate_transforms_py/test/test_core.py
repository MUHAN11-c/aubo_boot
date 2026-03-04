"""单元测试：coordinate_transforms_py.core 接口（非 ROS2）。"""
import numpy as np
import sys
import os
# 保证能 import 到包（开发时未 install 时）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from coordinate_transforms_py.core import (
    CameraIntrinsics,
    project_3d_to_2d,
    project_3d_to_2d_batch,
    unproject_2d_to_3d,
    unproject_2d_to_3d_batch,
    reprojection_error,
    reprojection_error_batch,
    reprojection_error_rms,
    rpy_to_quaternion,
    quaternion_to_rpy,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    transform_point,
    transform_direction,
    transform_points,
    transform_pose,
    transform_directions,
    transform_line,
    handedness_transform,
    right_to_left,
    left_to_right,
)
from coordinate_transforms_py.core.point_transform import Handedness


def test_project_unproject_roundtrip():
    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    p3d = np.array([0.1, 0.0, 1.0])
    u, v = project_3d_to_2d(p3d, K)
    p3d_back = unproject_2d_to_3d(u, v, 1.0, K)
    np.testing.assert_allclose(p3d, p3d_back, rtol=1e-5)


def test_reprojection_error():
    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    p3d = np.array([0.1, 0.0, 1.0])
    uv_proj = project_3d_to_2d(p3d, K)
    err = reprojection_error(uv_proj, p3d, K)
    assert abs(err) < 1e-6
    uv_obs = uv_proj + np.array([2.0, 0.0])
    err2 = reprojection_error(uv_obs, p3d, K)
    assert abs(err2 - 2.0) < 1e-5


def test_batch_project_unproject():
    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    P = np.array([[0.1, 0, 1], [0, 0.1, 1], [-0.05, -0.05, 0.8]])
    uv = project_3d_to_2d_batch(P, K)
    assert uv.shape == (3, 2)
    depths = P[:, 2]
    P_back = unproject_2d_to_3d_batch(uv, depths, K)
    np.testing.assert_allclose(P, P_back, rtol=1e-5)


def test_rotation_rpy_quat_matrix():
    rpy = np.array([0.1, 0.2, 0.3])
    q = rpy_to_quaternion(*rpy)
    rpy_back = quaternion_to_rpy(q)
    np.testing.assert_allclose(rpy, rpy_back, rtol=1e-5)
    R = quaternion_to_rotation_matrix(q)
    q_from_R = rotation_matrix_to_quaternion(R)
    np.testing.assert_allclose(np.abs(q), np.abs(q_from_R), rtol=1e-5)


def test_transform_point():
    T = np.eye(4)
    T[0:3, 3] = [1, 2, 3]
    p = np.array([0.0, 0.0, 0.0])
    p2 = transform_point(p, T)
    np.testing.assert_allclose(p2, [1, 2, 3])


def test_transform_direction():
    R = np.eye(3)
    v = np.array([1.0, 0.0, 0.0])
    v2 = transform_direction(v, R)
    np.testing.assert_allclose(v2, v)


def test_handedness():
    R_right = right_to_left()
    assert R_right.shape == (3, 3)
    np.testing.assert_allclose(R_right @ np.array([1, 0, 0]), [1, 0, 0])
    np.testing.assert_allclose(R_right @ np.array([0, 0, 1]), [0, 0, -1])
    M = handedness_transform(Handedness.Right, Handedness.Left)
    np.testing.assert_allclose(M, R_right)


def test_transform_points():
    T = np.eye(4)
    T[0:3, 3] = [1, 0, 0]
    P = np.array([[0, 0, 0], [1, 0, 0]])
    P2 = transform_points(P, T)
    np.testing.assert_allclose(P2[0], [1, 0, 0])
    np.testing.assert_allclose(P2[1], [2, 0, 0])


def test_transform_pose():
    origin = np.array([0.0, 0.0, 0.0])
    R = np.eye(3)
    T_B_A = np.eye(4)
    T_B_A[0:3, 3] = [5, 0, 0]
    new_origin, new_R = transform_pose(origin, R, T_B_A)
    np.testing.assert_allclose(new_origin, [5, 0, 0])
    np.testing.assert_allclose(new_R, R)


def test_transform_line():
    point = np.array([0.0, 0.0, 0.0])
    direction = np.array([1.0, 0.0, 0.0])
    T = np.eye(4)
    T[0:3, 3] = [1, 2, 3]
    p2, d2 = transform_line(point, direction, T)
    np.testing.assert_allclose(p2, [1, 2, 3])
    np.testing.assert_allclose(d2, direction)


def test_reprojection_error_rms():
    K = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    uv_obs = np.array([[320, 240], [321, 240]])
    P = np.array([[0, 0, 1], [0.002, 0, 1]])
    uv_proj = project_3d_to_2d_batch(P, K)
    rms = reprojection_error_rms(uv_obs, P, K)
    assert rms >= 0 and np.isfinite(rms)


if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])
