"""TF 变换双实现一致性：手写原理版 vs tf_transformations 官方 API."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose_node import (
    _quat_to_matrix_handwritten,
    _rotation_to_quat,
    _rotation_to_quat_handwritten,
    _transform_msg_to_matrix,
)


class _FakeRot:
    def __init__(self, q):
        self.x, self.y, self.z, self.w = q


class _FakeTrans:
    def __init__(self, t):
        self.x, self.y, self.z = t


class _FakeTransform:
    def __init__(self, q, t):
        self.rotation = _FakeRot(q)
        self.translation = _FakeTrans(t)


def _handwritten_full(q_xyzw, t_xyz):
    """手写版 + 平移组装成 4×4（与 _transform_msg_to_matrix 内对比路径同构）."""
    T = np.eye(4)
    T[:3, :3] = _quat_to_matrix_handwritten(q_xyzw)
    T[:3, 3] = t_xyz
    return T


class TfMatrixTest(unittest.TestCase):
    def test_handwritten_matches_api_random(self):
        """随机 200 组单位四元数+平移：双实现一致."""
        rng = np.random.default_rng(42)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            t = rng.uniform(-5, 5, size=3)
            T_api = _transform_msg_to_matrix(_FakeTransform(q, t))
            T_hw = _handwritten_full(q, t)
            self.assertLess(float(np.abs(T_api - T_hw).max()), 1e-12)

    def test_api_normalizes_non_unit_quaternion(self):
        """非单位四元数：官方版按模长归一，手写版（先归一化后）与之一致."""
        rng = np.random.default_rng(7)
        q = rng.normal(size=4)
        qn = q / np.linalg.norm(q)
        t = rng.uniform(-5, 5, size=3)
        T_api = _transform_msg_to_matrix(_FakeTransform(q * 1.7, t))
        T_hw = _handwritten_full(qn, t)
        self.assertLess(float(np.abs(T_api - T_hw).max()), 1e-12)

    def test_known_value(self):
        """已知值锚点：绕 z 转 90° + 平移 (1,2,3)."""
        q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
        t = (1.0, 2.0, 3.0)
        T = _transform_msg_to_matrix(_FakeTransform(q, t))
        expected_R = np.array([[0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(T[:3, :3], expected_R, atol=1e-12)
        np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
        np.testing.assert_allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)


def _quat_diff(q1_xyzw, q2_xyzw):
    """四元数差异度量（符号二义性：q 与 -q 同旋转）."""
    d1 = float(np.linalg.norm(np.asarray(q1_xyzw) - np.asarray(q2_xyzw)))
    d2 = float(np.linalg.norm(np.asarray(q1_xyzw) + np.asarray(q2_xyzw)))
    return min(d1, d2)


class RotationToQuatTest(unittest.TestCase):
    def test_handwritten_matches_api_random(self):
        """随机 200 组旋转矩阵：手写 Shepperd 与官方 API 一致."""
        rng = np.random.default_rng(123)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            R = _quat_to_matrix_handwritten(q)   # 由已知四元数构造合法 R
            api = _rotation_to_quat(R)
            hw = _rotation_to_quat_handwritten(R)
            api_np = np.array([api.x, api.y, api.z, api.w])
            hw_np = np.array([hw.x, hw.y, hw.z, hw.w])
            self.assertLess(_quat_diff(api_np, hw_np), 1e-12)

    def test_branch_coverage(self):
        """Shepperd 四个分支各至少触发一次（含大角度旋转）."""
        mats = [
            np.eye(3),                                   # t>0 分支
            np.array([[-1.0, 0.0, 0.0],                  # x 最大对角元分支
                      [0.0, -1.0, 0.0],
                      [0.0, 0.0, 1.0]]),
            np.array([[-1.0, 0.0, 0.0],                  # y 最大对角元分支
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, -1.0]]),
            np.array([[1.0, 0.0, 0.0],                   # z 最大对角元分支
                      [0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0]]),
        ]
        for R in mats:
            api = _rotation_to_quat(R)
            hw = _rotation_to_quat_handwritten(R)
            diff = _quat_diff(np.array([api.x, api.y, api.z, api.w]),
                              np.array([hw.x, hw.y, hw.z, hw.w]))
            self.assertLess(diff, 1e-12)


if __name__ == '__main__':
    unittest.main()
