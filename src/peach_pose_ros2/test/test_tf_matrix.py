"""
TF 变换官方路径锚点测试：tf_transformations 已知值 + 往返一致性.

历史：本文件原为「手写原理版 vs 官方 API」双实现对照；手写实现已按生产化
要求移除（tf_utils.py 模块 docstring 有记录），现改为锚定官方路径的
已知旋转值、字段接线（x,y,z,w 顺序）与 R→q→R 往返一致性。
"""
import unittest

import numpy as np

from peach_pose_ros2.tf_utils import (
    _rotation_to_quat,
    _transform_msg_to_matrix,
)
from tf_transformations import quaternion_matrix


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


class TransformMsgToMatrixTest(unittest.TestCase):
    def test_known_value_z90(self):
        """已知值锚点：绕 z 转 90° + 平移 (1,2,3)，含 (x,y,z,w) 接线校验."""
        q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
        t = (1.0, 2.0, 3.0)
        T = _transform_msg_to_matrix(_FakeTransform(q, t))
        expected_R = np.array([[0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(T[:3, :3], expected_R, atol=1e-12)
        np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
        np.testing.assert_allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_identity(self):
        """零旋转 + 零平移 → 单位阵."""
        T = _transform_msg_to_matrix(_FakeTransform((0, 0, 0, 1), (0, 0, 0)))
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_api_normalizes_non_unit_quaternion(self):
        """非单位四元数：官方 quaternion_matrix 按模长归一，结果与归一化输入一致."""
        rng = np.random.default_rng(7)
        q = rng.normal(size=4)
        qn = q / np.linalg.norm(q)
        t = rng.uniform(-5, 5, size=3)
        T_scaled = _transform_msg_to_matrix(_FakeTransform(q * 1.7, t))
        np.testing.assert_allclose(
            T_scaled[:3, :3], quaternion_matrix(qn)[:3, :3], atol=1e-12)

    def test_random_transform_matches_official_composition(self):
        """随机 200 组：旋转块正交、平移透传、点映射语义 p_out = R@p + t."""
        rng = np.random.default_rng(42)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            t = rng.uniform(-5, 5, size=3)
            T = _transform_msg_to_matrix(_FakeTransform(q, t))
            R = T[:3, :3]
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
            np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
            p = rng.uniform(-1, 1, size=3)
            np.testing.assert_allclose(
                T @ np.append(p, 1.0), np.append(R @ p + t, 1.0), atol=1e-12)


class RotationToQuatTest(unittest.TestCase):
    def test_identity(self):
        """单位旋转 → (0, 0, 0, 1)."""
        q = _rotation_to_quat(np.eye(3))
        self.assertAlmostEqual(q.x, 0.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)
        self.assertAlmostEqual(q.z, 0.0, places=12)
        self.assertAlmostEqual(q.w, 1.0, places=12)

    def test_known_z90(self):
        """绕 z 转 90° → (0, 0, ±√2/2, √2/2)（±q 等价）."""
        R = np.array([[0.0, -1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0]])
        q = _rotation_to_quat(R)
        s = np.sqrt(0.5)
        diff = min(abs(q.z - s), abs(q.z + s))
        self.assertLess(diff, 1e-12)
        self.assertAlmostEqual(abs(q.w), s, places=12)
        self.assertAlmostEqual(q.x, 0.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)

    def test_known_x180(self):
        """绕 x 转 180° → (±1, 0, 0, 0)（±q 等价）."""
        q = _rotation_to_quat(np.diag([1.0, -1.0, -1.0]))
        self.assertAlmostEqual(abs(q.x), 1.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)
        self.assertAlmostEqual(q.z, 0.0, places=12)
        self.assertAlmostEqual(q.w, 0.0, places=12)

    def test_roundtrip_random(self):
        """随机 200 组合法旋转：R → Quaternion → quaternion_matrix 还原 R."""
        rng = np.random.default_rng(123)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            R = quaternion_matrix(q)[:3, :3]   # 官方生成合法旋转
            out = _rotation_to_quat(R)
            R_rt = quaternion_matrix((out.x, out.y, out.z, out.w))[:3, :3]
            np.testing.assert_allclose(R_rt, R, atol=1e-12)


if __name__ == '__main__':
    unittest.main()
