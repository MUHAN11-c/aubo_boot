"""
tf_utils 锚点测试：官方路径已知值 + 往返一致性 + 点/方向/重力变换.

合并自 peach_pose_ros2/test/test_tf_matrix.py 与
peach_reconstruction_ros2/test/test_tf_utils.py（两份重复实现的统一
守门），并新增 transform_point / transform_direction /
gravity_camera_from_R / QuaternionValue 用例。
"""
import unittest

import numpy as np

from peach_core.tf_utils import (
    gravity_camera_from_R,
    invert_transform,
    QuaternionValue,
    relative_motion,
    rotation_to_quat,
    transform_direction,
    transform_msg_to_matrix,
    transform_point,
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
        T = transform_msg_to_matrix(_FakeTransform(q, t))
        expected_R = np.array([[0.0, -1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(T[:3, :3], expected_R, atol=1e-12)
        np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
        np.testing.assert_allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_identity(self):
        """零旋转 + 零平移 → 单位阵."""
        T = transform_msg_to_matrix(_FakeTransform((0, 0, 0, 1), (0, 0, 0)))
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_api_normalizes_non_unit_quaternion(self):
        """非单位四元数：官方 quaternion_matrix 按模长归一，结果与归一化输入一致."""
        rng = np.random.default_rng(7)
        q = rng.normal(size=4)
        qn = q / np.linalg.norm(q)
        t = rng.uniform(-5, 5, size=3)
        T_scaled = transform_msg_to_matrix(_FakeTransform(q * 1.7, t))
        np.testing.assert_allclose(
            T_scaled[:3, :3], quaternion_matrix(qn)[:3, :3], atol=1e-12)

    def test_random_transform_matches_official_composition(self):
        """随机 200 组：旋转块正交、平移透传、点映射语义 p_out = R@p + t."""
        rng = np.random.default_rng(42)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            t = rng.uniform(-5, 5, size=3)
            T = transform_msg_to_matrix(_FakeTransform(q, t))
            R = T[:3, :3]
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
            np.testing.assert_allclose(T[:3, 3], t, atol=1e-12)
            p = rng.uniform(-1, 1, size=3)
            np.testing.assert_allclose(
                T @ np.append(p, 1.0), np.append(R @ p + t, 1.0), atol=1e-12)

    def test_inverse_is_rigid_inverse(self):
        """随机 50 组位姿：inv(T) @ T = I（正逆互反）."""
        rng = np.random.default_rng(7)
        for _ in range(50):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            t = rng.uniform(-2.0, 2.0, size=3)
            T = transform_msg_to_matrix(_FakeTransform(tuple(q), tuple(t)))
            np.testing.assert_allclose(
                invert_transform(T) @ T, np.eye(4), atol=1e-12)


class RelativeMotionTest(unittest.TestCase):
    def test_translation_and_rotation(self):
        """平移 30 mm + 绕 z 90°：相对量与解析值一致."""
        T_a = np.eye(4)
        T_b = np.eye(4)
        T_b[:3, :3] = np.array([[0.0, -1.0, 0.0],
                                [1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0]])
        T_b[:3, 3] = [0.03, 0.0, 0.0]
        trans, rot = relative_motion(T_b, T_a)
        self.assertAlmostEqual(trans, 0.03, places=12)
        self.assertAlmostEqual(rot, 90.0, places=9)

    def test_identity_is_zero(self):
        """同位姿相对量为零（重复视角的极端情形）."""
        trans, rot = relative_motion(np.eye(4), np.eye(4))
        self.assertAlmostEqual(trans, 0.0, places=12)
        self.assertAlmostEqual(rot, 0.0, places=12)


class RotationToQuatTest(unittest.TestCase):
    def test_identity(self):
        """单位旋转 → (0, 0, 0, 1)."""
        q = rotation_to_quat(np.eye(3))
        self.assertAlmostEqual(q.x, 0.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)
        self.assertAlmostEqual(q.z, 0.0, places=12)
        self.assertAlmostEqual(q.w, 1.0, places=12)

    def test_known_z90(self):
        """绕 z 转 90° → (0, 0, ±√2/2, √2/2)（±q 等价）."""
        R = np.array([[0.0, -1.0, 0.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0]])
        q = rotation_to_quat(R)
        s = np.sqrt(0.5)
        diff = min(abs(q.z - s), abs(q.z + s))
        self.assertLess(diff, 1e-12)
        self.assertAlmostEqual(abs(q.w), s, places=12)
        self.assertAlmostEqual(q.x, 0.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)

    def test_known_x180(self):
        """绕 x 转 180° → (±1, 0, 0, 0)（±q 等价）."""
        q = rotation_to_quat(np.diag([1.0, -1.0, -1.0]))
        self.assertAlmostEqual(abs(q.x), 1.0, places=12)
        self.assertAlmostEqual(q.y, 0.0, places=12)
        self.assertAlmostEqual(q.z, 0.0, places=12)
        self.assertAlmostEqual(q.w, 0.0, places=12)

    def test_roundtrip_random(self):
        """随机 200 组合法旋转：R → QuaternionValue → quaternion_matrix 还原 R."""
        rng = np.random.default_rng(123)
        for _ in range(200):
            q = rng.normal(size=4)
            q /= np.linalg.norm(q)
            R = quaternion_matrix(q)[:3, :3]   # 官方生成合法旋转
            out = rotation_to_quat(R)
            R_rt = quaternion_matrix(out.as_tuple())[:3, :3]
            np.testing.assert_allclose(R_rt, R, atol=1e-12)

    def test_quaternion_value_is_immutable(self):
        """四元数值对象冻结不可变（值对象语义）."""
        q = QuaternionValue(x=0.0, y=0.0, z=0.0, w=1.0)
        with self.assertRaises(AttributeError):
            q.x = 1.0


class TransformPointDirectionTest(unittest.TestCase):
    def test_point_applies_rotation_and_translation(self):
        """点：p_out = R@p + t."""
        T = np.eye(4)
        T[:3, :3] = np.array([[0.0, -1.0, 0.0],
                              [1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0]])
        T[:3, 3] = [1.0, 2.0, 3.0]
        np.testing.assert_allclose(
            transform_point(T, [1.0, 0.0, 0.0]), [1.0, 3.0, 3.0], atol=1e-12)

    def test_direction_ignores_translation_and_renormalizes(self):
        """方向：只乘 R 不加平移，归一化为单位向量."""
        T = np.eye(4)
        T[:3, :3] = np.diag([2.0, 1.0, 1.0])   # 缩放验证重新归一化
        T[:3, 3] = [10.0, 0.0, 0.0]
        np.testing.assert_allclose(
            transform_direction(T, [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0],
            atol=1e-12)

    def test_none_passthrough(self):
        """None 输入原样透传（缺失字段语义）."""
        self.assertIsNone(transform_point(np.eye(4), None))
        self.assertIsNone(transform_direction(np.eye(4), None))


class GravityCameraTest(unittest.TestCase):
    def test_identity_rotation_gravity_points_down(self):
        """相机系=output 系时重力即 [0, 0, -1]（竖直向下）."""
        np.testing.assert_allclose(
            gravity_camera_from_R(np.eye(3)), [0.0, 0.0, -1.0], atol=1e-12)

    def test_rotated_gravity_stays_unit(self):
        """随机旋转下重力向量仍为单位向量且方向正确."""
        rng = np.random.default_rng(5)
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = quaternion_matrix(q)[:3, :3]
        g = gravity_camera_from_R(R)
        self.assertAlmostEqual(float(np.linalg.norm(g)), 1.0, places=12)
        np.testing.assert_allclose(R @ g, [0.0, 0.0, -1.0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
