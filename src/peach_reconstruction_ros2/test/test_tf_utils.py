"""tf_utils：Transform→4×4 已知值锚点 + 正逆互反 + 相对运动量."""
import unittest

import numpy as np

from peach_reconstruction_ros2.tf_utils import (
    invert_transform,
    relative_motion,
    transform_msg_to_matrix,
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


class TransformMsgToMatrixTest(unittest.TestCase):
    def test_known_value(self):
        """绕 z 转 90° + 平移 (1,2,3) 的解析锚点."""
        q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
        T = transform_msg_to_matrix(_FakeTransform(q, (1.0, 2.0, 3.0)))
        np.testing.assert_allclose(T[:3, :3], [[0.0, -1.0, 0.0],
                                               [1.0, 0.0, 0.0],
                                               [0.0, 0.0, 1.0]], atol=1e-12)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)
        np.testing.assert_allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

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


if __name__ == '__main__':
    unittest.main()
