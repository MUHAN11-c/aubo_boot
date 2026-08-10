"""抓取几何 TF 变换契约：点/方向/行程终点变换规则 + 重力 tf 模式纯函数."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.contracts import BagGraspReference3D
from peach_pose_ros2.tf_utils import (
    _apply_T_to_grasp3d,
    _gravity_camera_from_R,
)


def _known_T():
    """已知变换：绕 z 转 90° + 非零平移 (1, 2, 3)."""
    T = np.eye(4)
    T[:3, :3] = np.array([[0.0, -1.0, 0.0],
                          [1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0]])
    T[:3, 3] = [1.0, 2.0, 3.0]
    return T


class ApplyTToGrasp3dTest(unittest.TestCase):
    def test_points_translated_directions_not(self):
        """点按 R@p+t 变换；方向只乘 R、不受平移影响."""
        T = _known_T()
        R, t = T[:3, :3], T[:3, 3]
        g = BagGraspReference3D(
            entry_start=np.array([0.1, 0.2, 0.3]),
            bag_bottom=np.array([0.0, 0.0, 0.5]),
            bag_neck=np.array([0.1, 0.0, 0.7]),
            points_centroid=np.array([0.05, 0.0, 0.6]),
            translation_direction=np.array([1.0, 0.0, 0.0]),
        )
        _apply_T_to_grasp3d(g, T)
        np.testing.assert_allclose(
            g.entry_start, R @ np.array([0.1, 0.2, 0.3]) + t, atol=1e-12)
        np.testing.assert_allclose(
            g.bag_bottom, R @ np.array([0.0, 0.0, 0.5]) + t, atol=1e-12)
        np.testing.assert_allclose(
            g.bag_neck, R @ np.array([0.1, 0.0, 0.7]) + t, atol=1e-12)
        # 身份锚点（前景点云质心）同为点，必须按 R@p+t 变换
        np.testing.assert_allclose(
            g.points_centroid, R @ np.array([0.05, 0.0, 0.6]) + t, atol=1e-12)
        # 绕 z 转 90°：+X → +Y；平移 (1,2,3) 不得混入方向
        np.testing.assert_allclose(
            g.translation_direction, [0.0, 1.0, 0.0], atol=1e-12)

    def test_travel_end_and_position_transformed(self):
        """
        suggested_travel_end 与 legacy position 同为点，必须按 R@p+t 变换.

        回归锚点：旧版漏变换 suggested_travel_end，导致 ~/markers 行程箭头
        终点留在相机系。
        """
        T = _known_T()
        R, t = T[:3, :3], T[:3, 3]
        g = BagGraspReference3D(
            entry_start=np.array([0.0, 0.0, 0.4]),
            suggested_travel_end=np.array([0.0, 0.0, 0.6]),
            position=np.array([0.1, 0.0, 0.4]),
        )
        _apply_T_to_grasp3d(g, T)
        np.testing.assert_allclose(
            g.suggested_travel_end, R @ np.array([0.0, 0.0, 0.6]) + t, atol=1e-12)
        np.testing.assert_allclose(
            g.position, R @ np.array([0.1, 0.0, 0.4]) + t, atol=1e-12)

    def test_orientation_left_multiplied_and_none_kept(self):
        """姿态矩阵左乘 R；None 字段原样保留（不抛异常）."""
        T = _known_T()
        R = T[:3, :3]
        R0 = np.eye(3)
        g = BagGraspReference3D(orientation=R0.copy())
        _apply_T_to_grasp3d(g, T)
        np.testing.assert_allclose(g.orientation, R @ R0, atol=1e-12)
        self.assertIsNone(g.entry_start)
        self.assertIsNone(g.suggested_travel_end)
        self.assertIsNone(g.position)
        self.assertIsNone(g.translation_direction)


class GravityCameraFromRTest(unittest.TestCase):
    def test_identity_rotation_gives_down(self):
        """单位旋转：相机系重力 = output 系约定 [0, 0, -1]."""
        np.testing.assert_allclose(
            _gravity_camera_from_R(np.eye(3)), [0.0, 0.0, -1.0], atol=1e-12)

    def test_pure_rotation_rotates_vector_only(self):
        """纯旋转输入：重力向量只随之旋转（绕 x 转 90° → [0,-1,0]），无平移项."""
        R = np.array([[1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 1.0, 0.0]])
        np.testing.assert_allclose(
            _gravity_camera_from_R(R), [0.0, -1.0, 0.0], atol=1e-12)

    def test_scaled_rotation_still_unit(self):
        """带缩放的旋转输入：输出仍为单位向量（normalize 生效）."""
        R = np.array([[1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 1.0, 0.0]])
        np.testing.assert_allclose(
            _gravity_camera_from_R(2.0 * R), [0.0, -1.0, 0.0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
