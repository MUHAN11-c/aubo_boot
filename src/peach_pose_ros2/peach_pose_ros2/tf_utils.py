"""
TF/旋转工具 — Transform 消息 ↔ 齐次矩阵、旋转矩阵 ↔ 四元数、抓取几何坐标变换.

职责:
  坐标变换纯函数层（依赖 geometry_msgs / tf_transformations / numpy，不依赖
  rclpy 节点）。旋转/四元数换算一律走官方 ``tf_transformations``
  （quaternion_matrix / quaternion_from_matrix / translation_matrix）；
  历史上曾保留手写 Shepperd/展开式做双实现对照（教学用途），已按生产化
  要求移除——数值一致性由 test_tf_matrix.py 的官方路径锚点测试保障。

坐标系/单位约定:
  齐次矩阵 T 为 (4, 4)（输出系←输入系）：点 p_out = R@p_in + t（米）；
  方向向量只乘 R 不加平移，并重新归一化。重力约定：output_frame（如
  base_link）内重力向量为 [0, 0, -1]（竖直向下）。四元数消息为 (x, y, z, w)，
  与 tf_transformations 的 (x, y, z, w) 数组约定一致。
"""
from __future__ import annotations

from geometry_msgs.msg import Quaternion
import numpy as np
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


def _transform_msg_to_matrix(t) -> np.ndarray:
    """
    geometry_msgs/Transform → 4×4 齐次矩阵 T（p_out = R@p_in + t）.

    官方 tf_transformations 组合：translation_matrix @ quaternion_matrix
    （后者内部按模长归一化，非单位四元数输入也安全）。

    Args:
        t: geometry_msgs/Transform（translation + rotation 四元数）.

    Returns
    -------
        (4, 4) 齐次变换矩阵（平移单位随消息，通常为米）.

    """
    tr = t.translation
    q = t.rotation
    q_xyzw = (q.x, q.y, q.z, q.w)
    return translation_matrix((tr.x, tr.y, tr.z)) @ quaternion_matrix(q_xyzw)


def _apply_T_to_grasp3d(g3d, T: np.ndarray) -> None:
    """
    抓取几何由相机系变到输出系（默认 base_link），原地修改 g3d.

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t（含 entry_start /
    bag_bottom / bag_neck / suggested_travel_end / legacy position /
    points_centroid）；方向只乘 R 并归一化（平移不影响方向）；姿态矩阵
    左乘 R。None 字段原样保留。

    Args:
        g3d: BagGraspReference3D（相机光学系，米）；被原地改写.
        T: (4, 4) 齐次矩阵，输出系←相机系.

    Returns
    -------
        None（结果写回 g3d）.

    """
    R, t = T[:3, :3], T[:3, 3]

    def _pt(p):
        if p is None:
            return None
        return R @ np.asarray(p, dtype=float) + t

    g3d.entry_start = _pt(g3d.entry_start)
    g3d.bag_bottom = _pt(g3d.bag_bottom)
    g3d.bag_neck = _pt(g3d.bag_neck)
    # 行程终点、legacy position 与身份锚点（前景点云质心）也是点，必须同步
    # 变换（漏改会让 ~/markers 的行程箭头终点留在相机系，与输出系几何错位；
    # 质心漏改则身份锚点掉到相机系，匹配半径在世界系下失真）
    g3d.suggested_travel_end = _pt(g3d.suggested_travel_end)
    g3d.position = _pt(g3d.position)
    g3d.points_centroid = _pt(g3d.points_centroid)
    if g3d.translation_direction is not None:
        d = R @ np.asarray(g3d.translation_direction, dtype=float)
        n = float(np.linalg.norm(d))
        g3d.translation_direction = d / n if n > 1e-9 else d
    if g3d.orientation is not None:
        g3d.orientation = R @ np.asarray(g3d.orientation, dtype=float)


def _gravity_camera_from_R(R_out_cam: np.ndarray) -> np.ndarray:
    """
    由 output←camera 旋转反推相机系重力方向（gravity_mode='tf' 用）.

    约定 output_frame（如 base_link）内重力向量为 [0, 0, -1]（竖直向下）；
    方向向量只乘旋转、不加平移：g_cam = normalize(R_out_cam.T @ g_out)。

    Args:
        R_out_cam: (3, 3) 旋转矩阵，output_frame←相机系.

    Returns
    -------
        (3,) 相机系单位重力向量；退化（近零）时原样返回.

    """
    g = np.asarray(R_out_cam, dtype=float).T @ np.array([0.0, 0.0, -1.0])
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else g


def _rotation_to_quat(R: np.ndarray) -> Quaternion:
    """
    3×3 旋转矩阵 → geometry_msgs/Quaternion（官方 quaternion_from_matrix）.

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        单位四元数 Quaternion 消息.

    """
    m4 = np.eye(4, dtype=float)
    m4[:3, :3] = np.asarray(R, dtype=float)
    q = quaternion_from_matrix(m4)            # numpy [x, y, z, w]
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
