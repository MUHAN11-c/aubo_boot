"""
抓取几何坐标变换 — 感知专属的 TF 残留层（基于 peach_core.tf_utils 原语）.

职责:
  通用坐标变换原语（Transform 消息 ↔ 齐次矩阵、点/方向变换、旋转 ↔
  四元数值对象、重力反推）已统一迁移到 ``peach_core.tf_utils``（重构阶段
  A3，消除与重建包的重复实现）；本模块只保留感知专属的两件事：
    1. ``_apply_T_to_grasp3d``：BagGraspReference3D 抓取几何整体的
       相机系 → 输出系变换（字段级规则为感知数据合约专属，不入纯核）；
    2. ``_rotation_to_quat``：peach_core ``rotation_to_quat`` 返回的
       QuaternionValue → geometry_msgs/Quaternion 消息的薄包装
       （纯核不 import geometry_msgs，消息构造属编排层职责）。

坐标系/单位约定:
  与 peach_core.tf_utils 一致：T 为 (4, 4)（输出系←输入系），点
  p_out = R@p_in + t（米）；方向只乘 R 并归一化；四元数 (x, y, z, w)。

线程模型:
  纯函数，无共享状态，任意线程安全。
"""
from __future__ import annotations

from geometry_msgs.msg import Quaternion
import numpy as np
from peach_core.tf_utils import (
    rotation_to_quat,
    transform_direction,
    transform_point,
)


def _apply_T_to_grasp3d(g3d, T: np.ndarray) -> None:
    """
    抓取几何由相机系变到输出系（默认 base_link），原地修改 g3d.

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t（含 entry_start /
    bag_bottom / bag_neck / suggested_travel_end / legacy position /
    points_centroid，走 peach_core transform_point）；方向只乘 R 并归一化
    （transform_direction：平移不影响方向）；姿态矩阵左乘 R。None 字段
    原样保留。

    Args:
        g3d: BagGraspReference3D（相机光学系，米）；被原地改写.
        T: (4, 4) 齐次矩阵，输出系←相机系.

    Returns
    -------
        None（结果写回 g3d）.

    """
    # 行程终点、legacy position 与身份锚点（前景点云质心）也是点，必须同步
    # 变换（漏改会让 markers 的行程箭头终点留在相机系，与输出系几何错位；
    # 质心漏改则身份锚点掉到相机系，匹配半径在世界系下失真）
    g3d.entry_start = transform_point(T, g3d.entry_start)
    g3d.bag_bottom = transform_point(T, g3d.bag_bottom)
    g3d.bag_neck = transform_point(T, g3d.bag_neck)
    g3d.suggested_travel_end = transform_point(T, g3d.suggested_travel_end)
    g3d.position = transform_point(T, g3d.position)
    g3d.points_centroid = transform_point(T, g3d.points_centroid)
    g3d.translation_direction = transform_direction(
        T, g3d.translation_direction)
    if g3d.orientation is not None:
        g3d.orientation = (
            T[:3, :3] @ np.asarray(g3d.orientation, dtype=float))


def _rotation_to_quat(R: np.ndarray) -> Quaternion:
    """
    3×3 旋转矩阵 → geometry_msgs/Quaternion（peach_core 值对象的消息包装）.

    数值路径与重构前完全一致：官方 quaternion_from_matrix（见
    peach_core.tf_utils.rotation_to_quat），此处仅把 QuaternionValue
    组装成消息（纯核不 import geometry_msgs）。

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        单位四元数 Quaternion 消息（x, y, z, w）.

    """
    q = rotation_to_quat(R)
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)
