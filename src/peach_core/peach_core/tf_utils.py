"""
坐标变换纯函数层 — Transform 消息 ↔ 齐次矩阵、旋转 ↔ 四元数、点/方向变换.

职责:
  全链路坐标变换的**单份事实源**（统一原 peach_pose_ros2/tf_utils.py 与
  peach_reconstruction_ros2/tf_utils.py 两份重复实现，重构阶段 A1）。
  仅依赖 numpy 与纯数学库 tf_transformations（transformations.py 移植，
  无 rclpy/消息依赖，沿用重建包纯核 guard 的放行结论）；不 import
  geometry_msgs —— 输入走鸭子类型，输出用 :class:`QuaternionValue` 值对象。

输入/输出契约:
  - 齐次矩阵 T 为 (4, 4)（输出系←输入系）：点 p_out = R@p_in + t（米）；
    方向向量只乘 R 不加平移，并重新归一化；
  - Transform 输入为鸭子类型：只需 .translation.x/y/z 与
    .rotation.x/y/z/w 属性（geometry_msgs/Transform、
    TransformStamped.transform 或测试假对象均可）；
  - 四元数约定 (x, y, z, w)，与 tf_transformations 数组约定一致；
  - 重力约定：output_frame（如 base_link）内重力向量为 [0, 0, -1]
    （竖直向下）。

协议条款:
  纯核零 ROS import（test_pure_core.py AST 强制）；tf_transformations
  属纯数学库，明确放行。

线程模型:
  全部纯函数 + 不可变值对象，无共享状态，任意线程安全。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


@dataclass(frozen=True)
class QuaternionValue:
    """
    单位四元数不可变值对象（geometry_msgs/Quaternion 的纯核替代）.

    纯核不得 import geometry_msgs，故 rotation_to_quat 返回本值对象；
    字段语义与消息一致 (x, y, z, w)。编排层需要消息时自行构造：
    ``Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)``。
    """

    x: float
    y: float
    z: float
    w: float

    def as_tuple(self) -> tuple:
        """返回 (x, y, z, w) 元组，供 tf_transformations 等数组接口使用."""
        return (self.x, self.y, self.z, self.w)


def transform_msg_to_matrix(transform) -> np.ndarray:
    """
    Transform（鸭子类型）→ 4×4 齐次矩阵 T（p_out = R@p_in + t）.

    官方 tf_transformations 组合：translation_matrix @ quaternion_matrix
    （后者内部按模长归一化，非单位四元数输入也安全）。统一自原
    peach_pose_ros2.tf_utils._transform_msg_to_matrix 与
    peach_reconstruction_ros2.tf_utils.transform_msg_to_matrix（两者
    数值等价：quaternion_matrix 不写平移列）。

    Args:
        transform: 带 .translation.x/y/z 与 .rotation.x/y/z/w 的对象.

    Returns
    -------
        (4, 4) float64 齐次矩阵（平移单位随消息，通常为米）.

    """
    tr = transform.translation
    q = transform.rotation
    q_xyzw = (q.x, q.y, q.z, q.w)
    return translation_matrix((tr.x, tr.y, tr.z)) @ quaternion_matrix(q_xyzw)


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    4×4 齐次矩阵求逆：T_camera_base = inv(T_base_camera).

    官方 np.linalg.inv（通用 4×4 求逆）：刚体矩阵上数值误差 ~1e-16，
    与手写 [R.T, -R.T@t] 在测试锚点精度（atol=1e-12）内无差别；
    输入的刚性由 test_tf_utils 正逆互反用例守门，无需自造刚体特化。
    （实现沿用原 peach_reconstruction_ros2.tf_utils.invert_transform。）

    Args:
        T: (4, 4) 齐次矩阵.

    Returns
    -------
        (4, 4) float64 逆矩阵.

    """
    return np.linalg.inv(np.asarray(T, dtype=np.float64))


def relative_motion(T_a: np.ndarray, T_b: np.ndarray) -> tuple:
    """
    两个 base←camera 位姿间的相对运动量（视角过滤用）.

    保留 numpy 闭式（官方无等价物）：tf_transformations 没有「两旋转
    夹角」直出 API，须绕 quaternion_from_matrix → 2·arccos(|w|) 取角，
    反而多一次四元数往返；trace 闭式 R_rel→arccos((tr−1)/2) 是教科书
    标准式，单次矩阵乘即得。concatenate_matrices 仅为矩阵乘语法糖，
    无语义收益，不用。
    （实现沿用原 peach_reconstruction_ros2.tf_utils.relative_motion。）

    Args:
        T_a: (4, 4) 本帧位姿.
        T_b: (4, 4) 参考帧位姿（上一已采帧）.

    Returns
    -------
        (translation_m, rotation_deg)：平移差范数 [m] 与相对旋转角 [deg].

    """
    R_rel = T_a[:3, :3] @ T_b[:3, :3].T
    cos_angle = float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
    rotation_deg = float(np.degrees(np.arccos(cos_angle)))
    translation_m = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    return translation_m, rotation_deg


def rotation_to_quat(R: np.ndarray) -> QuaternionValue:
    """
    3×3 旋转矩阵 → 单位四元数值对象（官方 quaternion_from_matrix）.

    Args:
        R: (3, 3) 旋转矩阵（非正交输入的行为随官方实现，调用方保证刚性）.

    Returns
    -------
        QuaternionValue（x, y, z, w），模长为 1.

    """
    m4 = np.eye(4, dtype=float)
    m4[:3, :3] = np.asarray(R, dtype=float)
    q = quaternion_from_matrix(m4)            # numpy [x, y, z, w]
    return QuaternionValue(
        x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def transform_point(T: np.ndarray, point) -> np.ndarray:
    """
    点按齐次矩阵变换：p_out = R@p_in + t（None 透传）.

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        point: (3,) 点坐标（米），None 原样返回.

    Returns
    -------
        (3,) float64 输出系点坐标；输入 None 时返回 None.

    """
    if point is None:
        return None
    return T[:3, :3] @ np.asarray(point, dtype=float) + T[:3, 3]


def transform_direction(T: np.ndarray, direction) -> np.ndarray:
    """
    方向向量按齐次矩阵变换：只乘 R 不加平移，并重新归一化（None 透传）.

    平移不影响方向；近零退化向量不归一化（防除零），原样返回旋转结果。

    Args:
        T: (4, 4) 齐次矩阵，输出系←输入系.
        direction: (3,) 方向向量，None 原样返回.

    Returns
    -------
        (3,) float64 输出系单位方向向量；输入 None 时返回 None.

    """
    if direction is None:
        return None
    d = T[:3, :3] @ np.asarray(direction, dtype=float)
    n = float(np.linalg.norm(d))
    return d / n if n > 1e-9 else d


def gravity_camera_from_R(R_out_cam: np.ndarray) -> np.ndarray:
    """
    由 output←camera 旋转反推相机系重力方向（gravity_mode='tf' 用）.

    约定 output_frame（如 base_link）内重力向量为 [0, 0, -1]（竖直向下）；
    方向向量只乘旋转、不加平移：g_cam = normalize(R_out_cam.T @ g_out)。
    （实现沿用原 peach_pose_ros2.tf_utils._gravity_camera_from_R。）

    Args:
        R_out_cam: (3, 3) 旋转矩阵，output_frame←相机系.

    Returns
    -------
        (3,) 相机系单位重力向量；退化（近零）时原样返回.

    """
    g = np.asarray(R_out_cam, dtype=float).T @ np.array([0.0, 0.0, -1.0])
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else g
