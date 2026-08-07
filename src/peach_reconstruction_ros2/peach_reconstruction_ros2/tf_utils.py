"""
TF 消息与 4×4 齐次矩阵互转 — 本包独立副本（实现参考 peach_pose_node）.

约定：T_base_camera 为 base←camera 的 4×4 齐次矩阵，p_base = T @ p_camera。
仅依赖 numpy / tf_transformations，不依赖 rclpy，便于离线单测。
"""
from __future__ import annotations

import numpy as np
from tf_transformations import quaternion_matrix


def transform_msg_to_matrix(transform) -> np.ndarray:
    """
    geometry_msgs/Transform（或同构鸭子类型）→ 4×4 齐次矩阵.

    四元数经 tf_transformations.quaternion_matrix（内部自动归一化）。
    鸭子类型：只需 .translation.x/y/z 与 .rotation.x/y/z/w 属性。

    Args:
        transform: geometry_msgs/Transform 或 TransformStamped.transform.

    Returns
    -------
        (4, 4) float64 齐次矩阵（平移单位随消息，通常 [m]）.

    """
    tr = transform.translation
    q = transform.rotation
    T = quaternion_matrix((q.x, q.y, q.z, q.w))
    T[0, 3], T[1, 3], T[2, 3] = float(tr.x), float(tr.y), float(tr.z)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    4×4 齐次矩阵求逆：T_camera_base = inv(T_base_camera).

    刚体变换逆 = [R.T, -R.T @ t]，比 np.linalg.inv 数值更稳、语义更明确。

    Args:
        T: (4, 4) 齐次矩阵.

    Returns
    -------
        (4, 4) float64 逆矩阵.

    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def relative_motion(T_a: np.ndarray, T_b: np.ndarray) -> tuple:
    """
    两个 base←camera 位姿间的相对运动量（视角过滤用）.

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
