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

    官方 np.linalg.inv（通用 4×4 求逆）：刚体矩阵上数值误差 ~1e-16，
    与手写 [R.T, -R.T@t] 在测试锚点精度（atol=1e-12）内无差别；
    输入的刚性由 test_tf_utils 正逆互反用例守门，无需自造刚体特化。

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
