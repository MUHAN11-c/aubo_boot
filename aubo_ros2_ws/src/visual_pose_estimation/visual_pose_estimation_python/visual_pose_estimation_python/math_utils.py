"""
数学工具模块

统一的旋转矩阵、四元数、欧拉角转换函数。
消除 ros2_communication / pose_estimator / template_standardizer 中的重复实现。
"""

from __future__ import annotations

import numpy as np
from typing import List, Tuple


def quaternion_to_rotation_matrix(q: List[float]) -> np.ndarray:
    """四元数 [x, y, z, w] → 3x3 旋转矩阵

    Args:
        q: 四元数 [x, y, z, w]

    Returns:
        3x3 旋转矩阵
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → 四元数 [x, y, z, w]

    Args:
        R: 3x3 旋转矩阵

    Returns:
        四元数 [x, y, z, w]
    """
    trace = float(np.trace(R))

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return np.array([x, y, z, w], dtype=np.float64)


def rotation_matrix_to_euler_rpy(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → 欧拉角 RPY (ZYX 顺序)

    带有万向节锁奇点检测。

    Args:
        R: 3x3 旋转矩阵

    Returns:
        欧拉角 [roll, pitch, yaw] (弧度)
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    return np.array([roll, pitch, yaw], dtype=np.float64)


def normalize_angle_to_180(angle_deg: float) -> float:
    """将角度归一化到 [-180, 180] 区间

    Args:
        angle_deg: 角度（度）

    Returns:
        归一化后的角度（度）
    """
    a = float(angle_deg)
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def normalize_angle_to_pi(angle_rad: float) -> float:
    """将角度归一化到 [-π, π] 区间

    Args:
        angle_rad: 角度（弧度）

    Returns:
        归一化后的角度（弧度）
    """
    a = float(angle_rad)
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def filter_components_by_params(
    components: List[np.ndarray],
    min_area: float,
    max_area: float,
    min_aspect: float,
    max_aspect: float,
    min_width: float,
    min_height: float,
    max_count: int = 0,
) -> List[np.ndarray]:
    """根据面积、宽高比等条件筛选连通域（Preprocessor / FeatureExtractor 共用）

    Args:
        components: 连通域掩码列表
        min_area: 最小面积（像素²）
        max_area: 最大面积（像素²）
        min_aspect: 最小长宽比
        max_aspect: 最大长宽比
        min_width: 最小宽度（像素）
        min_height: 最小高度（像素）
        max_count: 最大数量（0 = 不限制）

    Returns:
        筛选后的连通域列表（按面积降序）
    """
    import cv2

    candidates = []
    for mask in components:
        area = float(cv2.countNonZero(mask))
        if area < min_area or area > max_area:
            continue

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            continue

        x, y, w, h = cv2.boundingRect(contours[0])
        if w < min_width or h < min_height:
            continue

        aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0.0
        if aspect < min_aspect or aspect > max_aspect:
            continue

        candidates.append((area, mask))

    candidates.sort(key=lambda x: x[0], reverse=True)
    if max_count > 0 and len(candidates) > max_count:
        candidates = candidates[:max_count]

    return [m for _, m in candidates]
