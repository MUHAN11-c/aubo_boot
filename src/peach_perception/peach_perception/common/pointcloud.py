"""
点云原语：RGB 位打包与刚体变换（纯核，零 ROS import）.

pack_rgb_bgr 无官方等价物（sensor_msgs_py 不提供 RViz float32 位打包，
integrate.py 历版注释已考证），此为唯一保留的手写位运算。
"""
from __future__ import annotations

import numpy as np


def pack_rgb_bgr(colors_bgr: np.ndarray) -> np.ndarray:
    """
    (N, 3) uint8 BGR → (N,) float32 位打包（0xRRGGBB，RViz RGB8 约定）.

    统一自原 integrate.pack_rgb_bgr 与 visualization._pack_rgb_bgr（两者
    逐位等价，后者无空输入短路）。

    Args:
        colors_bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB）；空输入给 (0,) 空数组.

    """
    colors = np.asarray(colors_bgr, dtype=np.uint8).reshape(-1, 3)
    if colors.shape[0] == 0:
        return np.zeros((0,), dtype=np.float32)
    b = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    r = colors[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """
    对 (N, 3) 点应用齐次刚体变换 p_out = R@p_in + t，不修改输入.

    统一自原 integrate.transform_points（transform_camera_points 为其
    未被调用的重复实现，已删除）。

    Args:
        points: (N, 3) 点.
        transform: (4, 4) 齐次矩阵，输出系←输入系.

    Returns
    -------
        (N, 3) float64 变换后点；空输入给 (0, 3) 空数组.

    """
    xyz = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    T = np.asarray(transform, dtype=np.float64)
    return xyz @ T[:3, :3].T + T[:3, 3]
