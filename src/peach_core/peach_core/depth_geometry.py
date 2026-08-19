"""
深度图归一化 — 管线统一的 uint16 毫米输入约定.

职责:
  深度图归一化的单份事实源（自 peach_pose_ros2.peach_pose.depth_geometry
  迁移，行为不变，重构阶段 A1）。

单位约定:
  - 算法管线内部全部按 uint16「毫米」工作，需要米处 /1000 转米；
  - 传感器输入分两路：uint16 原始值（``raw × depth_scale_unit`` = 毫米，
    Percipio DepthScaleUnit 常见 0.25）与 32FC1 浮点深度（单位 [m]，
    ×1000 转毫米，此时 depth_scale_unit 不生效）。

协议条款:
  纯核零 ROS import（test_pure_core.py AST 强制），仅依赖 numpy。

线程模型:
  纯函数，无共享状态，任意线程安全。
"""
from __future__ import annotations

import numpy as np

_UINT16_MAX_MM = 65535.0  # uint16 深度上限（毫米）；饱和值下游视为无效


def normalize_depth_to_uint16_mm(depth: np.ndarray,
                                 depth_scale_unit: float) -> np.ndarray:
    """
    深度图归一化为 uint16 毫米（管线统一输入约定）.

    uint16 路径：``raw × depth_scale_unit`` = 毫米（Percipio 常见 0.25；
    数据集回放的真毫米深度设 1.0），round + clip 到 [0, 65535]；
    scale==1.0 时原样返回（零拷贝）。

    浮点路径（32FC1 等）：输入视为「米」，×1000 转毫米，depth_scale_unit
    不生效；NaN/±Inf/负值一律置 0（无效深度），round + clip 到 uint16。

    Args:
        depth: (H, W) 深度图，uint16（原始值）或浮点（米）.
        depth_scale_unit: uint16 路径的比例因子（毫米/单位）；浮点路径忽略.

    Returns
    -------
        (H, W) uint16 深度，单位毫米.

    Raises
    ------
        ValueError: 不支持的 dtype（既非 uint16 也非浮点）.

    """
    if depth.dtype == np.uint16:
        if abs(depth_scale_unit - 1.0) > 1e-9:
            return np.clip(
                np.round(depth.astype(np.float32) * depth_scale_unit),
                0.0, _UINT16_MAX_MM).astype(np.uint16)
        return depth
    if np.issubdtype(depth.dtype, np.floating):
        mm = depth.astype(np.float64) * 1000.0
        mm = np.where(np.isfinite(mm) & (mm > 0.0), mm, 0.0)
        return np.clip(np.round(mm), 0.0, _UINT16_MAX_MM).astype(np.uint16)
    raise ValueError(f'不支持的深度 dtype {depth.dtype}（仅支持 uint16/浮点）')
