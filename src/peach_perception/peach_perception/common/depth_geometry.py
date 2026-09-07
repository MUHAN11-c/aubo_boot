from __future__ import annotations
"""几何原语：拟合、深度单位、TF、点云 RGB 打包。"""

from typing import Optional

import numpy as np


# ═══════════════════════════════════════════════════════════════
# 向量/轴线原语（各子模块共用）
# ═══════════════════════════════════════════════════════════════

# 四元数模长平方回退阈值：与原 tf_transformations.quaternion_matrix 的
# _EPS（numpy.finfo(float64).eps）一致；低于此值回退单位旋转。
_QUAT_NORM_EPS = float(np.finfo(np.float64).eps)


def unit_vector(vector) -> Optional[np.ndarray]:
    """
    有限非零向量 → 单位向量；None/非三维/含非有限/近零 → None.

    近零判废阈值为 norm < 1e-9（收敛自各处 ``_unit`` 的多数口径；
    refine 侧原 ``<= 1.0e-9`` 为浮点测度零的等值边界差，随统一收敛，
    见 refine.py 对应辅助函数的边界注）。

    Args:
        vector: 长度 3 序列；None 原样返回 None.

    Returns
    -------
        (3,) float64 单位向量，或 None（不可用输入）.

    """
    if vector is None:
        return None
    value = np.asarray(vector, dtype=np.float64).reshape(-1)
    if value.size != 3 or not np.all(np.isfinite(value)):
        return None
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        return None
    return value / norm


def angle_between_deg(first, second) -> Optional[float]:
    """
    两向量夹角 [deg]（arccos 点积，不取绝对值）；退化输入 → None.

    Args:
        first / second: 长度 3 向量（内部各自归一化）.

    Returns
    -------
        夹角 [deg] ∈ [0, 180]；任一向量 unit_vector 判不可用时 None.

    """
    a = unit_vector(first)
    b = unit_vector(second)
    if a is None or b is None:
        return None
    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def axis_radial_distance(points: np.ndarray, axis: np.ndarray,
                         origin) -> np.ndarray:
    """
    点集到轴线（过 origin、方向 axis）的垂直距离.

    理论: delta = p − origin 的垂直分量模 ‖delta − (delta·a)a‖；origin
    只需在轴线上（取截面质心或锚点均可，浮点末位级差异）。

    Args:
        points: (N, 3) 点.
        axis: (3,) 单位轴向.
        origin: (3,) 轴上一点（如截面质心/锚点）.

    Returns
    -------
        (N,) float64 距离（与输入同单位）.

    """
    delta = np.asarray(points, dtype=np.float64) - np.asarray(
        origin, dtype=np.float64)
    return np.linalg.norm(delta - np.outer(delta @ axis, axis), axis=1)


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
