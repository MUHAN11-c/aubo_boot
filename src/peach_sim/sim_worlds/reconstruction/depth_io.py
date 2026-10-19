"""合成深度导出：米制浮点深度 → uint16 毫米 PNG（无效像素 0）.

口径与 PeachDataSet / `output/reference_depth_mm.png` 一致（Azure Kinect
毫米惯例）。本轮不加噪声模型——评测报告须注明深度为理想渲染值。
遮挡序判断直接用 Depth pass（射线 Z）做相对比较，与光轴深度单调一致。
"""

import numpy as np


def camera_look_axis(matrix_world):
    """World unit vector along the optical axis. Blender cameras look -Z."""
    return (-matrix_world[0][2], -matrix_world[1][2], -matrix_world[2][2])


def optical_depth_m(position_xyz, location, look_axis):
    """Per-pixel camera-forward depth in metres (Kinect optical Z)."""
    loc = np.asarray(location, dtype=np.float64).reshape(1, 1, 3)
    axis = np.asarray(look_axis, dtype=np.float64).reshape(1, 1, 3)
    delta = position_xyz.astype(np.float64) - loc
    return np.sum(delta * axis, axis=-1)


def to_uint16_mm(depth_m: np.ndarray, cap_m: float = 65.0) -> np.ndarray:
    """米 → uint16 毫米；非有限、<=0、超 cap 一律置 0（无效）."""
    if depth_m.dtype not in (np.float32, np.float64):
        depth_m = depth_m.astype(np.float64)
    valid = np.isfinite(depth_m) & (depth_m > 0) & (depth_m < cap_m)
    out = np.zeros(depth_m.shape, dtype=np.uint16)
    out[valid] = np.round(depth_m[valid] * 1000.0).astype(np.uint16)
    return out
