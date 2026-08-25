"""Registry 记忆锚点 → 袋抓取几何（纯核，零 ROS）."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .contracts import compute_entry_start
from .pipeline import grasp_frame_from_axis


def first_point(*candidates) -> Optional[np.ndarray]:
    """返回第一个非 None 的 3D 点."""
    for point in candidates:
        if point is not None:
            return np.asarray(point, dtype=np.float64)
    return None


@dataclass(frozen=True)
class MemoryGrasp:
    """身份表记忆还原的袋底/颈/轴/入口."""

    bottom: np.ndarray
    neck: np.ndarray
    axis: np.ndarray
    entry_start: np.ndarray
    rotation: np.ndarray


def memory_grasp(entry: dict, standoff: float) -> Optional[MemoryGrasp]:
    """从 TargetRegistry 条目还原可规划锚点；缺位置则 None."""
    if not entry or entry.get('position') is None:
        return None
    center = np.asarray(entry['position'], dtype=np.float64)
    axis = entry.get('axis')
    axis = (np.array([0.0, 0.0, 1.0]) if axis is None
            else np.asarray(axis, dtype=np.float64))
    half = 0.5 * float(entry.get('diameter') or 0.06)
    rotation = grasp_frame_from_axis(axis)
    zg = rotation[:, 2]
    bottom = center - zg * half
    neck = center + zg * half
    return MemoryGrasp(
        bottom=bottom,
        neck=neck,
        axis=zg,
        entry_start=compute_entry_start(bottom, zg, standoff),
        rotation=rotation,
    )
