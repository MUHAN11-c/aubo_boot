"""预抓取残差：工具轴/套筒口/刀片面相对袋模型（纯核）."""
from __future__ import annotations

from typing import Optional

import numpy as np


def _unit(vector) -> Optional[np.ndarray]:
    """有限非零向量 → 单位向量."""
    if vector is None:
        return None
    value = np.asarray(vector, dtype=np.float64).reshape(-1)
    if value.size != 3 or not np.all(np.isfinite(value)):
        return None
    norm = float(np.linalg.norm(value))
    if norm < 1e-9:
        return None
    return value / norm


def axis_angle_deg(tool_axis, bag_axis) -> float:
    """两轴夹角（度）."""
    first = _unit(tool_axis)
    second = _unit(bag_axis)
    if first is None or second is None:
        return 180.0
    cosine = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def lateral_axial_errors(
        tool_point, model_point, bag_axis) -> tuple:
    """工具点相对模型点的侧向/轴向残差 [m]."""
    axis = _unit(bag_axis)
    if axis is None or tool_point is None or model_point is None:
        return 1.0, 1.0
    delta = np.asarray(tool_point, dtype=np.float64) - np.asarray(
        model_point, dtype=np.float64)
    axial = float(np.dot(delta, axis))
    lateral = float(np.linalg.norm(delta - axial * axis))
    return lateral, axial


def evaluate_pregrasp(
        tool_axis, bag_axis, sleeve_mouth, bag_bottom, cutting_plane,
        cut_plane_point, radial_margin_m: float, axial_margin_m: float,
        previous=None, max_angle_deg: float = 2.0,
        max_lateral_m: float = 0.003) -> dict:
    """
    停稳后的预抓取定量残差（工具变换相对冻结模型）.

    径向/轴向预算是接触许可，不参与本残差是否通过。
    previous 为上一帧同结构 dict 时检查两帧一致性。
    """
    angle = axis_angle_deg(tool_axis, bag_axis)
    lat_b, _ = lateral_axial_errors(sleeve_mouth, bag_bottom, bag_axis)
    lat_c, ax_c = lateral_axial_errors(
        cutting_plane, cut_plane_point, bag_axis)
    lateral = max(lat_b, lat_c)
    consistent = True
    if previous is not None:
        consistent = (
            abs(angle - float(previous.get('axis_angle_deg', angle))) < 1.5
            and abs(lateral - float(previous.get('lateral_error_m', lateral)))
            < 0.004)
    passed = (
        consistent
        and angle <= max_angle_deg
        and lateral <= max_lateral_m)
    needs = (not passed) and consistent
    reason = 'pregrasp_verified'
    failure_code = 0
    if not consistent:
        reason = 'pregrasp_frames_inconsistent'
        failure_code = 4
    elif not passed:
        reason = 'pregrasp_residual'
        failure_code = 4
    return {
        'axis_angle_deg': float(angle),
        'lateral_error_m': float(lateral),
        'axial_error_m': float(ax_c),
        'radial_margin_m': float(radial_margin_m),
        'axial_margin_m': float(axial_margin_m),
        'frames_consistent': bool(consistent),
        'needs_correction': bool(needs),
        'passed': bool(passed),
        'reason': reason,
        'failure_code': int(failure_code),
    }
