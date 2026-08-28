"""空心圆柱工具的逐目标动态径向/轴向预算（纯核，无 ROS）."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ToolBudgetParams:
    """与 hollow_cylinder_v1 对齐的误差与净空项，单位米或无量纲."""

    d_inner: float = 0.104
    wall_clearance: float = 0.002
    l_insert: float = 0.200
    tool_runout95: float = 0.001
    tcp_calibration_error95: float = 0.002
    hand_eye_error95: float = 0.003
    bag_deformation_margin95: float = 0.002
    blade_capture_half_width: float = 0.008
    axial_safety_margin: float = 0.004
    blade_plane_calibration_error95: float = 0.002
    robot_axial_error95: float = 0.002
    target_motion95: float = 0.003
    fruit_safety_clearance: float = 0.012
    diagnostic_axis_deg: float = 35.0


def radial_available(d_inner: float, d_bag95: float,
                     wall_clearance: float) -> float:
    """工具内壁相对袋 D95 的可用径向净空 [m]."""
    return 0.5 * float(d_inner) - 0.5 * float(d_bag95) - float(wall_clearance)


def radial_error95(center_lateral95: float, axis_error_deg: float,
                   length_m: float, params: ToolBudgetParams) -> float:
    """套入区间上的 95% 径向误差合成 [m]."""
    axis_rad = math.radians(max(0.0, float(axis_error_deg)))
    return (
        float(center_lateral95)
        + float(length_m) * math.sin(axis_rad)
        + params.tool_runout95
        + params.tcp_calibration_error95
        + params.hand_eye_error95
        + params.bag_deformation_margin95
    )


def axial_error95(neck_position95: float, params: ToolBudgetParams) -> float:
    """刀片面相对袋颈的 95% 轴向误差合成 [m]."""
    return (
        float(neck_position95)
        + params.blade_plane_calibration_error95
        + params.robot_axial_error95
        + params.target_motion95
    )


def evaluate_sleeve_cut(
        d_bag95: float, length_m: float, center_lateral95: float,
        axis_error_deg: float, neck_position95: float,
        cut_to_fruit_m: float,
        params: ToolBudgetParams | None = None) -> dict:
    """
    逐目标套入/剪切许可.

    固定 35° 只作完全错轴诊断，不授权接触。
    """
    cfg = params or ToolBudgetParams()
    available = radial_available(cfg.d_inner, d_bag95, cfg.wall_clearance)
    rad_err = radial_error95(
        center_lateral95, axis_error_deg, length_m, cfg)
    ax_err = axial_error95(neck_position95, cfg)
    radial_margin = available - rad_err
    axial_margin = (
        cfg.blade_capture_half_width - cfg.axial_safety_margin - ax_err)
    diagnostic_mismatch = float(axis_error_deg) > cfg.diagnostic_axis_deg
    too_wide = available <= 0.0
    fruit_ok = float(cut_to_fruit_m) > cfg.fruit_safety_clearance
    sleeve_ok = (not too_wide) and radial_margin > 0.0
    cut_ok = axial_margin > 0.0 and fruit_ok
    reason = 'dynamic_budget_accept'
    failure_code = 0
    if too_wide:
        reason = 'bag_d95_exceeds_tool'
        failure_code = 12
    elif radial_margin <= 0.0:
        reason = 'radial_budget_negative'
        failure_code = 12
    elif not fruit_ok:
        reason = 'cut_plane_fruit_clearance'
        failure_code = 16
    elif axial_margin <= 0.0:
        reason = 'axial_budget_negative'
        failure_code = 12
    return {
        'sleeve_ok': bool(sleeve_ok),
        'cut_ok': bool(cut_ok),
        'allowed': bool(sleeve_ok and cut_ok),
        'radial_available_m': float(available),
        'radial_error95_m': float(rad_err),
        'radial_margin_m': float(radial_margin),
        'axial_error95_m': float(ax_err),
        'axial_margin_m': float(axial_margin),
        'diagnostic_axis_mismatch': bool(diagnostic_mismatch),
        'reason': reason,
        'failure_code': int(failure_code),
        'shadow': False,
    }
