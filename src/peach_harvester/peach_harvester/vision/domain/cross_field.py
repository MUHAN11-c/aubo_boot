"""跨字段参数校验（零 ROS）。非法返回原因串，合法返回 None."""
from __future__ import annotations

from peach_harvester.vision.common.tool_budget import evaluate_sleeve_cut, ToolBudgetParams


def check_min_max(min_value, max_value, min_key: str, max_key: str):
    """Min <= max."""
    if float(min_value) > float(max_value):
        return f'{min_key} <= {max_key} required'
    return None


def check_enable_deps(execution_enabled: bool, grasp_enabled: bool,
                      tool_enabled: bool):
    """Tool enable requires grasp then execution; all-off is legal."""
    if tool_enabled and not grasp_enabled:
        return 'tool_enabled requires grasp_enabled'
    if grasp_enabled and not execution_enabled:
        return 'grasp_enabled requires execution_enabled'
    return None


def check_depth_window(min_depth_m: float, max_depth_m: float):
    """感知深度窗."""
    return check_min_max(
        min_depth_m, max_depth_m, 'pipeline.min_depth_m', 'pipeline.max_depth_m')


def check_reach_window(min_m: float, max_m: float):
    """选果可达窗."""
    return check_min_max(
        min_m, max_m, 'selection_reach_min_m', 'selection_reach_max_m')


def cut_capability_report(params: ToolBudgetParams | None = None,
                          neck_position95: float = 0.0) -> dict:
    """
    Report whether the default profile can claim cut_feasible.

    Auto-cut remaining infeasible is the correct F05 result; configure may
    still reach READY.
    """
    cfg = params or ToolBudgetParams()
    raw = evaluate_sleeve_cut(
        d_bag95=0.08, length_m=0.15, center_lateral95=0.002,
        axis_error_deg=2.0, neck_position95=neck_position95,
        cut_to_fruit_m=0.04, params=cfg)
    return {
        'cut_feasible': bool(raw['cut_ok']),
        'sleeve_feasible': bool(raw['sleeve_ok']),
        'axial_margin_m': float(raw['axial_margin_m']),
        'ready_ok': True,
        'claim_cut': bool(raw['cut_ok']),
    }
