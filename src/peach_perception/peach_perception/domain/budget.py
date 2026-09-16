"""工具预算能力三态（零 ROS）。cut 不可行仍可 READY，不得声称 cut_feasible."""
from __future__ import annotations

from peach_perception.common.tool_budget import (
    evaluate_sleeve_cut,
    ToolBudgetParams,
)
from peach_perception.domain.model_contract import allowed_from_capabilities

CAPABILITY_VALID = 0
CAPABILITY_INVALID = 1
CAPABILITY_UNKNOWN = 2


def capability_from_ok(ok, known: bool = True) -> int:
    """bool/None → VALID/INVALID/UNKNOWN."""
    if not known or ok is None:
        return CAPABILITY_UNKNOWN
    return CAPABILITY_VALID if ok else CAPABILITY_INVALID


def evaluate_capabilities(
        d_bag95: float, length_m: float, center_lateral95: float,
        axis_error_deg: float, neck_position95: float,
        cut_to_fruit_m: float,
        params: ToolBudgetParams | None = None) -> dict:
    """预算评估 + 能力三态。allowed 只走 allowed_from_capabilities."""
    raw = evaluate_sleeve_cut(
        d_bag95, length_m, center_lateral95, axis_error_deg,
        neck_position95, cut_to_fruit_m, params)
    sleeve = capability_from_ok(raw['sleeve_ok'])
    cut = capability_from_ok(raw['cut_ok'])
    geometry = capability_from_ok(True)
    allowed = allowed_from_capabilities(geometry, geometry, sleeve, cut)
    return {
        **raw,
        'geometry_capability': geometry,
        'pregrasp_capability': geometry,
        'sleeve_capability': sleeve,
        'cut_capability': cut,
        'cut_feasible': cut == CAPABILITY_VALID,
        'sleeve_feasible': sleeve == CAPABILITY_VALID,
        'allowed': allowed,
        'ready_ok': True,
    }
