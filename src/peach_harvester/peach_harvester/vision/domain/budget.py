"""工具预算能力三态（零 ROS）。cut 不可行仍可 READY，不得声称 cut_feasible."""
from __future__ import annotations

from peach_harvester.vision.common.tool_budget import (
    evaluate_sleeve_cut,
    ToolBudgetParams,
)
from peach_harvester.vision.domain.model_contract import allowed_from_capabilities

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
    # TODO(W13-A 占位)：geometry 恒 VALID 是占位语义——本评估器只见点云
    # 拟合质量，几何/预抓可达性（IK、场景碰撞）尚无独立信号源接入；
    # 接入真实信号后此处应改为按观测推导，不得默认放行。
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
        # TODO(W13-A 占位)：ready_ok 恒 True 是占位语义——预算三态尚不
        # 产出「就绪」判定（现由 allowed 单独承担门控）；引入真实就绪
        # 条件（如标定/模型版本核对）前，消费方不得把 True 当校验结论。
        'ready_ok': True,
    }
