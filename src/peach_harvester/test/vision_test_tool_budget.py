"""Characterization: default cut budget is infeasible (F05). Do not widen constants."""
from peach_harvester.vision.common.tool_budget import (
    evaluate_sleeve_cut,
    ToolBudgetParams,
)
from peach_harvester.vision.domain.budget import evaluate_capabilities
from peach_harvester.vision.domain.model_contract import (
    allowed_from_capabilities,
    CAPABILITY_VALID,
)


def _cut_infeasible(d_inner: float, neck_position95: float) -> None:
    # 袋径/轴向误差取宽松值：径向可过门时仍应因轴向余量拒绝剪切。
    params = ToolBudgetParams(d_inner=d_inner)
    result = evaluate_sleeve_cut(
        d_bag95=0.06,
        length_m=0.05,
        center_lateral95=0.0,
        axis_error_deg=0.0,
        neck_position95=neck_position95,
        cut_to_fruit_m=0.05,
        params=params,
    )
    assert result['axial_margin_m'] < 0.0
    assert result['cut_ok'] is False
    assert 'allowed' not in result


def test_hollow_inner_cut_infeasible_at_zero_and_fused_sigma():
    # 能力暂不支持自动剪切是正确结果；R5 才按标定重核常数。
    _cut_infeasible(0.104, 0.0)
    _cut_infeasible(0.104, 0.006)


def test_adaptive_inner_cut_infeasible_at_zero_and_fused_sigma():
    _cut_infeasible(0.116, 0.0)
    _cut_infeasible(0.116, 0.006)


def test_evaluate_capabilities_allowed_is_contract_only():
    budget = evaluate_capabilities(
        d_bag95=0.06, length_m=0.05, center_lateral95=0.0,
        axis_error_deg=0.0, neck_position95=0.0, cut_to_fruit_m=0.05)
    expected = allowed_from_capabilities(
        budget['geometry_capability'], budget['pregrasp_capability'],
        budget['sleeve_capability'], budget['cut_capability'])
    assert budget['allowed'] == expected
    assert budget['cut_capability'] != CAPABILITY_VALID
    assert budget['allowed'] is False
