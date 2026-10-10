# 批次2（2026-09-23 重构）新契约：轴向预算维度拆分 + 结构性诊断旗。
# 旧版 characterization（"零误差也不可剪"）锁的是 sig_p 双喂缺陷行为，
# 已随 G1 修复作废；本文件锁定修复后的语义。
from peach_harvester.vision.common.tool_budget import (
    evaluate_sleeve_cut,
    ToolBudgetParams,
)
from peach_harvester.vision.domain.budget import evaluate_capabilities
from peach_harvester.vision.domain.model_contract import (
    allowed_from_capabilities,
    CAPABILITY_VALID,
)


def _run(d_inner: float, neck_position95: float, **kw) -> dict:
    params = ToolBudgetParams(d_inner=d_inner, **kw)
    return evaluate_sleeve_cut(
        d_bag95=0.06, length_m=0.05, center_lateral95=0.0,
        axis_error_deg=0.0, neck_position95=neck_position95,
        cut_to_fruit_m=0.05, params=params)


def test_zero_neck_error_has_headroom_not_structural():
    # 拆维度+安全余量去重后：颈误差 0 时 8−0−7=+1mm 余量（旧版恒负缺陷已修）
    r = _run(0.104, 0.0)
    assert r['axial_margin_m'] > 0.0
    assert r['cut_ok'] is True
    assert r['axial_structural'] is False


def test_lateral_floor_no_longer_poisons_axial():
    # 旧缺陷：横向 6mm 下限经 sig_p 双喂进轴向。现在颈轴向 6mm（真散布）
    # 才拒，且 reason 是普通负预算（8−7−6<0 但 8−7−0>0 非结构性）
    r = _run(0.104, 0.006)
    assert r['axial_margin_m'] < 0.0
    assert r['cut_ok'] is False
    assert r['reason'] == 'axial_budget_negative'
    assert r['axial_structural'] is False


def test_structural_flag_when_fixed_budget_exceeds_capture():
    # 常数/工艺不成立（工具侧固定常数 ≥ 捕获带，感知零误差也救不回）→
    # 结构性诊断。恢复旧轴向安全余量 4mm 即触发：8 − 11 < 0。
    r = _run(0.104, 0.0, axial_safety_margin=0.004)
    assert r['axial_structural'] is True
    assert r['reason'] == 'axial_budget_structurally_unsatisfiable'
    # 颈下限再大也不改变结构性判定（不属工艺侧）
    r2 = _run(0.104, 0.006, axial_safety_margin=0.004)
    assert r2['axial_structural'] is True


def test_increasing_error_never_raises_permission():
    # FINAL_PLAN §20 最重要反例：增大任一误差项不得提高许可。
    base = _run(0.104, 0.0)
    worse = _run(0.104, 0.002)
    assert worse['axial_margin_m'] < base['axial_margin_m']
    assert (not worse['cut_ok']) or base['cut_ok']
    wider = _run(0.104, 0.0, target_motion95=0.010)
    assert wider['axial_margin_m'] < base['axial_margin_m']


def test_legacy_axial_safety_restorable_for_ab():
    # 旧值 0.004 可经参数恢复对拍（语义重复扣减已注明）
    r = _run(0.104, 0.0, axial_safety_margin=0.004)
    assert abs(r['axial_margin_m'] - (
        0.008 - 0.004 - (0.002 + 0.002 + 0.003))) < 1e-12


def test_evaluate_capabilities_allowed_is_contract_only():
    # 颈 0（完美感知）→ cut VALID、allowed True（旧"恒拒"是缺陷行为）
    budget = evaluate_capabilities(
        d_bag95=0.06, length_m=0.05, center_lateral95=0.0,
        axis_error_deg=0.0, neck_position95=0.0, cut_to_fruit_m=0.05)
    expected = allowed_from_capabilities(
        budget['geometry_capability'], budget['pregrasp_capability'],
        budget['sleeve_capability'], budget['cut_capability'])
    assert budget['allowed'] == expected
    assert budget['cut_capability'] == CAPABILITY_VALID
    assert budget['allowed'] is True
    # 颈 ≥ 3mm（默认轴向下限）→ cut INVALID、allowed False
    fused = evaluate_capabilities(
        d_bag95=0.06, length_m=0.05, center_lateral95=0.0,
        axis_error_deg=0.0, neck_position95=0.003, cut_to_fruit_m=0.05)
    assert fused['cut_capability'] != CAPABILITY_VALID
    assert fused['allowed'] is False
