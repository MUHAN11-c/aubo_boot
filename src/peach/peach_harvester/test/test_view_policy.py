"""视点两档策略纯核测试（重写轮 3c；随用户单独验证轮运行）."""
from __future__ import annotations

from peach_harvester.cycle_core.view_policy import (
    decide_fast,
    ViewDecision,
    ViewPolicyState,
    ViewSignals,
)


def _signals(**kw):
    base = {
        'bbox_area_ratio': 0.10, 'mask_foreground_ratio': 0.6,
        'tf_ok': True, 'bbox_valid': True}
    base.update(kw)
    return ViewSignals(**base)


def test_fast_good_single_view_enough():
    assert decide_fast(_signals(), ViewPolicyState()) is ViewDecision.ENOUGH


def test_fast_min_views_gates_enough():
    """P1-B：机位未达 supervisor 门限时好单视也不收，补视到门限才收."""
    state1 = ViewPolicyState(used_views=1)
    assert decide_fast(
        _signals(), state1, min_views=2) is ViewDecision.SUPPLEMENT
    state2 = ViewPolicyState(used_views=2)
    assert decide_fast(
        _signals(), state2, min_views=2) is ViewDecision.ENOUGH
    # 门限默认 1=旧语义（单视合格即收）
    assert decide_fast(
        _signals(), state1) is ViewDecision.ENOUGH


def test_fast_min_views_above_cap_reaches_cap():
    """门限高于补视封顶：按封顶停（交许可门否决），不无限补视."""
    state3 = ViewPolicyState(used_views=3)
    assert decide_fast(
        _signals(), state3, min_views=4) is ViewDecision.CAP_REACHED


def test_fast_small_bbox_supplements_then_caps():
    state = ViewPolicyState(used_views=1)
    assert decide_fast(
        _signals(bbox_area_ratio=0.02), state) is ViewDecision.SUPPLEMENT
    state3 = ViewPolicyState(used_views=3)
    assert decide_fast(
        _signals(bbox_area_ratio=0.02), state3) is ViewDecision.CAP_REACHED


def test_fast_tf_unavailable_does_not_count():
    assert decide_fast(
        _signals(tf_ok=False), ViewPolicyState()) is ViewDecision.SUPPLEMENT


def test_fast_mask_low_boundary():
    assert decide_fast(
        _signals(mask_foreground_ratio=0.39),
        ViewPolicyState()) is ViewDecision.SUPPLEMENT
    assert decide_fast(
        _signals(mask_foreground_ratio=0.41),
        ViewPolicyState()) is ViewDecision.ENOUGH
