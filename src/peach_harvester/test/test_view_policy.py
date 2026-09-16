"""视点两档策略纯核测试（重写轮 3c；随用户单独验证轮运行）。"""
from __future__ import annotations

import math

from peach_harvester.cycle_core.view_policy import (
    FastViewConfig,
    ViewDecision,
    ViewPolicyState,
    ViewSignals,
    conservative_should_continue,
    decide_fast,
    supplemental_viewpoint,
)


def _signals(**kw):
    base = dict(bbox_area_ratio=0.10, mask_foreground_ratio=0.6,
                tf_ok=True, bbox_valid=True)
    base.update(kw)
    return ViewSignals(**base)


def test_fast_good_single_view_enough():
    assert decide_fast(_signals(), ViewPolicyState()) is ViewDecision.ENOUGH


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


def test_supplemental_step_capped():
    target = [0.5, -0.6, 0.55]
    camera = [0.3, -0.3, 0.7]
    axis = [0.0, 0.0, 1.0]
    goal = supplemental_viewpoint(target, camera, axis)
    travel = math.dist(goal, camera)
    assert travel <= FastViewConfig().max_camera_step_m + 1.0e-6


def test_supplemental_axial_degenerate_falls_back_to_line():
    target = [0.0, 0.0, 0.0]
    camera = [0.0, 0.0, 0.5]
    axis = [0.0, 0.0, 1.0]
    goal = supplemental_viewpoint(target, camera, axis)
    # 视线沿轴：候选 B 无定义 → 直线截距（向目标收一步）
    assert math.dist(goal, [0.0, 0.0, 0.35]) < 1.0e-6


def test_conservative_stop_rules():
    assert conservative_should_continue(
        covered=True, station_count=1, maximum_moves_used=0,
        maximum_moves=2) is ViewDecision.ENOUGH
    assert conservative_should_continue(
        covered=False, station_count=2, maximum_moves_used=1,
        maximum_moves=2) is ViewDecision.CAP_REACHED
    assert conservative_should_continue(
        covered=False, station_count=1, maximum_moves_used=2,
        maximum_moves=2) is ViewDecision.CAP_REACHED
    assert conservative_should_continue(
        covered=False, station_count=1, maximum_moves_used=0,
        maximum_moves=2) is ViewDecision.SUPPLEMENT
