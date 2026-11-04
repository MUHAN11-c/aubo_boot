"""
Fast 档质量与机位数判定纯核（零 ROS）.

本模块只判停止、补视或封顶；实际补视机位由 view_planner.generate
生成。conservative 观察流程仍由 executor_node 的臂侧观察动作承担。
质量门和总视数上限保持原值，不承载第二套补视几何。
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class ViewDecision(Enum):
    """单点决策三态（fast/conservative 共用返回类型）."""

    ENOUGH = 'enough'
    """质量足够，停止观察."""
    SUPPLEMENT = 'supplement'
    """需要补视."""
    CAP_REACHED = 'cap_reached'
    """补视封顶，停止（质量未达标也停，交许可门否决）."""


@dataclass(frozen=True)
class ViewSignals:
    """单视质量信号（来自当前机位的观测/分割）."""

    bbox_area_ratio: float = 0.0
    """检测框面积/画面；<0.04 视为太远."""
    mask_foreground_ratio: float = -1.0
    """分割前景占比；<0.40 视为不足；-1=缺省不判."""
    tf_ok: bool = True
    """本帧 TF 可用（unavailable 不作数）."""
    bbox_valid: bool = True
    """检测框合法."""

    def too_small(self) -> bool:
        """检测框占画面比例过低（太远/残片）."""
        return self.bbox_valid and self.bbox_area_ratio < 0.04

    def mask_low(self) -> bool:
        """掩膜前景占比不足（-1=缺省不判，三态设计）."""
        return (self.mask_foreground_ratio >= 0.0
                and self.mask_foreground_ratio < 0.40)

    def acceptable(self) -> bool:
        """单视可用：TF 可用且不太小且掩膜不缺."""
        return self.tf_ok and not self.too_small() and not self.mask_low()


@dataclass(frozen=True)
class FastViewConfig:
    """fast 档常数（现行口径原值；调整须重封回放基线）."""

    max_supplemental_views: int = 2
    """补视数上限（总视数 ≤3）."""


@dataclass
class ViewPolicyState:
    """fast 档决策状态（当前已用机位数与历史）."""

    used_views: int = 1
    """当前已用机位数（fast 从拍照位 1 起）."""


def decide_fast(
    signals: ViewSignals,
    state: ViewPolicyState,
    config: Optional[FastViewConfig] = None,
    min_views: int = 1,
) -> ViewDecision:
    """
    Fast 档单点决策：质量可收且机位数达门限才收；不足则补视.

    min_views（调用方传 supervisor reconstruction_min_views；默认 1=不
    约束，兼容旧语义）：好单视不再直接收口——消除「fast 好单视不移动 ×
    supervisor min_views=2 × 重建基线门」三层矛盾（09-17 真机 P1-B：
    单站永不收口，宽限后被按质量跳过）。机位未达门限时即使质量合格也
    补视；门限高于补视封顶时按 CAP_REACHED 停（交许可门否决）。
    """
    cfg = config or FastViewConfig()
    cap = 1 + cfg.max_supplemental_views
    if signals.acceptable() and state.used_views >= max(int(min_views), 1):
        return ViewDecision.ENOUGH
    if state.used_views >= cap:
        return ViewDecision.CAP_REACHED
    return ViewDecision.SUPPLEMENT
