"""视点策略两档（清洁重写轮 3c 纯核，零 ROS）。

fast（重写轮新默认）：拍照位单视决策——单视质量信号够即收，不足才补视，
补视封顶 3 固定视（荔枝三视先例）；补视方向取沿当前相机直线截
max_camera_step_m（现行口径原值）与绕袋轴 ±30° 两个候选里行程短者。
conservative：现行多视观察循环（覆盖门 8°/独立机位/maximum_moves 原值），
判定委托重建侧覆盖门，本模块只给出「继续/停止」的框架性判定。

质量信号阈值与 stop 准则常数 = 现行 arm 侧 view_planner 同值
（bbox 面积占比 0.04、掩膜前景 0.40、框心偏移推动），原值移植不调参。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Sequence


class ViewDecision(Enum):
    ENOUGH = 'enough'            # 质量足够，停止观察
    SUPPLEMENT = 'supplement'    # 需要补视
    CAP_REACHED = 'cap_reached'  # 补视封顶，停止（质量未达标也停，交许可门否决）


@dataclass(frozen=True)
class ViewSignals:
    """单视质量信号（来自当前机位的观测/分割）。"""

    bbox_area_ratio: float = 0.0      # 检测框面积/画面（<0.04 视为太远）
    mask_foreground_ratio: float = -1.0  # 分割前景占比（<0.40 视为掩膜不足）
    tf_ok: bool = True                 # 本帧 TF 三态（unavailable 不作数）
    bbox_valid: bool = True

    def too_small(self) -> bool:
        return self.bbox_valid and self.bbox_area_ratio < 0.04

    def mask_low(self) -> bool:
        return (self.mask_foreground_ratio >= 0.0
                and self.mask_foreground_ratio < 0.40)

    def acceptable(self) -> bool:
        """单视可用：TF 可用且不太小且掩膜不缺。"""
        return self.tf_ok and not self.too_small() and not self.mask_low()


@dataclass(frozen=True)
class FastViewConfig:
    """fast 档常数（现行口径原值；调整须重封回放基线）。"""

    max_supplemental_views: int = 2   # 补视数上限（总视数 ≤3）
    max_camera_step_m: float = 0.15   # 沿当前相机直线截距
    azimuth_sweep_deg: float = 30.0   # 绕袋轴候选摆角


@dataclass
class ViewPolicyState:
    used_views: int = 1                # 当前已用机位数（fast 从拍照位 1 起）
    history: list = field(default_factory=list)


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _scale(a, s):
    return [x * s for x in a]


def decide_fast(
    signals: ViewSignals,
    state: ViewPolicyState,
    config: Optional[FastViewConfig] = None,
) -> ViewDecision:
    """fast 档单点决策：质量可收即收；不足且未封顶则补视。"""
    cfg = config or FastViewConfig()
    if signals.acceptable():
        return ViewDecision.ENOUGH
    if state.used_views >= 1 + cfg.max_supplemental_views:
        return ViewDecision.CAP_REACHED
    return ViewDecision.SUPPLEMENT


def supplemental_viewpoint(
    target_xyz: Sequence[float],
    camera_xyz: Sequence[float],
    bag_axis: Sequence[float],
    camera_front: Optional[Sequence[float]] = None,
    config: Optional[FastViewConfig] = None,
) -> list[float]:
    """补视机位（base 系）：沿当前相机直线截 max_camera_step 与绕轴 ±30°
    两候选中行程较短者。纯几何，常数原值；不绕球面、不对侧兜圈。"""
    cfg = config or FastViewConfig()
    axis_len = _norm(bag_axis)
    unit_axis = [x / (axis_len or 1.0) for x in bag_axis]

    # 候选 A：沿当前相机→目标直线前进 max_camera_step（行程封顶）
    toward = _sub(target_xyz, camera_xyz)
    dist = _norm(toward)
    if dist <= cfg.max_camera_step_m or dist < 1.0e-9:
        cand_a = list(target_xyz)
    else:
        cand_a = _add(
            camera_xyz, _scale(toward, cfg.max_camera_step_m / dist))

    # 候选 B：绕袋轴 ±azimuth 摆（视线在垂直轴平面的分量旋转），行程同封顶
    front = camera_front if camera_front is not None else _sub(
        camera_xyz, target_xyz)
    f = _norm(front)
    if f < 1.0e-9:
        return cand_a
    unit_front = [x / f for x in front]
    keep = sum(unit_front[i] * unit_axis[i] for i in range(3))
    radial = _sub(unit_front, _scale(unit_axis, keep))
    r = _norm(radial)
    if r < 1.0e-6:
        return cand_a  # 视线沿轴：绕轴无定义，用直线截距
    unit_radial = [x / r for x in radial]
    other = [
        unit_axis[1] * unit_radial[2] - unit_axis[2] * unit_radial[1],
        unit_axis[2] * unit_radial[0] - unit_axis[0] * unit_radial[2],
        unit_axis[0] * unit_radial[1] - unit_axis[1] * unit_radial[0]]
    theta = math.radians(cfg.azimuth_sweep_deg)
    rotated = _add(
        _scale(unit_radial, math.cos(theta)), _scale(other, math.sin(theta)))
    direction = _add(rotated, _scale(unit_axis, keep))
    d = _norm(direction)
    if d < 1.0e-9:
        return cand_a
    cand_b = _add(camera_xyz, _scale(direction, cfg.max_camera_step_m / d))

    travel_a = _norm(_sub(cand_a, camera_xyz))
    travel_b = _norm(_sub(cand_b, camera_xyz))
    return cand_a if travel_a <= travel_b else cand_b


def conservative_should_continue(
    covered: bool,
    station_count: int,
    maximum_moves_used: int,
    maximum_moves: int,
) -> ViewDecision:
    """conservative 档框架判定：覆盖达标即停；moves 用尽即停（现行口径）。"""
    if covered:
        return ViewDecision.ENOUGH
    if maximum_moves_used >= maximum_moves or station_count >= 2:
        return ViewDecision.CAP_REACHED
    return ViewDecision.SUPPLEMENT
