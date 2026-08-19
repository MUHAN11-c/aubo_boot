"""
观测跟踪状态分类与光照质量统计（阶段 D1 室外感知强化；纯 Python，零 ROS）.

职责:
  1. ``classify_tracking_status``：PeachTargetObservation.tracking_status 的
     四分类细分（协议阶段 D1 第 4 条）——OCCLUDED（掩膜缺失但检测框在）、
     OUT_OF_VIEW（检测框触图像边缘后消失）、DEPTH_VOID（掩膜在但有效深度
     占比低于阈值）、LOST（其余消失）；纯核返回字符串 token，由节点映射到
     msg 常量（纯核禁 import ROS 消息，test_pure_core AST 强制）。
  2. ``bbox_touches_image_edge``：OUT_OF_VIEW 判定的边缘触碰原语。
  3. ``LightingMeter``：锁定集目标的光照质量逐帧统计（掩膜内有效深度
     占比均值 + 检测置信度均值的跨帧 EMA）与连续低质判定；结果是
     **观测指标**（harvest_state.lighting / low_light_quality），不打
     阻断旗标——低质现场处置是补光/曝光，感知侧只暴露事实。

时钟约定（协议 I3）：本模块不取时钟；EMA 只依赖逐帧注入的样本序列。
"""
from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

# 跟踪状态 token：节点映射到 PeachTargetObservation.msg 同名常量
STATUS_OBSERVED = 'OBSERVED'
STATUS_OCCLUDED = 'OCCLUDED'
STATUS_LOST = 'LOST'
STATUS_OUT_OF_VIEW = 'OUT_OF_VIEW'
STATUS_DEPTH_VOID = 'DEPTH_VOID'


def bbox_touches_image_edge(bbox: Tuple[int, int, int, int],
                            width: int, height: int) -> bool:
    """
    检测框是否触及图像边缘（含出界裁剪后贴边）.

    判定 OUT_OF_VIEW 的证据：目标走出视野前最后一帧的检测框必然贴在
    图像某侧边缘上；被枝叶遮挡/检测漏检而消失的目标框一般在图内。

    Args:
        bbox: (x1, y1, x2, y2) 像素框.
        width: 图像宽（像素）.
        height: 图像高（像素）.

    Returns
    -------
        任一边贴到图像边界（x1<=0 / y1<=0 / x2>=width / y2>=height）为真.

    """
    x1, y1, x2, y2 = (int(v) for v in bbox)
    return x1 <= 0 or y1 <= 0 or x2 >= int(width) or y2 >= int(height)


def classify_tracking_status(has_observation: bool, has_mask: bool,
                             mask_depth_ratio: Optional[float],
                             min_depth_ratio: float,
                             last_bbox_touched_edge: bool) -> str:
    """
    单目标跟踪状态四分类（阶段 D1；优先级自上而下首个命中即返回）.

    Args:
        has_observation: 本帧该目标是否有检测/几何输出（payload 非空）.
        has_mask: 本帧是否有可用 SAM 掩膜（仅 has_observation 为真时有意义）.
        mask_depth_ratio: 掩膜内有效深度占比 [0,1]；无掩膜时可为 None.
        min_depth_ratio: DEPTH_VOID 判定的有效深度占比下限.
        last_bbox_touched_edge: 目标消失前最后一帧检测框是否触图像边缘.

    Returns
    -------
        状态 token（本模块 STATUS_* 常量）：
        无观测 → OUT_OF_VIEW（触边消失）/ LOST（其余消失）；
        有观测无掩膜 → OCCLUDED；掩膜内有效深度占比低于阈值 → DEPTH_VOID；
        否则 OBSERVED.

    """
    if not has_observation:
        return STATUS_OUT_OF_VIEW if last_bbox_touched_edge else STATUS_LOST
    if not has_mask:
        return STATUS_OCCLUDED
    ratio = mask_depth_ratio
    if ratio is None or not math.isfinite(float(ratio)):
        # 占比缺失按 0 处理：无法证明有足够实测深度，保守判 DEPTH_VOID
        ratio = 0.0
    if float(ratio) < min_depth_ratio:
        return STATUS_DEPTH_VOID
    return STATUS_OBSERVED


class LightingMeter:
    """
    锁定集目标光照质量统计：逐帧均值 + 跨帧 EMA + 连续低质判定.

    每帧由节点注入两个样本序列（仅锁定集中本帧带掩膜观测的目标）：
    掩膜内有效深度占比与检测置信度；帧内取均值后以 EMA（α 默认 0.3，
    与帧率/耗时埋点同纪律）平滑。判定：深度占比 EMA < min_depth_ratio
    或置信度 EMA < min_conf_mean 的帧记一帧低质，连续 bad_frames 帧
    低质 → low_quality=True；一帧达标即清零连击（与摆动判定的对称
    连击同一风格）。无样本帧（锁定集目标全部无掩膜观测）不进 EMA也
    不计低质——没有观测不等于低质。

    线程安全：无内部锁，与调用方（节点 _plan_lock 保护区）同一把锁。
    """

    def __init__(self, alpha: float = 0.3, min_depth_ratio: float = 0.35,
                 min_conf_mean: float = 0.3, bad_frames: int = 5):
        """建表；α∈(0,1]、阈值∈[0,1]、连击帧数≥1 校验."""
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        if not 0.0 <= min_depth_ratio <= 1.0:
            raise ValueError(f'min_depth_ratio 须在 [0,1]: {min_depth_ratio}')
        if not 0.0 <= min_conf_mean <= 1.0:
            raise ValueError(f'min_conf_mean 须在 [0,1]: {min_conf_mean}')
        if bad_frames < 1:
            raise ValueError(f'bad_frames 须 ≥ 1: {bad_frames}')
        self.alpha = float(alpha)
        self.min_depth_ratio = float(min_depth_ratio)
        self.min_conf_mean = float(min_conf_mean)
        self.bad_frames = int(bad_frames)
        self._depth_ema: Optional[float] = None
        self._conf_ema: Optional[float] = None
        self._bad_streak = 0

    @staticmethod
    def _finite_mean(samples: Iterable[float]) -> Optional[float]:
        """有限样本均值；空集/全非有限返回 None（脏样本不进 EMA）."""
        values = [float(s) for s in samples if math.isfinite(float(s))]
        if not values:
            return None
        return sum(values) / len(values)

    def update(self, depth_ratios: Iterable[float],
               confidences: Iterable[float]) -> None:
        """
        注入本帧锁定集目标的观测样本并刷新 EMA 与低质连击.

        Args:
            depth_ratios: 各目标掩膜内有效深度占比 [0,1]（可空序列）.
            confidences: 各目标检测置信度 [0,1]（可空序列）.

        Returns
        -------
            无返回值（None）；两序列均空（本帧无有效观测）时整帧跳过，
            EMA 与连击保持不变.

        """
        depth_mean = self._finite_mean(depth_ratios)
        conf_mean = self._finite_mean(confidences)
        if depth_mean is None and conf_mean is None:
            return
        if depth_mean is not None:
            self._depth_ema = (
                depth_mean if self._depth_ema is None
                else (1.0 - self.alpha) * self._depth_ema
                + self.alpha * depth_mean)
        if conf_mean is not None:
            self._conf_ema = (
                conf_mean if self._conf_ema is None
                else (1.0 - self.alpha) * self._conf_ema
                + self.alpha * conf_mean)
        # 尚无 EMA 的分量按达标处理（无法证明低质时不冤枉现场光照）
        bad = (
            (self._depth_ema is not None
             and self._depth_ema < self.min_depth_ratio)
            or (self._conf_ema is not None
                and self._conf_ema < self.min_conf_mean))
        self._bad_streak = self._bad_streak + 1 if bad else 0

    @property
    def low_quality(self) -> bool:
        """连续 bad_frames 帧低质（EMA 维度任一不达标）."""
        return self._bad_streak >= self.bad_frames

    def snapshot(self) -> dict:
        """
        harvest_state JSON 的 lighting 子对象.

        Returns
        -------
            dict：depth_ratio / conf_mean 为 EMA（无样本为 None）、
            bad_streak 为当前低质连击帧数、low_quality 为判定结果.

        """
        return {
            'depth_ratio': self._depth_ema,
            'conf_mean': self._conf_ema,
            'bad_streak': self._bad_streak,
            'low_quality': self.low_quality,
        }
