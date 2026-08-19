"""
帧率 EMA 估计与自适应超时 — 三份重复实现的统一（协议 I4）.

职责:
  统一以下三处现网重复实现（重构阶段 A1）：
    1. 感知节点 peach_pose_node._publish_target_observations 的帧间隔
       EMA（α=0.3，异常间隔 ≤1ms 或 >30s 不进 EMA）+ max_collect_s
       自适应 max(0.4×cfg, (min+settle+3)×ema)；
    2. 能力端 C++ safety_gate.adaptive_timeout_s（clamp(floor,
       mult×ema+margin, cap)）与 approach_grasp_node 的帧间隔 EMA；
    3. 重建侧目标观测龄自适应 clamp(1.0, 2.5×ema+0.5, 10.0)。

输入/输出契约:
  RateEstimator.update(now) 注入单调时钟秒；interval/rate_hz 无有效
  样本时为 None（None 安全）。AdaptiveTimeout.value(estimated_interval)
  无实测时返回 upper（回退档），有实测时返回
  clamp(lower, factor×interval+offset, upper)。

协议条款:
  I3（时钟唯一）：全部时间由调用方注入 now，**禁止 time.monotonic /
  time.time**（test_pure_core.py AST 强制本文件不 import time）；
  I4（超时协议）：超时 = clamp(下限, f(实测EMA), 上限)。

线程模型:
  实例不可变配置 + 可变估计状态，无内部同步；同一估计器实例须单线程
  调用（现网各调用点均为单回调线程）。
"""
from __future__ import annotations

from typing import Optional


class RateEstimator:
    """
    帧/事件间隔 EMA 估计器（帧率以运行状态为准）.

    异常间隔过滤沿用 peach_pose_node 帧率 EMA 纪律：间隔 ≤1ms（同帧
    重复/时钟噪声）或 >30s（暂停后首帧/时钟跳变）不进 EMA，防污染
    估计；但「上次时刻」始终更新，保证暂停恢复后下一帧间隔重新有效。

    生命周期：构造后可长期持有，随每个事件调用 update；无重置需求
    （EMA 自然跟踪缓变）。线程安全：无内部锁，单写者使用。
    """

    def __init__(self, alpha: float = 0.3, *,
                 min_interval_s: float = 1e-3,
                 max_interval_s: float = 30.0):
        """
        创建估计器；alpha 为新样本权重（现网三处均为 0.3）.

        Raises
        ------
            ValueError: alpha 不在 (0, 1] 或异常过滤区间非法.

        """
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        if min_interval_s <= 0.0 or min_interval_s >= max_interval_s:
            raise ValueError(
                f'过滤区间非法: ({min_interval_s}, {max_interval_s})')
        self._alpha = float(alpha)
        self._min_interval_s = float(min_interval_s)
        self._max_interval_s = float(max_interval_s)
        self._last_now: Optional[float] = None
        self._ema: Optional[float] = None

    def update(self, now: float) -> None:
        """
        注入一个事件的单调时钟秒；内部完成间隔计算与 EMA 更新.

        异常间隔（≤min_interval_s 或 >max_interval_s）不进 EMA；
        首个合法样本直接作 EMA 初值（与现网 ``ema if None else …`` 一致）。
        now 为注入时钟的当前秒（协议 I3：禁止内部自行取时钟）。
        """
        if self._last_now is not None:
            dt = now - self._last_now
            if self._min_interval_s < dt < self._max_interval_s:
                self._ema = (
                    dt if self._ema is None
                    else (1.0 - self._alpha) * self._ema + self._alpha * dt)
        # 上次时刻始终更新：暂停后首帧虽不进 EMA，但恢复后下一帧间隔有效
        self._last_now = now

    @property
    def interval(self) -> Optional[float]:
        """间隔 EMA（秒）；尚无有效样本时返回 None."""
        return self._ema

    @property
    def rate_hz(self) -> Optional[float]:
        """估计频率（Hz）；尚无有效样本时返回 None（None 安全）."""
        if self._ema is None or self._ema <= 0.0:
            return None
        return 1.0 / self._ema


class AdaptiveTimeout:
    """
    自适应超时取值器（协议 I4：clamp(下限, f(实测EMA), 上限)）.

    构造后不可变，可跨线程只读共享。三处现网用法映射：
      - 视点等待 frame_wait：AdaptiveTimeout(
            lower=2.0, upper=<scan.frame_wait_s 配置>, factor=4.0,
            offset=1.0)；ema 未测得时回退配置值（=upper）；
      - 收齐窗口 max_collect_s：factor=(min_collect+settle+3)、offset=0、
            lower=0.4×配置、upper=float('inf')（现网只设下限；无实测时
            由调用方保留配置值，勿用本类 None 回退档）；
      - 目标观测龄 target_observation_max_age：AdaptiveTimeout(
            lower=1.0, upper=10.0, factor=2.5, offset=0.5)。
    """

    def __init__(self, *, lower: float, upper: float,
                 factor: float, offset: float = 0.0):
        """
        创建取值器；lower ≤ upper，factor ≥ 0，均有限（upper 可为 inf）.

        Raises
        ------
            ValueError: 参数区间非法.

        """
        if lower > upper:
            raise ValueError(f'lower 不得大于 upper: {lower} > {upper}')
        if factor < 0.0:
            raise ValueError(f'factor 不得为负: {factor}')
        self._lower = float(lower)
        self._upper = float(upper)
        self._factor = float(factor)
        self._offset = float(offset)

    def value(self, estimated_interval: Optional[float]) -> float:
        """
        按实测间隔 EMA 求超时秒.

        无实测（None）返回 upper（回退档，对应现网「ema 未测得回退配置
        值」——各用法的配置上限即 upper）；有实测返回
        clamp(lower, factor×estimated_interval + offset, upper)。

        Args:
            estimated_interval: RateEstimator.interval（秒）或 None.

        Returns
        -------
            超时秒数，保证落在 [lower, upper].

        """
        if estimated_interval is None:
            return self._upper
        raw = self._factor * estimated_interval + self._offset
        return min(self._upper, max(self._lower, raw))
