"""
流观测原语：帧率/超时/分段耗时/光照质量的 EMA 统计（纯核）.

统一使用 peach_perception.common.runtime.ScalarEma 递推（首个样本播种、
α=0.3 在响应速度与抗单帧抖动间取折中），异常样本过滤口径沿用
原 peach_scene_perception_node 的埋点纪律。
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Optional

from peach_perception.common.runtime import ScalarEma


class RateEstimator:
    """
    帧/事件间隔 EMA 估计器（帧率以运行状态为准）.

    异常间隔过滤沿用 peach_scene_perception_node 帧率 EMA 纪律：间隔 ≤1ms（同帧
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
        self._min_interval_s = float(min_interval_s)
        self._max_interval_s = float(max_interval_s)
        self._last_now: Optional[float] = None
        self._ema = ScalarEma(alpha)

    def update(self, now: float) -> None:
        """
        注入一个事件的单调时钟秒；内部完成间隔计算与 EMA 更新.

        异常间隔（≤min_interval_s 或 >max_interval_s）不进 EMA；
        首个合法样本直接作 EMA 初值。
        now 为注入时钟的当前秒（协议 I3：禁止内部自行取时钟）。
        """
        if self._last_now is not None:
            dt = now - self._last_now
            if self._min_interval_s < dt < self._max_interval_s:
                self._ema.update(dt)
        # 上次时刻始终更新：暂停后首帧虽不进 EMA，但恢复后下一帧间隔有效
        self._last_now = now

    @property
    def interval(self) -> Optional[float]:
        """间隔 EMA（秒）；尚无有效样本时返回 None."""
        return self._ema.value

    @property
    def rate_hz(self) -> Optional[float]:
        """估计频率（Hz）；尚无有效样本时返回 None（None 安全）."""
        ema = self._ema.value
        if ema is None or ema <= 0.0:
            return None
        return 1.0 / ema


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


class TimingMetrics:
    """
    分段耗时 EMA 记录器（键 → 毫秒 EMA）.

    构造参数 alpha 为新样本权重（0, 1]；与现网帧率 EMA 同取 0.3）。
    record() 逐帧注入各段耗时；snapshot() 返回含全部分段键与 fps 的
    可序列化 dict（键固定排序，便于下游 diff/测试断言）。
    """

    def __init__(self, alpha: float = 0.3):
        """建空记录器；alpha 校验（须在 (0, 1]）."""
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        self._alpha = float(alpha)
        self._ema: Dict[str, ScalarEma] = {}

    def record(self, key: str, sample_ms: float) -> None:
        """
        记录一段耗时样本（毫秒）；首个样本直接作 EMA 初值.

        Args:
            key: 分段名（如 'detect_ms'）.
            sample_ms: 本帧该段耗时（毫秒，调用方用注入时钟测量）.

        Returns
        -------
            无返回值（None）；脏样本（nan/inf/负值）静默丢弃.

        """
        value = float(sample_ms)
        if not math.isfinite(value) or value < 0.0:
            return
        ema = self._ema.setdefault(key, ScalarEma(self._alpha))
        ema.update(value)

    def snapshot(self, fps: Optional[float] = None) -> dict:
        """
        返回可 JSON 序列化快照：各段 EMA 毫秒（3 位小数）+ 实测 fps.

        Args:
            fps: 实测帧率（Hz，来自帧间隔 EMA）；None/非正数记 0.0.

        Returns
        -------
            dict：{<分段键>: EMA 毫秒, ..., 'fps': 实测帧率}；尚无样本时
            仅含 'fps' 键.

        """
        out = {key: round(ema.value, 3)
               for key, ema in sorted(self._ema.items())}
        out['fps'] = round(float(fps), 2) if fps and fps > 0.0 else 0.0
        return out


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
        self._depth_ema = ScalarEma(alpha)
        self._conf_ema = ScalarEma(alpha)
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
            self._depth_ema.update(depth_mean)
        if conf_mean is not None:
            self._conf_ema.update(conf_mean)
        # 尚无 EMA 的分量按达标处理（无法证明低质时不冤枉现场光照）
        bad = (
            (self._depth_ema.seeded
             and self._depth_ema.value < self.min_depth_ratio)
            or (self._conf_ema.seeded
                and self._conf_ema.value < self.min_conf_mean))
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
            'depth_ratio': self._depth_ema.value,
            'conf_mean': self._conf_ema.value,
            'bad_streak': self._bad_streak,
            'low_quality': self.low_quality,
        }
