"""
推理耗时分项 EMA 记录器（零 ROS import；协议 I3：时钟由调用方注入）.

解决的问题:
  感知节点 ``_process_rgbd`` 各段（detect/segment/geometry/total）耗时此前
  无任何记录，帧率下降或偶发卡顿无法定位到具体段。本模块提供低开销的
  分项 EMA 记录与可 JSON 序列化快照，节点把快照挂进 harvest_state 的
  ``timing`` 子对象下发（不新增话题）。

设计要点:
  - EMA 平滑单帧抖动（默认 α=0.3，与 RateEstimator 同纪律）；
    首个样本直接作 EMA 初值；
  - 脏样本（nan/inf/负值）直接丢弃，不污染估计；
  - 本模块只记录数值，**不自行取时钟**——耗时样本由调用方用注入时钟
    （节点侧 RclpyClockAdapter.now()）测量后以毫秒传入；
  - 线程模型：无内部锁，现网仅推理 worker 单线程写、经节点同一把
    plan 锁读快照。
"""
from __future__ import annotations

import math
from typing import Dict, Optional


class TimingMetrics:
    """
    分段耗时 EMA 记录器（键 → 毫秒 EMA）.

    构造参数 alpha 为新样本权重（0, 1]；与现网帧率 EMA 同取 0.3。
    record() 逐帧注入各段耗时；snapshot() 返回含全部分段键与 fps 的
    可序列化 dict（键固定排序，便于下游 diff/测试断言）。
    """

    def __init__(self, alpha: float = 0.3):
        """建空记录器；alpha 校验（须在 (0, 1]）."""
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha 必须在 (0, 1] 内: {alpha}')
        self._alpha = float(alpha)
        self._ema: Dict[str, float] = {}

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
        old = self._ema.get(key)
        self._ema[key] = (
            value if old is None
            else (1.0 - self._alpha) * old + self._alpha * value)

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
        out = {key: round(value, 3) for key, value in sorted(self._ema.items())}
        out['fps'] = round(float(fps), 2) if fps and fps > 0.0 else 0.0
        return out
