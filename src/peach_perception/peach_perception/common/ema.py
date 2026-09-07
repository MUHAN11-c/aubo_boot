"""标量指数滑动平均（纯核，调用方保证单线程）."""
from __future__ import annotations

from typing import Optional


class ScalarEma:
    """
    标量指数滑动平均（thread-unsafe：调用方自行保证单线程访问）.

    α 取值口径：0.3 在响应速度与抗单帧抖动间取折中（沿用原
    capture.EMA_ALPHA / integrate._CORR_EMA_ALPHA 注释）。
    首个有效样本直接作初值，其后
    ``value ← α·sample + (1−α)·value``（α 为新样本权重）。
    """

    def __init__(self, alpha: float = 0.3):
        """初始化未播种的 EMA；alpha 为新样本权重 ∈ (0, 1]."""
        self._alpha = float(alpha)
        self._value: Optional[float] = None

    def update(self, sample: float) -> float:
        """
        注入一个样本并返回更新后的 EMA；首个样本直接作初值.

        Args:
            sample: 本次样本值（调用方保证非负/有限；脏样本先过滤）.

        Returns
        -------
            更新后的 EMA.

        """
        value = float(sample)
        if self._value is None:
            self._value = value
        else:
            self._value = (self._alpha * value
                           + (1.0 - self._alpha) * self._value)
        return self._value

    def reset(self) -> None:
        """清空播种状态（回到无样本）."""
        self._value = None

    @property
    def value(self) -> Optional[float]:
        """当前 EMA；尚无样本时 None."""
        return self._value

    @property
    def seeded(self) -> bool:
        """是否已有样本（未播种时按调用方约定处理，如视为达标）."""
        return self._value is not None
