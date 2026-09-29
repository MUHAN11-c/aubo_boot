"""
纯核观测原语：计数 + 周期摘要（EventMeter）.

纯核模块的失败/拒绝决策点不能逐条打日志（热路径刷屏），也不能完全静默
（排障断链）。EventMeter 折中：第 1 次必记，之后每 every 次记一条汇总，
走模块 stdlib logger（经 peach_common.ros_log_bridge 桥进 /rosout）。

用法（纯核模块顶部）::

    _logger = logging.getLogger(__name__)
    _fit_fail = EventMeter(_logger, '袋位姿拟合失败')

    def fit_bag(...):
        ...
        _fit_fail.hit('inliers<min')
        return None
"""
from __future__ import annotations

import logging


class EventMeter:
    """同一类事件的计数器：第 1 次与每 every 次发一条 WARN 摘要."""

    def __init__(self, logger, name: str, every: int = 50,
                 level: int = logging.WARNING):
        """构造计数器；logger 为 stdlib logger，name 为事件短名（日志行前缀）."""
        self._logger = logger
        self._name = str(name)
        self._every = max(1, int(every))
        self._level = int(level)
        self.count = 0
        self.last_detail = ''

    def hit(self, detail: str = '') -> None:
        """记一次事件；达到阈值时发摘要（含最近一次 detail）."""
        self.count += 1
        self.last_detail = str(detail)
        if self.count == 1 or self.count % self._every == 0:
            suffix = f'（最近: {detail}）' if detail else ''
            self._logger.log(
                self._level, '%s 累计 %d 次%s', self._name, self.count,
                suffix)

    def snapshot(self) -> dict:
        """只读投影（诊断/测试用）."""
        return {'name': self._name, 'count': self.count,
                'last_detail': self.last_detail}
