"""单调时钟抽象：真机注入 ROS 钟，测试用 ManualClock."""
from __future__ import annotations

from abc import ABC, abstractmethod


class Clock(ABC):
    """
    单调时钟抽象基类（协议 I3）.

    生命周期：通常与节点同寿，由编排层构造并注入各纯核组件。
    线程安全：实现方保证 now() 可并发调用。可替换性：真机用
    RclpyClockAdapter，单测/回放开 ManualClock。
    """

    @abstractmethod
    def now(self) -> float:
        """返回当前单调秒（float）."""


class ManualClock(Clock):
    """
    测试用手动时钟：时间只在 advance() 时前进，确定性可复现.

    仅供单线程测试/回放使用（无内部同步）。
    """

    def __init__(self, start: float = 0.0):
        """创建虚拟时钟，起始时刻 start 秒."""
        self._now = float(start)

    def now(self) -> float:
        """返回当前虚拟时刻（不自动前进）."""
        return self._now

    def advance(self, dt: float) -> float:
        """
        虚拟时间前进 dt 秒并返回新时刻.

        Raises
        ------
            ValueError: dt 为负（单调语义禁止倒退）.

        """
        if dt < 0.0:
            raise ValueError(f'ManualClock 禁止倒退: dt={dt}')
        self._now += dt
        return self._now
