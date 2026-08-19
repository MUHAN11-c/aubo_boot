"""
rclpy 时钟适配器 — Clock 抽象的 ROS 运行时实现（协议 I3）.

职责:
  把 rclpy 节点时钟（node.get_clock()）适配为纯核 :class:`Clock`
  接口，使纯核组件在 ROS 节点内仍走注入时钟，不自行取时间。

输入/输出契约:
  构造接受任何带 ``now()`` 且返回值有 ``.nanoseconds`` 属性的时钟
  对象（rclpy.clock.Clock 满足；测试可用假对象，无需 rclpy 初始化）；
  now() 返回 float 秒。

协议条款:
  I3（时钟唯一）；本文件位于 peach_common_py.ros 子包——纯核 guard 的唯一
  豁免区。实现本体零 rclpy import（鸭子类型），rclpy 依赖由调用方
  （编排层节点）持有。

线程模型:
  无内部状态，线程安全性取决于被包装的 rclpy 时钟（其 now() 线程安全）。
"""
from __future__ import annotations

from peach_common_py.clock import Clock


class RclpyClockAdapter(Clock):
    """
    包装 rclpy 节点时钟的 Clock 实现（ROS 运行时注入用）.

    用法::

        clock = RclpyClockAdapter(node.get_clock())
        estimator.update(clock.now())

    生命周期：与被包装时钟同寿（弱语义，仅持引用）。线程安全：委托
    rclpy 时钟（其 now() 线程安全）。
    """

    def __init__(self, rclpy_clock):
        """
        包装一个 rclpy.clock.Clock（或同构鸭子类型）.

        入参须带 now() 方法，其返回值带 .nanoseconds 属性。
        """
        self._clock = rclpy_clock

    def now(self) -> float:
        """返回节点时钟当前秒（nanoseconds × 1e-9，与现网写法一致）."""
        return self._clock.now().nanoseconds * 1e-9
