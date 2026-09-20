"""
QoS profile factories; rclpy is imported lazily inside each function.

模块顶层零 ROS 依赖（纯核环境可 import）；调用工厂时才要求 rclpy：

- ``latched()``：RELIABLE + TRANSIENT_LOCAL + depth 1（晚订户取最后一帧，
  如 ``HarvestState`` / ``GraspDecision``）。
- ``stream(depth=10)`` 与 ``reliable(depth=10)``：RELIABLE + VOLATILE +
  KEEP_LAST(depth)，命令 / 状态流用；``reliable`` 是 ``stream`` 的显式别名。
"""

from __future__ import annotations

from typing import Any


def latched() -> Any:
    """RELIABLE + TRANSIENT_LOCAL + depth 1（晚到订阅者拿最后一帧）."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1)


def stream(depth: int = 10) -> Any:
    """RELIABLE + VOLATILE + KEEP_LAST(depth)（命令 / 状态流）."""
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth)


def reliable(depth: int = 10) -> Any:
    """``stream`` 的显式别名（同 profile，名字表意 RELIABLE）."""
    return stream(depth=depth)
