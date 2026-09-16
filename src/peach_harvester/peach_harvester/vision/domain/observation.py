"""场景观测值对象（零 ROS）."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence


@dataclass(frozen=True)
class Observation:
    """一帧里的一颗候选：位置/轴/检测身份，不含 ROS Header."""

    target_id: str = ''
    class_id: int = 0
    position: Optional[Sequence[float]] = None
    axis: Optional[Sequence[float]] = None
    diameter_m: float = 0.0
    tracking_status: str = 'unknown'
    flags: tuple = field(default_factory=tuple)
