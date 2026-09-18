"""执行窗口看门狗：robot_status 新鲜度、模型有效期、取消态."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WatchdogSample:
    """单调时钟样本；与 ROS stamp 分离."""

    now_mono_s: float
    robot_status_mono_s: float = 0.0
    robot_status_seen: bool = False
    model_valid_until_s: float = 0.0
    cancel_requested: bool = False
    timeout_s: float = 0.5


def robot_status_fresh(sample: WatchdogSample) -> bool:
    """柜侧状态流在超时窗内（未见/超龄=False）."""
    if not sample.robot_status_seen:
        return False
    return (sample.now_mono_s - sample.robot_status_mono_s) <= sample.timeout_s


def model_still_valid(sample: WatchdogSample) -> bool:
    """模型有效期未过（valid_until=0 视为未提供=False）."""
    return sample.model_valid_until_s > 0.0 and sample.now_mono_s <= sample.model_valid_until_s


def execution_window_ok(sample: WatchdogSample) -> tuple[bool, str]:
    """阻塞 execute() 窗口的最后一环；失败走取消 + RobotMoveStop，不得称 e-stop."""
    if sample.cancel_requested:
        return False, 'cancel_requested'
    if not robot_status_fresh(sample):
        return False, 'robot_status_stale'
    if sample.model_valid_until_s > 0.0 and not model_still_valid(sample):
        return False, 'model_expired'
    return True, 'ok'
