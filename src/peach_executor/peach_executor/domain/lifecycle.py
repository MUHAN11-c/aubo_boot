"""Lifecycle 拆除收敛与心跳看门狗（bond 等价，零 ROS）."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class DeactivatePlan:
    """deactivate：拒新 goal → 取消在途 → 锁外有截止等待 → 清旧许可."""

    refuse_new_goals: bool = False
    cancel_inflight: bool = False
    wait_deadline_s: float = 0.0
    clear_permits: bool = False
    recovery_required: bool = False


def plan_deactivate(now_s: float, deadline_s: float, inflight: bool) -> DeactivatePlan:
    """停不稳则保留 recovery_required."""
    remaining = max(0.0, deadline_s - now_s)
    return DeactivatePlan(
        refuse_new_goals=True,
        cancel_inflight=True,
        wait_deadline_s=remaining,
        clear_permits=not inflight or remaining <= 0.0,
        recovery_required=inflight and remaining <= 0.0,
    )


@dataclass
class HeartbeatWatchdog:
    """进程心跳：超时则视为托管节点丢失（Nav2 bond 等价）."""

    timeout_s: float = 4.0
    last_beat: dict = field(default_factory=dict)

    def beat(self, name: str, now_s: float) -> None:
        self.last_beat[name] = now_s

    def missing(self, names, now_s: float) -> list:
        lost = []
        for name in names:
            last = self.last_beat.get(name)
            if last is None or (now_s - last) > self.timeout_s:
                lost.append(name)
        return lost


# 与 peach_interfaces/srv/ManageLifecycleNodes.srv 常量同值（纯核不 import IDL）。
CMD_STARTUP = 0
CMD_PAUSE = 1
CMD_RESUME = 2
CMD_RESET = 3
CMD_SHUTDOWN = 4


def watchdog_armed_after(command: int, success: bool) -> bool:
    """STARTUP/RESUME/RESET 成功才武装；PAUSE/SHUTDOWN 或失败则撤防."""
    if command in (CMD_STARTUP, CMD_RESUME, CMD_RESET):
        return bool(success)
    return False
