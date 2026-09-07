from __future__ import annotations
"""采帧：门禁、帧栈、绑定防抖、自动采集。"""

import time
from typing import Callable, Optional


class BindSwitchHoldoff:
    """selected 切换防抖状态机（注入时钟；每消息 arbitrate 一次）."""

    # 动作枚举（字符串，节点按此分支；保持纯数据便于日志/测试断言）
    FOLLOW = 'follow'  # 直通跟随（无可毁会话 / 未偏离 / 无挂起）
    PEND = 'pend'      # 开始挂起（含改挂到另一个新 ID 重新记时）
    WAIT = 'wait'      # 挂起中未到期，维持旧绑定
    CANCEL = 'cancel'  # holdoff 内切回原 ID，取消挂起
    COMMIT = 'commit'  # 挂起到期，执行放弃重绑

    def __init__(self, holdoff_s: float = 2.0,
                 now: Callable[[], float] = time.monotonic):
        """
        注入挂起时长与时钟.

        Args:
            holdoff_s: 切换挂起时长 [s]；requested 须持续超过本时长才
                commit。0 等价于「下一条维持新 ID 的观测即到期」，近似
                旧版立即跟随行为。
            now: 单调时钟（秒），节点注入 RclpyClockAdapter.now（I3）.

        Returns
        -------
            无返回值（None）.

        """
        self.holdoff_s = float(holdoff_s)
        self._now = now
        self._pending_id: Optional[str] = None  # 挂起中的候选新 ID（含 ''）
        self._pending_since: float = 0.0        # 挂起起始时刻 [s]

    @property
    def pending_id(self) -> Optional[str]:
        """当前挂起的新 ID；None 表示无挂起（诊断/测试观察口）."""
        return self._pending_id

    def arbitrate(self, requested_id: str, bound_id: str,
                  session_active: bool) -> str:
        """
        对一条观测的 selected_target_id 做防抖仲裁.

        Args:
            requested_id: 本帧感知全局计划 selected_target_id（可为 ''）.
            bound_id: 节点当前跟随的目标 ID（_preferred_target_id；空串
                表示尚未跟随任何目标）.
            session_active: 是否有进行中会话（collector.state != IDLE）；
                False 时无会话可毁，一切变化立即 follow 并清挂起.

        Returns
        -------
            动作：FOLLOW / PEND / WAIT / CANCEL / COMMIT（见类常量）.

        """
        if requested_id == bound_id:
            # 切回（或本就未偏离）绑定 ID：取消挂起；无挂起即常规直通
            if self._pending_id is None:
                return self.FOLLOW
            self._pending_id = None
            return self.CANCEL if session_active else self.FOLLOW
        if not session_active or not bound_id:
            # IDLE（无会话可毁）或从未跟随任何目标：立即跟随，不挂起
            self._pending_id = None
            return self.FOLLOW
        # requested 偏离绑定 ID 且有进行中会话：进入/维持挂起
        if self._pending_id != requested_id:
            # 新候选（含从某个挂起 ID 改挂到另一个）：重新记时
            self._pending_id = requested_id
            self._pending_since = self._now()
            return self.PEND
        if self._now() - self._pending_since >= self.holdoff_s:
            # 同一新 ID 持续超过挂起时长：到期，执行放弃重绑
            self._pending_id = None
            return self.COMMIT
        return self.WAIT
