# Copyright 2026 wjz
"""可配置背压策略的单线程有界 worker."""

from collections import deque
import logging
import threading
import traceback
from typing import Callable, Generic, TypeVar


Item = TypeVar('Item')

# 纯核不能 import ROS，走 stdlib logging（print 会污染 stdout）
_logger = logging.getLogger(__name__)


class BoundedWorker(Generic[Item]):
    """在独立线程串行执行任务并显式统计丢弃项."""

    def __init__(
            self, process: Callable[[Item], None], *, capacity: int,
            drop_oldest: bool):
        """创建 worker；容量必须为正数."""
        if capacity < 1:
            raise ValueError('capacity 必须大于零')
        self._process = process
        self._capacity = capacity
        self._drop_oldest = drop_oldest
        self._queue = deque()
        self._condition = threading.Condition()
        self._closing = False
        self._drain = True
        self.dropped = 0
        self._thread = threading.Thread(
            target=self._run, name='bounded-worker', daemon=True)
        self._thread.start()

    def submit(self, item: Item) -> bool:
        """提交任务；满队列时按策略替换旧项或拒绝新项."""
        with self._condition:
            if self._closing:
                return False
            if len(self._queue) >= self._capacity:
                self.dropped += 1
                if not self._drop_oldest:
                    return False
                self._queue.popleft()
            self._queue.append(item)
            self._condition.notify()
            return True

    def close(self, *, drain: bool) -> None:
        """停止接收并等待线程；drain 决定是否处理剩余任务."""
        with self._condition:
            self._closing = True
            self._drain = drain
            if not drain:
                self.dropped += len(self._queue)
                self._queue.clear()
            self._condition.notify_all()
        self._thread.join()

    def _run(self) -> None:
        consecutive_errors = 0
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._queue or self._closing)
                if self._closing and (not self._drain or not self._queue):
                    return
                item = self._queue.popleft()
            # 任务异常不得杀 worker 线程（否则 submit 照常返回 True、节点静默
            # 无输出）：记错误日志（含 traceback）后继续处理后续任务；连续异常
            # 计数用于日志节流——第 1 次必打，之后每 10 次打一次
            try:
                self._process(item)
            except Exception:  # noqa: BLE001
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 10 == 0:
                    _logger.error(
                        'bounded-worker 任务处理异常（连续第 %d 次）:\n%s',
                        consecutive_errors, traceback.format_exc())
            else:
                consecutive_errors = 0
