# Copyright 2026 wjz
"""可配置背压策略的单线程有界 worker."""

from collections import deque
import threading
from typing import Callable, Generic, TypeVar


Item = TypeVar('Item')


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
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._queue or self._closing)
                if self._closing and (not self._drain or not self._queue):
                    return
                item = self._queue.popleft()
            self._process(item)
