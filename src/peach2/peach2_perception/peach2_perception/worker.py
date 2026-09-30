"""Capacity-1, drop-oldest hand-off between ROS callbacks and the inference thread (no ROS)."""
from __future__ import annotations

import threading
from typing import Generic, TypeVar

T = TypeVar('T')


class LatestSlot(Generic[T]):
    """
    Holds at most one pending item; put() replaces an unconsumed item (counted as dropped).

    Callbacks never wait on inference: a slow model only means older frames are skipped.
    """

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item: T | None = None
        self._closed = False
        self._dropped = 0

    @property
    def dropped(self) -> int:
        with self._cond:
            return self._dropped

    def put(self, item: T) -> bool:
        """Offer an item; True when it displaced a pending one. Ignored after close()."""
        with self._cond:
            if self._closed:
                return False
            displaced = self._item is not None
            if displaced:
                self._dropped += 1
            self._item = item
            self._cond.notify()
            return displaced

    def get(self, timeout: float | None = None) -> T | None:
        """Take the pending item; None on timeout or once closed."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._item is not None or self._closed, timeout):
                return None
            if self._closed:
                return None
            item, self._item = self._item, None
            return item

    def clear(self) -> None:
        with self._cond:
            self._item = None

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._item = None
            self._cond.notify_all()

    def reopen(self) -> None:
        with self._cond:
            self._closed = False
