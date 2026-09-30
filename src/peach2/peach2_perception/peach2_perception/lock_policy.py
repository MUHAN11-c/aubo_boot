"""
Scene target-set lock and scene_epoch rules (no ROS).

begin_scene() starts a new epoch (+1) and records its start stamp; frames stamped before it
belong to the previous scene and are refused (the old stack could fold a pre-BeginScene frame
into the new scene). The confirmed target set locks when all of these hold:
  * >= min_stationary_frames consecutive stationary frames,
  * the confirmed-ID set unchanged (and the arm stationary) for >= stable_s,
  * no tentative tracks pending confirmation, and the set is non-empty.
After max_collect_s the current set locks regardless (possibly empty). Once locked the set is
frozen until the next begin_scene(); observations keep flowing either way.
"""
from __future__ import annotations

from dataclasses import dataclass
import enum
from typing import Iterable


class LockState(enum.Enum):
    IDLE = 'idle'  # no scene started yet
    COLLECTING = 'collecting'
    LOCKED = 'locked'


@dataclass(frozen=True)
class LockStatus:
    state: LockState
    scene_epoch: int
    locked_ids: frozenset[str]
    reason: str  # '', 'stable' or 'timeout'
    stationary_frames: int
    stable_for_s: float

    @property
    def locked(self) -> bool:
        return self.state is LockState.LOCKED


class LockPolicy:
    def __init__(self, min_stationary_frames: int, stable_s: float, max_collect_s: float) -> None:
        if min_stationary_frames < 1 or stable_s < 0.0 or max_collect_s <= 0.0:
            raise ValueError('invalid lock parameters')
        self._n_min = int(min_stationary_frames)
        self._stable_s = float(stable_s)
        self._max_s = float(max_collect_s)
        self._epoch = 0
        self._state = LockState.IDLE
        self._start_s = float('-inf')
        self._first_s: float | None = None
        self._last_change_s = 0.0
        self._ids: frozenset[str] = frozenset()
        self._n_stationary = 0
        self._reason = ''

    @property
    def scene_epoch(self) -> int:
        return self._epoch

    @property
    def state(self) -> LockState:
        return self._state

    def begin_scene(self, start_stamp_s: float) -> int:
        self._epoch += 1
        self._state = LockState.COLLECTING
        self._start_s = float(start_stamp_s)
        self._first_s = None
        self._ids = frozenset()
        self._n_stationary = 0
        self._reason = ''
        return self._epoch

    def accepts(self, stamp_s: float) -> bool:
        return stamp_s >= self._start_s

    def status(self, stamp_s: float | None = None) -> LockStatus:
        stable = 0.0
        if stamp_s is not None and self._first_s is not None:
            stable = max(stamp_s - self._last_change_s, 0.0)
        return LockStatus(self._state, self._epoch, self._ids, self._reason,
                          self._n_stationary, stable)

    def update(self, stamp_s: float, stationary: bool, confirmed_ids: Iterable[str],
               pending: int) -> LockStatus:
        """Feed one processed frame (stamp >= scene start); returns the lock status after it."""
        if self._state is not LockState.COLLECTING:
            return self.status(stamp_s)
        ids = frozenset(confirmed_ids)
        if self._first_s is None:
            self._first_s = stamp_s
            self._last_change_s = stamp_s
            self._ids = ids
        if ids != self._ids or not stationary:
            self._last_change_s = stamp_s
        self._ids = ids
        self._n_stationary = self._n_stationary + 1 if stationary else 0
        stable_for = stamp_s - self._last_change_s
        if (self._n_stationary >= self._n_min and stable_for >= self._stable_s
                and pending == 0 and ids):
            self._state, self._reason = LockState.LOCKED, 'stable'
        elif stamp_s - self._first_s >= self._max_s:
            self._state, self._reason = LockState.LOCKED, 'timeout'
        return self.status(stamp_s)
