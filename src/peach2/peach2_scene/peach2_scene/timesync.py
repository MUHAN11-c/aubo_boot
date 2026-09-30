"""
Joint-state history with interpolation at an image stamp (zero ROS).

Speeds come from the finite difference of the bracketing samples, not from
JointState.velocity (which may be empty).
"""
from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class JointSample:
    stamp_s: float
    positions: dict[str, float]     # [rad]
    speeds: dict[str, float]        # [rad/s], absolute

    @property
    def max_speed(self) -> float:
        return max(self.speeds.values(), default=0.0)


class JointStateBuffer:
    def __init__(self, window_s: float) -> None:
        if window_s <= 0.0:
            raise ValueError('window_s must be > 0')
        self._window_s = float(window_s)
        self._stamps: deque[float] = deque()
        self._positions: deque[dict[str, float]] = deque()

    def add(self, stamp_s: float, names: list[str], positions: list[float]) -> None:
        """Append one sample; out-of-order samples are dropped (stamps must increase)."""
        if len(names) != len(positions) or not names:
            return
        if self._stamps and stamp_s <= self._stamps[-1]:
            return
        self._stamps.append(float(stamp_s))
        self._positions.append({n: float(p) for n, p in zip(names, positions)})
        while self._stamps and self._stamps[0] < stamp_s - self._window_s:
            self._stamps.popleft()
            self._positions.popleft()

    def latest_stamp(self) -> float | None:
        return self._stamps[-1] if self._stamps else None

    def __len__(self) -> int:
        return len(self._stamps)

    def sample(self, t_s: float, max_gap_s: float) -> JointSample | None:
        """
        Linear interpolation at t_s, or None when t_s is outside the history.

        Also None when the bracketing samples are more than max_gap_s apart (a joint_states gap
        means the arm pose at t_s is unknown, not constant).
        """
        stamps = self._stamps
        if len(stamps) < 2 or t_s < stamps[0] or t_s > stamps[-1]:
            return None
        k = bisect_left(stamps, t_s)
        if k == 0:
            k = 1
        t0, t1 = stamps[k - 1], stamps[k]
        if t1 - t0 > max_gap_s:
            return None
        q0, q1 = self._positions[k - 1], self._positions[k]
        a = (t_s - t0) / (t1 - t0)
        names = [n for n in q0 if n in q1]
        pos = {n: q0[n] + a * (q1[n] - q0[n]) for n in names}
        spd = {n: abs(q1[n] - q0[n]) / (t1 - t0) for n in names}
        return JointSample(t_s, pos, spd)
