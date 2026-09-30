"""
Arm stationarity at an image stamp from a /joint_states buffer (no ROS).

The stop-and-go camera cadence only integrates frames taken while the arm is still. Velocity
is the reported joint velocity when present, else a finite difference of positions; it is
linearly interpolated at the image stamp and checked over a trailing window.
"""
from __future__ import annotations

from collections import deque
import enum
from typing import Sequence

import numpy as np

JOINT_NAMES = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint',
               'wrist2_joint', 'wrist3_joint')


class Motion(enum.Enum):
    STATIONARY = 'stationary'
    MOVING = 'moving'
    UNKNOWN = 'unknown'  # no joint data bracketing the stamp


class JointMotionBuffer:
    def __init__(self, horizon_s: float, joint_names: Sequence[str] = JOINT_NAMES) -> None:
        if horizon_s <= 0.0:
            raise ValueError('horizon_s must be > 0')
        self._names = tuple(joint_names)
        self._horizon = float(horizon_s)
        self._t: deque[float] = deque()
        self._q: deque[np.ndarray] = deque()
        self._v: deque[np.ndarray] = deque()

    def __len__(self) -> int:
        return len(self._t)

    def clear(self) -> None:
        self._t.clear()
        self._q.clear()
        self._v.clear()

    def add(self, stamp_s: float, names: Sequence[str], positions: Sequence[float],
            velocities: Sequence[float] = ()) -> bool:
        """Append one JointState sample; False when joints are missing or the stamp regresses."""
        index = {n: i for i, n in enumerate(names)}
        if any(n not in index for n in self._names) or len(positions) != len(names):
            return False
        if self._t and stamp_s <= self._t[-1]:
            return False
        q = np.array([positions[index[n]] for n in self._names], dtype=np.float64)
        if len(velocities) == len(names):
            v = np.array([velocities[index[n]] for n in self._names], dtype=np.float64)
        else:
            v = np.full(len(self._names), np.nan)
        self._t.append(float(stamp_s))
        self._q.append(q)
        self._v.append(v)
        while self._t and self._t[0] < stamp_s - self._horizon:
            self._t.popleft()
            self._q.popleft()
            self._v.popleft()
        return True

    def _velocities(self) -> tuple[np.ndarray, np.ndarray]:
        t = np.fromiter(self._t, dtype=np.float64)
        v = np.array(self._v, dtype=np.float64).reshape(t.size, len(self._names))
        if t.size >= 2:
            fd = np.gradient(np.array(self._q), t, axis=0)
            v = np.where(np.isfinite(v), v, fd)
        return t, v

    def speed_at(self, stamp_s: float, max_gap_s: float) -> float | None:
        """Max |joint velocity| [rad/s] at stamp_s; None when no sample lies within max_gap_s."""
        if not self._t:
            return None
        t, v = self._velocities()
        speeds = self._interp(t, v, stamp_s, max_gap_s)
        return None if speeds is None else float(np.max(np.abs(speeds)))

    @staticmethod
    def _interp(t: np.ndarray, v: np.ndarray, stamp_s: float,
                max_gap_s: float) -> np.ndarray | None:
        i = int(np.searchsorted(t, stamp_s))
        if i == 0 or i == t.size:
            j = 0 if i == 0 else t.size - 1
            if abs(t[j] - stamp_s) > max_gap_s:
                return None
            row = v[j]
        else:
            t0, t1 = t[i - 1], t[i]
            if t1 - t0 > 2.0 * max_gap_s:
                return None
            a = (stamp_s - t0) / (t1 - t0)
            row = (1.0 - a) * v[i - 1] + a * v[i]
        return row if np.all(np.isfinite(row)) else None

    def classify(self, stamp_s: float, threshold_rad_s: float, window_s: float,
                 max_gap_s: float) -> Motion:
        """STATIONARY when every joint speed in [stamp - window, stamp] is <= threshold."""
        if not self._t:
            return Motion.UNKNOWN
        t, v = self._velocities()
        at = self._interp(t, v, stamp_s, max_gap_s)
        if at is None:
            return Motion.UNKNOWN
        peak = float(np.max(np.abs(at)))
        inside = (t >= stamp_s - window_s) & (t <= stamp_s)
        if np.any(inside):
            win = v[inside]
            if not np.all(np.isfinite(win)):
                return Motion.UNKNOWN
            peak = max(peak, float(np.max(np.abs(win))))
        return Motion.STATIONARY if peak <= threshold_rad_s else Motion.MOVING
