"""
3D multi-target tracker.

Constant-velocity Kalman filter per target, Mahalanobis gating on the innovation covariance
(P_pred + R), Hungarian assignment with a category constraint.

Confirmation needs `confirm_hits` consecutive associations (a tentative track that misses one
update is dropped); confirmed tracks coast until `ttl_s` seconds without an update. Time is the
caller's stamp (image stamp), never frame counts.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import linear_sum_assignment

# 1-sigma prior on the velocity of a newly born track [m/s]; bags are quasi-static.
_INIT_VEL_SIGMA = 0.2
_BIG_COST = 1e9


@dataclass
class Detection3D:
    position: np.ndarray  # (3,) [m]
    cov: np.ndarray  # (3, 3) [m^2]
    category: int
    payload: object = None


@dataclass
class Track:
    track_id: str
    position: np.ndarray
    velocity: np.ndarray
    cov: np.ndarray  # (3, 3) position covariance
    category: int
    hits: int
    confirmed: bool
    last_update_s: float
    payload: object = None


class _KF:
    __slots__ = ('x', 'P', 'stamp_s')

    def __init__(self, position: np.ndarray, cov: np.ndarray, stamp_s: float):
        self.x = np.concatenate([position, np.zeros(3)])
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = cov
        self.P[3:, 3:] = _INIT_VEL_SIGMA ** 2 * np.eye(3)
        self.stamp_s = stamp_s

    def predicted(self, stamp_s: float, accel_sigma: float) -> tuple[np.ndarray, np.ndarray]:
        dt = max(stamp_s - self.stamp_s, 0.0)
        F = np.eye(6)
        F[:3, 3:] = dt * np.eye(3)
        q = accel_sigma ** 2
        Q = np.zeros((6, 6))
        Q[:3, :3] = 0.25 * dt ** 4 * q * np.eye(3)
        Q[:3, 3:] = Q[3:, :3] = 0.5 * dt ** 3 * q * np.eye(3)
        Q[3:, 3:] = dt ** 2 * q * np.eye(3)
        return F @ self.x, F @ self.P @ F.T + Q

    def update(self, x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray,
               stamp_s: float) -> None:
        S = P[:3, :3] + R
        K = P[:, :3] @ np.linalg.inv(S)
        self.x = x + K @ (z - x[:3])
        IKH = np.eye(6)
        IKH[:, :3] -= K
        # Joseph form keeps P symmetric positive semi-definite.
        self.P = IKH @ P @ IKH.T + K @ R @ K.T
        self.stamp_s = stamp_s


def _snapshot(track: Track) -> Track:
    """Copy with private arrays; the payload is shared (it may hold large masks)."""
    return replace(track, position=track.position.copy(), velocity=track.velocity.copy(),
                   cov=track.cov.copy())


def _sanitize_cov(cov: np.ndarray) -> np.ndarray:
    c = np.asarray(cov, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(c)):
        raise ValueError('detection covariance must be finite')
    c = 0.5 * (c + c.T)
    return c + 1e-10 * np.eye(3)


class Tracker:
    def __init__(self, confirm_hits: int = 3, ttl_s: float = 3.0, gate_chi2: float = 11.34,
                 process_accel_sigma: float = 0.5, id_prefix: str = 'target_'):
        if confirm_hits < 1 or ttl_s <= 0.0 or gate_chi2 <= 0.0 or process_accel_sigma < 0.0:
            raise ValueError('invalid tracker parameters')
        self._confirm_hits = int(confirm_hits)
        self._ttl_s = float(ttl_s)
        self._gate = float(gate_chi2)
        self._accel = float(process_accel_sigma)
        self._prefix = id_prefix
        self._next_id = 1
        self._tracks: list[Track] = []
        self._filters: dict[str, _KF] = {}

    def reset(self) -> None:
        self._tracks = []
        self._filters = {}

    def tracks(self) -> list[Track]:
        return [_snapshot(t) for t in self._tracks]

    def update(self, stamp_s: float, detections: list[Detection3D]) -> list[Track]:
        """
        Associate one frame of detections; returns the tracks updated or born this frame.

        Each returned track carries the payload of the detection it absorbed.
        """
        stamp_s = float(stamp_s)
        expired = {t.track_id for t in self._tracks
                   if t.confirmed and stamp_s - t.last_update_s > self._ttl_s}
        for tid in expired:
            del self._filters[tid]
        self._tracks = [t for t in self._tracks if t.track_id not in expired]
        preds = [self._filters[t.track_id].predicted(stamp_s, self._accel)
                 for t in self._tracks]
        dets = [(np.asarray(d.position, dtype=np.float64).reshape(3), _sanitize_cov(d.cov), d)
                for d in detections]
        n_t, n_d = len(self._tracks), len(dets)
        cost = np.full((n_t, n_d), _BIG_COST)
        for i, (track, (x, P)) in enumerate(zip(self._tracks, preds)):
            for j, (z, R, det) in enumerate(dets):
                if int(det.category) != track.category:
                    continue
                y = z - x[:3]
                S = P[:3, :3] + R
                try:
                    d2 = float(y @ np.linalg.solve(S, y))
                except np.linalg.LinAlgError:
                    continue
                if d2 <= self._gate:
                    cost[i, j] = d2
        matched_t: set[int] = set()
        matched_d: set[int] = set()
        updated: list[Track] = []
        if n_t and n_d:
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] >= _BIG_COST:
                    continue
                track = self._tracks[i]
                z, R, det = dets[j]
                x, P = preds[i]
                kf = self._filters[track.track_id]
                kf.update(x, P, z, R, stamp_s)
                track.hits += 1
                track.confirmed = track.confirmed or track.hits >= self._confirm_hits
                track.last_update_s = stamp_s
                track.payload = det.payload
                self._sync(track)
                matched_t.add(i)
                matched_d.add(j)
                updated.append(track)
        survivors = []
        for i, track in enumerate(self._tracks):
            if i in matched_t or track.confirmed:
                survivors.append(track)
            else:
                del self._filters[track.track_id]
        self._tracks = survivors
        for j, (z, R, det) in enumerate(dets):
            if j in matched_d:
                continue
            tid = f'{self._prefix}{self._next_id}'
            self._next_id += 1
            self._filters[tid] = _KF(z, R, stamp_s)
            track = Track(track_id=tid, position=z.copy(), velocity=np.zeros(3), cov=R.copy(),
                          category=int(det.category), hits=1,
                          confirmed=self._confirm_hits <= 1, last_update_s=stamp_s,
                          payload=det.payload)
            self._tracks.append(track)
            updated.append(track)
        return [_snapshot(t) for t in updated]

    def _sync(self, track: Track) -> None:
        kf = self._filters[track.track_id]
        track.position = kf.x[:3].copy()
        track.velocity = kf.x[3:].copy()
        track.cov = kf.P[:3, :3].copy()
