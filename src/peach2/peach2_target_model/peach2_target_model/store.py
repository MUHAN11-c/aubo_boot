"""
Per-target observation history, viewpoint segmentation and model revisions (no ROS).

Consecutive frames from one camera pose are strongly correlated, so they are aggregated into
one ViewSample per viewpoint before fusion; otherwise the MAD term would shrink with frame
count and n_views would reach max_views without the camera ever moving. A new viewpoint starts
on a time gap > view_gap_s, or when TargetObservation.camera_pose moved from the first frame of
the current viewpoint by more than view_translation_change_m or view_rotation_change_deg.

Swing only uses frames with swing_known = true (their reported amplitude and their bottom
positions for the sine fit); with no such frame the swing is unknown (NaN).

Threading: ModelStore is not thread-safe; the node holds one lock around every call and runs
fuse_job() (the expensive part) outside of it.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np
from peach2_core.fusion import fuse_views, FusedModel, ViewSample
from peach2_core.swing import estimate_swing
from peach2_core.types import Landmark, Landmarks3D, unit

from .decision import convergence_failures, ModelRecord, Remeasure
from .params import TargetModelParams

_MIN_SWING_SAMPLES = 8


@dataclass
class Frame:
    stamp_s: float
    landmarks: Landmarks3D
    camera_distance_m: float
    camera_position: np.ndarray
    camera_quat_xyzw: np.ndarray
    swing_known: bool
    swing_amplitude_m: float


@dataclass
class FusionJob:
    target_id: str
    epoch: int
    views: list[list[Frame]]
    last_obs_s: float


@dataclass
class FusionOutput:
    target_id: str
    epoch: int
    fused: FusedModel
    swing_amplitude_m: float
    last_obs_s: float
    unconverged: list[str]
    fruit_top_offset_m: float = float('nan')


def _landmark(valid: bool, position, cov, confidence: float) -> Landmark | str:
    if not valid:
        return Landmark(False, np.full(3, np.nan), np.full((3, 3), np.nan), 0.0)
    p = np.asarray(position, dtype=np.float64).reshape(3)
    c = np.asarray(cov, dtype=np.float64).reshape(3, 3)
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(c))):
        return 'non_finite'
    c = 0.5 * (c + c.T)
    if np.linalg.eigvalsh(c).min() <= 0.0:
        return 'bad_covariance'
    return Landmark(True, p, c, float(confidence))


def _camera_pose(camera_pose: tuple) -> tuple[np.ndarray, np.ndarray] | None:
    position, quat = camera_pose
    p = np.asarray(position, dtype=np.float64).reshape(3)
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
        return None
    norm = float(np.linalg.norm(q))
    # An all-zero quaternion is the IDL default: the producer did not fill camera_pose.
    if not 0.9 <= norm <= 1.1:
        return None
    return p, q / norm


def rotation_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Angle of the relative rotation between two unit quaternions (sign-invariant)."""
    c = min(abs(float(np.dot(q1, q2))), 1.0)
    return float(np.degrees(2.0 * math.acos(c)))


def build_frame(stamp_s: float, bottom: tuple, neck: tuple, tie: tuple, axis,
                diameter95_m: float, camera_distance_m: float, camera_pose: tuple,
                swing_known: bool, swing_amplitude_m: float) -> Frame | str:
    """
    Validate one observation; landmarks are (valid, position, cov 3x3/9, confidence).

    camera_pose is (position xyz, quaternion xyzw) of the camera optical frame in base_link.
    Returns a Frame, or a stable reject reason for diagnostics.
    """
    pose = _camera_pose(camera_pose)
    if pose is None:
        return 'camera_pose_invalid'
    lms = [_landmark(*lm) for lm in (bottom, neck, tie)]
    for lm in lms[:2]:
        if isinstance(lm, str):
            return lm
    b, n = lms[0], lms[1]
    t = lms[2] if isinstance(lms[2], Landmark) else _landmark(False, None, None, 0.0)
    if not (b.valid and n.valid):
        return 'landmark_invalid'
    line = n.position - b.position
    a = unit(np.asarray(axis, dtype=np.float64).reshape(3))
    if a is None:
        a = unit(line)
    if a is None:
        return 'axis_invalid'
    if float(line @ a) < 0.0:
        a = -a
    length = float(line @ a)
    if length <= 0.0:
        return 'length_nonpositive'
    if not (math.isfinite(diameter95_m) and diameter95_m >= 0.0):
        return 'diameter_invalid'
    known = bool(swing_known) and math.isfinite(swing_amplitude_m) and swing_amplitude_m >= 0.0
    lm3 = Landmarks3D(ok=True, bottom=b, neck=n, tie=t, axis=a, d95_m=float(diameter95_m),
                      length_m=length, flags=[])
    return Frame(float(stamp_s), lm3, float(camera_distance_m), pose[0], pose[1], known,
                 float(swing_amplitude_m) if known else float('nan'))


def _representative_cov(covs: list[np.ndarray]) -> np.ndarray:
    traces = np.array([np.trace(c) for c in covs])
    return covs[int(np.argsort(traces)[len(traces) // 2])].copy()


def _aggregate_landmark(lms: list[Landmark]) -> Landmark:
    good = [lm for lm in lms if lm.valid]
    if 2 * len(good) <= len(lms) or not good:
        return Landmark(False, np.full(3, np.nan), np.full((3, 3), np.nan), 0.0)
    pos = np.median(np.array([lm.position for lm in good]), axis=0)
    return Landmark(True, pos, _representative_cov([lm.cov for lm in good]),
                    float(np.mean([lm.confidence for lm in good])))


def aggregate_view(frames: list[Frame]) -> ViewSample:
    """
    One ViewSample per viewpoint: component-wise median positions, median-trace covariance.

    The covariance is a single-frame one on purpose: frames of one view are not independent.
    """
    lms = [f.landmarks for f in frames]
    ref = lms[-1].axis
    axes = np.array([a if a @ ref >= 0.0 else -a for a in (lm.axis for lm in lms)])
    axis = unit(axes.sum(axis=0))
    bottom = _aggregate_landmark([lm.bottom for lm in lms])
    neck = _aggregate_landmark([lm.neck for lm in lms])
    tie = _aggregate_landmark([lm.tie for lm in lms])
    ok = bottom.valid and neck.valid and axis is not None
    length = float((neck.position - bottom.position) @ axis) if ok else float('nan')
    lm3 = Landmarks3D(ok=ok, bottom=bottom, neck=neck, tie=tie,
                      axis=axis if axis is not None else np.full(3, np.nan),
                      d95_m=float(np.median([lm.d95_m for lm in lms])), length_m=length,
                      flags=[])
    return ViewSample(landmarks=lm3, stamp_s=frames[-1].stamp_s,
                      camera_distance_m=float(np.median([f.camera_distance_m for f in frames])))


def swing_from_views(views: list[list[Frame]], window_s: float) -> float:
    """
    Swing over the last window_s: max(reported amplitudes, sine fit on bottom positions).

    Only swing_known frames count. Reported amplitudes are taken from every viewpoint in the
    window; the sine fit only uses the latest viewpoint (stationary camera). NaN when no
    swing_known frame is in the window.
    """
    known = [f for view in views for f in view if f.swing_known]
    if not known:
        return float('nan')
    t_end = max(f.stamp_s for f in known)
    reported = max(f.swing_amplitude_m for f in known if t_end - f.stamp_s <= window_s)
    latest = [f for f in (views[-1] if views else [])
              if f.swing_known and t_end - f.stamp_s <= window_s]
    if len(latest) < _MIN_SWING_SAMPLES:
        return reported
    t = np.array([f.stamp_s for f in latest])
    p = np.array([f.landmarks.bottom.position for f in latest])
    amp, _ = estimate_swing(t, p)
    return max(reported, float(amp)) if math.isfinite(amp) else reported


def fuse_job(job: FusionJob, params: TargetModelParams) -> FusionOutput:
    samples = [aggregate_view(frames) for frames in job.views if frames]
    fused = fuse_views(samples, 1)
    swing = swing_from_views(job.views, params.swing_window_s)
    # No TargetObservation field carries fruit evidence yet (fruit keypoint / fruit mask), so
    # the offset stays unknown and the decision falls back to the length - d95 proxy.
    return FusionOutput(job.target_id, job.epoch, fused, swing, job.last_obs_s,
                        convergence_failures(fused, params), fruit_top_offset_m=float('nan'))


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(float(a @ b), -1.0, 1.0))))


def _differs(x: float, y: float, tol: float) -> bool:
    """NaN-aware scalar change test (unknown <-> known counts as a change)."""
    if math.isnan(x) or math.isnan(y):
        return math.isnan(x) != math.isnan(y)
    return abs(x - y) > tol


def significant_change(old: ModelRecord, new: FusionOutput, params: TargetModelParams) -> bool:
    """Whether a fusion result changes the published content enough to bump model_revision."""
    a, b = old.fused, new.fused
    if a.ok != b.ok or a.n_views != b.n_views or old.converged != (not new.unconverged):
        return True
    if not b.ok:
        return False
    tol = params.revision_position_tol_m
    for pa, pb in ((a.bottom, b.bottom), (a.neck, b.neck)):
        if np.linalg.norm(pa - pb) > tol:
            return True
    if (a.tie is None) != (b.tie is None):
        return True
    if a.tie is not None and np.linalg.norm(a.tie - b.tie) > tol:
        return True
    scalars = ((a.d95_m, b.d95_m), (a.length_m, b.length_m),
               (a.sigma_lateral95_m, b.sigma_lateral95_m),
               (a.sigma_axial95_m, b.sigma_axial95_m),
               (old.swing_amplitude_m, new.swing_amplitude_m),
               (old.fruit_top_offset_m, new.fruit_top_offset_m))
    if any(_differs(x, y, tol) for x, y in scalars):
        return True
    ang_tol = params.revision_angle_tol_deg
    return (_angle_deg(a.axis, b.axis) > ang_tol
            or abs(a.theta95_deg - b.theta95_deg) > ang_tol)


class _History:
    def __init__(self) -> None:
        self.views: list[list[Frame]] = []
        self.last_obs_s = -math.inf
        self.dirty = False
        self.remeasure = Remeasure.NONE


class ModelStore:
    def __init__(self, params: TargetModelParams) -> None:
        self._p = params
        self.epoch: int | None = None
        self._hist: dict[str, _History] = {}
        self._records: dict[str, ModelRecord] = {}
        # Never reset across epochs so (target_id, model_revision) stays unique for the run.
        self._revisions: dict[str, int] = {}

    def set_epoch(self, epoch: int) -> bool:
        """Adopt `epoch`; a changed epoch drops every history and model. True if cleared."""
        if self.epoch == epoch:
            return False
        cleared = self.epoch is not None and bool(self._hist or self._records)
        self.epoch = epoch
        self._hist.clear()
        self._records.clear()
        return cleared

    def add_frame(self, target_id: str, frame: Frame) -> None:
        h = self._hist.setdefault(target_id, _History())
        p = self._p
        last = h.views[-1] if h.views else None
        same_view = (last is not None
                     and frame.stamp_s - last[-1].stamp_s <= p.view_gap_s
                     and np.linalg.norm(frame.camera_position - last[0].camera_position)
                     <= p.view_translation_change_m
                     and rotation_angle_deg(frame.camera_quat_xyzw, last[0].camera_quat_xyzw)
                     <= p.view_rotation_change_deg)
        if same_view:
            last.append(frame)
            if len(last) > p.max_frames_per_view:
                del last[0]
        else:
            h.views.append([frame])
            if len(h.views) > p.max_views_per_target:
                del h.views[0]
        h.last_obs_s = max(h.last_obs_s, frame.stamp_s)
        h.dirty = True

    def take_jobs(self) -> list[FusionJob]:
        """Snapshot every target with new frames since the last call and mark it clean."""
        jobs = []
        for tid, h in self._hist.items():
            if not h.dirty:
                continue
            h.dirty = False
            jobs.append(FusionJob(tid, self.epoch, [list(v) for v in h.views], h.last_obs_s))
        return jobs

    def commit(self, outputs: list[FusionOutput]) -> list[str]:
        """Store fusion results; returns ids whose model_revision was bumped."""
        changed = []
        for out in outputs:
            if out.epoch != self.epoch or out.target_id not in self._hist:
                continue
            old = self._records.get(out.target_id)
            if old is not None and not significant_change(old, out, self._p):
                old.last_obs_s = max(old.last_obs_s, out.last_obs_s)
                continue
            rev = self._revisions.get(out.target_id, 0) + 1
            self._revisions[out.target_id] = rev
            self._records[out.target_id] = ModelRecord(
                target_id=out.target_id, revision=rev, fused=out.fused,
                swing_amplitude_m=out.swing_amplitude_m, last_obs_s=out.last_obs_s,
                converged=not out.unconverged, unconverged=list(out.unconverged),
                fruit_top_offset_m=out.fruit_top_offset_m)
            changed.append(out.target_id)
        return changed

    def record(self, target_id: str) -> ModelRecord | None:
        rec = self._records.get(target_id)
        if rec is None:
            return None
        return replace(rec, unconverged=list(rec.unconverged),
                       remeasure=self._hist[target_id].remeasure)

    def records(self) -> list[ModelRecord]:
        return [self.record(tid) for tid in sorted(self._records)]

    def frames_since(self, target_id: str, t0_s: float) -> list[Frame]:
        h = self._hist.get(target_id)
        if h is None:
            return []
        return [f for view in h.views for f in view if f.stamp_s >= t0_s]

    def set_remeasure(self, target_id: str, state: Remeasure) -> None:
        h = self._hist.get(target_id)
        if h is None:
            return
        if h.remeasure is Remeasure.MISMATCH and state is not Remeasure.MISMATCH:
            return
        h.remeasure = state
