"""
Per-frame instance -> TargetObservation fields (no ROS).

measure(): stateless per-frame geometry. For a bag with a mask:
  * 2D landmarks from the mask, gravity = base -Z projected into the image at the bag,
  * 3D landmarks from the mask's confident depth points, transformed to base_link with the TF
    at the image stamp (covariances therefore come out in base_link),
  * 2D/3D cross-check: a 3D landmark whose base_link position is > cross_check_tol_m off the
    ray of its 2D pixel (or in front of / too far behind the surface seen along that ray) is
    marked invalid and flagged; fusion never sees it as valid.
  * Tracking anchor = 3D bottom (fallbacks: 2D bottom back-projected, then the box centre).
associate(): tracker update (image-stamp time, never frame counts) and swing estimation over
the stationary window from the raw 3D-bottom anchors of each track.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np
from peach2_core.depth import backproject, depth_sigma_m, mask_points
from peach2_core.landmarks2d import landmarks_from_mask
from peach2_core.landmarks3d import landmarks_from_points
from peach2_core.swing import estimate_swing
from peach2_core.tracker import Detection3D, Tracker
from peach2_core.types import invalid_landmark, Landmark, Landmarks2D, Landmarks3D, unit
from scipy.spatial.transform import Rotation

from .detector import box_iou, Detection
from .segmenter import MaskResult
from .stationary import Motion

CATEGORY_BAG = 0
CATEGORY_NOBAG = 1
GRAVITY_BASE = np.array([0.0, 0.0, -1.0])

ANCHOR_3D = '3d'
ANCHOR_2D = '2d'
ANCHOR_BOX = 'box'

# A bag fills roughly 0.5-0.75 of its axis-aligned box; below this fraction the mask is
# treated as incomplete (occluded or under-segmented) in mask_quality.
_FULL_AREA_RATIO = 0.4
_EDGE_TOUCH_PENALTY = 0.5


@dataclass(frozen=True)
class BuilderParams:
    bag_class_id: int
    n_bins_2d: int
    n_bins_3d: int
    taper_ratio: float
    point_stride: int
    cross_check_tol_m: float
    cross_check_inset_px: int
    backproject_win: int
    baseline_m: float
    stereo_fx_px: float
    disparity_sigma_px: float
    confirm_hits: int
    ttl_s: float
    gate_chi2: float
    process_accel_sigma: float
    swing_window_s: float
    swing_min_span_s: float


@dataclass
class FrameData:
    stamp_s: float
    K: np.ndarray  # (3, 3) of the (registered) colour image
    T_base_cam: np.ndarray  # (4, 4) base_link <- camera optical frame at stamp_s
    depth_m: np.ndarray  # (H, W) metres, 0 where invalid or below min_confidence
    confidence: np.ndarray  # (H, W) 0..1

    @property
    def shape(self) -> tuple[int, int]:
        return self.depth_m.shape[:2]


@dataclass
class Measurement:
    detection: Detection
    category: int
    mask: np.ndarray | None
    landmarks2d: Landmarks2D | None
    landmarks3d: Landmarks3D | None
    bottom: Landmark  # base_link, after the cross-check
    neck: Landmark
    tie: Landmark
    axis: np.ndarray  # (3,) unit bottom -> neck in base_link; zeros if unknown
    d95_m: float
    camera_distance_m: float
    mask_quality: float
    depth_coverage: float
    edge_touch: bool
    anchor: np.ndarray | None  # (3,) base_link
    anchor_cov: np.ndarray | None
    anchor_source: str
    flags: list[str] = field(default_factory=list)
    cross_check_dev_m: dict[str, float] = field(default_factory=dict)


@dataclass
class ObservationRecord:
    measurement: Measurement
    target_id: str
    confirmed: bool
    swing_known: bool
    swing_amplitude_m: float  # 0 when not swing_known
    swing_period_s: float  # 0 when not swing_known


def transform_matrix(translation: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    """4x4 homogeneous transform from a translation and a unit quaternion (x, y, z, w)."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(np.asarray(quat_xyzw, dtype=np.float64)).as_matrix()
    T[:3, 3] = np.asarray(translation, dtype=np.float64)
    return T


def pose_from_matrix(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (translation (3,), unit quaternion (x, y, z, w) with w >= 0) of a 4x4 transform."""
    T = np.asarray(T, dtype=np.float64)
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    if q[3] < 0.0:
        q = -q
    return T[:3, 3].copy(), q


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    return p @ T[:3, :3].T + T[:3, 3]


def rotate_cov(R: np.ndarray, cov: np.ndarray) -> np.ndarray:
    return R @ cov @ R.T


def gravity_in_image(K: np.ndarray, R_base_cam: np.ndarray, uv: Sequence[float],
                     z_m: float) -> np.ndarray | None:
    """
    Project base -Z into the image at pixel uv / depth z (unit derivative of the projection).

    None when gravity is (nearly) along the viewing ray, i.e. the camera looks straight down
    or up and the image carries no usable gravity direction.
    """
    K = np.asarray(K, dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if not (np.isfinite(z_m) and z_m > 0.0):
        return None
    p = np.array([(uv[0] - cx) * z_m / fx, (uv[1] - cy) * z_m / fy, z_m])
    g = np.asarray(R_base_cam, dtype=np.float64).T @ GRAVITY_BASE
    du = fx * (g[0] * p[2] - p[0] * g[2]) / p[2] ** 2
    dv = fy * (g[1] * p[2] - p[1] * g[2]) / p[2] ** 2
    d = np.array([du, dv])
    # |d| is px per metre of motion along gravity; < 5% of the focal length means gravity is
    # within ~3 deg of the ray.
    if float(np.linalg.norm(d)) < 0.05 * min(fx, fy) / z_m:
        return None
    return unit(d)


def cross_check_landmark(position_base: np.ndarray, pixel: Sequence[float],
                         depth_m: np.ndarray, K: np.ndarray, T_base_cam: np.ndarray,
                         confidence: np.ndarray | None, win: int, d95_m: float,
                         tol_m: float) -> tuple[bool | None, float]:
    """
    (consistent?, deviation [m]) of a base_link 3D landmark against its 2D pixel.

    The pixel is back-projected with the (mask-restricted) depth. Deviation is the larger of the
    3D point's perpendicular distance to the pixel ray and how far its range along the ray
    leaves [surface - tol, surface + d95 + tol]: the landmark lies on the bag axis, behind the
    visible surface by at most the diameter. None when the pixel has no depth.
    """
    xyz, _ = backproject(float(pixel[0]), float(pixel[1]), depth_m, K, confidence, win)
    if xyz is None:
        return None, float('nan')
    T_cam_base = np.linalg.inv(T_base_cam)
    p = transform_points(T_cam_base, position_base)[0]
    ray = xyz / np.linalg.norm(xyz)
    along = float(p @ ray)
    perp = float(np.linalg.norm(p - along * ray))
    behind = along - float(np.linalg.norm(xyz))
    d95 = d95_m if np.isfinite(d95_m) else 0.0
    out_of_range = max(-behind, behind - d95, 0.0)
    deviation = max(perp, out_of_range)
    return deviation <= tol_m, deviation


def _copy_landmark(lm: Landmark) -> Landmark:
    return Landmark(lm.valid, lm.position.copy(), lm.cov.copy(), lm.confidence)


def _edge_touch(det: Detection, mask: np.ndarray | None, shape: tuple[int, int]) -> bool:
    h, w = shape
    if mask is not None:
        return bool(mask[0, :].any() or mask[-1, :].any() or mask[:, 0].any()
                    or mask[:, -1].any())
    x1, y1, x2, y2 = det.bbox
    return x1 <= 0 or y1 <= 0 or x2 >= w or y2 >= h


class ObservationBuilder:
    def __init__(self, params: BuilderParams) -> None:
        self._p = params
        self._tracker = Tracker(confirm_hits=params.confirm_hits, ttl_s=params.ttl_s,
                                gate_chi2=params.gate_chi2,
                                process_accel_sigma=params.process_accel_sigma)
        self._swing: dict[str, deque] = {}

    # ------------------------------------------------------------------ per frame
    def measure(self, frame: FrameData,
                instances: Sequence[tuple[Detection, MaskResult | None]]) -> list[Measurement]:
        boxes = [d.bbox for d, _ in instances]
        out = []
        for i, (det, seg) in enumerate(instances):
            overlap = max((box_iou(det.bbox, b) for j, b in enumerate(boxes) if j != i),
                          default=0.0)
            out.append(self._measure_one(frame, det, seg, overlap))
        return out

    def _measure_one(self, frame: FrameData, det: Detection, seg: MaskResult | None,
                     overlap: float) -> Measurement:
        p = self._p
        category = CATEGORY_BAG if det.class_id == p.bag_class_id else CATEGORY_NOBAG
        mask = seg.mask if seg is not None else None
        flags = list(seg.flags) if seg is not None else []
        m = Measurement(
            detection=det, category=category, mask=mask, landmarks2d=None, landmarks3d=None,
            bottom=invalid_landmark(), neck=invalid_landmark(), tie=invalid_landmark(),
            axis=np.zeros(3), d95_m=0.0, camera_distance_m=0.0, mask_quality=0.0,
            depth_coverage=0.0, edge_touch=_edge_touch(det, mask, frame.shape), anchor=None,
            anchor_cov=None, anchor_source='', flags=flags)
        if mask is not None:
            m.depth_coverage = float(np.count_nonzero(mask & (frame.depth_m > 0.0))
                                     / max(np.count_nonzero(mask), 1))
        if category == CATEGORY_BAG and mask is not None:
            self._bag_geometry(frame, m, overlap)
        if m.anchor is None:
            self._box_anchor(frame, m)
        return m

    def _bag_geometry(self, frame: FrameData, m: Measurement, overlap: float) -> None:
        p = self._p
        K, T = frame.K, frame.T_base_cam
        R = T[:3, :3]
        mask = m.mask
        pts_cam = mask_points(mask, frame.depth_m, K, stride=p.point_stride)
        vs, us = np.nonzero(mask)
        z_med = float(np.median(pts_cam[:, 2])) if pts_cam.shape[0] else float('nan')
        if pts_cam.shape[0]:
            m.camera_distance_m = float(np.linalg.norm(np.median(pts_cam, axis=0)))
        g_px = gravity_in_image(K, R, (float(us.mean()), float(vs.mean())), z_med)
        if g_px is None:
            m.flags.append('gravity_unobservable')
        lm2 = landmarks_from_mask(mask, g_px if g_px is not None else np.full(2, np.nan),
                                  p.n_bins_2d)
        m.landmarks2d = lm2
        m.flags.extend(f'2d_{f}' for f in lm2.flags)
        m.mask_quality = self._mask_quality(m, lm2, overlap)

        pts_base = transform_points(T, pts_cam)
        depth_in_mask = np.where(mask, frame.depth_m, 0.0)
        hint = None
        bottom_px = neck_px = tie_px = None
        if lm2.ok:
            # The 2D bottom and tie are silhouette tips; step inside the mask to find depth.
            bottom_px = lm2.bottom_px + p.cross_check_inset_px * lm2.axis_px
            neck_px = lm2.neck_px
            tie_px = lm2.tie_px - p.cross_check_inset_px * lm2.axis_px
            b3, _ = backproject(*bottom_px, depth_in_mask, K, None, p.backproject_win)
            n3, _ = backproject(*neck_px, depth_in_mask, K, None, p.backproject_win)
            if b3 is not None and n3 is not None:
                hint = R @ (n3 - b3)
        sigma = None
        if np.isfinite(z_med):
            sigma = np.atleast_1d(depth_sigma_m(z_med, p.stereo_fx_px, p.baseline_m,
                                                p.disparity_sigma_px))
        lm3 = landmarks_from_points(pts_base, GRAVITY_BASE, axis_hint=hint, sigma_point_m=sigma,
                                    n_bins=p.n_bins_3d, taper_ratio=p.taper_ratio)
        m.landmarks3d = lm3
        m.flags.extend(f'3d_{f}' for f in lm3.flags)
        if not lm3.ok:
            self._anchor_from_2d(frame, m, lm2)
            return
        m.axis = lm3.axis.copy()
        m.d95_m = float(lm3.d95_m)
        # The tie keeps the core's own covariance (axial = one profile bin, lateral from the axis
        # fit at the tip); it is never derived from the neck.
        m.bottom, m.neck, m.tie = (_copy_landmark(lm3.bottom), _copy_landmark(lm3.neck),
                                   _copy_landmark(lm3.tie))
        if m.tie.valid and not np.all(np.isfinite(m.tie.cov)):
            m.tie.valid = False
            m.flags.append('tie_cov_invalid')
        mismatch = False
        if lm2.ok:
            for name, lm, px in (('bottom', m.bottom, bottom_px), ('neck', m.neck, neck_px),
                                 ('tie', m.tie, tie_px)):
                if not lm.valid:
                    continue
                ok, dev = cross_check_landmark(lm.position, px, depth_in_mask, K, T,
                                               frame.confidence, p.backproject_win, m.d95_m,
                                               p.cross_check_tol_m)
                m.cross_check_dev_m[name] = dev
                if ok is None:
                    m.flags.append(f'{name}_cross_check_no_depth')
                elif not ok:
                    lm.valid = False
                    # The anchor is the bottom; only bottom/neck disagreement inflates it.
                    mismatch = mismatch or name != 'tie'
                    m.flags.append(f'{name}_2d3d_mismatch')
        else:
            m.flags.append('cross_check_unavailable')
        m.anchor = lm3.bottom.position.copy()
        cov = lm3.bottom.cov.copy()
        if mismatch:
            cov = cov + p.cross_check_tol_m ** 2 * np.eye(3)
        m.anchor_cov = cov
        m.anchor_source = ANCHOR_3D

    def _anchor_from_2d(self, frame: FrameData, m: Measurement, lm2: Landmarks2D) -> None:
        p = self._p
        if not lm2.ok:
            return
        px = lm2.bottom_px + p.cross_check_inset_px * lm2.axis_px
        depth_in_mask = np.where(m.mask, frame.depth_m, 0.0)
        xyz, cov = backproject(float(px[0]), float(px[1]), depth_in_mask, frame.K,
                               frame.confidence, max(p.backproject_win, 2))
        if xyz is None:
            return
        widths = lm2.widths_px[np.isfinite(lm2.widths_px)]
        radius = 0.5 * float(widths[0]) * xyz[2] / frame.K[0, 0] if widths.size else 0.0
        # The visible surface is ~one radius in front of the axis bottom.
        xyz = xyz + radius * xyz / np.linalg.norm(xyz)
        cov = cov + (0.5 * radius) ** 2 * np.eye(3)
        R = frame.T_base_cam[:3, :3]
        m.anchor = transform_points(frame.T_base_cam, xyz)[0]
        m.anchor_cov = rotate_cov(R, cov)
        m.anchor_source = ANCHOR_2D
        m.flags.append('anchor_from_2d')

    def _box_anchor(self, frame: FrameData, m: Measurement) -> None:
        det = m.detection
        u, v = det.centre
        depth = frame.depth_m if m.mask is None else np.where(m.mask, frame.depth_m, 0.0)
        win = max(min(det.width, det.height) // 6, 1)
        xyz, cov = backproject(u, v, depth, frame.K, frame.confidence, win)
        if xyz is None:
            m.flags.append('no_depth_at_box')
            return
        if m.camera_distance_m == 0.0:
            m.camera_distance_m = float(np.linalg.norm(xyz))
        m.anchor = transform_points(frame.T_base_cam, xyz)[0]
        m.anchor_cov = rotate_cov(frame.T_base_cam[:3, :3], cov)
        m.anchor_source = ANCHOR_BOX
        m.flags.append('anchor_box_centre')

    @staticmethod
    def _mask_quality(m: Measurement, lm2: Landmarks2D, overlap: float) -> float:
        det = m.detection
        area = float(np.count_nonzero(m.mask))
        s_area = min(area / max(det.area * _FULL_AREA_RATIO, 1.0), 1.0)
        s_edge = _EDGE_TOUCH_PENALTY if m.edge_touch else 1.0
        if lm2.ok and lm2.widths_px.size:
            s_profile = float(np.count_nonzero(np.isfinite(lm2.widths_px))) / lm2.widths_px.size
        else:
            s_profile = 0.5
        return float(np.clip(s_area * s_edge * s_profile * (1.0 - overlap), 0.0, 1.0))

    # ------------------------------------------------------------------ tracking
    def associate(self, stamp_s: float, measurements: Sequence[Measurement],
                  motion: Motion) -> list[ObservationRecord]:
        """
        Track update for one frame; returns records for tracks updated or born this frame.

        Frames taken while the arm moves keep feeding the tracker but their landmarks are
        invalidated: colour, depth (up to the sync slop apart) and the TF sample are then no
        longer the same instant, and motion blur widens the mask.
        """
        stationary = motion is Motion.STATIONARY
        for m in measurements:
            if motion is Motion.MOVING:
                m.bottom.valid = m.neck.valid = m.tie.valid = False
                m.flags.append('arm_moving')
            elif motion is Motion.UNKNOWN:
                m.flags.append('arm_motion_unknown')
        dets = [Detection3D(m.anchor, m.anchor_cov, m.category, payload=m)
                for m in measurements if m.anchor is not None]
        updated = self._tracker.update(stamp_s, dets)
        live = {t.track_id for t in self._tracker.tracks()}
        if not stationary:
            self._swing.clear()
        for tid in list(self._swing):
            if tid not in live:
                del self._swing[tid]
        p = self._p
        records = []
        for track in updated:
            m: Measurement = track.payload
            known = False
            amp = period = 0.0
            if stationary and m.anchor_source == ANCHOR_3D:
                hist = self._swing.setdefault(track.track_id, deque())
                hist.append((stamp_s, m.anchor.copy()))
                while hist and hist[0][0] < stamp_s - p.swing_window_s:
                    hist.popleft()
                if hist[-1][0] - hist[0][0] >= p.swing_min_span_s:
                    times = np.array([h[0] for h in hist])
                    pos = np.array([h[1] for h in hist])
                    a, T = estimate_swing(times, pos)
                    if np.isfinite(a) and np.isfinite(T):
                        known, amp, period = True, float(a), float(T)
            records.append(ObservationRecord(m, track.track_id, track.confirmed, known, amp,
                                             period))
        return records

    def track_summary(self) -> tuple[frozenset[str], int]:
        """Return (confirmed track IDs, number of tentative tracks)."""
        tracks = self._tracker.tracks()
        confirmed = frozenset(t.track_id for t in tracks if t.confirmed)
        return confirmed, sum(1 for t in tracks if not t.confirmed)

    def reset(self) -> None:
        """Drop all tracks and swing history (IDs keep counting, so they are never reused)."""
        self._tracker.reset()
        self._swing.clear()
