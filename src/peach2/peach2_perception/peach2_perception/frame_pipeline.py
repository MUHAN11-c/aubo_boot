"""
One synchronized RGB-D frame -> per-instance measurements (no ROS).

depth units -> validity -> confidence (published or surrogate) -> YOLO -> MobileSAM (bags first)
-> ObservationBuilder.measure. Tracking and the scene lock stay in the caller, under its lock.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from .depth_quality import (confident_depth, depth_to_metres, normalise_confidence,
                            surrogate_confidence, valid_depth_mask)
from .detector import Detection, Detector
from .observation_builder import BuilderParams, FrameData, Measurement, ObservationBuilder
from .params import PerceptionParams
from .segmenter import MaskResult, Segmenter

CONFIDENCE_PUBLISHED = 'published'
CONFIDENCE_SURROGATE = 'surrogate'


@dataclass
class FrameInput:
    stamp_s: float
    bgr: np.ndarray  # (H, W, 3) uint8
    depth_raw: np.ndarray  # (H, W) uint16 counts or float metres, registered to bgr
    K: np.ndarray  # (3, 3)
    T_base_cam: np.ndarray  # (4, 4) at stamp_s
    confidence: np.ndarray | None = None  # (H, W) mono8 / float, or None when not published


@dataclass
class FrameResult:
    frame: FrameData
    detections: list[Detection]
    measurements: list[Measurement]
    confidence_source: str
    sam_truncated: int
    timings_ms: dict[str, float] = field(default_factory=dict)


def builder_params(p: PerceptionParams) -> BuilderParams:
    lm, tr = p.landmarks, p.tracker
    return BuilderParams(
        bag_class_id=p.detector.bag_class_id, n_bins_2d=lm.n_bins_2d, n_bins_3d=lm.n_bins_3d,
        taper_ratio=lm.taper_ratio, point_stride=lm.point_stride,
        cross_check_tol_m=lm.cross_check_tol_m, cross_check_inset_px=lm.cross_check_inset_px,
        backproject_win=lm.backproject_win, baseline_m=p.depth_noise.baseline_m,
        stereo_fx_px=p.depth_noise.fx_px, disparity_sigma_px=p.depth_noise.disparity_sigma_px,
        confirm_hits=tr.confirm_hits, ttl_s=tr.ttl_s, gate_chi2=tr.gate_chi2,
        process_accel_sigma=tr.process_accel_sigma, swing_window_s=p.swing.window_s,
        swing_min_span_s=p.swing.min_span_s)


class FramePipeline:
    def __init__(self, params: PerceptionParams, detector: Detector, segmenter: Segmenter,
                 builder: ObservationBuilder) -> None:
        self._p = params
        self._detector = detector
        self._segmenter = segmenter
        self._builder = builder

    def run(self, inp: FrameInput) -> FrameResult:
        p = self._p
        if inp.bgr.shape[:2] != inp.depth_raw.shape[:2]:
            raise ValueError('colour and depth sizes differ (depth must be registered)')
        t0 = time.perf_counter()
        depth_m = depth_to_metres(inp.depth_raw, p.depth_unit_m)
        valid = valid_depth_mask(depth_m, p.min_depth_m, p.max_depth_m)
        if inp.confidence is not None:
            if inp.confidence.shape[:2] != depth_m.shape:
                raise ValueError('confidence and depth sizes differ')
            conf = normalise_confidence(inp.confidence)
            source = CONFIDENCE_PUBLISHED
        else:
            sc = p.surrogate_confidence
            conf = surrogate_confidence(depth_m, valid, sc.jump_rel_lo, sc.jump_rel_hi)
            source = CONFIDENCE_SURROGATE
        depth_conf = confident_depth(depth_m, valid, conf, p.min_confidence)
        depth_ok = depth_conf > 0.0
        t1 = time.perf_counter()

        dets = self._detector.detect(inp.bgr)
        t2 = time.perf_counter()
        bag_id = p.detector.bag_class_id
        # Bags first (by score) so SAM truncation only ever drops nobag or the weakest bags.
        ordered = sorted(dets, key=lambda d: (d.class_id != bag_id, -d.score))
        to_segment = [d for d in ordered
                      if d.class_id == bag_id or p.segmenter.segment_nobag]
        masks, truncated = self._segmenter.segment(inp.bgr, [d.bbox for d in to_segment],
                                                   depth_conf, depth_ok)
        by_det: dict[int, MaskResult] = {id(d): m for d, m in zip(to_segment, masks)}
        instances = [(d, by_det.get(id(d))) for d in ordered]
        t3 = time.perf_counter()

        frame = FrameData(stamp_s=inp.stamp_s, K=np.asarray(inp.K, dtype=np.float64),
                          T_base_cam=np.asarray(inp.T_base_cam, dtype=np.float64),
                          depth_m=depth_conf, confidence=conf)
        measurements = self._builder.measure(frame, instances)
        t4 = time.perf_counter()
        timings = {'depth': 1e3 * (t1 - t0), 'detect': 1e3 * (t2 - t1),
                   'segment': 1e3 * (t3 - t2), 'geometry': 1e3 * (t4 - t3),
                   'total': 1e3 * (t4 - t0)}
        return FrameResult(frame, ordered, measurements, source, truncated, timings)
