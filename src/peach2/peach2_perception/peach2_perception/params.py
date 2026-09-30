"""
config/perception.yaml loader with explicit validation (no ROS).

Every key is required and unknown keys are rejected, so a typo cannot silently fall back to a
code default. `$(find-pkg-share <pkg>)` in model paths is expanded by a caller-supplied resolver
(the node passes ament_index; tests pass a stub). Topic names are fixed in the node, never here.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math
import re
from typing import Callable

import yaml

_FIND_PKG_SHARE = re.compile(r'\$\(find-pkg-share\s+([A-Za-z0-9_]+)\)')

STR = 'str'  # non-empty string
STR_OPT = 'str?'  # string, may be empty
STR_LIST = 'str[]'


@dataclass(frozen=True)
class DepthNoiseParams:
    baseline_m: float
    fx_px: float
    disparity_sigma_px: float


@dataclass(frozen=True)
class SurrogateConfidenceParams:
    jump_rel_lo: float
    jump_rel_hi: float


@dataclass(frozen=True)
class DetectorParams:
    model_path: str
    engine_path: str
    device: str
    allow_cpu: bool
    imgsz: int
    half: bool
    conf: float
    iou: float
    class_names: tuple[str, ...]
    bag_class_id: int
    dedup_ios: float
    dedup_frag_ios: float
    dedup_area_ratio: float


@dataclass(frozen=True)
class SegmenterParams:
    model_path: str
    imgsz: int
    max_boxes: int
    box_expand_frac: float
    neg_point_offset_px: int
    min_area_px: int
    depth_jump_rel: float
    depth_jump_abs_m: float
    seed_frac: float
    morph_kernel_px: int
    segment_nobag: bool


@dataclass(frozen=True)
class LandmarkParams:
    n_bins_2d: int
    n_bins_3d: int
    taper_ratio: float
    point_stride: int
    cross_check_tol_m: float
    cross_check_inset_px: int
    backproject_win: int


@dataclass(frozen=True)
class TrackerParams:
    confirm_hits: int
    ttl_s: float
    gate_chi2: float
    process_accel_sigma: float


@dataclass(frozen=True)
class StationaryParams:
    joint_speed_threshold_rad_s: float
    window_s: float
    max_gap_s: float
    buffer_s: float


@dataclass(frozen=True)
class SwingParams:
    window_s: float
    min_span_s: float


@dataclass(frozen=True)
class LockParams:
    min_stationary_frames: int
    stable_s: float
    max_collect_s: float


@dataclass(frozen=True)
class DebugParams:
    publish_image: bool
    downscale: float


@dataclass(frozen=True)
class PerceptionParams:
    frame_id: str
    camera_frame: str
    depth_unit_m: float
    min_depth_m: float
    max_depth_m: float
    min_confidence: float
    confidence_match_tol_s: float
    sync_slop_s: float
    sync_queue_size: int
    sensor_qos_reliable: bool
    tf_timeout_s: float
    depth_noise: DepthNoiseParams
    surrogate_confidence: SurrogateConfidenceParams
    detector: DetectorParams
    segmenter: SegmenterParams
    landmarks: LandmarkParams
    tracker: TrackerParams
    stationary: StationaryParams
    swing: SwingParams
    lock: LockParams
    debug: DebugParams


# 'section.key' -> (type, lower bound, lower bound exclusive, upper bound)
_Rule = tuple[object, float | None, bool, float | None]
_RULES: dict[str, _Rule] = {
    'frame_id': (STR, None, False, None),
    'camera_frame': (STR_OPT, None, False, None),
    'depth_unit_m': (float, 0.0, True, 0.01),
    'min_depth_m': (float, 0.0, True, 10.0),
    'max_depth_m': (float, 0.0, True, 10.0),
    'min_confidence': (float, 0.0, False, 1.0),
    'confidence_match_tol_s': (float, 0.0, False, 0.1),
    'sync_slop_s': (float, 0.0, True, 0.5),
    'sync_queue_size': (int, 1, False, 100),
    'sensor_qos_reliable': (bool, None, False, None),
    'tf_timeout_s': (float, 0.0, True, 2.0),
    'depth_noise.baseline_m': (float, 0.0, True, 1.0),
    'depth_noise.fx_px': (float, 0.0, True, 10000.0),
    'depth_noise.disparity_sigma_px': (float, 0.0, True, 5.0),
    'surrogate_confidence.jump_rel_lo': (float, 0.0, False, 1.0),
    'surrogate_confidence.jump_rel_hi': (float, 0.0, True, 1.0),
    'detector.model_path': (STR, None, False, None),
    'detector.engine_path': (STR_OPT, None, False, None),
    'detector.device': (STR, None, False, None),
    'detector.allow_cpu': (bool, None, False, None),
    'detector.imgsz': (int, 32, False, 4096),
    'detector.half': (bool, None, False, None),
    'detector.conf': (float, 0.0, True, 1.0),
    'detector.iou': (float, 0.0, True, 1.0),
    'detector.class_names': (STR_LIST, None, False, None),
    'detector.bag_class_id': (int, 0, False, 1000),
    'detector.dedup_ios': (float, 0.0, True, 1.0),
    'detector.dedup_frag_ios': (float, 0.0, True, 1.0),
    'detector.dedup_area_ratio': (float, 0.0, False, 1.0),
    'segmenter.model_path': (STR, None, False, None),
    'segmenter.imgsz': (int, 256, False, 1024),
    'segmenter.max_boxes': (int, 1, False, 64),
    'segmenter.box_expand_frac': (float, 0.0, False, 0.5),
    'segmenter.neg_point_offset_px': (int, 0, False, 100),
    'segmenter.min_area_px': (int, 1, False, 1000000),
    'segmenter.depth_jump_rel': (float, 0.0, True, 0.5),
    'segmenter.depth_jump_abs_m': (float, 0.0, True, 0.5),
    'segmenter.seed_frac': (float, 0.0, True, 1.0),
    'segmenter.morph_kernel_px': (int, 0, False, 31),
    'segmenter.segment_nobag': (bool, None, False, None),
    'landmarks.n_bins_2d': (int, 6, False, 200),
    'landmarks.n_bins_3d': (int, 4, False, 100),
    'landmarks.taper_ratio': (float, 0.0, True, 0.999),
    'landmarks.point_stride': (int, 1, False, 8),
    'landmarks.cross_check_tol_m': (float, 0.0, True, 0.1),
    'landmarks.cross_check_inset_px': (int, 0, False, 20),
    'landmarks.backproject_win': (int, 0, False, 5),
    'tracker.confirm_hits': (int, 1, False, 50),
    'tracker.ttl_s': (float, 0.0, True, 60.0),
    'tracker.gate_chi2': (float, 0.0, True, 100.0),
    'tracker.process_accel_sigma': (float, 0.0, False, 10.0),
    'stationary.joint_speed_threshold_rad_s': (float, 0.0, True, 1.0),
    'stationary.window_s': (float, 0.0, False, 2.0),
    'stationary.max_gap_s': (float, 0.0, True, 1.0),
    'stationary.buffer_s': (float, 0.0, True, 60.0),
    'swing.window_s': (float, 0.0, True, 30.0),
    'swing.min_span_s': (float, 0.0, True, 30.0),
    'lock.min_stationary_frames': (int, 1, False, 1000),
    'lock.stable_s': (float, 0.0, False, 60.0),
    'lock.max_collect_s': (float, 0.0, True, 600.0),
    'debug.publish_image': (bool, None, False, None),
    'debug.downscale': (float, 0.0, True, 1.0),
}


def _check(key: str, value: object) -> object:
    kind, lo, lo_excl, hi = _RULES[key]
    if kind is bool:
        if not isinstance(value, bool):
            raise ValueError(f'{key} must be true/false')
        return value
    if kind in (STR, STR_OPT):
        if value is None and kind == STR_OPT:
            return ''
        if not isinstance(value, str) or (kind == STR and not value.strip()):
            raise ValueError(f'{key} must be a {"non-empty " if kind == STR else ""}string')
        return value.strip()
    if kind == STR_LIST:
        if (not isinstance(value, list) or not value
                or not all(isinstance(v, str) and v.strip() for v in value)):
            raise ValueError(f'{key} must be a non-empty list of strings')
        return tuple(v.strip() for v in value)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a number')
    if kind is int and not isinstance(value, int) and not float(value).is_integer():
        raise ValueError(f'{key} must be an integer')
    num = int(value) if kind is int else float(value)
    if not math.isfinite(num):
        raise ValueError(f'{key} must be finite')
    if lo is not None and (num < lo or (lo_excl and num == lo)):
        raise ValueError(f'{key}={num} below {"(" if lo_excl else "["}{lo}')
    if hi is not None and num > hi:
        raise ValueError(f'{key}={num} above {hi}')
    return num


def _section(cls, raw: object, prefix: str):
    if not isinstance(raw, dict):
        raise ValueError(f'{prefix or "config root"} must be a mapping')
    names = {f.name for f in fields(cls)}
    unknown = sorted(set(raw) - names)
    missing = sorted(names - set(raw))
    where = f' in {prefix}' if prefix else ''
    if unknown:
        raise ValueError(f'unknown keys{where}: {unknown}')
    if missing:
        raise ValueError(f'missing keys{where}: {missing}')
    values = {}
    for f in fields(cls):
        key = f'{prefix}.{f.name}' if prefix else f.name
        sub = _SECTIONS.get(key)
        values[f.name] = _section(sub, raw[f.name], key) if sub else _check(key, raw[f.name])
    return cls(**values)


_SECTIONS = {
    'depth_noise': DepthNoiseParams,
    'surrogate_confidence': SurrogateConfidenceParams,
    'detector': DetectorParams,
    'segmenter': SegmenterParams,
    'landmarks': LandmarkParams,
    'tracker': TrackerParams,
    'stationary': StationaryParams,
    'swing': SwingParams,
    'lock': LockParams,
    'debug': DebugParams,
}


def expand_path(value: str, resolve_share: Callable[[str], str]) -> str:
    return _FIND_PKG_SHARE.sub(lambda m: resolve_share(m.group(1)), value)


def params_from_dict(raw: dict, resolve_share: Callable[[str], str]) -> PerceptionParams:
    p = _section(PerceptionParams, raw, '')
    if p.frame_id != 'base_link':
        raise ValueError('frame_id must be base_link (observations are published in base_link)')
    if p.min_depth_m >= p.max_depth_m:
        raise ValueError('min_depth_m must be < max_depth_m')
    sc = p.surrogate_confidence
    if sc.jump_rel_lo >= sc.jump_rel_hi:
        raise ValueError('surrogate_confidence.jump_rel_lo must be < jump_rel_hi')
    d = p.detector
    if d.imgsz % 32:
        raise ValueError('detector.imgsz must be a multiple of 32')
    if d.bag_class_id >= len(d.class_names):
        raise ValueError('detector.bag_class_id out of range of class_names')
    if len(set(d.class_names)) != len(d.class_names):
        raise ValueError('detector.class_names must be unique')
    if d.dedup_frag_ios > d.dedup_ios:
        raise ValueError('detector.dedup_frag_ios must be <= dedup_ios')
    s = p.segmenter
    if s.imgsz % 64:
        raise ValueError('segmenter.imgsz must be a multiple of 64')
    if s.morph_kernel_px not in (0, 1) and s.morph_kernel_px % 2 == 0:
        raise ValueError('segmenter.morph_kernel_px must be odd (or 0/1 to disable)')
    if p.swing.min_span_s > p.swing.window_s:
        raise ValueError('swing.min_span_s must be <= swing.window_s')
    if p.stationary.window_s >= p.stationary.buffer_s:
        raise ValueError('stationary.window_s must be < stationary.buffer_s')
    detector = DetectorParams(**{**d.__dict__,
                                 'model_path': expand_path(d.model_path, resolve_share),
                                 'engine_path': expand_path(d.engine_path, resolve_share)})
    segmenter = SegmenterParams(**{**s.__dict__,
                                   'model_path': expand_path(s.model_path, resolve_share)})
    return PerceptionParams(**{**p.__dict__, 'detector': detector, 'segmenter': segmenter})


def load_params(path: str, resolve_share: Callable[[str], str]) -> PerceptionParams:
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    return params_from_dict(raw, resolve_share)
