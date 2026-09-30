"""
config/scene.yaml loader with explicit validation (no ROS).

Every key is required and unknown keys are rejected, so a typo cannot silently fall back to a
code default. Topic and service names are fixed in code, never parameters.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math

import yaml

from .scene_core import SceneParams

QOS_BEST_EFFORT = 'best_effort'
QOS_RELIABLE = 'reliable'


@dataclass(frozen=True)
class NodeParams:
    base_frame: str
    tcp_frame: str
    camera_frame: str
    depth_unit_m: float
    image_qos_reliability: str
    frame_timeout_s: float
    tf_timeout_s: float
    max_joint_speed_rad_s: float
    joint_buffer_s: float
    joint_max_gap_s: float
    self_sample_spacing_m: float
    max_frames_per_batch: int
    apply_timeout_s: float
    scene: SceneParams


# key -> (type, lower bound, lower bound exclusive, upper bound)
_RULES: dict[str, tuple[type, float | None, bool, float | None]] = {
    'base_frame': (str, None, False, None),
    'tcp_frame': (str, None, False, None),
    'camera_frame': (str, None, False, None),
    'depth_unit_m': (float, 0.0, True, 0.01),
    'image_qos_reliability': (str, None, False, None),
    'frame_timeout_s': (float, 0.0, True, 30.0),
    'tf_timeout_s': (float, 0.0, True, 5.0),
    'max_joint_speed_rad_s': (float, 0.0, True, 3.0),
    'joint_buffer_s': (float, 0.0, True, 60.0),
    'joint_max_gap_s': (float, 0.0, True, 1.0),
    'self_sample_spacing_m': (float, 0.0, True, 0.05),
    'max_frames_per_batch': (int, 1, False, 64),
    'apply_timeout_s': (float, 0.0, True, 60.0),
    'voxel_size_m': (float, 0.0, True, 0.20),
    'min_points_per_voxel': (int, 1, False, 1000),
    'workspace_radius_m': (float, 0.0, True, 5.0),
    'self_margin_m': (float, 0.0, False, 0.20),
    'target_margin_m': (float, 0.0, False, 0.30),
    'target_axial_margin_m': (float, 0.0, False, 0.30),
    'target_neck_overshoot_m': (float, 0.0, False, 0.30),
    'hard_min_thickness_m': (float, 0.0, True, 0.20),
    'linearity_min': (float, 0.0, True, 1.0),
    'flat_thickness_max_m': (float, 0.0, False, 0.10),
    'soft_max_extent_m': (float, 0.0, True, 5.0),
    'branch_vote_min': (float, 0.0, True, 1.0),
    'leaf_vote_min': (float, 0.0, True, 1.0),
    'leaf_mask_max_width_m': (float, 0.0, True, 1.0),
    'fit_min_fill_ratio': (float, 0.0, True, 1.0),
    'max_objects': (int, 1, False, 10000),
    'pixel_stride': (int, 1, False, 16),
    'min_confidence': (float, 0.0, False, 1.0),
    'min_depth_m': (float, 0.0, False, 10.0),
    'max_depth_m': (float, 0.0, True, 10.0),
}
# Empty means "use the depth image header.frame_id".
_OPTIONAL_STR = {'camera_frame'}

_SCENE_KEYS = {f.name for f in fields(SceneParams)}
_NODE_KEYS = {f.name for f in fields(NodeParams)} - {'scene'}


def _check(key: str, value: object) -> object:
    kind, lo, lo_excl, hi = _RULES[key]
    if kind is str:
        if value is None and key in _OPTIONAL_STR:
            return ''
        if not isinstance(value, str) or (not value.strip() and key not in _OPTIONAL_STR):
            raise ValueError(f'{key} must be a non-empty string')
        return value.strip()
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a number')
    if kind is int and not float(value).is_integer():
        raise ValueError(f'{key} must be an integer')
    num = int(value) if kind is int else float(value)
    if not math.isfinite(num):
        raise ValueError(f'{key} must be finite')
    if lo is not None and (num < lo or (lo_excl and num == lo)):
        raise ValueError(f'{key}={num} below {"(" if lo_excl else "["}{lo}')
    if hi is not None and num > hi:
        raise ValueError(f'{key}={num} above {hi}')
    return num


def params_from_dict(raw: dict) -> NodeParams:
    if not isinstance(raw, dict):
        raise ValueError('config root must be a mapping')
    names = _SCENE_KEYS | _NODE_KEYS
    unknown = sorted(set(raw) - names)
    missing = sorted(names - set(raw))
    if unknown:
        raise ValueError(f'unknown keys: {unknown}')
    if missing:
        raise ValueError(f'missing keys: {missing}')
    v = {k: _check(k, raw[k]) for k in names}
    if v['image_qos_reliability'] not in (QOS_BEST_EFFORT, QOS_RELIABLE):
        raise ValueError(f'image_qos_reliability must be {QOS_BEST_EFFORT} or {QOS_RELIABLE}')
    if v['tf_timeout_s'] >= v['frame_timeout_s']:
        raise ValueError('tf_timeout_s must be < frame_timeout_s')
    if v['self_sample_spacing_m'] > v['self_margin_m'] + 1e-12 and v['self_margin_m'] > 0.0:
        raise ValueError('self_sample_spacing_m must be <= self_margin_m (sample gaps would '
                         'leak robot points past the self filter)')
    if v['hard_min_thickness_m'] >= v['leaf_mask_max_width_m']:
        raise ValueError('hard_min_thickness_m must be < leaf_mask_max_width_m')
    if v['soft_max_extent_m'] < v['voxel_size_m']:
        raise ValueError('soft_max_extent_m must be >= voxel_size_m')
    if v['min_depth_m'] >= v['max_depth_m']:
        raise ValueError('min_depth_m must be < max_depth_m')
    scene = SceneParams(**{k: v[k] for k in _SCENE_KEYS})
    return NodeParams(scene=scene, **{k: v[k] for k in _NODE_KEYS})


def load_params(path: str) -> NodeParams:
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    return params_from_dict(raw)
