"""
config/target_model.yaml loader with explicit validation (no ROS).

Every key is required and unknown keys are rejected, so a typo cannot silently fall back to a
code default. `$(find-pkg-share <pkg>)` in directory values is expanded by a caller-supplied
resolver (the node passes ament_index; tests pass a stub).
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math
import re
from typing import Callable

import yaml

_FIND_PKG_SHARE = re.compile(r'\$\(find-pkg-share\s+([A-Za-z0-9_]+)\)')


@dataclass(frozen=True)
class TargetModelParams:
    frame_id: str
    validity_s: float
    pregrasp_standoff_m: float
    converge_sigma_lateral95_m: float
    converge_theta95_deg: float
    converge_sigma_axial95_m: float
    min_views: int
    default_max_views: int
    max_views_per_target: int
    observe_timeout_s: float
    view_gap_s: float
    view_translation_change_m: float
    view_rotation_change_deg: float
    max_frames_per_view: int
    remeasure_mismatch_m: float
    remeasure_max_camera_distance_m: float
    remeasure_min_frames: int
    cut_requires_remeasure: bool
    require_known_swing: bool
    swing_max_m: float
    swing_window_s: float
    approach_radial_slack_m: float
    revision_position_tol_m: float
    revision_angle_tol_deg: float
    description_config_dir: str
    calibration_dir: str


# key -> (type, lower bound, lower bound exclusive, upper bound)
_RULES: dict[str, tuple[type, float | None, bool, float | None]] = {
    'validity_s': (float, 0.0, True, 3600.0),
    'pregrasp_standoff_m': (float, 0.0, False, 0.30),
    'converge_sigma_lateral95_m': (float, 0.0, True, 0.05),
    'converge_theta95_deg': (float, 0.0, True, 45.0),
    'converge_sigma_axial95_m': (float, 0.0, True, 0.05),
    'min_views': (int, 1, False, 20),
    'default_max_views': (int, 1, False, 20),
    'max_views_per_target': (int, 1, False, 100),
    'observe_timeout_s': (float, 0.0, True, 600.0),
    'view_gap_s': (float, 0.0, True, 60.0),
    'view_translation_change_m': (float, 0.0, True, 1.0),
    'view_rotation_change_deg': (float, 0.0, True, 180.0),
    'max_frames_per_view': (int, 1, False, 1000),
    'remeasure_mismatch_m': (float, 0.0, True, 0.10),
    'remeasure_max_camera_distance_m': (float, 0.0, True, 2.0),
    'remeasure_min_frames': (int, 1, False, 100),
    'cut_requires_remeasure': (bool, None, False, None),
    'require_known_swing': (bool, None, False, None),
    'swing_max_m': (float, 0.0, True, 0.20),
    'swing_window_s': (float, 0.0, True, 60.0),
    'approach_radial_slack_m': (float, 0.0, False, 0.05),
    'revision_position_tol_m': (float, 0.0, False, 0.01),
    'revision_angle_tol_deg': (float, 0.0, False, 5.0),
    'frame_id': (str, None, False, None),
    'description_config_dir': (str, None, False, None),
    'calibration_dir': (str, None, False, None),
}


def _check(key: str, value: object) -> object:
    kind, lo, lo_excl, hi = _RULES[key]
    if kind is bool:
        if not isinstance(value, bool):
            raise ValueError(f'{key} must be true/false')
        return value
    if kind is str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{key} must be a non-empty string')
        return value.strip()
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a number')
    if kind is int and (not isinstance(value, int) and not float(value).is_integer()):
        raise ValueError(f'{key} must be an integer')
    num = int(value) if kind is int else float(value)
    if not math.isfinite(num):
        raise ValueError(f'{key} must be finite')
    if lo is not None and (num < lo or (lo_excl and num == lo)):
        raise ValueError(f'{key}={num} below {"(" if lo_excl else "["}{lo}')
    if hi is not None and num > hi:
        raise ValueError(f'{key}={num} above {hi}')
    return num


def expand_dirs(value: str, resolve_share: Callable[[str], str]) -> str:
    return _FIND_PKG_SHARE.sub(lambda m: resolve_share(m.group(1)), value)


def params_from_dict(raw: dict, resolve_share: Callable[[str], str]) -> TargetModelParams:
    if not isinstance(raw, dict):
        raise ValueError('config root must be a mapping')
    names = {f.name for f in fields(TargetModelParams)}
    unknown = sorted(set(raw) - names)
    missing = sorted(names - set(raw))
    if unknown:
        raise ValueError(f'unknown keys: {unknown}')
    if missing:
        raise ValueError(f'missing keys: {missing}')
    values = {k: _check(k, raw[k]) for k in names}
    for key in ('description_config_dir', 'calibration_dir'):
        values[key] = expand_dirs(values[key], resolve_share)
    if values['frame_id'] != 'base_link':
        raise ValueError('frame_id must be base_link (observations are published in base_link)')
    if values['min_views'] > values['max_views_per_target']:
        raise ValueError('min_views must be <= max_views_per_target')
    if values['default_max_views'] > values['max_views_per_target']:
        raise ValueError('default_max_views must be <= max_views_per_target')
    if values['remeasure_min_frames'] > values['max_frames_per_view']:
        raise ValueError('remeasure_min_frames must be <= max_frames_per_view')
    return TargetModelParams(**values)


def load_params(path: str, resolve_share: Callable[[str], str]) -> TargetModelParams:
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    return params_from_dict(raw, resolve_share)
