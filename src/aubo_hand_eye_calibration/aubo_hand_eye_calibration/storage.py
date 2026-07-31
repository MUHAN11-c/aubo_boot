# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Versioned, atomic calibration result storage."""

from datetime import datetime, timezone
import os
from pathlib import Path
import tempfile

import numpy as np
import yaml

from .transforms import transform_to_xyz_quat


SCHEMA_VERSION = 1


def default_storage_directory():
    """
    标定结果存储目录.

    优先级: AUBO_HAND_EYE_DIR 环境变量 > 项目内 <工作区根>/hand_eye/
    > ~/.ros/aubo_e5/hand_eye（布局无法识别时的兜底）。

    项目内定位相对本文件向上查找（含 src/aubo_hand_eye_calibration 的
    目录即工作区根），源码树与 colcon install 布局均成立，不硬编码
    绝对路径。
    """
    override = os.environ.get('AUBO_HAND_EYE_DIR')
    if override:
        return Path(override)
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src' / 'aubo_hand_eye_calibration').is_dir():
            return parent / 'hand_eye'
    return Path.home() / '.ros' / 'aubo_e5' / 'hand_eye'


def _transform_data(transform):
    xyz, quaternion = transform_to_xyz_quat(transform)
    return {
        'xyz_m': [float(value) for value in xyz],
        'quaternion_xyzw': [float(value) for value in quaternion],
        'matrix': np.asarray(transform, dtype=float).tolist(),
    }


def write_candidate(
    candidate_id,
    wrist_from_camera_optical,
    wrist_from_camera_root,
    base_from_target,
    result,
    frames,
    board,
    samples,
    directory=None,
    method_scores=None,
    sample_errors=None,
    refine_stats=None,
):
    directory = Path(directory or default_storage_directory())
    candidates = directory / 'candidates'
    candidates.mkdir(parents=True, exist_ok=True)
    path = candidates / f'{candidate_id}.yaml'
    document = {
        'schema_version': SCHEMA_VERSION,
        'candidate_id': candidate_id,
        'created_at': datetime.now(timezone.utc).isoformat(),
        'calibration_type': 'eye_in_hand',
        'quality_passed': bool(result.passed),
        'failures': list(result.failures),
        'frames': dict(frames),
        'checkerboard': dict(board),
        'method': result.method,
        'metrics': {
            'accepted_samples': len(result.accepted_indices),
            'rejected_samples': len(result.rejected_indices),
            'translation_rms_m': result.translation_rms_m,
            'rotation_rms_deg': result.rotation_rms_deg,
            'reprojection_rms_px': result.reprojection_rms_px,
            'rotation_span_deg': result.rotation_span_deg,
        },
        'transforms': {
            'wrist_from_camera_optical':
                _transform_data(wrist_from_camera_optical),
            'wrist_from_camera_root':
                _transform_data(wrist_from_camera_root),
            'base_from_target': _transform_data(base_from_target),
        },
        # 求解明细: 与 status JSON result 区段同构;
        # 旧调用方未传时给空兜底, 读取侧用 .get() 缺省处理
        'method_scores': list(method_scores or []),
        'sample_errors': list(sample_errors or []),
        'refine_stats': dict(refine_stats or {}),
        'samples': list(samples),
    }
    _atomic_yaml(path, document)
    return path


def _atomic_yaml(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=path.parent)
    try:
        with os.fdopen(file_descriptor, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


def load_candidate(candidate_id, directory=None):
    if not candidate_id or any(
            token in candidate_id for token in ('/', '\\', '..')):
        raise ValueError('invalid candidate id')
    path = Path(directory or default_storage_directory()) / (
        f'candidates/{candidate_id}.yaml')
    with path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if document.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('unsupported calibration schema')
    return path, document


def activate_candidate(candidate_id, directory=None):
    directory = Path(directory or default_storage_directory())
    candidate_path, document = load_candidate(candidate_id, directory)
    if not document.get('quality_passed', False):
        raise ValueError('candidate did not pass quality gates')
    active = directory / 'active.yaml'
    document = dict(document)
    document['activated_at'] = datetime.now(timezone.utc).isoformat()
    document['source_candidate'] = str(candidate_path)
    _atomic_yaml(active, document)
    return active
