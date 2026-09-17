"""Zero-ROS tests for candidate/active result storage."""

from aubo_hand_eye_calibration.solver import CalibrationResult
from aubo_hand_eye_calibration.storage import (
    activate_candidate,
    atomic_write_yaml,
    load_candidate,
    write_candidate,
)
import numpy as np
import pytest
import yaml

FRAMES = {
    'base': 'base_link',
    'wrist': 'wrist3_Link',
    'camera_root': 'camera_link',
    'camera_optical': 'camera_color_optical_frame',
}
BOARD = {'inner_corners': [11, 8], 'square_size_m': 0.02}
CANDIDATE_ID = '20260917T000000_000000Z'


def make_result(passed=True):
    return CalibrationResult(
        gripper_from_camera=np.eye(4),
        base_from_target=np.eye(4),
        method='tsai',
        accepted_indices=[0, 1, 2],
        rejected_indices=[],
        translation_rms_m=0.001,
        rotation_rms_deg=0.2,
        reprojection_rms_px=0.4,
        rotation_span_deg=45.0,
        passed=passed,
        failures=[] if passed else ['accepted samples 3 < 12'],
    )


@pytest.fixture
def storage_dir(tmp_path, monkeypatch):
    monkeypatch.setenv('AUBO_HAND_EYE_DIR', str(tmp_path))
    return tmp_path


def test_write_load_round_trip(storage_dir):
    path = write_candidate(
        CANDIDATE_ID, np.eye(4), np.eye(4), np.eye(4), make_result(),
        FRAMES, BOARD, [{'pose_index': 1, 'accepted': True}])
    assert path == storage_dir / 'candidates' / f'{CANDIDATE_ID}.yaml'
    loaded_path, document = load_candidate(CANDIDATE_ID)
    assert document['candidate_id'] == CANDIDATE_ID
    assert document['frames'] == FRAMES
    assert document['checkerboard'] == BOARD
    assert document['quality_passed'] is True
    assert document['transforms']['wrist_from_camera_optical']['xyz_m'] == [
        0.0, 0.0, 0.0]


def test_activate_requires_quality_pass(storage_dir):
    write_candidate(
        CANDIDATE_ID, np.eye(4), np.eye(4), np.eye(4),
        make_result(passed=False), FRAMES, BOARD, [])
    with pytest.raises(ValueError, match='quality gates'):
        activate_candidate(CANDIDATE_ID)


def test_activate_copies_to_active(storage_dir):
    write_candidate(
        CANDIDATE_ID, np.eye(4), np.eye(4), np.eye(4), make_result(),
        FRAMES, BOARD, [])
    active = activate_candidate(CANDIDATE_ID)
    assert active == storage_dir / 'active.yaml'
    document = yaml.safe_load(active.read_text(encoding='utf-8'))
    assert document['candidate_id'] == CANDIDATE_ID
    assert 'activated_at' in document
    assert document['source_candidate'].endswith(f'{CANDIDATE_ID}.yaml')


def test_load_rejects_path_traversal(storage_dir):
    with pytest.raises(ValueError, match='invalid candidate id'):
        load_candidate('../../etc/passwd')


def test_load_rejects_unknown_schema(storage_dir):
    target = storage_dir / 'candidates'
    target.mkdir(parents=True)
    (target / 'legacy.yaml').write_text(
        'schema_version: 99\n', encoding='utf-8')
    with pytest.raises(ValueError, match='unsupported calibration schema'):
        load_candidate('legacy')


def test_atomic_write_replaces_existing_file(tmp_path):
    target = tmp_path / 'out.yaml'
    atomic_write_yaml(target, {'value': 1})
    atomic_write_yaml(target, {'value': 2})
    assert target.read_text(encoding='utf-8') == 'value: 2\n'
    assert list(tmp_path.glob('.*.tmp')) == []
