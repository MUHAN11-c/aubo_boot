"""Results files: complete keys, and design_reference values trace to the tool profiles."""
import math
import os

from peach2_calibration import RESULT_KEYS
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, '..', 'results')
PROFILES = os.path.join(HERE, '..', '..', '..', 'aubo_description', 'config')
TOOLS = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')

# results key -> (profile section, profile key)
TRACE = {
    'w_capture_m': ('geometry_m', 'blade_capture_half_width'),
    'e_blade_m': ('error_m', 'blade_plane_calibration_error95'),
    'e_robot_m': ('error_m', 'robot_axial_error95'),
    'e_tcp_m': ('error_m', 'tcp_calibration_error95'),
    'e_handeye_m': ('error_m', 'hand_eye_error95'),
    'e_runout_m': ('error_m', 'tool_runout95'),
    'c_wall_m': ('geometry_m', 'wall_clearance'),
    'fruit_clearance_m': ('geometry_m', 'fruit_safety_clearance'),
}


def _load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('tool_id', TOOLS)
def test_keys_complete_and_finite(tool_id):
    data = _load(os.path.join(RESULTS, f'{tool_id}.yaml'))
    assert data['tool_id'] == tool_id
    for key in RESULT_KEYS:
        assert key in data, key
    for key in RESULT_KEYS:
        if key != 'status':
            assert math.isfinite(float(data[key])) and float(data[key]) >= 0.0
    assert float(data['w_capture_m']) > 0.0


@pytest.mark.parametrize('tool_id', TOOLS)
def test_design_reference_traces_to_profile(tool_id):
    data = _load(os.path.join(RESULTS, f'{tool_id}.yaml'))
    profile_path = os.path.join(PROFILES, f'{tool_id}.yaml')
    if not os.path.exists(profile_path):
        pytest.skip('aubo_description sources not available')
    if data['status'] != 'design_reference':
        pytest.skip('calibrated values no longer trace to the design profile')
    profile = _load(profile_path)
    for key, (section, pkey) in TRACE.items():
        assert float(data[key]) == pytest.approx(float(profile[section][pkey])), key
