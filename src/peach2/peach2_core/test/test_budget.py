import dataclasses
import math
import os

import numpy as np
from peach2_core import codes
from peach2_core.budget import evaluate_budget, tcp_target_for_blade
from peach2_core.fusion import FusedModel
from peach2_core.tool import load_tool_calibration, load_tool_geometry
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES = os.path.join(HERE, '..', '..', '..', 'aubo_description', 'config')
CALIB = os.path.join(HERE, '..', '..', 'peach2_calibration', 'results')
TOOLS = ('shear_v1', 'bite_shear_v1', 'adaptive_shear_v1')

# Old stack (tool_budget.py) constants that have no calibration key: v2 replaces them with
# the measured neck sigma and swing amplitude.
OLD_AXIAL_NECK_FLOOR = 0.003
OLD_TARGET_MOTION = 0.003
OLD_BAG_DEFORMATION = 0.002


def _model(d95=0.05, length=0.13, s_lat=0.004, s_ax=0.003, theta=2.0, ok=True):
    axis = np.array([0.0, 0.0, 1.0])
    bottom = np.array([0.5, 0.1, 0.8])
    return FusedModel(ok=ok, bottom=bottom, neck=bottom + length * axis, tie=None, axis=axis,
                      d95_m=d95, length_m=length, sigma_lateral95_m=s_lat,
                      sigma_axial95_m=s_ax, theta95_deg=theta, n_views=3, flags=[])


def _tool(tool_id):
    return load_tool_geometry(tool_id, PROFILES)


def _calib(tool_id, **over):
    c = load_tool_calibration(tool_id, CALIB)
    return dataclasses.replace(c, **over)


@pytest.mark.parametrize('tool_id', TOOLS)
def test_linear_sum_negative_but_rss_positive(tool_id):
    tool, calib = _tool(tool_id), _calib(tool_id, status='calibrated')
    linear_axial = calib.w_capture_m - (OLD_AXIAL_NECK_FLOOR + calib.e_blade_m + calib.e_robot_m
                                        + OLD_TARGET_MOTION)
    assert linear_axial < 0.0
    r = evaluate_budget(_model(s_ax=OLD_AXIAL_NECK_FLOOR), tool, calib,
                        swing_amplitude_m=OLD_TARGET_MOTION)
    expected = calib.w_capture_m - math.sqrt(OLD_AXIAL_NECK_FLOOR ** 2 + calib.e_blade_m ** 2
                                             + calib.e_robot_m ** 2 + OLD_TARGET_MOTION ** 2)
    assert r.axial_margin_m == pytest.approx(expected)
    assert r.axial_margin_m > 0.0
    assert r.sleeve_ok and r.cut_ok and not r.structural
    assert r.failure_code == codes.NONE and r.reason == 'ok'


def test_shear_radial_linear_negative_rss_positive():
    tool, calib = _tool('shear_v1'), _calib('shear_v1', status='calibrated')
    m = _model()
    lever = m.length_m * math.sin(math.radians(m.theta95_deg))
    available = 0.5 * tool.d_inner_m - 0.5 * m.d95_m - calib.c_wall_m
    linear = available - (m.sigma_lateral95_m + lever + calib.e_runout_m + calib.e_tcp_m
                          + calib.e_handeye_m + OLD_BAG_DEFORMATION)
    assert linear < 0.0
    r = evaluate_budget(m, tool, calib, swing_amplitude_m=OLD_TARGET_MOTION)
    rss = math.sqrt(m.sigma_lateral95_m ** 2 + lever ** 2 + calib.e_tcp_m ** 2
                    + calib.e_handeye_m ** 2 + calib.e_runout_m ** 2 + OLD_TARGET_MOTION ** 2)
    assert r.radial_margin_m == pytest.approx(available - rss)
    assert r.radial_margin_m > 0.0 and r.sleeve_ok


@pytest.mark.parametrize('tool_id', TOOLS)
def test_design_reference_blocks_cut_only(tool_id):
    r = evaluate_budget(_model(), _tool(tool_id), _calib(tool_id), swing_amplitude_m=0.003)
    assert r.sleeve_ok
    assert not r.cut_ok
    assert r.failure_code == codes.BUDGET_STRUCTURAL
    assert r.reason == 'calibration_pending'
    assert r.axial_margin_m > 0.0


@pytest.mark.parametrize('tool_id,d95,reason', [
    ('shear_v1', 0.080, 'bag_d95_exceeds_tool'),
    ('shear_v1', 0.066, 'radial_budget_negative'),
    ('bite_shear_v1', 0.110, 'bag_d95_exceeds_tool'),
    ('adaptive_shear_v1', 0.105, 'radial_budget_negative'),
    ('adaptive_shear_v1', 0.0, 'bag_d95_missing'),
])
def test_radial_negative(tool_id, d95, reason):
    r = evaluate_budget(_model(d95=d95), _tool(tool_id), _calib(tool_id, status='calibrated'))
    assert not r.sleeve_ok and not r.cut_ok
    assert r.failure_code == codes.BUDGET_RADIAL_NEGATIVE
    assert r.reason == reason


@pytest.mark.parametrize('tool_id', TOOLS)
def test_axial_negative_and_swing(tool_id):
    tool, calib = _tool(tool_id), _calib(tool_id, status='calibrated')
    r = evaluate_budget(_model(s_ax=0.010), tool, calib)
    assert r.sleeve_ok and not r.cut_ok
    assert r.failure_code == codes.BUDGET_AXIAL_NEGATIVE
    assert r.reason == 'axial_budget_negative'
    base = evaluate_budget(_model(), tool, calib)
    windy = evaluate_budget(_model(), tool, calib, swing_amplitude_m=0.015)
    assert windy.axial_margin_m < base.axial_margin_m
    assert windy.radial_margin_m < base.radial_margin_m
    assert not windy.cut_ok


def test_structural_cases():
    tool = _tool('adaptive_shear_v1')
    r = evaluate_budget(_model(), tool,
                        _calib('adaptive_shear_v1', status='calibrated', w_capture_m=0.003))
    assert r.structural and not r.cut_ok and r.sleeve_ok
    assert r.failure_code == codes.BUDGET_STRUCTURAL and r.reason == 'axial_structural'
    r = evaluate_budget(_model(), tool,
                        _calib('adaptive_shear_v1', status='calibrated', e_tcp_m=0.07))
    assert r.structural and not r.sleeve_ok
    assert r.failure_code == codes.BUDGET_STRUCTURAL and r.reason == 'radial_structural'


def test_fruit_clearance_and_invalid_model():
    tool, calib = _tool('bite_shear_v1'), _calib('bite_shear_v1', status='calibrated')
    r = evaluate_budget(_model(), tool, calib, cut_to_fruit_m=0.005)
    assert r.sleeve_ok and not r.cut_ok
    assert r.failure_code == codes.BUDGET_AXIAL_NEGATIVE
    assert r.reason == 'cut_plane_fruit_clearance'
    r = evaluate_budget(_model(ok=False), tool, calib)
    assert not r.sleeve_ok and r.failure_code == codes.MODEL_NOT_CONVERGED
    r = evaluate_budget(_model(), tool, calib, swing_amplitude_m=float('nan'))
    assert not r.sleeve_ok and r.reason == 'model_invalid'


@pytest.mark.parametrize('tool_id,l_blade', [
    ('shear_v1', 0.030), ('bite_shear_v1', 0.037), ('adaptive_shear_v1', 0.079)])
def test_tcp_target_for_blade(tool_id, l_blade):
    neck = np.array([0.4, -0.1, 0.9])
    axis = np.array([0.0, 0.3, 0.4])
    tcp = tcp_target_for_blade(neck, axis, _tool(tool_id))
    assert np.allclose(tcp, neck + l_blade * np.array([0.0, 0.6, 0.8]))
    with pytest.raises(ValueError):
        tcp_target_for_blade(neck, np.zeros(3), _tool(tool_id))
