import dataclasses

import numpy as np
from peach2_core import codes
from peach2_core.budget import evaluate_budget
from peach2_core.tool import CALIBRATED, load_tool_calibration, load_tool_geometry
from peach2_target_model.decision import (build_decision, convergence_failures, cut_to_fruit_m,
                                          pregrasp_position, quat_z_to_axis, Remeasure,
                                          remeasure_deviation)
import pytest
from tm_fixtures import AXIS, BOTTOM, CALIB, fused, NECK, params, PROFILES, record, TOOLS

NOW = 130.0


def _tool(tool_id):
    return load_tool_geometry(tool_id, PROFILES)


def _calib(tool_id, calibrated=False):
    c = load_tool_calibration(tool_id, CALIB)
    return dataclasses.replace(c, status=CALIBRATED) if calibrated else c


def _rotate(q, v):
    x, y, z, w = q
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def test_convergence_gates_are_strict():
    p = params()
    assert convergence_failures(fused(), p) == []
    edge = fused(s_lat=p.converge_sigma_lateral95_m, theta=p.converge_theta95_deg,
                 s_ax=p.converge_sigma_axial95_m)
    assert convergence_failures(edge, p) == ['sigma_lateral', 'theta', 'sigma_axial']
    assert convergence_failures(fused(ok=False), p) == ['model_invalid']
    assert 'too_few_views' in convergence_failures(
        fused(n_views=0), dataclasses.replace(p, min_views=1))


@pytest.mark.parametrize('axis', [
    [0.0, 0.0, 1.0], [0.3, -0.2, 0.9], [1.0, 0.0, 0.0], [0.0, -1.0, 0.2], [0.1, 0.0, -1.0]])
def test_pregrasp_orientation_aligns_tcp_z_with_axis(axis):
    a = np.asarray(axis) / np.linalg.norm(axis)
    q = quat_z_to_axis(a)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert np.allclose(_rotate(q, np.array([0.0, 0.0, 1.0])), a, atol=1e-9)


def test_quat_antiparallel_and_invalid():
    q = quat_z_to_axis(np.array([0.0, 0.0, -1.0]))
    assert np.allclose(_rotate(q, np.array([0.0, 0.0, 1.0])), [0.0, 0.0, -1.0])
    with pytest.raises(ValueError):
        quat_z_to_axis(np.zeros(3))
    with pytest.raises(ValueError):
        pregrasp_position(BOTTOM, np.zeros(3), 0.03)


@pytest.mark.parametrize('tool_id', TOOLS)
def test_geometry_fields(tool_id):
    p = params()
    tool = _tool(tool_id)
    axis = np.array([0.2, 0.1, 0.95])
    axis /= np.linalg.norm(axis)
    model = fused(axis=axis)
    d = build_decision(record(model), tool, _calib(tool_id), p, NOW)
    assert np.allclose(d.pregrasp_position, model.bottom - p.pregrasp_standoff_m * axis)
    assert np.allclose(d.blade_target, model.neck)
    assert d.insert_travel_m == pytest.approx(
        model.length_m + tool.l_blade_m + p.pregrasp_standoff_m)
    assert np.allclose(_rotate(d.pregrasp_quat_xyzw, np.array([0.0, 0.0, 1.0])), axis)
    assert d.valid_until_s == pytest.approx(NOW + p.validity_s)
    assert d.tool_id == tool_id and d.model_revision == 3 and d.target_id == 'target_1'


@pytest.mark.parametrize('tool_id', TOOLS)
def test_design_reference_calibration_blocks_cut_only(tool_id):
    d = build_decision(record(remeasure=Remeasure.PASSED), _tool(tool_id), _calib(tool_id),
                       params(), NOW)
    assert d.approach_allowed and d.sleeve_allowed and not d.cut_allowed
    assert d.failure_code == codes.BUDGET_STRUCTURAL and d.reason == 'calibration_pending'


@pytest.mark.parametrize('tool_id', TOOLS)
def test_calibrated_and_remeasured_allows_cut(tool_id):
    d = build_decision(record(remeasure=Remeasure.PASSED), _tool(tool_id),
                       _calib(tool_id, calibrated=True), params(), NOW)
    assert d.approach_allowed and d.sleeve_allowed and d.cut_allowed
    assert d.failure_code == codes.NONE and d.reason == 'ok'
    assert d.radial_margin_m > 0.0 and d.axial_margin_m > 0.0


def test_remeasure_gating():
    tool, calib, p = _tool('shear_v1'), _calib('shear_v1', calibrated=True), params()
    pending = build_decision(record(), tool, calib, p, NOW)
    assert pending.sleeve_allowed and not pending.cut_allowed
    assert pending.failure_code == codes.NECK_REMEASURE_PENDING == 28
    assert pending.reason == 'neck_remeasure_pending'
    relaxed = dataclasses.replace(p, cut_requires_remeasure=False)
    assert build_decision(record(), tool, calib, relaxed, NOW).cut_allowed
    mismatch = build_decision(record(remeasure=Remeasure.MISMATCH), tool, calib, relaxed, NOW)
    assert mismatch.sleeve_allowed and not mismatch.cut_allowed
    assert mismatch.reason == 'neck_remeasure_mismatch'
    assert mismatch.failure_code == codes.NECK_REMEASURE_MISMATCH


def test_unknown_swing():
    tool, calib, p = _tool('shear_v1'), _calib('shear_v1', calibrated=True), params()
    rec = record(swing=float('nan'), remeasure=Remeasure.PASSED)
    blocked = build_decision(rec, tool, calib, p, NOW)
    assert not blocked.approach_allowed
    assert blocked.failure_code == codes.SWING_TOO_LARGE and blocked.reason == 'swing_unknown'
    assert 'swing_unknown' in blocked.flags and np.isfinite(blocked.radial_margin_m)
    relaxed = build_decision(rec, tool, calib, dataclasses.replace(p, require_known_swing=False),
                             NOW)
    assert relaxed.cut_allowed and 'swing_unknown' in relaxed.flags
    known = build_decision(record(remeasure=Remeasure.PASSED), tool, calib, p, NOW)
    assert 'swing_unknown' not in known.flags


def test_not_converged_expired_invalid_block_everything():
    tool, calib, p = _tool('shear_v1'), _calib('shear_v1', calibrated=True), params()
    cases = [
        (record(converged=False), codes.MODEL_NOT_CONVERGED, 'model_not_converged'),
        (record(last_obs_s=NOW - p.validity_s - 0.1), codes.MODEL_EXPIRED, 'model_expired'),
        (record(model=fused(ok=False)), codes.MODEL_NOT_CONVERGED, 'model_invalid'),
        (record(swing=p.swing_max_m + 1e-3), codes.SWING_TOO_LARGE, 'swing_too_large'),
    ]
    for rec, code, reason in cases:
        d = build_decision(rec, tool, calib, p, NOW)
        assert not (d.approach_allowed or d.sleeve_allowed or d.cut_allowed), reason
        assert d.failure_code == code and d.reason == reason
    at_limit = build_decision(record(last_obs_s=NOW - p.validity_s), tool, calib, p, NOW)
    assert at_limit.approach_allowed


def test_radial_structural_blocks_approach():
    tool = _tool('shear_v1')
    calib = dataclasses.replace(_calib('shear_v1', calibrated=True), e_tcp_m=tool.d_inner_m)
    d = build_decision(record(), tool, calib, params(), NOW)
    assert not d.approach_allowed and d.failure_code == codes.BUDGET_STRUCTURAL
    assert d.reason == 'radial_structural'


def test_radial_slack_band():
    tool, calib, p = _tool('shear_v1'), _calib('shear_v1', calibrated=True), params()
    m0 = evaluate_budget(fused(d95=0.0), tool, calib).radial_margin_m
    inside = build_decision(record(model=fused(d95=2.0 * (m0 + 0.5 * p.approach_radial_slack_m))),
                            tool, calib, p, NOW)
    assert inside.radial_margin_m == pytest.approx(-0.5 * p.approach_radial_slack_m)
    assert inside.approach_allowed and not inside.sleeve_allowed
    assert inside.failure_code == codes.BUDGET_RADIAL_NEGATIVE
    outside = build_decision(record(model=fused(d95=2.0 * (m0 + 2.0 * p.approach_radial_slack_m))),
                             tool, calib, p, NOW)
    assert not outside.approach_allowed
    assert outside.reason == 'radial_margin_below_approach_slack'


def test_fruit_clearance_proxy_blocks_cut():
    tool, calib, p = _tool('adaptive_shear_v1'), _calib('adaptive_shear_v1', True), params()
    model = fused(d95=0.05, length=0.05 + 0.5 * calib.fruit_clearance_m)
    dist, proxy = cut_to_fruit_m(model, float('nan'))
    assert proxy and dist == pytest.approx(0.5 * calib.fruit_clearance_m)
    d = build_decision(record(model=model, remeasure=Remeasure.PASSED), tool, calib, p, NOW)
    assert d.sleeve_allowed and not d.cut_allowed
    assert d.reason == 'cut_plane_fruit_clearance' and 'fruit_top_proxy' in d.flags
    dist, proxy = cut_to_fruit_m(fused(ok=False), 0.05)
    assert np.isnan(dist) and not proxy


def test_known_fruit_top_offset_overrides_proxy():
    tool, calib, p = _tool('adaptive_shear_v1'), _calib('adaptive_shear_v1', True), params()
    model = fused(d95=0.05, length=0.05 + 0.5 * calib.fruit_clearance_m)
    far = 2.0 * calib.fruit_clearance_m
    assert cut_to_fruit_m(model, far) == (far, False)
    d = build_decision(record(model=model, remeasure=Remeasure.PASSED, fruit_top_offset_m=far),
                       tool, calib, p, NOW)
    assert d.cut_allowed and 'fruit_top_proxy' not in d.flags
    near = build_decision(record(remeasure=Remeasure.PASSED,
                                 fruit_top_offset_m=0.5 * calib.fruit_clearance_m),
                          tool, calib, p, NOW)
    assert not near.cut_allowed and near.reason == 'cut_plane_fruit_clearance'


def test_remeasure_deviation():
    close = [NECK + [0.0, 0.0, 0.004], NECK + [0.0, 0.0, 0.006], NECK + [0.0, 0.0, 0.5]]
    assert remeasure_deviation(NECK, close) == pytest.approx(0.006)
    assert np.isnan(remeasure_deviation(NECK, []))
    assert remeasure_deviation(NECK, [NECK + 0.003 * AXIS]) == pytest.approx(0.003)
