"""Regression gates and independent diagnostics for landmark fusion (zero ROS)."""
import json

import numpy as np
from sim_perception.vision.common.bag_landmarks import BagLandmarks
from sim_perception.vision.target_reconstruction.refine import (
    BagModel, fuse_bag_views, merge_fused_bag_model, RefitResult,
    STATUS_ACCEPT, STATUS_REOBSERVE,
)
import pytest


def _axis(degrees):
    radians = np.deg2rad(degrees)
    return np.array([np.sin(radians), 0.0, np.cos(radians)])


def _refit(axis=None, status=STATUS_ACCEPT, ok=True):
    return RefitResult(
        ok=ok, status=status, kind='cylinder', n_points=300,
        axis=np.array([0.0, 0.0, 1.0]) if axis is None else axis.copy(),
        rmse=0.002, inlier_ratio=0.85)


def _fused(axis):
    bottom = np.array([0.0, 0.0, 0.4])
    neck = bottom + 0.2 * axis
    return BagModel(
        ok=True, axis=axis.copy(), bottom=bottom, neck=neck,
        d95_m=0.07, length_m=0.2, rmse=0.03, inlier_ratio=0.0,
        budget={'allowed': True}, allowed=True, view_count=1)


def test_merge_preserves_point_fit_quality_and_original_refit_snapshot():
    original_axis = _axis(10.0)
    result = merge_fused_bag_model(
        _refit(original_axis), _fused(_axis(15.0)), 1, _axis(0.0))
    assert result.rmse == 0.002
    assert result.inlier_ratio == 0.85
    np.testing.assert_allclose(result.refit_axis, original_axis)
    assert result.refit_status == STATUS_ACCEPT
    assert result.refit_ok
    assert result.refit_rmse == 0.002
    assert result.refit_inlier_ratio == 0.85
    assert result.fused_refit_axis_angle_deg == pytest.approx(5.0)
    exported = result.as_dict()
    assert exported['refit_axis'] == original_axis.tolist()
    assert exported['refit_status'] == STATUS_ACCEPT
    json.dumps(exported, allow_nan=False)


@pytest.mark.parametrize('hint', [_axis(0.0), None])
def test_105_degree_conflict_against_accepted_refit_clears_authority(hint):
    fused = _fused(_axis(105.0))
    result = merge_fused_bag_model(_refit(), fused, 1, hint)
    assert result.status == STATUS_REOBSERVE
    assert not result.ok
    assert result.reason == 'fused_axis_conflict'
    assert 'fused_axis_conflict' in result.flags
    assert result.budget == {}
    assert not fused.ok
    assert not fused.allowed
    assert fused.budget == {}
    assert result.fused_refit_axis_angle_deg == pytest.approx(105.0)
    np.testing.assert_allclose(result.axis, fused.axis)


@pytest.mark.parametrize('degrees,hint_degrees', [
    (45.0, -50.0), (90.0, -20.0), (70.0, -40.0)])
def test_tilted_or_horizontal_bag_consistent_with_refit_is_not_hint_vetoed(
        degrees, hint_degrees):
    axis = _axis(degrees)
    result = merge_fused_bag_model(_refit(axis), _fused(axis), 1, _axis(hint_degrees))
    assert result.status == STATUS_ACCEPT
    assert result.ok
    assert result.budget['allowed']
    assert result.fused_axis_angle_deg == pytest.approx(degrees - hint_degrees)
    np.testing.assert_allclose(result.axis, axis)


@pytest.mark.parametrize('degrees, rejected', [
    (34.99, False), (35.0, False), (35.01, True)])
def test_accepted_refit_conflict_gate_uses_configured_angle_boundary(degrees, rejected):
    result = merge_fused_bag_model(_refit(), _fused(_axis(degrees)), 1, None)
    assert (result.status == STATUS_REOBSERVE) is rejected
    assert ('fused_axis_conflict' in result.flags) is rejected


@pytest.mark.parametrize('degrees, hint, rejected', [
    (70.0, _axis(0.0), False), (90.0, _axis(0.0), False),
    (90.01, _axis(0.0), True), (105.0, None, False),
])
def test_without_accepted_refit_only_hint_polarity_conflict_is_vetoed(degrees, hint, rejected):
    result = merge_fused_bag_model(
        _refit(status=STATUS_REOBSERVE), _fused(_axis(degrees)), 1, hint)
    assert (result.status == STATUS_REOBSERVE) is rejected
    assert ('fused_axis_conflict' in result.flags) is rejected


def _view(lateral):
    return BagLandmarks(
        bottom_center=np.array([lateral, 0.0, 0.4]),
        neck_center=np.array([lateral, 0.0, 0.6]),
        bag_axis=np.array([0.0, 0.0, 1.0]), d95_m=0.07,
        sigma_position_m=0.005, sigma_axis_deg=1.0)


def _cloud(lateral):
    theta = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
    return np.array([[lateral + 0.035 * np.cos(t), 0.035 * np.sin(t), z]
                     for z in np.linspace(0.4, 0.6, 41) for t in theta])


def test_single_pose_keeps_landmark_line_without_claiming_multiview_consistency():
    fused = fuse_bag_views([_view(0.0)], cloud_xyz=_cloud(0.04))
    assert fused.ok
    assert fused.fusion_lateral_shift_m == pytest.approx(0.0)
    assert fused.fusion_pose_count == 1
    assert not fused.fusion_consistency_available
    assert fused.fusion_landmark_scatter_rmse_m is None
    assert fused.fusion_landmark_inlier_ratio is None
    result = merge_fused_bag_model(_refit(), fused, 1, None)
    exported = result.as_dict()
    assert exported['fusion_lateral_shift_m'] == pytest.approx(0.0)
    assert exported['fusion_pose_count'] == 1
    assert exported['fusion_consistency_available'] is False
    assert exported['fusion_landmark_scatter_rmse_m'] is None
    assert exported['fusion_landmark_inlier_ratio'] is None
    assert exported['rmse'] == 0.002
    assert exported['inlier_ratio'] == 0.85
    json.dumps(exported, allow_nan=False)


def test_multiview_scatter_uses_huber_endpoints_without_surface_translation():
    fused = fuse_bag_views([_view(-0.01), _view(0.01)], cloud_xyz=_cloud(0.04))
    assert fused.ok
    assert fused.fusion_pose_count == 2
    assert fused.fusion_consistency_available
    assert fused.fusion_lateral_shift_m == pytest.approx(0.0)
    assert fused.fusion_landmark_scatter_rmse_m == pytest.approx(0.01)
    assert fused.fusion_landmark_inlier_ratio == 1.0


def test_multiview_scatter_precedes_upper_hemisphere_polarity_adjustment():
    views = [_view(0.0), _view(0.0)]
    for view in views:
        view.bottom_center, view.neck_center = view.neck_center, view.bottom_center
        view.bag_axis = -view.bag_axis
    fused = fuse_bag_views(views)
    assert 'polarity_upper_hemisphere' in fused.flags
    assert fused.fusion_landmark_scatter_rmse_m == 0.0
    assert fused.fusion_landmark_inlier_ratio == 1.0
    np.testing.assert_allclose(fused.axis, [0.0, 0.0, 1.0])


def test_conflicted_fusion_is_unavailable_to_target_model_and_pregrasp_consumers():
    from types import SimpleNamespace

    from sim_perception.vision.target_reconstruction.publish import (
        build_pregrasp_verification, fill_target_model,
    )
    from peach_interfaces.msg import TargetModel
    from rclpy.time import Time
    from std_msgs.msg import Header

    fused = _fused(_axis(105.0))
    result = merge_fused_bag_model(_refit(), fused, 1, _axis(0.0))
    params = SimpleNamespace(tool=SimpleNamespace(profile_id='shear_v1'))
    model = TargetModel()
    fill_target_model(model, fused, result, params, Time(seconds=1.0))
    assert model.geometry_capability == TargetModel.CAPABILITY_UNKNOWN
    assert model.pregrasp_capability == TargetModel.CAPABILITY_UNKNOWN
    assert model.sleeve_capability == TargetModel.CAPABILITY_UNKNOWN
    assert model.cut_capability == TargetModel.CAPABILITY_UNKNOWN
    verification, _ = build_pregrasp_verification(
        Header(), fused, result, None, None, params)
    assert not verification.passed
    assert verification.reason == 'bag_model_unavailable'
