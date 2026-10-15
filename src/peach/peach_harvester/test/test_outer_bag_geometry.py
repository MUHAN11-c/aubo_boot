"""Outer bag contact uses observed endpoints, never an inferred hidden fruit."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from peach_common.yaml_params import dict_to_ns, load_ros_parameters
from peach_harvester.vision.common.bag_landmarks import (
    BagLandmarks, estimate_bag_landmarks,
)
from peach_harvester.vision.common.tool_budget import ToolBudgetParams
from peach_harvester.vision.scene_perception.contracts import (
    compute_travel_range,
)
from peach_harvester.vision.scene_perception.params import ScenePerceptionParams
from peach_harvester.vision.scene_perception.pipeline import PerceptionPipeline, SyncedRgbd
from peach_harvester.vision.target_reconstruction.refine import fuse_bag_views
import pytest
from std_msgs.msg import Header


def _view(bottom=None, neck=None):
    return BagLandmarks(
        bottom_center=np.array([0.0, 0.0, 0.4]) if bottom is None else bottom,
        neck_center=np.array([0.0, 0.0, 0.6]) if neck is None else neck,
        bag_axis=np.array([0.0, 0.0, 1.0]), d95_m=0.07,
        sigma_position_m=0.008, sigma_axis_deg=3.0, occlusion_class='clear')


def _surface():
    # The visible side of a tapered bag bends; its surface centroid is not
    # the physical centreline. The declared axis remains the process axis.
    z, theta = np.meshgrid(np.linspace(0.0, 0.2, 40),
                           np.linspace(-0.9, 0.9, 30))
    radius = 0.045 - 0.12 * z
    cloud = np.column_stack((
        (radius * np.cos(theta)).ravel(),
        (radius * np.sin(theta)).ravel(), z.ravel()))
    return cloud + [0.0, 0.0, 0.4]


def test_bag_process_axis_does_not_follow_local_cylinder_fit(monkeypatch):
    from peach_harvester.vision.scene_perception import pose_pipelines
    from peach_harvester.vision.scene_perception.contracts import BagObservation
    depth = np.full((160, 120), 700, dtype=np.uint16)
    mask = np.zeros(depth.shape, dtype=bool)
    mask[20:140, 40:80] = True
    observation = BagObservation(
        rgb=np.zeros((160, 120, 3), dtype=np.uint8), depth=depth,
        camera_K={'fx': 500, 'fy': 500, 'cx': 60, 'cy': 80},
        gravity_hint=np.array([0.0, 1.0, 0.0]))
    axes = []
    for fitted_axis in ([1.0, 0.0, 0.0], [0.6, -0.8, 0.0], [0.0, 0.0, 1.0]):
        monkeypatch.setattr(pose_pipelines, 'fit_cylinder_robust',
                            lambda *_, axis=fitted_axis: {
                                'axis': np.array(axis), 'q0': np.array([0., 0., .7]),
                                'inlier_ratio': .9, 'rms': .001, 'radius': .03})
        result = pose_pipelines.RobustBagPosePipeline().estimate(
            observation, 't', (30, 10, 90, 150), mask, 'sam')
        axes.append(result.grasp_3d.translation_direction)
    np.testing.assert_allclose(axes[0], axes[1], atol=1e-9)
    np.testing.assert_allclose(axes[0], axes[2], atol=1e-9)
    np.testing.assert_allclose(axes[0], [0, -1, 0], atol=0.05)


def test_observed_bag_mouth_authorizes_contact_without_hidden_fruit_sphere():
    fused = fuse_bag_views([_view()])
    assert fused.ok
    assert fused.allowed
    assert fused.budget['sleeve_capability'] == 0
    assert fused.budget['cut_capability'] == 0
    np.testing.assert_allclose(fused.cut_pose, [0.0, 0.0, 0.6])
    assert fused.cut_travel_m == pytest.approx(0.2)
    assert fused.fruit_prior_radius_m == 0.0
    assert 'cut_band_unavailable' not in fused.flags


def test_single_sided_surface_envelope_is_diagnostic_not_contact_veto():
    z = np.linspace(0.0, 0.2, 40)
    theta = np.linspace(-0.9, 0.9, 30)
    cloud = np.array([[0.035 * np.cos(t) + 0.5 * h,
                       0.035 * np.sin(t), 0.4 + h]
                      for h in z for t in theta])
    fused = fuse_bag_views([_view()], cloud_xyz=cloud)
    assert fused.axis_conflict_deg > 12.0
    assert fused.budget['sleeve_capability'] == 0
    assert fused.allowed
    assert fused.sigma_axis_deg == pytest.approx(3.0)


@pytest.mark.parametrize('neck', [
    np.array([0.0, 0.0, 0.4]),
    np.array([float('nan'), 0.0, 0.6]),
])
def test_degenerate_or_nonfinite_endpoints_never_authorize_contact(neck):
    fused = fuse_bag_views([_view(neck=neck)])
    assert not fused.ok
    assert not fused.allowed
    assert not fused.budget


def test_primary_axis_is_not_rotated_by_single_sided_endpoint_centroids():
    landmarks = estimate_bag_landmarks(
        _surface(), gravity=np.array([0.0, 0.0, -1.0]),
        axis_hint=np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(landmarks.bag_axis, [0.0, 0.0, 1.0], atol=1e-8)
    delta = landmarks.neck_center - landmarks.bottom_center
    np.testing.assert_allclose(delta[:2], [0.0, 0.0], atol=1e-8)
    assert delta[2] > 0.1


def test_conditioned_whole_bag_axis_is_not_rotated_by_endpoint_centroids():
    cloud = _surface()
    _, _, directions = np.linalg.svd(cloud - np.median(cloud, axis=0), full_matrices=False)
    expected = directions[0]
    if expected[2] < 0:
        expected = -expected
    landmarks = estimate_bag_landmarks(cloud, gravity=[0, 0, -1])
    np.testing.assert_allclose(landmarks.bag_axis, expected, atol=1e-9)


def test_tcp_at_cutting_plane_does_not_subtract_legacy_mechanical_blade_length():
    snapshot = SimpleNamespace(
        tool=SimpleNamespace(D_inner=0.104, L_insert=0.03, L_blade=0.037,
                             entry_d_tool=0.0, entry_d_s=0.0,
                             clearance_min=0.005, margin_neck=0.015,
                             version='1.2'),
        gravity_hint_xyz='0.0,0.0,-1.0', gravity_mode='tf',
        pipeline=SimpleNamespace(min_depth_m=0.3, max_depth_m=1.5))
    tool, _, _ = ScenePerceptionParams._derive(snapshot)
    _, travel = compute_travel_range(
        np.array([0.0, 0.0, 0.4]), np.array([0.0, 0.0, 0.445]),
        np.array([0.0, 0.0, 1.0]), tool)
    assert travel == pytest.approx(0.03)
    assert tool.blade_offset_m == 0.0


@pytest.mark.parametrize('offset,expected', [(0.01, 0.61), (-0.015, 0.585)])
def test_cut_reference_uses_one_explicit_process_offset(offset, expected):
    fused = fuse_bag_views(
        [_view()], params=ToolBudgetParams(cut_reference_offset_m=offset))
    assert fused.allowed
    np.testing.assert_allclose(fused.cut_pose, [0.0, 0.0, expected])
    assert fused.cut_travel_m == pytest.approx(expected - 0.4)


def test_auxiliary_bag_surface_sphere_does_not_claim_internal_fruit_clearance():
    view = _view()
    view.fruit_prior_center = np.array([0.0, 0.0, 0.58])
    view.fruit_prior_radius_m = 0.04
    fused = fuse_bag_views([view])
    assert fused.cut_to_fruit_m < 0.0
    assert fused.allowed
    np.testing.assert_allclose(fused.cut_pose, [0.0, 0.0, 0.6])


@pytest.mark.parametrize('axis', [np.zeros(3), np.array([0.0, 0.0, -1.0])])
def test_invalid_or_opposed_declared_axis_cannot_authorize_endpoints(axis):
    view = _view()
    view.bag_axis = axis
    fused = fuse_bag_views([view])
    assert not fused.ok
    assert not fused.allowed


def test_detected_bag_without_depth_is_reported_but_never_a_motion_candidate():
    params = ScenePerceptionParams.from_params(dict_to_ns(load_ros_parameters(
        Path(__file__).resolve().parents[1] / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')))
    pipeline = PerceptionPipeline(params, SimpleNamespace(now=lambda: 1.0))
    mask = np.zeros((48, 64), dtype=bool)
    mask[10:35, 20:40] = True
    detection = {'bbox': (18, 8, 42, 37), 'class_id': 0,
                 'class_name': 'peach_bag', 'conf': 0.9}
    # Only GPU inference is doubled. Geometry, registration, message
    # construction and the confirmed-candidate policy remain real.
    pipeline.engine = SimpleNamespace(
        detect=lambda _rgb: [detection],
        segment=lambda _rgb, boxes: [(mask, boxes[0])])
    header = Header(frame_id='camera_color_optical_frame')
    frame = SyncedRgbd(
        rgb=np.zeros((48, 64, 3), dtype=np.uint8),
        depth=np.zeros((48, 64), dtype=np.uint16),
        K={'fx': 500.0, 'fy': 500.0, 'cx': 32.0, 'cy': 24.0, 'width': 64, 'height': 48},
        cam_frame=header.frame_id, out_frame='base_link',
        geometry_stamp=header.stamp, img_header=header, header=header,
        T_out_cam=np.eye(4), tf_status='ok', gravity_hint=np.array([0.0, 1.0, 0.0]))
    result = pipeline.process(frame)
    assert len(result.kept) == 1
    assert not result.candidates.candidates
    assert len(result.fittings.fittings) == 1
    assert result.fittings.fittings[0].target_id == 'untracked_0'
    assert 'mask_unavailable' in result.fittings.fittings[0].diagnostic_flags


def test_primary_centreline_origin_is_not_replaced_by_visible_surface():
    landmarks = estimate_bag_landmarks(
        _surface(), gravity=np.array([0.0, 0.0, -1.0]),
        axis_hint=np.array([0.0, 0.0, 1.0]), axis_origin=np.array([0.0, 0.0, 0.4]))
    np.testing.assert_allclose(landmarks.bottom_center[:2], [0.0, 0.0], atol=1e-8)
    np.testing.assert_allclose(landmarks.neck_center[:2], [0.0, 0.0], atol=1e-8)


def test_fusion_does_not_move_cut_point_to_single_visible_side():
    fused = fuse_bag_views([_view()], cloud_xyz=_surface())
    assert fused.allowed
    np.testing.assert_allclose(fused.cut_pose, [0.0, 0.0, 0.6], atol=1e-8)


@pytest.mark.parametrize('reason', [
    'bag_axis_invalid', 'bag_endpoints_invalid', 'cut_reference_invalid',
    'fusion_failed', 'landmark_axis_conflict',
    'bag_axis_unobservable', 'bag_axis_envelope_ill_conditioned',
    'bag_axis_silhouette_unobservable', 'fused_axis_silhouette_conflict',
])
def test_new_invalid_geometry_revokes_previous_node_contact_authority(reason):
    from peach_harvester.vision.target_reconstruction.refine import BagModel, RefitResult
    from peach_harvester.vision.target_reconstruction.target_reconstruction_node import (
        TargetReconstructionNode,
    )
    previous = RefitResult(ok=True, budget={'allowed': True})
    failed = RefitResult(ok=False, reason=reason)
    bad_model = BagModel(ok=False, reason=reason, allowed=False)
    host = SimpleNamespace(
        _refined=previous, _bag_model=BagModel(ok=True, allowed=True),
        _tsdf_cloud_cache=None, _bound_axis_hint=np.array([0.0, 0.0, 1.0]),
        collector=SimpleNamespace(frames=[], target_center=None, target_id='t1'),
        params=SimpleNamespace(
            refit=SimpleNamespace(entry_standoff_m=0.0, pregrasp_standoff_m=0.03),
            tool=SimpleNamespace(budget=ToolBudgetParams())),
        _resolve_target_kind=lambda: ('bag', False),
        _refit_orchestrator=SimpleNamespace(run=lambda **_: (failed, bad_model)),
        _on_view_geometry=lambda *_: None, _log_geometry_row=lambda *_: None,
        _bump_products_version=lambda: None, _products_force_publish=False)
    TargetReconstructionNode._run_refit(host, keep_last_good=True)
    assert host._refined is failed
    assert host._bag_model is bad_model
    assert not host._bag_model.allowed
