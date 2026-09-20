"""Zero-ROS tests for RefitResult/BagModel dataclasses + merge/collect (W4)."""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.common.bag_landmarks import BagLandmarks
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    collect_bag_views,
    merge_fused_bag_model,
    RefitResult,
    STATUS_ACCEPT,
    STATUS_REJECT,
    STATUS_REOBSERVE,
)


def _landmark(bottom, neck, d95=0.07):
    axis = neck - bottom
    axis = axis / np.linalg.norm(axis)
    return BagLandmarks(
        bottom_center=np.asarray(bottom, dtype=np.float64),
        neck_center=np.asarray(neck, dtype=np.float64),
        bag_axis=axis,
        d95_m=d95,
    )


def _fused_ok(bottom=(0.0, 0.0, 0.4), neck=(0.0, 0.0, 0.6), d95=0.07):
    bottom = np.asarray(bottom, dtype=np.float64)
    neck = np.asarray(neck, dtype=np.float64)
    axis = neck - bottom
    axis = axis / np.linalg.norm(axis)
    return BagModel(
        ok=True, bottom=bottom, neck=neck, axis=axis,
        entry=bottom - 0.01 * axis, pregrasp=bottom - 0.11 * axis,
        cut_plane_point=neck.copy(), cut_pose=neck.copy(),
        cut_travel_m=0.2, cut_to_fruit_m=0.01, d95_m=d95, length_m=0.2,
        sigma_position_m=0.005, sigma_axis_deg=4.0, rmse=0.004,
        inlier_ratio=0.9, view_count=3,
        budget={'allowed': True, 'reason': 'ok', 'failure_code': 0},
        allowed=True, radial_margin_m=0.01, axial_margin_m=0.02,
        corridor_clear=True, flags=['x'],
    )


def test_refit_result_defaults_match_legacy_fail_dict():
    result = RefitResult(ok=False, reason='empty_cloud', kind='')
    assert result.status == STATUS_REJECT
    assert result.radius == -1.0
    assert result.diameter == -1.0
    assert result.span_m == -1.0
    assert result.rmse == -1.0
    assert result.inlier_ratio == -1.0
    assert result.n_points == 0
    assert result.flags == []
    assert result.budget == {}
    assert result.axis_angle_deg is None


def test_as_dict_keeps_diag_keyset_and_json_native():
    result = merge_fused_bag_model(
        RefitResult(ok=True, kind='cylinder', status=STATUS_REOBSERVE,
                    n_points=100,
                    center=np.zeros(3), axis=np.array([0.0, 0.0, 1.0]),
                    bottom=np.zeros(3), neck=np.array([0.0, 0.0, 0.2]),
                    diameter=0.07, span_m=0.2, rmse=0.003,
                    inlier_ratio=0.8, flags=['low_inlier_ratio']),
        _fused_ok(), views_count=3, bound_axis_hint=None, target_id='t1')
    as_dict = result.as_dict()
    # 旧 _refined_diag_dict 键集全保留
    for key in ('ok', 'kind', 'status', 'center', 'axis', 'bottom', 'neck',
                'diameter', 'span_m', 'rmse', 'inlier_ratio', 'n_points',
                'flags', 'axis_angle_deg', 'axis_conflict_deg',
                'envelope_conditioned', 'envelope_reason', 'perception_axis'):
        assert key in as_dict, key
    # N1：双字段入投影；兼容键 = 融合值（此处 hint=None → None）
    assert 'refit_axis_angle_deg' in as_dict
    assert 'fused_axis_angle_deg' in as_dict
    # numpy → 原生类型（可直接 json.dumps）
    import json
    json.dumps(as_dict)
    assert as_dict['final'] is False


def test_n1_dual_axis_fields_not_overwritten():
    hint = np.array([0.0, 0.0, 1.0])
    result = RefitResult(
        ok=True, kind='cylinder', status=STATUS_ACCEPT, n_points=50,
        center=np.zeros(3),
        axis=np.array([0.0, 0.0871557, 0.9961947]),  # 与 hint 夹角约 5°
        bottom=np.zeros(3), neck=np.array([0.0, 0.0174, 0.1992]),
        diameter=0.07, span_m=0.2, rmse=0.002, inlier_ratio=0.9)
    result.refit_axis_angle_deg = 5.0
    fused = _fused_ok(bottom=(0.0, 0.0, 0.4), neck=(0.0, 0.0, 0.6))
    merged = merge_fused_bag_model(
        result, fused, views_count=2, bound_axis_hint=hint, target_id='t')
    # 融合轴与 hint 同向（0°）；refit 值 5° 不得被覆写（N1）
    assert merged.refit_axis_angle_deg == 5.0
    assert abs(merged.fused_axis_angle_deg) < 1e-6
    assert abs(merged.axis_angle_deg) < 1e-6  # 兼容键取融合值


def test_n1_compat_key_falls_back_to_refit_value_when_fusion_failed():
    hint = np.array([0.0, 0.0, 1.0])
    result = RefitResult(ok=True, kind='cylinder', status=STATUS_ACCEPT,
                         n_points=50, center=np.zeros(3),
                         axis=np.array([0.0, 0.0, 1.0]), bottom=np.zeros(3),
                         neck=np.array([0.0, 0.0, 0.2]), diameter=0.07,
                         span_m=0.2, rmse=0.002, inlier_ratio=0.9)
    result.refit_axis_angle_deg = 7.5
    failed = BagModel(ok=False, reason='no_landmark_views', allowed=False)
    merged = merge_fused_bag_model(
        result, failed, views_count=0, bound_axis_hint=hint, target_id='t')
    assert not merged.ok
    assert merged.status == STATUS_REOBSERVE
    assert 'bag_fusion_required' in merged.flags
    assert merged.budget == {}
    assert merged.fused_axis_angle_deg is None
    assert merged.axis_angle_deg == 7.5  # 旧语义：失败路径保留 refit 值


def test_n6_merge_preserves_refit_kind():
    sphere = RefitResult(
        ok=True, kind='sphere', status=STATUS_ACCEPT, n_points=80,
        center=np.zeros(3), axis=np.array([0.0, 0.0, 1.0]),
        bottom=np.zeros(3), neck=np.array([0.0, 0.0, 0.14]),
        radius=0.035, diameter=0.07, span_m=0.07, rmse=0.002,
        inlier_ratio=0.9)
    merged = merge_fused_bag_model(
        sphere, _fused_ok(), views_count=2, bound_axis_hint=None,
        target_id='fruit-1')
    assert merged.kind == 'sphere'  # N6：不再强制 'cylinder'


def test_merge_success_geometry_contract():
    fused = _fused_ok()
    result = RefitResult(ok=True, kind='cylinder', status=STATUS_ACCEPT,
                         n_points=120, center=np.zeros(3),
                         axis=np.array([0.0, 0.0, 1.0]), bottom=np.zeros(3),
                         neck=np.array([0.0, 0.0, 0.2]), diameter=0.06,
                         span_m=0.2, rmse=0.004, inlier_ratio=0.7)
    merged = merge_fused_bag_model(
        result, fused, views_count=3, bound_axis_hint=None, target_id='t-9')
    assert merged.ok
    assert merged.model_revision == 't-9:3'
    assert merged.status == STATUS_ACCEPT
    np.testing.assert_allclose(merged.bottom, fused.bottom)
    np.testing.assert_allclose(merged.d95_m, fused.d95_m)
    assert merged.diameter == fused.d95_m
    assert merged.radius == 0.5 * fused.d95_m
    assert merged.span_m == fused.length_m
    assert merged.budget == fused.budget
    assert merged.entry is fused.entry
    assert merged.cut_travel_m == fused.cut_travel_m
    assert merged.cut_to_fruit_m == fused.cut_to_fruit_m
    # center 重算为融合底/颈中点
    np.testing.assert_allclose(
        merged.center, 0.5 * (fused.bottom + fused.neck))


def test_bag_model_drops_dead_keys_by_schema():
    fused = _fused_ok()
    dataclass_fields = set(fused.__dataclass_fields__)
    for dead in ('cut_normal', 'fruit_prior_auxiliary', 'envelope_span_m',
                 'envelope_d95_m', 'detection_conflict_deg', 'axis_point'):
        assert dead not in dataclass_fields, dead
    result_fields = set(RefitResult.__dataclass_fields__)
    assert 'axis_point' not in result_fields
    assert 'diagnostic_axis_mismatch' not in result_fields


def test_collect_bag_views_selects_best_member_per_cluster():
    class _Frame:
        def __init__(self, ratio, cloud, position):
            self.valid_depth_ratio = ratio
            self.cloud_base = cloud
            self.camera_position_base = np.asarray(position)
            self.stamp = 1.0
            self.registration = {}
            self.diagnostic_flags = []

    cloud = np.random.default_rng(0).random((200, 3)) * 0.05
    # 两个机位（方向差 > 5°），各两帧同簇（同帧堆叠），同簇取 ratio 最高者
    frames = [
        _Frame(0.5, cloud, [0.3, 0.0, 0.6]),
        _Frame(0.9, cloud, [0.3, 0.001, 0.6]),  # 同簇更优
        _Frame(0.6, cloud, [-0.3, 0.0, 0.6]),
    ]
    seen = []
    views = collect_bag_views(
        frames, np.zeros(3), on_view=lambda lm, f: seen.append(f))
    # 每簇估计一次 landmarks；on_view 逐视角回调
    assert len(views) == 2
    assert seen == [frames[1], frames[2]]
    for view in views:
        assert view.bottom_center is not None
