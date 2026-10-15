"""Zero-ROS tests for RefitOrchestrator.run (fake refitters, no open3d)."""
from __future__ import annotations

import numpy as np

from peach_harvester.vision.common.tool_budget import ToolBudgetParams
from peach_harvester.vision.target_reconstruction.capture import TimingStats
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    merge_fused_bag_model,
    RefitConfig,
    RefitResult,
    STATUS_ACCEPT,
)
from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
    RefitOrchestrator,
)
import pytest


class _FakeRefitter:
    """按预设返回结果的假拟合线（绕开 open3d 依赖）."""

    def __init__(self, product=None, error=None):
        self.product = product
        self.error = error
        self.calls = []

    def refit(self, xyz, kind, config, axis_hint):
        self.calls.append((xyz.shape[0], kind, axis_hint is not None))
        if self.error is not None:
            raise self.error
        return self.product


class _Logger:
    def __init__(self):
        self.records = []

    def info(self, msg, **kwargs):
        self.records.append(('info', msg))

    def warning(self, msg, **kwargs):
        self.records.append(('warning', msg))

    def debug(self, msg, **kwargs):
        self.records.append(('debug', msg))


def _ok_refit(kind='cylinder', n=500):
    return RefitResult(
        ok=True, kind=kind, status=STATUS_ACCEPT, n_points=n,
        center=np.zeros(3), axis=np.array([0.0, 0.0, 1.0]),
        bottom=np.array([0.0, 0.0, 0.4]),
        neck=np.array([0.0, 0.0, 0.6]),
        diameter=0.07, span_m=0.2, rmse=0.002, inlier_ratio=0.9,
        flags=['from_refit'])


def _bag_cloud(seed=1, n=420):
    """可见半圆柱面点云（z 0.40→0.60，半径 0.032；袋状细长体）."""
    rng = np.random.default_rng(seed)
    z = rng.uniform(0.40, 0.60, size=n)
    theta = rng.uniform(-np.pi / 2, np.pi / 2, size=n)  # +x 侧可见半周
    radius = 0.032 + rng.normal(0.0, 0.001, size=n)
    return np.stack([
        radius * np.cos(theta), radius * np.sin(theta), z], axis=1)


def _frames(n=2):
    cloud = _bag_cloud()
    frames = []
    for i in range(n):
        position = np.array(
            [0.25 if i % 2 == 0 else -0.25, 0.0, 0.6])

        class _Frame:
            valid_depth_ratio = 0.8
            cloud_base = cloud
            stamp = 2.0
            registration = {}
            diagnostic_flags = []

        _Frame.camera_position_base = position
        frames.append(_Frame())
    return frames


def _make(refitter, clock_values=None):
    clock = iter(clock_values or [0.0] * 100)
    return RefitOrchestrator(
        {'cylinder': refitter, 'sphere': refitter},
        RefitConfig(), TimingStats(), _Logger(), now=lambda: next(clock))


def test_accepted_reconstruction_axis_owns_landmarks_despite_noisy_live_hint():
    primary = _ok_refit()
    primary.center = np.array([0.0, 0.0, 0.5])
    hint = np.array([0.8, 0.0, 0.6])
    first = _silhouette_frame()
    second = _silhouette_frame()
    second.T_base_camera = second.T_base_camera.copy()
    second.T_base_camera[0, 3] = .1
    second.camera_position_base = second.T_base_camera[:3, 3]
    second.target_mask = np.roll(second.target_mask, -80, axis=1)
    result, fused = _make(_FakeRefitter(product=primary)).run(
        tsdf_xyz=_bag_cloud(), frames=[first, second], target_center=[0.0, 0.0, 0.5],
        kind='cylinder', kind_defaulted=False, bound_axis_hint=hint,
        target_id='t1', entry_standoff_m=0.0, pregrasp_standoff_m=0.03,
        budget_params=ToolBudgetParams(), mark_final=True)
    assert result.ok
    assert fused.ok
    np.testing.assert_allclose(fused.axis, [0.0, 0.0, 1.0], atol=1e-9)
    np.testing.assert_allclose(fused.cut_pose[:2], [0.0, 0.0], atol=1e-9)
    assert 'axis_from_reconstruction' in result.flags


def test_run_success_path_merges_and_marks_final():
    refitter = _FakeRefitter(product=_ok_refit())
    orch = _make(refitter)
    xyz = _bag_cloud(n=600)
    result, fused = orch.run(
        tsdf_xyz=xyz, frames=_frames(), target_center=np.zeros(3),
        kind='bag', kind_defaulted=False, bound_axis_hint=None,
        target_id='tgt-1', entry_standoff_m=0.0, pregrasp_standoff_m=0.0,
        budget_params=ToolBudgetParams(), mark_final=True)
    assert refitter.calls == [(600, 'bag', False)]
    assert fused.ok
    assert result.ok
    assert result.final is True
    assert result.budget  # 融合预算并入
    assert result.model_revision == 'tgt-1:2:1'  # G3：首轮计数=1
    assert 'from_refit' in result.flags
    # 分支日志命中成功行
    assert any('REFINING 完成' in msg for _, msg in orch._logger.records)


def test_run_no_tsdf_cloud_fuses_from_views_only():
    refitter = _FakeRefitter(product=_ok_refit())
    orch = _make(refitter)
    result, fused = orch.run(
        tsdf_xyz=None, frames=_frames(), target_center=np.zeros(3),
        kind='bag', kind_defaulted=False, bound_axis_hint=None,
        target_id='t', entry_standoff_m=0.0, pregrasp_standoff_m=0.0,
        budget_params=ToolBudgetParams())
    assert refitter.calls == []  # 无云不跑拟合线
    assert fused.ok
    assert result.ok  # 融合成功即 ok（旧 no_tsdf_cloud 失败被 merge 覆盖）
    assert result.model_revision == 't:2:1'


def test_run_exception_path_records_timing_and_fuses():
    refitter = _FakeRefitter(error=RuntimeError('boom'))
    orch = _make(refitter)
    result, fused = orch.run(
        tsdf_xyz=np.zeros((50, 3)), frames=_frames(), target_center=None,
        kind='bag', kind_defaulted=False, bound_axis_hint=None,
        target_id='t', entry_standoff_m=0.0, pregrasp_standoff_m=0.0,
        budget_params=ToolBudgetParams())
    assert result.reason == 'bag_axis_unobservable'
    assert not result.ok
    assert any('refit 异常' in msg for _, msg in orch._logger.records)
    assert orch._timing.snapshot()['refit_ms_last'] == 0.0


def test_run_keep_last_good_branch_logs_retention():
    bottom = np.array([0.0, 0.0, 0.4])
    neck = np.array([0.0, 0.0, 0.6])
    good_fused = BagModel(
        ok=True, bottom=bottom, neck=neck, axis=np.array([0.0, 0.0, 1.0]),
        entry=bottom, pregrasp=bottom, cut_plane_point=neck, cut_pose=neck,
        cut_travel_m=0.2, d95_m=0.065, length_m=0.2,
        budget={'allowed': True, 'reason': 'ok'}, allowed=True)
    good = merge_fused_bag_model(
        _ok_refit(), good_fused, views_count=2,
        bound_axis_hint=None, target_id='t')
    # 让融合失败：空 views → no_landmark_views

    class _EmptyFrame:
        valid_depth_ratio = 0.8
        cloud_base = None
        camera_position_base = np.array([0.3, 0.0, 0.6])
        stamp = 1.0
        registration = {}
        diagnostic_flags = []

    orch = _make(_FakeRefitter(product=_ok_refit()))
    result, fused = orch.run(
        tsdf_xyz=np.zeros((50, 3)), frames=[_EmptyFrame()],
        target_center=None, kind='bag', kind_defaulted=False,
        bound_axis_hint=None, target_id='t', entry_standoff_m=0.0,
        pregrasp_standoff_m=0.0, budget_params=ToolBudgetParams(),
        previous=good)
    assert not result.ok
    assert not fused.ok
    # 节点侧按同一判据（previous.ok and previous.budget）保持缓存不动；
    # 编排器只打保留日志
    assert any('保留上一帧袋模型' in msg for _, msg in orch._logger.records)


def test_run_kind_defaulted_appends_flag():
    refitter = _FakeRefitter(product=_ok_refit())
    orch = _make(refitter)
    result, _ = orch.run(
        tsdf_xyz=np.zeros((50, 3)), frames=[], target_center=None,
        kind='bag', kind_defaulted=True, bound_axis_hint=None,
        target_id='t', entry_standoff_m=0.0, pregrasp_standoff_m=0.0,
        budget_params=ToolBudgetParams())
    assert 'target_kind_defaulted' in result.flags


def test_run_view_sink_called_per_view():
    seen = []
    refitter = _FakeRefitter(product=_ok_refit())
    orch = _make(refitter)
    orch.run(
        tsdf_xyz=None, frames=_frames(2), target_center=np.zeros(3),
        kind='bag', kind_defaulted=False, bound_axis_hint=None,
        target_id='t', entry_standoff_m=0.0, pregrasp_standoff_m=0.0,
        budget_params=ToolBudgetParams(), on_view=lambda lm, f: seen.append(f))
    assert len(seen) == 2


def _run_kwargs(**overrides):
    kwargs = {
        'tsdf_xyz': None, 'frames': _frames(), 'target_center': np.zeros(3),
        'kind': 'bag', 'kind_defaulted': False, 'bound_axis_hint': None,
        'target_id': 't', 'entry_standoff_m': 0.0, 'pregrasp_standoff_m': 0.0,
        'budget_params': ToolBudgetParams()}
    kwargs.update(overrides)
    return kwargs


def test_revision_monotonic_same_target_and_views():
    """G3：同目标两次 finalize 且聚类机位数相同 → revision 必不同."""
    orch = _make(_FakeRefitter(product=_ok_refit()))
    results = [orch.run(**_run_kwargs())[0] for _ in range(3)]
    revisions = [r.model_revision for r in results]
    assert len(set(revisions)) == 3  # 两两不同
    # 前两段不变（target:views），尾段为严格递增计数
    assert all(rev.startswith('t:2:') for rev in revisions)
    counters = [int(rev.rsplit(':', 1)[1]) for rev in revisions]
    assert counters == [1, 2, 3]


def test_revision_counter_survives_reset_and_target_switch():
    """
    G3：reset_reconstruction 不清计数（orchestrator 无 reset 口）.

    模拟现场序：Build A → reset（节点清帧栈/缓存，计数不动）→ 重 Build A
    （同机位数）→ 切目标 B——revision 尾段全程单调，无重复。
    """
    orch = _make(_FakeRefitter(product=_ok_refit()))
    first, _ = orch.run(**_run_kwargs(target_id='A'))
    # reset_reconstruction 在节点侧只清 collector/产物缓存，不触碰
    # orchestrator；此处无操作即等价模拟
    rebuild, _ = orch.run(**_run_kwargs(target_id='A'))
    other, _ = orch.run(**_run_kwargs(target_id='B'))
    assert first.model_revision != rebuild.model_revision
    assert first.model_revision != other.model_revision
    counters = [
        int(rev.rsplit(':', 1)[1])
        for rev in (first.model_revision, rebuild.model_revision,
                    other.model_revision)]
    assert counters == sorted(counters) == [1, 2, 3]


def test_run_passes_configured_fused_refit_axis_gate_and_does_not_log_retention(monkeypatch):
    refit = _ok_refit()
    refit.axis = np.array([np.sin(np.deg2rad(25)), 0.0, np.cos(np.deg2rad(25))])
    orch = _make(_FakeRefitter(product=refit))
    orch._refit_config.max_axis_angle_deg = 10.0
    # Independently contradictory endpoint evidence must still be rejected.
    from peach_harvester.vision.common.bag_landmarks import BagLandmarks
    from peach_harvester.vision.target_reconstruction import refit_orchestrator
    monkeypatch.setattr(refit_orchestrator, 'collect_bag_views', lambda *_, **__: [
        BagLandmarks(bottom_center=np.array([0.0, 0.0, 0.4]),
                     neck_center=np.array([0.0, 0.0, 0.6]),
                     bag_axis=np.array([0.0, 0.0, 1.0]), d95_m=0.07)])
    refit.center = np.array([0.0, 0.0, 0.5])
    frame = _silhouette_frame()
    y, x = np.ogrid[:320, :320]
    angle = np.deg2rad(25.0)
    along = np.sin(angle) * (x - 160) - np.cos(angle) * (y - 160)
    across = np.cos(angle) * (x - 160) + np.sin(angle) * (y - 160)
    frame.target_mask = ((np.abs(along) < 80) & (np.abs(across) < 28)).astype(np.uint8)
    second = _silhouette_frame()
    second.T_base_camera = second.T_base_camera.copy()
    second.T_base_camera[0, 3] = .1
    second.camera_position_base = second.T_base_camera[:3, 3]
    second.target_mask = np.roll(frame.target_mask, -80, axis=1)
    result, fused = orch.run(**_run_kwargs(
        tsdf_xyz=_bag_cloud(), frames=[frame, second], target_center=[0, 0, .5],
        previous=_ok_refit()))
    assert result.reason == 'fused_axis_conflict'
    assert not result.ok
    assert not result.budget
    assert not fused.allowed
    assert not any('保留上一帧袋模型' in msg for _, msg in orch._logger.records)


def test_same_pose_burst_is_not_multiview_consistency():
    frames = _frames(4)
    for frame in frames:
        frame.camera_position_base = np.array([0.25, 0.0, 0.6])
    result, fused = _make(_FakeRefitter()).run(**_run_kwargs(frames=frames))
    assert fused.view_count == 1
    assert fused.fusion_pose_count == 1
    assert not result.fusion_consistency_available
    assert result.fusion_landmark_scatter_rmse_m is None


def test_unavailable_pose_coverage_does_not_claim_distinct_views():
    result, fused = _make(_FakeRefitter()).run(**_run_kwargs(target_center=None))
    assert fused.view_count == 2  # 原有 fallback 仍可融合逐帧关键点
    assert result.fusion_pose_count == 0
    assert not result.fusion_consistency_available
    assert result.fusion_landmark_scatter_rmse_m is None
    assert result.fusion_landmark_inlier_ratio is None


def _silhouette_frame():
    from types import SimpleNamespace
    mask = np.zeros((320, 320), dtype=np.uint8)
    mask[80:240, 132:188] = 255
    transform = np.eye(4)
    transform[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    transform[:3, 3] = [0, -0.5, 0.5]
    return SimpleNamespace(
        target_mask=mask, camera_K={'fx': 400, 'fy': 400, 'cx': 160, 'cy': 160},
        T_base_camera=transform, camera_position_base=transform[:3, 3],
        valid_depth_ratio=0.8, cloud_base=_bag_cloud(), stamp=2.0,
        registration={}, diagnostic_flags=[])


def test_low_residual_crosswise_refit_cannot_replace_observed_bag_axis():
    primary = _ok_refit()
    primary.center = np.array([0.0, 0.0, 0.5])
    primary.axis = np.array([1.0, 0.0, 0.0])
    result, fused = _make(_FakeRefitter(product=primary)).run(
        tsdf_xyz=_bag_cloud(), frames=[_silhouette_frame()],
        target_center=[0.0, 0.0, 0.5], kind='cylinder', kind_defaulted=False,
        bound_axis_hint=np.array([0.0, 0.0, 1.0]), target_id='t1',
        entry_standoff_m=0.0, pregrasp_standoff_m=0.03,
        budget_params=ToolBudgetParams(), mark_final=True)
    assert fused.ok
    assert fused.axis[2] > np.cos(np.radians(3.0))
    assert result.ok
    assert 'refit_axis_silhouette_conflict' in result.flags
    assert 'axis_from_reconstruction' not in result.flags
    np.testing.assert_allclose(result.refit_axis, [1.0, 0.0, 0.0])


def test_refit_without_image_evidence_does_not_own_the_axis():
    primary = _ok_refit()
    primary.center = np.array([0.0, 0.0, 0.5])
    primary.axis = np.array([1.0, 0.0, 0.0])
    result, fused = _make(_FakeRefitter(product=primary)).run(
        tsdf_xyz=_bag_cloud(), frames=_frames(1), target_center=[0.0, 0.0, 0.5],
        kind='cylinder', kind_defaulted=False, bound_axis_hint=[0.0, 0.0, 1.0],
        target_id='t1', entry_standoff_m=0.0, pregrasp_standoff_m=0.03,
        budget_params=ToolBudgetParams(), mark_final=True)
    assert fused.ok
    assert fused.axis[2] > np.cos(np.radians(3.0))
    assert 'axis_from_reconstruction' not in result.flags
    assert 'refit_axis_support_unavailable' in result.flags


def test_parallel_axis_outside_original_silhouette_is_not_supported():
    from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
        reconstruction_axis_image_support,
    )
    fit = _ok_refit()
    fit.center = np.array([0.1, 0.0, 0.5])
    angle, invalid_views = reconstruction_axis_image_support([_silhouette_frame()], fit)
    assert angle == 90.0
    assert invalid_views == 0


def test_single_view_keeps_observed_center_when_local_fit_center_changes():
    primary = _ok_refit()
    fit_center = np.array([0., -.025, .5])
    primary.center = fit_center.copy()
    primary.axis = np.array([.35, 0., np.sqrt(1. - .35**2)])
    result, fused = _make(_FakeRefitter(product=primary)).run(
        **_run_kwargs(tsdf_xyz=_bag_cloud(), frames=[_silhouette_frame()],
                      target_center=[0, 0, .5], bound_axis_hint=[0, 0, 1]))
    assert result.ok
    assert 'axis_from_captured_shape' in result.flags
    assert fused.axis[2] > np.cos(np.radians(3.0))
    assert abs(float(fused.bottom[1] - fit_center[1])) > .015
    np.testing.assert_allclose(result.refit_center, [0., -.025, .5])
    assert np.degrees(np.arccos(np.dot(result.refit_axis, result.axis))) > 15


def test_auxiliary_single_view_cylinder_does_not_veto_valid_observed_shape():
    primary = _ok_refit()
    primary.center = np.array([0., 0., .5])
    primary.axis = np.array([0., -.8, .6])
    result, fused = _make(_FakeRefitter(product=primary)).run(
        **_run_kwargs(tsdf_xyz=_bag_cloud(), frames=[_silhouette_frame()],
                      target_center=[0, 0, .5], bound_axis_hint=[0, 0, 1]))
    assert result.refit_ok
    assert result.fused_refit_axis_angle_deg > 35
    assert result.ok
    assert fused.allowed
    assert 'axis_from_captured_shape' in result.flags


def test_same_view_burst_cannot_outvote_an_independent_silhouette_conflict():
    frames = []
    for x in (0., .003, .006):
        frame = _silhouette_frame()
        frame.T_base_camera = frame.T_base_camera.copy()
        frame.T_base_camera[0, 3] = x
        frame.camera_position_base = frame.T_base_camera[:3, 3]
        frame.target_mask = np.roll(frame.target_mask, -int(round(800*x)), axis=1)
        frames.append(frame)
    second = _silhouette_frame()
    second.T_base_camera = second.T_base_camera.copy()
    second.T_base_camera[0, 3] = .1
    second.camera_position_base = second.T_base_camera[:3, 3]
    second.target_mask = np.zeros((320, 320), np.uint8)
    second.target_mask[132:188, :160] = 255
    frames.append(second)
    refit = _ok_refit()
    refit.center = np.array([0., 0., .5])
    result, fused = _make(_FakeRefitter(product=refit)).run(
        **_run_kwargs(tsdf_xyz=_bag_cloud(), frames=frames,
                      target_center=[0, 0, .5], bound_axis_hint=[0, 0, 1]))
    assert not result.ok
    assert not fused.allowed
    assert result.reason == 'fused_axis_silhouette_conflict'


def test_unobservable_second_view_cannot_certify_multiview_refit_axis():
    from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
        reconstruction_axis_image_support,
    )
    first = _silhouette_frame()
    second = _silhouette_frame()
    second.T_base_camera = second.T_base_camera.copy()
    second.T_base_camera[0, 3] = .1
    second.camera_position_base = second.T_base_camera[:3, 3]
    y, x = np.ogrid[:320, :320]
    second.target_mask = ((x-80)**2+(y-160)**2 < 45**2).astype(np.uint8)
    refit = _ok_refit()
    refit.center = np.array([0., 0., .5])
    angle, invalid = reconstruction_axis_image_support(
        [first, second], refit, min_supported_views=2)
    assert angle is None
    assert invalid == 0


def test_round_silhouette_cannot_certify_refit_axis():
    from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
        reconstruction_axis_image_support,
    )
    frame = _silhouette_frame()
    frame.target_mask.fill(0)
    y, x = np.ogrid[:320, :320]
    frame.target_mask[(x - 160)**2 + (y - 160)**2 < 50**2] = 255
    fit = _ok_refit()
    fit.center = np.array([0.0, 0.0, 0.5])
    assert reconstruction_axis_image_support([frame], fit) == (None, 0)


def test_failed_refit_fallback_must_still_agree_with_original_silhouette():
    frame = _silhouette_frame()
    frame.cloud_base = _bag_cloud()[:, [2, 1, 0]] + [-.5, 0, .5]
    result, fused = _make(_FakeRefitter(product=RefitResult(
        ok=False, reason='cylinder_fit_failed', flags=['refit_failed']))).run(
            **_run_kwargs(tsdf_xyz=frame.cloud_base, frames=[frame],
                          target_center=[0, 0, .5], bound_axis_hint=[1, 0, 0]))
    assert not result.ok
    assert not fused.allowed
    assert not result.budget
    assert result.reason == 'fused_axis_silhouette_conflict'


def test_failed_refit_and_unconditioned_envelope_cannot_authorize_contact():
    from peach_harvester.vision.target_reconstruction.refine import (
        CONTACT_GEOMETRY_INVALID_REASONS,
    )
    bottom = np.array([0., 0., .4])
    neck = np.array([0., 0., .5])
    fused = BagModel(
        ok=True, bottom=bottom, neck=neck, axis=np.array([0., 0., 1.]),
        entry=bottom, pregrasp=bottom, cut_pose=neck, cut_plane_point=neck,
        cut_travel_m=.1, length_m=.1, d95_m=.11, envelope_conditioned=False,
        envelope_reason='envelope_axis_ill_conditioned',
        budget={'allowed': True}, allowed=True)
    result = merge_fused_bag_model(
        RefitResult(ok=False, reason='cylinder_fit_failed'),
        fused, 1, [0, 0, 1], 't')
    assert not result.ok
    assert not fused.allowed
    assert result.reason == 'bag_axis_unobservable'
    assert result.reason in CONTACT_GEOMETRY_INVALID_REASONS


@pytest.mark.parametrize('bad_field', ['transform', 'intrinsics', 'mask'])
def test_malformed_image_metadata_does_not_abort_refit(bad_field):
    from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
        reconstruction_axis_image_support,
    )
    frame = _silhouette_frame()
    if bad_field == 'transform':
        frame.T_base_camera = np.zeros((4, 4))
    elif bad_field == 'intrinsics':
        frame.camera_K = {'fx': 400, 'fy': 400, 'cy': 160}
    else:
        frame.target_mask = np.zeros((4, 4, 4))
    fit = _ok_refit()
    fit.center = np.array([0.0, 0.0, 0.5])
    assert reconstruction_axis_image_support([frame], fit) == (None, 1)
