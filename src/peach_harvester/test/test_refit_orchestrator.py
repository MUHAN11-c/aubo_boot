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


def test_run_success_path_merges_and_marks_final():
    refitter = _FakeRefitter(product=_ok_refit())
    orch = _make(refitter)
    xyz = np.random.default_rng(2).random((600, 3)) * 0.04 + [0, 0, 0.4]
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
    assert result.reason.startswith('exception:')
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
