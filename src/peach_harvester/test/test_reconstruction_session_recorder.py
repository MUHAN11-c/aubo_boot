"""Zero-ROS tests for SessionRecorder + params snapshot wiring (W4)."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from peach_common.yaml_params import dict_to_ns, snapshot
from peach_harvester.vision.common.bag_landmarks import BagLandmarks
from peach_harvester.vision.target_reconstruction.capture import (
    CollectorConfig,
    FrameCollector,
)
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    merge_fused_bag_model,
    RefitResult,
    STATUS_ACCEPT,
)
from peach_harvester.vision.target_reconstruction.session_recorder import (
    SessionRecorder,
)


def _fused_ok():
    bottom = np.array([0.0, 0.0, 0.4])
    neck = np.array([0.0, 0.0, 0.6])
    axis = np.array([0.0, 0.0, 1.0])
    return BagModel(
        ok=True, bottom=bottom, neck=neck, axis=axis,
        entry=bottom, pregrasp=bottom, cut_plane_point=neck,
        cut_pose=neck, cut_travel_m=0.2, d95_m=0.065, length_m=0.2,
        sigma_position_m=0.005, sigma_axis_deg=4.0, view_count=3,
        budget={'allowed': True, 'reason': 'ok'}, allowed=True,
        radial_margin_m=0.01, axial_margin_m=0.02, corridor_clear=True)


def _merged():
    result = RefitResult(
        ok=True, kind='cylinder', status=STATUS_ACCEPT, n_points=100,
        center=np.zeros(3), axis=np.array([0.0, 0.0, 1.0]),
        bottom=np.array([0.0, 0.0, 0.4]),
        neck=np.array([0.0, 0.0, 0.6]), diameter=0.065, span_m=0.2,
        rmse=0.002, inlier_ratio=0.9)
    return merge_fused_bag_model(
        result, _fused_ok(), views_count=3, bound_axis_hint=None,
        target_id='t1')


def test_snapshot_flattens_namespace_to_native_types():
    ns = dict_to_ns({
        'frames': {'base_frame': ' base_link '},
        'icp': {'min_points': 300, 'voxel': 0.006},
        'tags': ['a', 'b'],
        'numpy_scalar': np.float64(1.5),
    })
    data = snapshot(ns)
    assert data['frames.base_frame'] == ' base_link '
    assert data['icp.min_points'] == 300
    assert data['icp.voxel'] == 0.006
    assert data['tags'] == ['a', 'b']
    assert data['numpy_scalar'] == 1.5
    assert isinstance(data['numpy_scalar'], float)
    # 快照与后续 set 解耦（浅拷贝）
    ns.icp.min_points = 999
    assert data['icp.min_points'] == 300


def test_recorder_geometry_row_writes_jsonl(tmp_path: Path):
    recorder = SessionRecorder(root_resolver=lambda: tmp_path)
    recorder.geometry_row(_merged(), _fused_ok(), 't1')
    rows = [
        __import__('json').loads(line)
        for line in (tmp_path / 'geometry.jsonl').read_text(
            encoding='utf-8').splitlines()]
    assert len(rows) == 1
    row = rows[0]
    assert row['target_id'] == 't1'
    assert row['fused'] is True
    assert row['view_count'] == 3
    assert abs(row['d95_m'] - 0.065) < 1e-12
    assert abs(row['length_m'] - 0.2) < 1e-12


def test_recorder_view_row_skips_incomplete_and_appends(tmp_path: Path):
    recorder = SessionRecorder(root_resolver=lambda: tmp_path)
    frame = NS(stamp=1.25)
    full = BagLandmarks(
        bottom_center=np.zeros(3), neck_center=np.array([0.0, 0.0, 0.2]),
        bag_axis=np.array([0.0, 0.0, 1.0]), d95_m=0.07)
    recorder.view_row(full, frame, 't1')
    recorder.view_row(BagLandmarks(), frame, 't1')  # 缺端点：跳过
    rows = [
        __import__('json').loads(line)
        for line in (tmp_path / 'geometry.jsonl').read_text(
            encoding='utf-8').splitlines()]
    assert len(rows) == 1
    assert rows[0]['fused'] is False
    assert rows[0]['stamp_sec'] == 1.25


def test_recorder_batch_root_sanitizes_run_id(tmp_path: Path):
    recorder = SessionRecorder(root_resolver=lambda: tmp_path)
    recorder.bind_executor_run_id('../evil')
    root = recorder.geometry_root()
    assert '..' not in root.parts
    assert root.parent.name != 'evil'
    recorder.bind_executor_run_id('')
    assert recorder.session_root() == tmp_path


def test_session_metadata_uses_params_snapshot(tmp_path: Path):
    recorder = SessionRecorder(root_resolver=lambda: tmp_path)
    collector = FrameCollector(CollectorConfig(min_views=1, max_views=4))
    collector.start('t1', np.zeros(3))
    parameters = {
        'frames.base_frame': 'base_link', 'icp.min_points': 300,
        'tsdf.voxel_length': 0.003,
    }
    metadata = recorder.session_metadata(
        collector=collector, parameters=parameters,
        harvest_run_id='run-1', selected_target_id='t1',
        target_mask_cache_size=2, tsdf_result={'points': 10},
        refined_result={'ok': True}, timing={'frames_timed': 0})
    assert metadata['parameters'] == parameters
    assert metadata['harvest_run_id'] == 'run-1'
    assert metadata['target_id'] == 't1'
    assert metadata['captured_views'] == 0
    assert metadata['frames'] == []
    assert 'created' in metadata and 'view_coverage' in metadata
