# Copyright 2026 wjz
"""recorder 纯函数与 Recorder 落盘行为测试（零 ROS 依赖）."""

import json
import threading
import time
from types import SimpleNamespace

import numpy as np
from peach_perception_web.recorder import (
    build_summary_csv,
    build_summary_markdown,
    build_target_rows,
    event_statistics,
    perception_stats,
    phase_durations,
    reconstruction_final,
    Recorder,
    session_folder,
    slim_targets,
    summarize_metrics,
    write_ply,
    xyz_rgb_from_pointcloud2,
)

_PLY_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
])
_POINTS = [(0.1, 0.2, 0.3, 255, 0, 0), (0.4, 0.5, 0.6, 0, 128, 255)]


def _fake_cloud(points):
    """构造 rgb float32 位打包的假 PointCloud2（duck-typed SimpleNamespace）."""
    fields = [
        SimpleNamespace(name='x', offset=0, datatype=7, count=1),
        SimpleNamespace(name='y', offset=4, datatype=7, count=1),
        SimpleNamespace(name='z', offset=8, datatype=7, count=1),
        SimpleNamespace(name='rgb', offset=12, datatype=7, count=1),
    ]
    data = bytearray()
    for x, y, z, r, g, b in points:
        data += np.array([x, y, z], dtype='<f4').tobytes()
        packed = np.array([(r << 16) | (g << 8) | b], dtype='<u4')
        data += packed.view('<f4').tobytes()
    return SimpleNamespace(
        width=len(points), height=1, fields=fields, point_step=16,
        row_step=16 * len(points), data=bytes(data), is_bigendian=False)


def _read_ply(path):
    """读回 binary PLY：校验头部并返回结构化数组."""
    blob = path.read_bytes()
    header, body = blob.split(b'end_header\n', 1)
    assert b'format binary_little_endian 1.0' in header
    return np.frombuffer(body, dtype=_PLY_DTYPE)


def _jsonl(path):
    """读取 jsonl 为 dict 列表."""
    return [json.loads(line) for line in
            path.read_text(encoding='utf-8').strip().splitlines()]


def test_session_folder_naming(tmp_path):
    """批次/空闲目录按 kind_<时间戳> 命名."""
    run = session_folder(tmp_path, 'run', now=0.0)
    idle = session_folder(tmp_path, 'idle', now=0.0)
    assert run.parent == tmp_path and run.name.startswith('run_')
    assert idle.name.startswith('idle_')


def test_slim_targets_drops_mask_keeps_candidate_fitting():
    """目标快照瘦身：mask 剔除，candidate/fitting 摘要保留."""
    value = {
        'stamp': 1.0, 'snapshot_id': 7, 'harvest_run_id': 'r',
        'target_set_locked': True, 'target_count': 1,
        'selected_target_id': 'p1',
        'observations': [{
            'target_id': 'p1', 'priority': 1, 'confirmed': True,
            'selected': True, 'harvest_status': 'PLANNED',
            'tracking_status': 'OBSERVED', 'camera_distance_m': 0.6,
            'confidence': 0.9, 'diagnostic_flags': ['x'],
            'candidate': {'entry_position': [0, 0, 0]},
            'fitting': {'status': 'ACCEPT'},
            'mask': {'width': 1},
        }],
    }
    item = slim_targets(value)['observations'][0]
    assert item['candidate'] == {'entry_position': [0, 0, 0]}
    assert item['fitting'] == {'status': 'ACCEPT'}
    assert 'mask' not in item


def test_pointcloud2_to_ply_roundtrip(tmp_path):
    """PointCloud2 解析 + PLY 写出可被 numpy 读回，点数与通道一致."""
    xyz, rgb = xyz_rgb_from_pointcloud2(_fake_cloud(_POINTS))
    assert xyz.shape == (2, 3)
    assert rgb.tolist() == [[255, 0, 0], [0, 128, 255]]
    path = tmp_path / 'cloud.ply'
    assert write_ply(path, xyz, rgb) == 2
    arr = _read_ply(path)
    assert len(arr) == 2
    assert np.allclose(arr['x'], [0.1, 0.4])
    assert arr['red'].tolist() == [255, 0]
    assert arr['blue'].tolist() == [0, 255]


def test_pointcloud2_empty_and_nan_skipped(tmp_path):
    """空云返回 None；非有限点被剔除."""
    assert xyz_rgb_from_pointcloud2(_fake_cloud([])) is None
    nan_cloud = _fake_cloud([(float('nan'), 0.0, 0.0, 1, 2, 3)] + _POINTS)
    xyz, _rgb = xyz_rgb_from_pointcloud2(nan_cloud)
    assert xyz.shape == (2, 3)
    assert write_ply(tmp_path / 'x.ply', xyz, None) == 2


def test_summarize_metrics_mean_and_max():
    """性能统计：均值/峰值，GPU 缺失自动跳过."""
    records = [
        {'cpu_percent': 10.0, 'memory_percent': 40.0,
         'gpu': {'utilization_percent': 20.0, 'memory_used_mb': 1000.0}},
        {'cpu_percent': 30.0, 'memory_percent': 60.0, 'gpu': None},
    ]
    stats = summarize_metrics(records)
    assert stats['samples'] == 2
    assert stats['cpu_percent'] == {'mean': 20.0, 'max': 30.0}
    assert stats['gpu_utilization_percent'] == {'mean': 20.0, 'max': 20.0}
    assert summarize_metrics([])['samples'] == 0


def test_build_target_rows_and_csv():
    """事件流 → 逐目标 outcome 表与 CSV 文本."""
    events = [
        {'code': 'target_dispatched', 'stamp': 11.0, 'target_id': 'p1'},
        {'code': 'target_succeeded', 'message': '抓取完成', 'stamp': 16.5,
         'target_id': 'p1'},
        {'code': 'target_dispatched', 'stamp': 20.0, 'target_id': 'p2'},
        {'code': 'target_failed', 'message': '规划无解', 'stamp': 25.0,
         'target_id': 'p2'},
    ]
    rows = build_target_rows(events, {'p1': 1, 'p2': 2})
    assert [row['target_id'] for row in rows] == ['p1', 'p2']
    assert rows[0]['outcome'] == 'succeeded'
    assert rows[0]['duration_s'] == 5.5
    assert rows[1]['outcome'] == 'failed'
    assert 'p1,1,succeeded,抓取完成' in build_summary_csv(rows)


def test_target_rows_distinguish_operator_skip():
    """A13 拆分：操作员跳过（target_operator_skipped）独立于系统判定跳过."""
    events = [
        {'code': 'target_dispatched', 'stamp': 11.0, 'target_id': 'p1'},
        {'code': 'target_operator_skipped', 'message': '操作员跳过当前目标',
         'stamp': 13.0, 'target_id': 'p1'},
        {'code': 'target_dispatched', 'stamp': 20.0, 'target_id': 'p2'},
        {'code': 'target_skipped', 'message': '质量不达标', 'stamp': 25.0,
         'target_id': 'p2'},
    ]
    rows = build_target_rows(events, {'p1': 1, 'p2': 2})
    assert rows[0]['outcome'] == 'operator_skipped'
    assert rows[1]['outcome'] == 'skipped'


def test_phase_durations_per_cycle():
    """target_phase 跃迁 → 逐周期各阶段耗时（终局相不计入）."""
    states = [
        {'recorded_at': 10.0, 'target_phase': 1, 'cycle_id': 'c1',
         'target_id': 'p1'},
        {'recorded_at': 11.0, 'target_phase': 2, 'cycle_id': 'c1',
         'target_id': 'p1'},
        {'recorded_at': 14.5, 'target_phase': 5, 'cycle_id': 'c1',
         'target_id': 'p1'},
        {'recorded_at': 20.0, 'target_phase': 9, 'cycle_id': 'c1',
         'target_id': 'p1'},
        {'recorded_at': 21.0, 'target_phase': 0, 'cycle_id': '',
         'target_id': ''},
    ]
    rows = phase_durations(states)
    assert len(rows) == 1
    assert rows[0]['cycle_id'] == 'c1'
    assert rows[0]['phases'] == {
        'SELECTING': 1.0, 'OBSERVING': 3.5, 'APPROACHING': 5.5}
    assert rows[0]['total_s'] == 10.0


def test_event_statistics_counts():
    """事件按 code/severity 计数."""
    events = [
        {'code': 'round_started', 'severity_name': 'INFO'},
        {'code': 'target_failed', 'severity_name': 'ERROR'},
        {'code': 'target_failed', 'severity_name': 'ERROR'},
    ]
    stats = event_statistics(events)
    assert stats['total'] == 3
    assert stats['by_code'] == {'round_started': 1, 'target_failed': 2}
    assert stats['by_severity'] == {'INFO': 1, 'ERROR': 2}


def test_perception_stats_frame_rate_and_count_range():
    """perception.jsonl → 帧间隔中位数/FPS 与目标数范围."""
    records = [
        {'kind': 'targets', 'recorded_at': 10.0,
         'data': {'stamp': 10.0, 'target_count': 3}},
        {'kind': 'harvest', 'recorded_at': 10.5, 'data': {}},
        {'kind': 'targets', 'recorded_at': 11.0,
         'data': {'stamp': 11.5, 'target_count': 5}},
        {'kind': 'targets', 'recorded_at': 12.0,
         'data': {'stamp': 13.0, 'target_count': 2}},
    ]
    stats = perception_stats(records)
    assert stats['frames'] == 3
    assert stats['median_interval_s'] == 1.5
    assert stats['fps'] == 0.67
    assert stats['target_count_min'] == 2
    assert stats['target_count_max'] == 5


def test_reconstruction_final_picks_last_diagnostics():
    """重建终值取最后一条 diagnostics 的关键键."""
    records = [
        {'topic': 'status', 'data': {'state': 'WARMUP'}},
        {'topic': 'diagnostics', 'data': {
            'state': 'COLLECTING', 'captured_views': 3, 'tf_failures': 1,
            'view_coverage': {'max_baseline_deg': 21.5},
            'tsdf': {'points': 100, 'integrate_time_s': 0.08},
            'grasp_decision': {'allowed': False, 'reason': 'not_ready'}}},
        {'topic': 'diagnostics', 'data': {
            'state': 'READY', 'captured_views': 8, 'rejected_views': 2,
            'tf_failures': 1, 'tf_latency_ms': 3.2, 'cloud_points': 5000,
            'view_coverage': {'max_baseline_deg': 45.0,
                              'mean_nearest_baseline_deg': 12.0},
            'grasp_decision': {'allowed': True,
                               'reason': 'refined_geometry_accept'}}},
    ]
    final = reconstruction_final(records)
    assert final['state'] == 'READY'
    assert final['captured_views'] == 8
    assert final['max_baseline_deg'] == 45.0
    assert final['grasp_allowed'] is True
    assert 'tsdf_points' not in final  # 最后一条无 tsdf 键则不出现
    assert reconstruction_final([]) == {}


def test_build_summary_markdown_sections():
    """Markdown 摘要包含全部统计区块."""
    markdown = build_summary_markdown(
        'run-a', 10.0, 40.0, 6, 2,
        [{'target_id': 'p1', 'priority': 1, 'outcome': 'succeeded',
          'reason': 'ok', 'duration_s': 5.5}],
        [{'cycle_id': 'c1', 'target_id': 'p1',
          'phases': {'OBSERVING': 3.5}, 'total_s': 3.5}],
        {'total': 4, 'by_code': {'round_started': 1},
         'by_severity': {'INFO': 3, 'ERROR': 1}},
        {'frames': 10, 'median_interval_s': 1.3, 'fps': 0.77,
         'target_count_min': 2, 'target_count_max': 5},
        {'state': 'READY', 'captured_views': 8},
        summarize_metrics([{'cpu_percent': 5.0}]))
    assert '复扫轮数：2' in markdown
    assert '总时长：30.0 s' in markdown
    assert '每阶段耗时统计' in markdown and 'OBSERVING' in markdown
    assert '事件统计' in markdown and '`round_started`：1' in markdown
    assert '帧间隔中位数：1.3 s' in markdown
    assert '重建关键指标终值' in markdown and 'captured_views：8' in markdown
    assert 'CPU %' in markdown


def test_recorder_batch_folder_lifecycle(tmp_path):
    """一次批次一个文件夹：轮次不拆目录，终局生成 summary，批次后进 idle."""
    recorder = Recorder(
        root_dir=tmp_path, enabled=True, save_images=False, save_clouds=True)
    # 批次前的记录进 idle
    recorder.handle_metrics({'stamp': 1.0, 'cpu_percent': 5.0})
    # 批次开始（DISCOVERY）
    recorder.handle_state({'revision': 1, 'run_id': 'r1', 'batch_state': 1,
                           'target_phase': 0, 'message': '收齐中'})
    # 第 1 轮
    recorder.handle_event({'code': 'round_started', 'message': '第1轮开始',
                           'stamp': 10.0, 'run_id': 'r1', 'target_id': '',
                           'severity': 0, 'severity_name': 'INFO'})
    recorder.handle_targets({
        'stamp': 10.5, 'snapshot_id': 1, 'harvest_run_id': 'r1',
        'target_count': 2, 'observations': [
            {'target_id': 'p1', 'priority': 1, 'harvest_status': 'PLANNED',
             'candidate': {}, 'fitting': {}, 'mask': {'width': 1}}]})
    recorder.handle_targets({
        'stamp': 11.5, 'snapshot_id': 2, 'harvest_run_id': 'r1',
        'target_count': 2, 'observations': [
            {'target_id': 'p2', 'priority': 2, 'harvest_status': 'PLANNED',
             'candidate': {}, 'fitting': {}, 'mask': {'width': 1}}]})
    recorder.handle_harvest({'target_count': 2, 'harvest_run_id': 'r1'})
    recorder.handle_harvest({'target_count': 2, 'harvest_run_id': 'r1'})
    recorder.handle_reconstruction('diagnostics', {
        'state': 'READY', 'captured_views': 8,
        'view_coverage': {'max_baseline_deg': 40.0}})
    recorder.handle_approach({'state': 'OBSERVING', 'message': '采帧'})
    recorder.handle_metrics({'stamp': 2.0, 'cpu_percent': 20.0})
    recorder.handle_state({'revision': 2, 'run_id': 'r1', 'batch_state': 2,
                           'target_phase': 2, 'cycle_id': 'c1',
                           'target_id': 'p1'})
    recorder.handle_event({'code': 'target_dispatched', 'stamp': 12.0,
                           'run_id': 'r1', 'target_id': 'p1',
                           'severity': 0, 'severity_name': 'INFO'})
    time.sleep(0.002)  # recorded_at 毫秒分辨率，让阶段跃迁产生正耗时
    recorder.handle_state({'revision': 3, 'run_id': 'r1', 'batch_state': 2,
                           'target_phase': 5, 'cycle_id': 'c1',
                           'target_id': 'p1'})
    time.sleep(0.002)
    recorder.handle_cloud(_fake_cloud(_POINTS))
    recorder.handle_event({'code': 'target_succeeded', 'message': '完成',
                           'stamp': 18.0, 'run_id': 'r1', 'target_id': 'p1',
                           'severity': 0, 'severity_name': 'INFO'})
    # 第 2 轮（复扫，同一文件夹）
    recorder.handle_event({'code': 'round_started', 'message': '第2轮开始',
                           'stamp': 20.0, 'run_id': 'r1', 'target_id': '',
                           'severity': 0, 'severity_name': 'INFO'})
    recorder.handle_event({'code': 'target_dispatched', 'stamp': 21.0,
                           'run_id': 'r1', 'target_id': 'p2',
                           'severity': 0, 'severity_name': 'INFO'})
    recorder.handle_event({'code': 'target_skipped', 'message': '质量不达标',
                           'stamp': 25.0, 'run_id': 'r1', 'target_id': 'p2',
                           'severity': 3, 'severity_name': 'AUDIT'})
    # 批次终局（COMPLETED）
    recorder.handle_state({'revision': 4, 'run_id': 'r1', 'batch_state': 6,
                           'target_phase': 0, 'message': '批次完成'})
    # 批次后的记录进新的 idle
    recorder.handle_metrics({'stamp': 3.0, 'cpu_percent': 6.0})
    recorder.close()

    run_dirs = [p for p in tmp_path.iterdir() if p.name.startswith('run_')]
    idle_dirs = [p for p in tmp_path.iterdir() if p.name.startswith('idle_')]
    assert len(run_dirs) == 1, '一个批次必须恰好一个 run 目录'
    assert len(idle_dirs) >= 1
    run_dir = run_dirs[0]

    events = _jsonl(run_dir / 'events.jsonl')
    codes = [item['code'] for item in events]
    assert codes.count('round_started') == 2, '两轮复扫在同一目录'
    assert all('recorded_at' in item for item in events)
    states = _jsonl(run_dir / 'state.jsonl')
    assert len(states) == 4
    perception = _jsonl(run_dir / 'perception.jsonl')
    assert [item['kind'] for item in perception] == [
        'targets', 'targets', 'harvest']  # 逐帧全量 + harvest 变化去重
    assert 'mask' not in perception[0]['data']['observations'][0]
    assert 'candidate' in perception[0]['data']['observations'][0]
    assert len(_jsonl(run_dir / 'reconstruction.jsonl')) == 1
    assert len(_jsonl(run_dir / 'approach.jsonl')) == 1
    # 批次内 metric 在 run 目录；批次前/后的进 idle
    run_metrics = _jsonl(run_dir / 'metrics.jsonl')
    assert len(run_metrics) == 1 and run_metrics[0]['cpu_percent'] == 20.0
    idle_metrics = [line for folder in idle_dirs
                    if (folder / 'metrics.jsonl').exists()
                    for line in _jsonl(folder / 'metrics.jsonl')]
    assert len(idle_metrics) == 2
    # target 终局点云两个目标各一份
    clouds = sorted(p.name for p in (run_dir / 'clouds').glob('*.ply'))
    assert len(clouds) == 2
    assert any(name.startswith('p1_') for name in clouds)
    assert any(name.startswith('p2_') for name in clouds)
    # 事件索引（终局事件各一行；无图像时 image 为 None）
    index = _jsonl(run_dir / 'image_index.jsonl')
    assert [item['event'] for item in index] == [
        'target_succeeded', 'target_skipped']
    # summary：内容齐全
    csv_text = (run_dir / 'summary.csv').read_text(encoding='utf-8')
    assert 'p1,1,succeeded,完成' in csv_text
    assert 'p2,2,skipped,质量不达标' in csv_text
    markdown = (run_dir / 'summary.md').read_text(encoding='utf-8')
    assert '复扫轮数：2' in markdown
    assert 'COMPLETED' in markdown
    assert '每阶段耗时统计' in markdown and 'c1' in markdown
    assert '事件统计' in markdown and 'round_started' in markdown
    assert '感知统计' in markdown and '帧间隔中位数' in markdown
    assert '重建关键指标终值' in markdown and 'captured_views：8' in markdown
    assert '运行性能统计' in markdown
    assert recorder.info()['directory'] != str(run_dir)  # 已切回 idle


def test_recorder_disabled_is_noop(tmp_path):
    """record.enabled=false 时不产生任何文件."""
    recorder = Recorder(root_dir=tmp_path, enabled=False)
    recorder.handle_event({'code': 'round_started', 'stamp': 1.0,
                           'run_id': 'run-x', 'target_id': ''})
    recorder.handle_metrics({'stamp': 1.0, 'cpu_percent': 1.0})
    recorder.close()
    assert list(tmp_path.iterdir()) == []
    assert recorder.info() == {'enabled': False, 'directory': None}


def test_recorder_async_write_does_not_block_caller(tmp_path, monkeypatch):
    """写线程执行慢任务时，回调入口仍只入队即时返回（不阻塞采集主路径）."""
    recorder = Recorder(
        root_dir=tmp_path, enabled=True, save_images=False, save_clouds=True)
    started = threading.Event()
    release = threading.Event()

    def slow_cloud(message, path):
        started.set()
        release.wait(5.0)

    monkeypatch.setattr(recorder, '_write_cloud', slow_cloud)
    recorder.handle_cloud(_fake_cloud(_POINTS))
    # target 终局事件触发点云落盘任务，写线程随即卡在 slow_cloud 上
    recorder.handle_event({'code': 'target_succeeded', 'stamp': 1.0,
                           'run_id': 'r', 'target_id': 'p1'})
    assert started.wait(2.0), '写线程应已开始执行慢任务'
    begin = time.monotonic()
    for index in range(500):
        recorder.handle_metrics({'stamp': float(index), 'cpu_percent': 1.0})
    elapsed = time.monotonic() - begin
    release.set()
    recorder.close()
    assert elapsed < 0.5, f'回调入口被写盘阻塞: {elapsed:.3f}s'


def test_recorder_write_failure_degrades_without_blocking(tmp_path):
    """单批落盘失败只降级告警并丢弃该批，后续采集与落盘不受影响."""
    warnings = []
    recorder = Recorder(
        root_dir=tmp_path, enabled=True, save_images=False, save_clouds=False,
        log_warning=warnings.append)
    recorder.handle_metrics({'stamp': 1.0, 'bad': object()})  # 不可序列化
    time.sleep(0.6)  # 等写线程空闲 flush 尝试并失败
    recorder.handle_metrics({'stamp': 2.0, 'cpu_percent': 3.0})
    recorder.close()
    assert any('落盘失败' in text for text in warnings)
    idle_dirs = [p for p in tmp_path.iterdir() if p.name.startswith('idle_')]
    lines = [line for folder in idle_dirs
             if (folder / 'metrics.jsonl').exists()
             for line in _jsonl(folder / 'metrics.jsonl')]
    assert [item['stamp'] for item in lines] == [2.0]


def test_recorder_close_drains_pending_writes(tmp_path):
    """退出 drain：close() 排空队列与写缓冲（含跨批量阈值），记录不丢."""
    recorder = Recorder(
        root_dir=tmp_path, enabled=True, save_images=False, save_clouds=False)
    total = 200  # 超过 _JSONL_BATCH_LINES=64，覆盖批量 flush + 退出 drain
    for index in range(total):
        recorder.handle_metrics({'stamp': float(index), 'cpu_percent': 1.0})
    recorder.close()
    idle_dirs = [p for p in tmp_path.iterdir() if p.name.startswith('idle_')]
    lines = [line for folder in idle_dirs
             for line in _jsonl(folder / 'metrics.jsonl')]
    assert len(lines) == total


def test_recorder_jsonl_visible_after_idle_flush(tmp_path):
    """未达批量阈值的记录在队列空闲一个轮询周期后落盘（不等 close）."""
    recorder = Recorder(
        root_dir=tmp_path, enabled=True, save_images=False, save_clouds=False)
    try:
        recorder.handle_metrics({'stamp': 7.0, 'cpu_percent': 9.0})
        deadline = time.monotonic() + 3.0
        found = []
        while time.monotonic() < deadline and not found:
            idle_dirs = [p for p in tmp_path.iterdir()
                         if p.name.startswith('idle_')]
            found = [line for folder in idle_dirs
                     if (folder / 'metrics.jsonl').exists()
                     for line in _jsonl(folder / 'metrics.jsonl')]
            time.sleep(0.05)
        assert found and found[0]['stamp'] == 7.0
    finally:
        recorder.close()
