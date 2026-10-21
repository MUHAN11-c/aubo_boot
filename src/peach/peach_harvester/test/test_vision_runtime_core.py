"""Zero-ROS tests for ManualClock, BoundedWorker, HarvestDataStore (V4), PF-5."""
import json
import threading

import cv2
import numpy as np

from peach_harvester.vision.common.runtime import (
    BoundedWorker,
    HarvestDataStore,
    ManualClock,
    view_stage_for_phase,
)
from peach_harvester.vision.target_reconstruction.capture import (
    CollectorConfig,
    FrameCollector,
)
import pytest


def test_manual_clock_advances_only_on_advance():
    clock = ManualClock(start=1.5)
    assert clock.now() == pytest.approx(1.5)
    assert clock.advance(0.25) == pytest.approx(1.75)
    assert clock.now() == pytest.approx(1.75)


def test_manual_clock_rejects_negative_dt():
    clock = ManualClock()
    with pytest.raises(ValueError):
        clock.advance(-0.1)


def test_bounded_worker_capacity_1_drop_oldest():
    processed = []
    started = threading.Event()
    release = threading.Event()

    def process(item):
        started.set()
        assert release.wait(timeout=2.0)
        processed.append(item)

    worker = BoundedWorker(process, capacity=1, drop_oldest=True)
    assert worker.submit('a')
    assert started.wait(timeout=2.0)
    assert worker.submit('b')
    assert worker.submit('c')
    assert worker.dropped >= 1
    release.set()
    worker.close(drain=True)
    assert processed[0] == 'a'
    assert processed[-1] == 'c'
    assert 'b' not in processed


def _store_with_run(tmp_path):
    store = HarvestDataStore(root=tmp_path)
    store.run_dir = tmp_path / 'run1'
    (store.run_dir / 'masks').mkdir(parents=True)
    return store


def test_store_append_event_async_ordered_and_close_drains(tmp_path):
    store = _store_with_run(tmp_path)
    for i in range(10):
        store.append_event({'source': 'reconstruction', 'event': f'e{i}'})
    store.close(drain=True)
    lines = (tmp_path / 'run1' / 'events.jsonl').read_text(
        encoding='utf-8').splitlines()
    assert len(lines) == 10
    events = [json.loads(line)['event'] for line in lines]
    assert events == [f'e{i}' for i in range(10)]  # FIFO 单写者
    assert store.latest_state['event'] == 'e9'


def test_store_save_mask_throttles_per_target(tmp_path):
    store = _store_with_run(tmp_path)
    mask = np.zeros((4, 4), dtype=np.uint8)
    first = store.save_mask('t1', 100, mask, min_interval_s=60.0)
    assert first == 'masks/100_t1.png'
    # 间隔内同目标节流返回空串；另一目标不受影响
    assert store.save_mask('t1', 101, mask, min_interval_s=60.0) == ''
    assert store.save_mask('t2', 102, mask, min_interval_s=60.0) != ''
    store.close(drain=True)
    saved = sorted(p.name for p in (tmp_path / 'run1' / 'masks').iterdir())
    assert saved == ['100_t1.png', '102_t2.png']


def test_store_save_mask_writes_matching_color(tmp_path):
    store = _store_with_run(tmp_path)
    mask = np.zeros((4, 4), dtype=np.uint8)
    color = np.full((4, 4, 3), 30, dtype=np.uint8)
    debug = np.full((4, 4, 3), 90, dtype=np.uint8)
    relative = store.save_mask(
        't1', 100, mask, color=color, debug=debug, min_interval_s=0.0)
    assert relative == 'masks/100_t1.png'
    assert store.save_mask(
        't1', 101, mask, color=color, debug=debug, min_interval_s=60.0) == ''
    store.close(drain=True)
    run = tmp_path / 'run1'
    assert (run / 'masks' / '100_t1.png').is_file()
    assert (run / 'colors' / '100_t1.png').is_file()
    assert (run / 'debug' / '100_t1.png').is_file()
    assert not (run / 'colors' / '101_t1.png').exists()
    assert not (run / 'debug' / '101_t1.png').exists()
    saved = cv2.imread(str(run / 'colors' / '100_t1.png'))
    overlay = cv2.imread(str(run / 'debug' / '100_t1.png'))
    assert saved.shape == (4, 4, 3)
    assert overlay.shape == (4, 4, 3)


def test_store_save_view_without_mask(tmp_path):
    store = _store_with_run(tmp_path)
    color = np.full((4, 4, 3), 40, dtype=np.uint8)
    debug = np.full((4, 4, 3), 80, dtype=np.uint8)
    relative = store.save_mask(
        't1', 100, None, color=color, debug=debug, min_interval_s=0.0)
    assert relative == 'colors/100_t1.png'
    store.close(drain=True)
    run = tmp_path / 'run1'
    assert not (run / 'masks' / '100_t1.png').exists()
    assert (run / 'colors' / '100_t1.png').is_file()
    assert (run / 'debug' / '100_t1.png').is_file()


def test_stage_view_once_per_phase_and_not_during_approach(tmp_path):
    assert view_stage_for_phase(2, True, set()) == 'observing'
    assert view_stage_for_phase(2, False, set()) is None
    assert view_stage_for_phase(5, True, set()) is None
    assert view_stage_for_phase(4, True, {'observing'}) == 'reconfirm'
    assert view_stage_for_phase(10, False, set()) == 'outcome'
    assert view_stage_for_phase(11, False, {'observing'}) is None
    store = _store_with_run(tmp_path)
    color = np.full((4, 4, 3), 20, dtype=np.uint8)
    first = store.save_stage_view('t1', 'observing', 100, color=color)
    assert first == 'colors/100_t1.png'
    assert store.save_stage_view('t1', 'observing', 101, color=color) == ''
    second = store.save_stage_view('t1', 'reconfirm', 200, color=color)
    assert second == 'colors/200_t1.png'
    store.close(drain=True)
    names = sorted(p.name for p in (tmp_path / 'run1' / 'colors').iterdir())
    assert names == ['100_t1.png', '200_t1.png']


def test_collector_accumulated_points_count_tracks_stack():
    collector = FrameCollector(CollectorConfig(min_views=1, max_views=8))
    assert collector.accumulated_points_count == 0

    class _Frame:
        def __init__(self, n):
            self.cloud_base = None if n is None else np.zeros((n, 3))
            self.valid_depth_ratio = 0.8
            self.stamp = 1.0

    assert collector.add_frame(_Frame(100))
    assert collector.add_frame(_Frame(None))
    assert collector.accumulated_points_count == 100
    assert collector.add_frame(_Frame(50))
    assert collector.accumulated_points_count == 150
    assert collector.accumulated_points_count == (
        collector.accumulated_cloud().shape[0])
    collector.remove_last()
    assert collector.accumulated_points_count == 100
    collector.reset()
    assert collector.accumulated_points_count == 0


def test_collector_finalize_includes_skip_summary():
    collector = FrameCollector(CollectorConfig(min_views=1, max_views=8))

    class _Frame:
        def __init__(self):
            self.cloud_base = np.zeros((4, 3))
            self.valid_depth_ratio = 0.8
            self.stamp = 1.0

    collector.start('t1')
    assert collector.add_frame(_Frame())
    collector.note_skip('missing_mask', 'no mask')
    collector.note_skip('stale_frame', 'old')
    ok, msg, _cloud = collector.finalize()
    assert ok
    assert '跳过 2 次' in msg
    assert 'missing_mask=1' in msg
    assert 'stale_frame=1' in msg
