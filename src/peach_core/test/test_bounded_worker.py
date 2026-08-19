# Copyright 2026 wjz
"""有界 worker 背压行为测试（自 peach_pose_ros2 迁移，行为不变）."""

import threading

from peach_core.bounded_worker import BoundedWorker


def test_latest_only_worker_replaces_pending_frame():
    """推理繁忙时保留最新帧而不是积压历史帧."""
    started = threading.Event()
    release = threading.Event()
    processed = []

    def process(value):
        processed.append(value)
        if value == 1:
            started.set()
            release.wait(1.0)

    worker = BoundedWorker(process, capacity=1, drop_oldest=True)
    worker.submit(1)
    assert started.wait(1.0)
    worker.submit(2)
    worker.submit(3)
    release.set()
    worker.close(drain=True)

    assert processed == [1, 3]
    assert worker.dropped == 1


def test_serial_worker_preserves_order_with_bounded_queue():
    """重建单写者按入队顺序处理可容纳的帧."""
    processed = []
    worker = BoundedWorker(processed.append, capacity=3, drop_oldest=False)
    assert worker.submit('a')
    assert worker.submit('b')
    assert worker.submit('c')
    worker.close(drain=True)

    assert processed == ['a', 'b', 'c']


def test_close_without_drain_drops_pending():
    """close(drain=False) 丢弃剩余任务并计入 dropped."""
    started = threading.Event()
    release = threading.Event()

    def process(value):
        started.set()
        release.wait(1.0)

    worker = BoundedWorker(process, capacity=4, drop_oldest=False)
    worker.submit(1)
    assert started.wait(1.0)
    worker.submit(2)
    worker.submit(3)
    release.set()
    worker.close(drain=False)

    assert worker.dropped == 2
    assert not worker.submit(4)


def test_capacity_must_be_positive():
    """容量必须为正：capacity < 1 抛 ValueError."""
    try:
        BoundedWorker(lambda item: None, capacity=0, drop_oldest=True)
    except ValueError:
        return
    raise AssertionError('capacity=0 应抛 ValueError')
