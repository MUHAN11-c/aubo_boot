# Copyright 2026 wjz
"""有界 worker 背压行为测试."""

import threading

from peach_pose_ros2.bounded_worker import BoundedWorker


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
