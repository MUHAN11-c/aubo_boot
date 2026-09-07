"""Zero-ROS tests for ManualClock and BoundedWorker."""
import threading

from peach_perception.common.bounded_worker import BoundedWorker
from peach_perception.common.clock import ManualClock
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
