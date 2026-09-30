import numpy as np
from peach2_perception.stationary import JOINT_NAMES, JointMotionBuffer, Motion
import pytest

NAMES = list(JOINT_NAMES)


def _fill(buf, times, speed_fn, report_velocity=True):
    q = np.zeros(6)
    last = None
    for t in times:
        v = np.full(6, speed_fn(t))
        if last is not None:
            q = q + v * (t - last)
        last = t
        buf.add(t, NAMES, q.tolist(), v.tolist() if report_velocity else [])


def test_stationary_with_reported_velocity():
    buf = JointMotionBuffer(5.0)
    _fill(buf, np.arange(0.0, 2.0, 0.01), lambda t: 0.0)
    assert buf.classify(1.5, 0.02, 0.2, 0.1) is Motion.STATIONARY
    assert buf.speed_at(1.5, 0.1) == pytest.approx(0.0)


def test_moving_in_window_detected():
    buf = JointMotionBuffer(5.0)
    _fill(buf, np.arange(0.0, 2.0, 0.01), lambda t: 0.5 if 1.35 < t < 1.4 else 0.0)
    assert buf.classify(1.5, 0.02, 0.2, 0.1) is Motion.MOVING
    assert buf.classify(1.8, 0.02, 0.2, 0.1) is Motion.STATIONARY


def test_finite_difference_when_velocity_missing():
    buf = JointMotionBuffer(5.0)
    _fill(buf, np.arange(0.0, 2.0, 0.01), lambda t: 0.3, report_velocity=False)
    assert buf.speed_at(1.0, 0.1) == pytest.approx(0.3, rel=1e-6)
    assert buf.classify(1.0, 0.02, 0.2, 0.1) is Motion.MOVING


def test_unknown_without_bracketing_data():
    buf = JointMotionBuffer(5.0)
    assert buf.classify(1.0, 0.02, 0.2, 0.1) is Motion.UNKNOWN
    _fill(buf, np.arange(0.0, 1.0, 0.01), lambda t: 0.0)
    assert buf.classify(1.5, 0.02, 0.2, 0.1) is Motion.UNKNOWN
    buf2 = JointMotionBuffer(5.0)
    buf2.add(0.0, NAMES, [0.0] * 6, [0.0] * 6)
    buf2.add(1.0, NAMES, [0.0] * 6, [0.0] * 6)
    assert buf2.classify(0.5, 0.02, 0.2, 0.1) is Motion.UNKNOWN  # 1 s gap


def test_interpolates_between_samples():
    buf = JointMotionBuffer(5.0)
    buf.add(0.0, NAMES, [0.0] * 6, [0.0] * 6)
    buf.add(0.1, NAMES, [0.0] * 6, [0.2] * 6)
    assert buf.speed_at(0.05, 0.1) == pytest.approx(0.1)


def test_rejects_missing_joints_and_regressing_stamps_and_prunes():
    buf = JointMotionBuffer(1.0)
    assert not buf.add(0.0, NAMES[:5], [0.0] * 5)
    assert buf.add(0.0, NAMES, [0.0] * 6)
    assert not buf.add(0.0, NAMES, [0.0] * 6)
    reordered = list(reversed(NAMES))
    assert buf.add(0.5, reordered, [0.0] * 6)
    for t in np.arange(0.6, 3.0, 0.1):
        buf.add(float(t), NAMES, [0.0] * 6)
    assert len(buf) <= 12
    with pytest.raises(ValueError):
        JointMotionBuffer(0.0)
