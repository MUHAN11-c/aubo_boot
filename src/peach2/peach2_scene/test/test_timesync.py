from peach2_scene.timesync import JointStateBuffer
import pytest

NAMES = ['shoulder_joint', 'upperArm_joint']


def _buffer() -> JointStateBuffer:
    b = JointStateBuffer(window_s=1.0)
    for k in range(11):
        t = 10.0 + 0.1 * k
        b.add(t, NAMES, [0.1 * k, 0.0])
    return b


def test_interpolates_positions_and_speeds():
    s = _buffer().sample(10.25, max_gap_s=0.2)
    assert s is not None
    assert s.positions['shoulder_joint'] == pytest.approx(0.25)
    assert s.speeds['shoulder_joint'] == pytest.approx(1.0)
    assert s.speeds['upperArm_joint'] == pytest.approx(0.0)
    assert s.max_speed == pytest.approx(1.0)
    assert _buffer().sample(10.0, 0.2).positions['shoulder_joint'] == pytest.approx(0.0)
    assert _buffer().sample(11.0, 0.2).positions['shoulder_joint'] == pytest.approx(1.0)


def test_outside_history_and_gaps_return_none():
    b = _buffer()
    assert b.sample(9.99, 0.2) is None
    assert b.sample(11.01, 0.2) is None
    assert b.sample(10.25, max_gap_s=0.05) is None


def test_window_and_ordering():
    b = _buffer()
    b.add(10.5, NAMES, [9.0, 9.0])          # out of order: dropped
    b.add(12.0, NAMES, [0.0, 0.0])          # evicts samples older than 11.0
    assert b.latest_stamp() == 12.0
    assert len(b) == 2
    b.add(12.1, NAMES, [0.0])               # malformed: dropped
    assert len(b) == 2
    with pytest.raises(ValueError):
        JointStateBuffer(0.0)
