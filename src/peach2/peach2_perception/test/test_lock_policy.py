from peach2_perception.lock_policy import LockPolicy, LockState
import pytest


def _run(policy, t0, n, dt=0.1, ids=('target_1',), stationary=True, pending=0):
    status = None
    for i in range(n):
        status = policy.update(t0 + i * dt, stationary, ids, pending)
    return status


def test_idle_until_begin_scene():
    p = LockPolicy(5, 1.0, 25.0)
    assert p.state is LockState.IDLE and p.scene_epoch == 0
    s = p.update(1.0, True, ['target_1'], 0)
    assert s.state is LockState.IDLE and not s.locked


def test_locks_after_stationary_frames_and_stable_set():
    p = LockPolicy(5, 1.0, 25.0)
    assert p.begin_scene(10.0) == 1
    s = _run(p, 10.0, 10)  # 0.9 s stable
    assert s.state is LockState.COLLECTING
    s = p.update(11.0, True, ['target_1'], 0)
    assert s.locked and s.reason == 'stable' and s.locked_ids == frozenset({'target_1'})
    s = p.update(11.1, True, ['target_1', 'target_2'], 0)
    assert s.locked_ids == frozenset({'target_1'})  # frozen after lock


def test_set_change_and_motion_restart_stability():
    p = LockPolicy(3, 1.0, 25.0)
    p.begin_scene(0.0)
    _run(p, 0.0, 8)
    s = p.update(0.8, True, ['target_1', 'target_2'], 0)
    assert s.stable_for_s == pytest.approx(0.0)
    s = p.update(0.9, False, ['target_1', 'target_2'], 0)
    assert s.stationary_frames == 0
    s = _run(p, 1.0, 9, ids=('target_1', 'target_2'))
    assert not s.locked
    s = p.update(2.0, True, ['target_1', 'target_2'], 0)
    assert s.locked


def test_pending_tracks_block_stable_lock():
    p = LockPolicy(3, 0.5, 25.0)
    p.begin_scene(0.0)
    s = _run(p, 0.0, 20, pending=1)
    assert not s.locked
    assert p.update(2.0, True, ['target_1'], 0).locked


def test_empty_scene_locks_only_on_timeout():
    p = LockPolicy(3, 0.5, 5.0)
    p.begin_scene(0.0)
    s = _run(p, 0.0, 50, ids=())
    assert not s.locked
    s = p.update(5.0, True, [], 0)
    assert s.locked and s.reason == 'timeout' and s.locked_ids == frozenset()


def test_timeout_locks_moving_scene():
    p = LockPolicy(3, 0.5, 2.0)
    p.begin_scene(0.0)
    s = _run(p, 0.0, 21, stationary=False)
    assert s.locked and s.reason == 'timeout'


def test_begin_scene_rejects_older_frames_and_increments_epoch():
    p = LockPolicy(3, 0.5, 2.0)
    p.begin_scene(5.0)
    assert not p.accepts(4.99) and p.accepts(5.0)
    _run(p, 5.0, 30)
    assert p.begin_scene(9.0) == 2
    assert p.state is LockState.COLLECTING
    assert p.status().locked_ids == frozenset()


def test_invalid_parameters():
    with pytest.raises(ValueError):
        LockPolicy(0, 1.0, 1.0)
    with pytest.raises(ValueError):
        LockPolicy(1, -1.0, 1.0)
    with pytest.raises(ValueError):
        LockPolicy(1, 1.0, 0.0)
