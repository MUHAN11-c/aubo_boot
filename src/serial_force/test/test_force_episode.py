"""零 ROS：五点力只在离开空载到归零之间成段，并跟正在写的采摘会话."""

import os

from serial_force.force_episode import (
    ForceEpisodeTracker, live_peach_session, next_episode_index)


def _touch_bag(session, age_s, now, with_metadata=False):
    bag = session / 'bag'
    bag.mkdir(parents=True)
    data = bag / 'bag_0.mcap'
    data.write_bytes(b'x')
    if with_metadata:
        (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n')
    stamp = now - age_s
    for path in (data, bag, *bag.iterdir()):
        os.utime(path, (stamp, stamp))


def test_idle_noise_is_not_recorded():
    tracker = ForceEpisodeTracker()
    assert tracker.update(0.0, 0.0, [0.0]) is None
    assert tracker.update(1.0, 0.14, [0.14]) is None
    assert tracker.phase == 'idle'
    assert tracker.rows == []


def test_episode_runs_from_onset_until_return():
    tracker = ForceEpisodeTracker(onset_n=0.15, release_n=0.08, hold_s=0.3)
    assert tracker.update(0.0, 0.0, ['idle']) is None
    assert tracker.update(1.0, 0.2, ['on']) is None
    assert tracker.phase == 'recording'
    assert tracker.update(1.2, 1.5, ['peak']) is None
    assert tracker.update(1.5, 0.05, ['fall']) is None
    closed = tracker.update(1.8, 0.0, ['zero'])
    assert closed is not None
    assert closed.reason == 'returned'
    assert [row[0] for row in closed.rows] == ['on', 'peak', 'fall', 'zero']
    assert abs(closed.contact_s - 0.2) < 1e-9
    assert abs(closed.duration_s - 0.8) < 1e-9
    assert tracker.phase == 'idle'


def test_brief_dip_does_not_split_one_grasp():
    tracker = ForceEpisodeTracker(onset_n=0.15, release_n=0.08, hold_s=0.3)
    tracker.update(0.0, 0.4, ['a'])
    tracker.update(0.1, 0.02, ['dip'])
    assert tracker.update(0.2, 0.5, ['back']) is None
    assert tracker.phase == 'recording'
    assert tracker.update(0.5, 0.0, ['fall']) is None
    closed = tracker.update(0.8, 0.0, ['end'])
    assert closed is not None
    assert closed.reason == 'returned'
    assert [row[0] for row in closed.rows] == ['a', 'dip', 'back', 'fall', 'end']


def test_second_episode_starts_after_return():
    tracker = ForceEpisodeTracker(onset_n=0.15, release_n=0.08, hold_s=0.2)
    tracker.update(0.0, 0.3, ['a'])
    assert tracker.update(0.25, 0.0, ['fall']) is None
    first = tracker.update(0.5, 0.0, ['z'])
    assert first is not None
    assert tracker.update(1.0, 0.0, ['gap']) is None
    assert tracker.update(1.25, 0.4, ['b']) is None
    assert tracker.update(1.5, 0.0, ['fall2']) is None
    second = tracker.update(1.75, 0.0, ['z2'])
    assert second is not None
    assert [row[0] for row in second.rows] == ['b', 'fall2', 'z2']


def test_live_session_is_the_fresh_bag(tmp_path):
    now = 1_000_000.0
    old = tmp_path / 'session_20261022_090000'
    live = tmp_path / 'session_20261022_110000'
    abandoned = tmp_path / 'session_20261022_100000'
    _touch_bag(old, age_s=3600, now=now, with_metadata=True)
    _touch_bag(abandoned, age_s=600, now=now, with_metadata=False)
    _touch_bag(live, age_s=1, now=now, with_metadata=False)
    assert live_peach_session(tmp_path, now=now) == live
    assert live_peach_session(tmp_path / 'missing', now=now) is None


def test_sealed_stale_session_is_not_reused(tmp_path):
    now = 1_000_000.0
    sealed = tmp_path / 'session_20261022_080000'
    _touch_bag(sealed, age_s=30, now=now, with_metadata=True)
    assert live_peach_session(tmp_path, now=now) is None


def test_episode_index_continues_in_the_same_session(tmp_path):
    force = tmp_path / 'force'
    force.mkdir()
    (force / 'episode_002.csv').write_text('t_s\n')
    (force / 'episode_007.csv').write_text('t_s\n')
    assert next_episode_index(force) == 8
    assert next_episode_index(tmp_path / 'empty') == 1


def test_stuck_load_closes_at_max_duration():
    tracker = ForceEpisodeTracker(
        onset_n=0.15, release_n=0.08, hold_s=0.3, max_s=1.0)
    tracker.update(0.0, 0.4, ['a'])
    assert tracker.update(0.5, 0.4, ['b']) is None
    closed = tracker.update(1.0, 0.4, ['c'])
    assert closed is not None
    assert closed.reason == 'max_duration'
    assert closed.contact_s == 1.0
    assert tracker.phase == 'idle'
