# Copyright 2026 wjz
"""DashboardState 事件环形缓冲与快照结构测试."""

import json

from peach_perception_web.state import DashboardState


def test_append_event_keeps_recent_limit():
    """事件环形缓冲超限后丢弃最旧，只保留最近 limit 条."""
    state = DashboardState()
    for index in range(5):
        state.append_event({'sequence': index}, limit=3)
    snapshot = state.snapshot()
    events = snapshot['orchestration']['events']
    assert [item['sequence'] for item in events] == [2, 3, 4]
    assert 'orchestration.events' in snapshot['system']['topic_age_s']


def test_snapshot_contains_new_sections():
    """快照包含事件/机械臂/性能区段，初始为空."""
    snapshot = DashboardState().snapshot()
    assert snapshot['orchestration']['events'] == []
    assert snapshot['robot']['status'] == {}
    assert snapshot['metrics']['sample'] == {}


def test_snapshot_serializable_and_schema_unchanged():
    """浅拷贝快照仍可直接 json.dumps，内容与键集和写侧一致（契约不破）."""
    state = DashboardState()
    state.update('perception', 'targets', {
        'target_count': 2, 'observations': [
            {'target_id': 'p1', 'candidate': {'entry_position': [0, 0, 0]}}]})
    state.update_params('/node', {'rate': 10})
    snapshot = state.snapshot()
    # HTTP 层唯一消费方式：整体序列化（必须一次成功，schema 与旧深拷贝相同）
    text = json.dumps(snapshot, ensure_ascii=False)
    assert 'p1' in text
    assert snapshot['perception']['targets']['target_count'] == 2
    assert snapshot['params']['/node'] == {'rate': 10}
    assert set(snapshot['system']) == {
        'revision', 'server_time', 'uptime_s', 'topic_age_s'}


def test_snapshot_section_level_isolation():
    """快照顶层/区层级改动不写穿缓存（区段 dict 与 events 列表均已复制）."""
    state = DashboardState()
    state.update('robot', 'status', {'mode': 1})
    state.append_event({'sequence': 1}, limit=10)
    snapshot = state.snapshot()
    snapshot['robot']['status'] = {'mode': 99}  # 区段键替换：不写穿
    snapshot['orchestration']['events'].append({'sequence': 99})
    snapshot['metrics'] = {}
    fresh = state.snapshot()
    assert fresh['robot']['status'] == {'mode': 1}
    assert [item['sequence'] for item in fresh['orchestration']['events']] == [1]
    assert fresh['metrics']['sample'] == {}


def test_snapshot_reflects_latest_writes():
    """写侧 copy-on-write：旧快照持有旧叶子，新快照看到整体替换后的值."""
    state = DashboardState()
    state.update('approach', 'status', {'state': 'OBSERVING'})
    old = state.snapshot()
    state.update('approach', 'status', {'state': 'APPROACHING'})
    new = state.snapshot()
    assert old['approach']['status']['state'] == 'OBSERVING'
    assert new['approach']['status']['state'] == 'APPROACHING'
    assert new['system']['revision'] > old['system']['revision']
