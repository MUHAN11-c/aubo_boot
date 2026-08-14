# Copyright 2026 wjz
"""DashboardState 事件环形缓冲与快照结构测试."""

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
