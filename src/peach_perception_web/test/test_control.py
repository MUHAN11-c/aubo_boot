# Copyright 2026 wjz
"""Web 控制边界与参数档案测试。."""

import json

from peach_perception_web.control import CommandGuard, ProfileStore
import pytest


def test_command_guard_rejects_wrong_nonce_and_stale_revision():
    """伪造会话或旧页面不能发送控制命令。."""
    guard = CommandGuard('session-secret')

    denied = guard.authorize('wrong', 12, 12, maintenance_required=False)
    assert denied == (False, '会话 nonce 无效')

    stale = guard.authorize('session-secret', 11, 12, maintenance_required=False)
    assert stale == (False, '状态版本已过期')


def test_debug_commands_require_maintenance_mode():
    """自动模式中的手动按钮不能绕过唯一调度所有权。."""
    guard = CommandGuard('session-secret')
    denied = guard.authorize(
        'session-secret', 4, 4, maintenance_required=True,
        operation_mode=0)
    assert denied == (False, '手动调试仅在维护模式可用')

    allowed = guard.authorize(
        'session-secret', 4, 4, maintenance_required=True,
        operation_mode=2)
    assert allowed == (True, '')


def test_profile_store_round_trips_named_profile(tmp_path):
    """合法命名档案能够持久化并原样读取。."""
    store = ProfileStore(tmp_path)
    values = {
        'orchestrator': {'execution_enabled': False},
        'reconstruction': {'min_views': 5},
    }
    store.save('sim_safe', values)

    assert store.list_names() == ['sim_safe']
    assert store.load('sim_safe') == values
    assert json.loads((tmp_path / 'sim_safe.json').read_text()) == values


@pytest.mark.parametrize('name', ['', '../escape', '有 空格', 'a/b'])
def test_profile_store_rejects_unsafe_names(tmp_path, name):
    """档案名不能逃逸 peach_profiles 目录。."""
    store = ProfileStore(tmp_path)
    with pytest.raises(ValueError, match='档案名'):
        store.save(name, {})
