# Copyright 2026 wjz
"""GatewayParams 不可变快照与 web.yaml 默认值同步测试（A9）."""

from dataclasses import FrozenInstanceError
from pathlib import Path

from peach_perception_web.params import DEFAULTS, load_params, TOPIC_NAMES
import pytest

YAML_PATH = (
    Path(__file__).resolve().parent.parent / 'config' / 'web.yaml')


class _FakeParam:
    """最小参数替身：load_params 只触 .value."""

    def __init__(self, value):
        """保存原始值."""
        self.value = value


class _FakeNode:
    """不依赖 ROS 上下文的节点替身：按字典供参."""

    def __init__(self, values):
        """以 {参数名: 值} 字典构造."""
        self._values = dict(values)

    def get_parameter(self, name):
        """返回 _FakeParam 包装的参数值."""
        return _FakeParam(self._values[name])


def _default_values():
    return {name: default for name, (default, _desc) in DEFAULTS.items()}


def test_yaml_defaults_match_declare_defaults():
    """web.yaml 与 DEFAULTS 键集一致、逐值相等（默认值权威源双向同步）."""
    import yaml  # 延迟 import：仅本测试需要 PyYAML
    doc = yaml.safe_load(YAML_PATH.read_text())
    yaml_params = doc['peach_perception_web']['ros__parameters']
    # record.* 在 yaml 里是嵌套组，展开成点号键再比对
    flat = {}
    for key, value in yaml_params.items():
        if isinstance(value, dict):
            for sub_key, sub_value in value.items():
                flat[f'{key}.{sub_key}'] = sub_value
        else:
            flat[key] = value
    assert set(flat) == set(DEFAULTS)
    for key, (default, _desc) in DEFAULTS.items():
        assert flat[key] == default, f'{key}: yaml={flat[key]!r} != {default!r}'


def test_load_params_builds_frozen_snapshot():
    """默认装载成功：topics 全覆盖、列表固化为 tuple、快照不可变."""
    params = load_params(_FakeNode(_default_values()))
    assert params.port == 8090
    assert set(params.topics) == set(TOPIC_NAMES)
    assert params.topics['orchestrator_events_topic'] == (
        '/peach_harvest_orchestrator/events')
    assert isinstance(params.metrics_process_patterns, tuple)
    with pytest.raises(FrozenInstanceError):
        params.port = 1
    with pytest.raises(TypeError):
        params.topics['port'] = 'x'


def test_load_params_rejects_invalid_values():
    """启动期校验：端口越界/缓冲与周期非正 → ValueError 整包拒绝."""
    for key, bad in (
            ('port', 0), ('port', 70000),
            ('event_buffer_size', 0),
            ('param_poll_period_s', 0.0), ('metrics_period_s', -1.0)):
        values = _default_values()
        values[key] = bad
        with pytest.raises(ValueError, match=key):
            load_params(_FakeNode(values))
