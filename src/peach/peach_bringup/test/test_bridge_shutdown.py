"""The flag bridge treats normal shutdown separately from callback failures."""
from types import SimpleNamespace

from peach_bringup import lifecycle_flag_bridge as bridge
import pytest
from rclpy.executors import ExternalShutdownException


@pytest.mark.parametrize('exception', [KeyboardInterrupt, ExternalShutdownException, RuntimeError])
@pytest.mark.parametrize('context_live', [True, False])
def test_bridge_shutdown_order(monkeypatch, exception, context_live):
    calls = []

    def spin(*args):
        raise exception('test interrupt')

    executor = SimpleNamespace(add_node=lambda node: None, spin=spin,
                               shutdown=lambda: calls.append('executor_shutdown'))
    monkeypatch.setattr(bridge, 'SingleThreadedExecutor', lambda: executor, raising=False)
    monkeypatch.setattr(bridge, 'LifecycleFlagBridge', lambda: SimpleNamespace(
        destroy_node=lambda: calls.append('destroy')))
    monkeypatch.setattr(bridge.rclpy, 'init', lambda **kwargs: calls.append('init'))
    monkeypatch.setattr(bridge.rclpy, 'spin', spin)
    monkeypatch.setattr(bridge.rclpy, 'ok', lambda: context_live)
    monkeypatch.setattr(bridge.rclpy, 'shutdown', lambda: calls.append('context_shutdown'))
    raised = None
    try:
        bridge.main()
    except BaseException as exc:
        raised = exc
    if exception is RuntimeError:
        assert isinstance(raised, RuntimeError)
    else:
        assert raised is None
    expected = ['init', 'executor_shutdown', 'destroy']
    if context_live:
        expected.append('context_shutdown')
    assert calls == expected
