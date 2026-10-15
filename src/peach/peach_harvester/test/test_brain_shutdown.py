"""Normal spin interruption quiesces callbacks before destroying ROS nodes."""
from types import SimpleNamespace

from peach_harvester import brain
import pytest
from rclpy.executors import ExternalShutdownException


@pytest.mark.parametrize('exception', [KeyboardInterrupt, ExternalShutdownException, RuntimeError])
@pytest.mark.parametrize('context_live', [True, False])
def test_shutdown_order_and_unexpected_errors(monkeypatch, exception, context_live):
    calls = []

    def node(name):
        return SimpleNamespace(destroy_node=lambda: calls.append('destroy:' + name))

    def spin():
        raise exception('test interrupt')

    executor = SimpleNamespace(
        add_node=lambda node: None, spin=spin,
        shutdown=lambda: calls.append('executor_shutdown'))
    monkeypatch.setattr(brain, 'ScenePerceptionNode', lambda: node('scene'))
    monkeypatch.setattr(brain, 'TargetReconstructionNode', lambda: node('recon'))
    monkeypatch.setattr(brain, 'TaskExecutorNode', lambda: node('supervisor'))
    monkeypatch.setattr(brain, 'MultiThreadedExecutor', lambda: executor)
    monkeypatch.setattr(brain.rclpy, 'init', lambda **kwargs: calls.append('init'))
    monkeypatch.setattr(brain.rclpy, 'ok', lambda: context_live)
    monkeypatch.setattr(brain.rclpy, 'shutdown', lambda: calls.append('context_shutdown'))
    raised = None
    try:
        brain.main()
    except BaseException as exc:
        raised = exc
    if exception is RuntimeError:
        assert isinstance(raised, RuntimeError)
    else:
        assert raised is None
    expected = ['init', 'executor_shutdown', 'destroy:scene', 'destroy:recon',
                'destroy:supervisor']
    if context_live:
        expected.append('context_shutdown')
    assert calls == expected
