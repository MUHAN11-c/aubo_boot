"""Automatic analysis cannot keep the ROS node alive after bag closure."""
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock

from peach_observability import bag_report
from peach_observability import observability_node as module
import pytest


def test_report_process_uses_current_python_and_sealed_bag(monkeypatch, tmp_path):
    process = Mock()
    process.poll.return_value = None
    spawn = Mock(return_value=process)
    monkeypatch.setattr(module, 'subprocess', SimpleNamespace(Popen=spawn), raising=False)
    monkeypatch.setattr(bag_report, 'generate_session_report', lambda *a, **kw: None)
    node = SimpleNamespace(_report_process=None, _report_thread=None, _params=None,
                           get_logger=lambda: SimpleNamespace(info=lambda s: None,
                                                              warning=lambda s: None))
    module.ObservabilityNode._spawn_report(node, tmp_path / 'bag')
    args = spawn.call_args.args[0]
    assert args[1:3] == ['-m', 'peach_observability.bag_report']
    assert str(tmp_path / 'bag') in args
    assert spawn.call_args.kwargs.get('start_new_session') is False
    assert node._report_process is process


@pytest.mark.parametrize('timeout_count', [0, 1, 2])
def test_report_shutdown_reaps_child_even_when_analysis_hangs(timeout_count, capsys):
    process = Mock()
    process.wait.side_effect = [subprocess.TimeoutExpired('report', 1)] * timeout_count + [0]
    node = SimpleNamespace(_report_process=process, _report_thread=None)
    module.ObservabilityNode.join_report(node, timeout=.01)
    assert process.wait.call_count == timeout_count + 1
    assert process.terminate.call_count == (1 if timeout_count else 0)
    assert process.kill.call_count == (1 if timeout_count == 2 else 0)
    assert node._report_process is None
    if timeout_count:
        assert '离线' in capsys.readouterr().err
