"""Operator driver regressions without DDS, motion or camera processes."""
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
from peach_interfaces.action import RunHarvest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import drive_harvest as driver  # noqa: E402


def _case(monkeypatch, wrapper, timeout='180'):
    handle = SimpleNamespace(accepted=True, canceled=False)
    handle.get_result_async = lambda: SimpleNamespace(result=lambda: wrapper)

    def cancel():
        handle.canceled = True
        return SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[]))

    handle.cancel_goal_async = cancel
    node = SimpleNamespace(destroy_node=lambda: None)
    client = SimpleNamespace(wait_for_server=lambda **kw: True)

    def send(goal):
        handle.goal = goal
        return SimpleNamespace(result=lambda: handle)

    client.send_goal_async = send
    monkeypatch.setattr(driver, 'Node', lambda name: node)
    monkeypatch.setattr(driver, 'ActionClient', lambda *a: client)
    monkeypatch.setattr(driver.rclpy, 'init', lambda: None)
    monkeypatch.setattr(driver.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(driver.rclpy, 'spin_until_future_complete', lambda *a, **k: None)
    monkeypatch.setattr(sys, 'argv', ['drive_harvest', '--request-id', 'offline',
                                    '--skip-enables', '--timeout', timeout])
    return handle


@pytest.mark.parametrize('status,success,expected', [(4, True, 0),
                                                   (4, False, 3), (5, True, 3)])
def test_result_uses_current_idl_and_product_success(monkeypatch, capsys,
                                                    status, success, expected):
    result = RunHarvest.Result(success=success, termination_reason='completed')
    result.summary.elapsed.sec = 12
    _case(monkeypatch, SimpleNamespace(status=status, result=result))
    assert driver.main() == expected
    assert 'elapsed=12.0s' in capsys.readouterr().out


def test_timeout_requests_cancel_before_node_shutdown(monkeypatch):
    handle = _case(monkeypatch, None)
    assert driver.main() == 2
    assert handle.canceled


def test_goal_carries_server_side_time_budgets(monkeypatch):
    result = RunHarvest.Result(success=True)
    handle = _case(monkeypatch, SimpleNamespace(status=4, result=result), '90')
    assert driver.main() == 0
    assert handle.goal.per_target_timeout_s == 90.0
    assert handle.goal.sector_timeout_s == 90.0


@pytest.mark.parametrize('timeout', ['0', '181'])
def test_invalid_cycle_budget_rejected(monkeypatch, timeout):
    _case(monkeypatch, None, timeout)
    with pytest.raises(SystemExit) as exc:
        driver.main()
    assert exc.value.code == 2


def test_rejected_enables_are_not_reported_as_success(monkeypatch):
    response = SimpleNamespace(accepted=False, message='dependency refused')
    client = SimpleNamespace(wait_for_service=lambda **kw: True,
                             call_async=lambda req: SimpleNamespace(result=lambda: response))
    node = SimpleNamespace(create_client=lambda *a: client)
    monkeypatch.setattr(driver.rclpy, 'spin_until_future_complete', lambda *a, **k: None)
    assert not driver._send_enables(node, True, 'offline check')
