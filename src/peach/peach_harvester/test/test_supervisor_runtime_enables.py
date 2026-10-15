"""运行时SetEnables必须在锁定/选果/派发边界生效，覆盖本地参数."""
from types import SimpleNamespace

from peach_harvester.supervisor.executor_node import TaskExecutorNode
from peach_harvester.supervisor.harvest_fsm import (
    COMPLETED, DISCOVERY, Event, react,
)
from peach_interfaces.msg import Enables
from peach_interfaces.srv import SetEnables
import pytest


def _node():
    """真实SetEnables回调与使能读取，ROS传输用内存记录替代."""
    node = SimpleNamespace(
        _enables_override={'execution': True},
        _params=SimpleNamespace(execution_enabled=True),
        _run_enabled=True, _cancel=False, _survey_only=False,
        _survey_goal=SimpleNamespace(scene_key='s'),
        _wait_lock=lambda: None, _publish_scene_snapshot=lambda *_args: None,
        _emit=lambda *_args, **_kwargs: None, _publish_state=lambda **_kwargs: None,
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
        _react=lambda event: react(DISCOVERY, event),
        _apply=lambda *_args: None,
    )
    node._publish_enables = lambda _reason: Enables(
        execution=node._enables_override['execution'])
    node._execution_enabled_effective = lambda: TaskExecutorNode._execution_enabled_effective(node)
    return node


def _disable(node):
    response = TaskExecutorNode._on_set_enables(
        node, SetEnables.Request(execution=False), SetEnables.Response())
    assert response.accepted


def test_set_enables_after_begin_is_read_at_lock_completion():
    """开批快照仍为true时，运行期关使能必须直接结算而非LOCK_READY."""
    node = _node()
    _disable(node)
    reaction, _persist = TaskExecutorNode._cmd_wait_lock(
        node, SimpleNamespace(request_id='r'), None)
    assert reaction.batch_state == COMPLETED
    assert node._params.execution_enabled is True
    assert node._execution_enabled_effective() is False


def test_selection_reads_runtime_disable_before_claiming_or_querying():
    """锁后选果前关闭时，不查询IK或claim目标，直接走FSM结算."""
    node = _node()
    _disable(node)
    reaction, _persist = TaskExecutorNode._cmd_select(
        node, SimpleNamespace(request_id='r'), None)
    assert reaction.batch_state == COMPLETED


@pytest.mark.parametrize('method,event', [('_cmd_dispatch', Event.OBSERVE_FAILED),
                                          ('_cmd_full', Event.FULL_SKIPPED)])
def test_disabled_dispatch_or_full_does_not_send_motion(method, event):
    """选果后瞬时关闭使能时，派发前拒绝，不创建动作goal."""
    node = _node()
    _disable(node)
    node._current_target_id = 't1'
    node._react = lambda actual: actual
    node._fail_dispatch = lambda reaction, *_args, **_kwargs: (reaction, True)
    actual, _persist = getattr(TaskExecutorNode, method)(
        node, SimpleNamespace(request_id='r'), None)
    assert actual == event


def test_runtime_override_remains_authoritative_over_parameter():
    """操作台明确打开时，本地参数false不能反向覆盖运行时许可."""
    node = _node()
    node._params.execution_enabled = False
    reaction, _persist = TaskExecutorNode._cmd_wait_lock(
        node, SimpleNamespace(request_id='r'), None)
    assert reaction.batch_state == DISCOVERY
    assert node._execution_enabled_effective() is True


def test_disable_during_reachability_prevents_claim_and_running(monkeypatch):
    """IK服务等待期间关使能，返回后不能再claim并派发已选目标."""
    from peach_harvester.supervisor import executor_node

    node = _node()
    node._outcomes, node._discovered, node._claimed = [], 1, set()
    node._batch_policy, node._observations = None, None
    node._lock_set_ready = lambda: True
    node._params = SimpleNamespace(
        execution_enabled=True, selection_depth_min_m=0., selection_depth_max_m=2.,
        selection_reach_min_m=0., selection_reach_max_m=2., execute_pregrasp_only=True)
    monkeypatch.setattr(executor_node, 'ratio_reached', lambda *_args: False)
    monkeypatch.setattr(executor_node, 'reach_queries', lambda *_args: [])
    monkeypatch.setattr(executor_node, 'next_target', lambda *_args, **_kwargs: ('t1', {}))

    def query(_requests):
        _disable(node)
        return {}, ''

    node._query_reachability = query
    reaction, _persist = TaskExecutorNode._cmd_select(
        node, SimpleNamespace(request_id='r', target_ids=[]), None)
    assert reaction.batch_state == COMPLETED
    assert node._claimed == set()
