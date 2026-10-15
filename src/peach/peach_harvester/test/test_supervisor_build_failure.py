"""并行Build质量失败应取消等待中的FULL并保留具体拒因."""
from types import SimpleNamespace

from peach_harvester.supervisor.executor_node import TaskExecutorNode
import pytest


class _Future:
    def __init__(self, result, done=True):
        self.value, self.ready = result, done

    def done(self):
        return self.ready

    def result(self):
        return self.value

    def add_done_callback(self, _callback):
        pass


@pytest.mark.parametrize('success', [False, True])
def test_waiting_full_is_canceled_only_if_parallel_build_fails(success):
    result = object()
    arm_future = _Future(SimpleNamespace(result=result, status=5), done=False)
    build_future = _Future(SimpleNamespace(result=SimpleNamespace(
        success=success, message='bag_axis_envelope_ill_conditioned; attempts=3')))
    handle = SimpleNamespace(get_result_async=lambda: arm_future)
    canceled = []
    host = SimpleNamespace(
        _cancel=False, _paused=False, _peek_skip=lambda: False, _poke=lambda: None,
        _idle=lambda *_args: setattr(arm_future, 'ready', True),
        _cancel_handle=lambda actual: canceled.append(actual),
        get_logger=lambda: SimpleNamespace(warning=lambda *_args: None),
    )
    actual = TaskExecutorNode._wait_result(
        host, handle, 5., prerequisite_future=build_future)
    assert actual is result
    assert canceled == ([] if success else [handle])


def test_already_failed_build_prevents_sending_new_full_goal():
    build = _Future(SimpleNamespace(result=SimpleNamespace(
        success=False, message='bag_axis_silhouette_unobservable; attempts=3')))
    host = SimpleNamespace(_send_goal=lambda *_args, **_kwargs: pytest.fail('sent FULL'))
    result = TaskExecutorNode._send_action(
        host, object(), object(), 5., prerequisite_future=build)
    assert result is None


def test_failed_build_is_recorded_with_specific_quality_reason_without_full_dispatch():
    from peach_interfaces.msg import FailureCode, TargetOutcome
    from peach_harvester.supervisor.harvest_fsm import Event

    reason = 'bag_axis_envelope_ill_conditioned; attempts=3'
    build_future = _Future(SimpleNamespace(result=SimpleNamespace(success=False, message=reason)))
    build_handle = SimpleNamespace(get_result_async=lambda: build_future)
    outcomes, canceled = [], []
    node = SimpleNamespace(
        _params=SimpleNamespace(action_timeout_s=180., execute_pregrasp_only=False,
                                skip_reconstruction=False, reconstruct_in_trajectory=True,
                                tool=SimpleNamespace(profile_id='bite_shear_v1')),
        _execution_enabled_effective=lambda: True,
        _target_deadline_exceeded=lambda _request: False,
        _current_target_id='t1', _run_id='r', _cycle_id='r:t1',
        _decision_cache=None, _scene_epoch=1, _action_generation=1, _cycle_plan_id='p1',
        _last_model_revision='pending:1:t1', _last_calibration_revision='cal',
        _last_config_revision='cfg', _exec=object(), _run_goal_handle=object(),
        _in_flight_build_handle=build_handle, _take_skip=lambda: False,
        _cycle_observe_extra={}, _cycle_dispatch_t0=0.,
        _send_goal=lambda *_args, **_kwargs: pytest.fail('FULL was dispatched'),
        _cancel_handle=canceled.append, _wait_result=lambda *_args, **_kwargs: None,
        _push_outcome=lambda outcome, extra: outcomes.append((outcome, extra)),
        _react=lambda event: event, _apply=lambda *_args: None,
    )
    node._send_action = lambda *args, **kwargs: TaskExecutorNode._send_action(
        node, *args, **kwargs)
    reaction, _persist = TaskExecutorNode._cmd_full(node, SimpleNamespace(request_id='r'), None)
    assert reaction == Event.FULL_SKIPPED
    outcome, extra = outcomes[0]
    assert outcome.outcome == TargetOutcome.SKIPPED_QUALITY
    assert outcome.reason == 'build_failed: ' + reason
    assert extra['failure_code_n'] == FailureCode.BUILD_FAILED
    assert canceled == [build_handle]
