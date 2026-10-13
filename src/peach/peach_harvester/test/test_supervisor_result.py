"""RunHarvest result tests without starting a node, executor, or ROS graph."""
import time
from types import SimpleNamespace

from peach_harvester.supervisor.executor_node import TaskExecutorNode
from peach_harvester.supervisor.harvest_fsm import (
    COMPLETED, DISCOVERY, Event, react,
)
from peach_interfaces.action import RunHarvest
from peach_interfaces.msg import TargetOutcome
import pytest


class _GoalHandle:
    """Record the terminal action status at the transport boundary."""

    def __init__(self, cancel_requested=False):
        self.is_cancel_requested = cancel_requested
        self.status = None

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'


def _settle(intent, outcomes=(), discovered=0, event=Event.EMPTY_LIMIT,
            cancel=False, action_cancel=False, pregrasp_only=False):
    """Exercise the real result builder and FSM state application, without I/O."""
    reaction = react(DISCOVERY, event)
    goal = RunHarvest.Goal(intent=intent)
    handle = _GoalHandle(action_cancel)
    node = SimpleNamespace(
        _params=SimpleNamespace(
            empty_survey_limit=1, execute_pregrasp_only=pregrasp_only),
        _execution_enabled_effective=lambda: True,
        _cancel=cancel,
        _run_id='result_test',
        _run_started=time.monotonic(),
        _outcomes=list(outcomes),
        _discovered=discovered,
        _batch_state=reaction.batch_state,
        _target_phase=reaction.target_phase,
        _operation_mode=reaction.operation_mode,
        _publish_state=lambda **kwargs: None,
    )
    node._apply = lambda *args, **kwargs: TaskExecutorNode._apply(
        node, *args, **kwargs)
    node._apply_state = lambda *args, **kwargs: TaskExecutorNode._apply_state(
        node, *args, **kwargs)
    result = TaskExecutorNode._run_harvest_body(node, handle, goal, reaction)
    return result, handle.status, node._batch_state


@pytest.mark.parametrize('intent', [
    RunHarvest.Goal.INTENT_PICK_ALL, RunHarvest.Goal.INTENT_PICK_SELECTED,
])
@pytest.mark.parametrize('discovered', [0, 1])
def test_harvest_without_success_fails_even_without_attempts(intent, discovered):
    """Empty and entirely filtered scenes do not fulfill a harvest request."""
    result, status, state = _settle(intent, discovered=discovered)
    assert result.summary.discovered == discovered
    assert result.summary.attempted == 0
    assert result.summary.succeeded == 0
    assert result.success is False
    assert result.termination_reason == 'no_targets_succeeded'
    # COMPLETED/SUCCEEDED describe orderly settlement; the result carries
    # whether the requested product was fulfilled.
    assert status == 'succeeded'
    assert state == COMPLETED


@pytest.mark.parametrize('pregrasp_only', [False, True])
def test_one_success_fulfills_full_or_pregrasp_request(pregrasp_only):
    outcome = TargetOutcome(target_id='t1', outcome=TargetOutcome.SUCCEEDED)
    result, status, state = _settle(
        RunHarvest.Goal.INTENT_PICK_ALL, [outcome], discovered=1,
        pregrasp_only=pregrasp_only)
    assert result.summary.succeeded == 1
    assert result.success is True
    assert result.termination_reason == 'completed'
    assert status == 'succeeded'
    assert state == COMPLETED


@pytest.mark.parametrize('code', [
    TargetOutcome.FAILED, TargetOutcome.SKIPPED_QUALITY,
    TargetOutcome.SKIPPED_UNREACHABLE, TargetOutcome.CANCELED,
])
def test_unsuccessful_target_outcome_does_not_fulfill_harvest(code):
    outcome = TargetOutcome(target_id='t1', outcome=code)
    result, status, _ = _settle(RunHarvest.Goal.INTENT_PICK_ALL, [outcome])
    assert result.summary.attempted == 1
    assert result.success is False
    assert result.termination_reason == 'no_targets_succeeded'
    assert status == 'succeeded'


@pytest.mark.parametrize('discovered', [0, 1])
def test_completed_survey_only_does_not_require_harvest(discovered):
    result, status, state = _settle(
        RunHarvest.Goal.INTENT_SURVEY_ONLY, discovered=discovered,
        event=Event.SURVEY_ONLY)
    assert result.summary.succeeded == 0
    assert result.success is True
    assert result.termination_reason == 'completed'
    assert status == 'succeeded'
    assert state == COMPLETED


@pytest.mark.parametrize('intent', [
    RunHarvest.Goal.INTENT_PICK_ALL, RunHarvest.Goal.INTENT_SURVEY_ONLY,
])
@pytest.mark.parametrize('event, reason', [
    (Event.SURVEY_FAILED, 'survey_failed'),
    (Event.BEGIN_FAILED, 'begin_scene_failed'),
    (Event.NAV_FAILED, 'navigate_failed'),
])
def test_abort_retains_cause_instead_of_no_product(intent, event, reason):
    result, status, _ = _settle(intent, event=event)
    assert result.success is False
    assert result.termination_reason == reason
    assert status == 'aborted'


@pytest.mark.parametrize('action_cancel, status', [
    (True, 'canceled'), (False, 'aborted'),
])
def test_cancel_overrides_previous_target_success(action_cancel, status):
    outcome = TargetOutcome(target_id='t1', outcome=TargetOutcome.SUCCEEDED)
    result, terminal_status, _ = _settle(
        RunHarvest.Goal.INTENT_PICK_ALL, [outcome], event=Event.CANCEL,
        cancel=True, action_cancel=action_cancel)
    assert result.summary.succeeded == 1
    assert result.success is False
    assert result.termination_reason == 'canceled'
    assert terminal_status == status
