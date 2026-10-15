"""Default dispatch avoids ageing a ready model during a second observation."""
from pathlib import Path
from types import SimpleNamespace

from peach_common.yaml_params import load_ros_parameters
from peach_harvester.supervisor.executor_node import TaskExecutorNode
from peach_harvester.supervisor.harvest_fsm import Event
from peach_interfaces.action import ExecuteTarget
import pytest


def deployed():
    return load_ros_parameters(Path(__file__).parents[1] / 'config/peach_supervisor.yaml',
                               'peach_supervisor')


def test_default_starts_build_then_defers_geometry_gate_to_arm():
    config = deployed()
    sequence = []
    build = object()

    def forbidden(*args, **kwargs):
        pytest.fail('default dispatch must not add serial observation after starting Build')
    node = SimpleNamespace(
        _execution_enabled_effective=lambda: True,
        _params=SimpleNamespace(**{key: config[key] for key in (
            'action_timeout_s', 'reconstruction_min_views', 'build_start_timeout_s',
            'observe_build_grace_s', 'skip_reconstruction', 'reconstruct_in_trajectory')}),
        _current_target_id='target_3', _scene_epoch=4, _action_generation=8,
        _target_deadline_exceeded=lambda request: False,
        _take_skip=lambda: False, _wait_target_in_locked_set=lambda *args: True,
        _batch_policy=SimpleNamespace(view_policy=0),
        _build=object(), _on_build_feedback=lambda *args: None, _in_flight=[],
        _send_goal=lambda *args, **kwargs: sequence.append('build_goal') or build,
        _wait_build_started=lambda *args: sequence.append('build_started') or True,
        _react=lambda event: sequence.append(event) or event,
        _apply=lambda *args: None,
        get_logger=lambda: SimpleNamespace(info=lambda *args: None),
        _dispatch_fast=forbidden, _dispatch_observe=forbidden,
        _wait_build_after_observe=forbidden,
    )
    reaction, persist = TaskExecutorNode._cmd_dispatch(node, SimpleNamespace(request_id='r'), None)
    assert reaction == Event.READY_FULL and persist
    assert sequence == ['build_goal', 'build_started', Event.READY_FULL]
    assert node._in_flight_build_handle is build
    assert node._last_model_revision == 'pending:4:target_3'


@pytest.mark.parametrize('pregrasp_only', [True, False])
def test_parallel_full_keeps_mode_and_does_not_attach_old_clearance(pregrasp_only):
    class Captured(Exception):
        pass
    calls = []

    def send(client, goal, *args, **kwargs):
        calls.append(goal)
        raise Captured()
    node = SimpleNamespace(
        _execution_enabled_effective=lambda: True,
        _params=SimpleNamespace(
            action_timeout_s=180., execute_pregrasp_only=pregrasp_only,
            skip_reconstruction=False,
            reconstruct_in_trajectory=deployed()['reconstruct_in_trajectory'],
            tool=SimpleNamespace(profile_id='bite_shear_v1')),
        _target_deadline_exceeded=lambda request: False,
        _current_target_id='target_3', _run_id='r', _cycle_id='r:target_3',
        _decision_cache=SimpleNamespace(target_id='target_3', allowed=True),
        _scene_epoch=4, _action_generation=8, _cycle_plan_id='r:target_3:8',
        _last_model_revision='pending:4:target_3', _last_calibration_revision='pending',
        _last_config_revision='pending', _exec=object(), _run_goal_handle=object(),
        _send_action=send,
    )
    with pytest.raises(Captured):
        TaskExecutorNode._cmd_full(node, SimpleNamespace(request_id='r'), None)
    goal = calls[0]
    expected_mode = (ExecuteTarget.Goal.PREGRASP_ONLY
                     if pregrasp_only else ExecuteTarget.Goal.FULL)
    assert goal.mode == expected_mode
    assert goal.skip_observation
    assert goal.clearance.target_id == ''
    assert not goal.clearance.allowed
    assert goal.target_id == 'target_3' and goal.scene_epoch == 4
