"""Zero-ROS tests for harvest_fsm.react."""
from peach_executor.harvest_fsm import (
    Command,
    DISCOVERY,
    Event,
    OBSERVING,
    react,
    RUNNING,
    TARGET_IDLE,
    VALIDATING,
    WAITING_READY,
)


def test_run_requested_navigates():
    hit = react(WAITING_READY, Event.RUN_REQUESTED)
    assert hit.batch_state == DISCOVERY
    assert hit.command == Command.NAVIGATE


def test_survey_at_pose_begins_scene():
    hit = react(DISCOVERY, Event.SURVEY_AT_POSE)
    assert hit.command == Command.BEGIN_SCENE
    assert hit.batch_state == DISCOVERY


def test_target_selected_dispatches():
    hit = react(DISCOVERY, Event.TARGET_SELECTED)
    assert hit.batch_state == RUNNING
    assert hit.target_phase == OBSERVING
    assert hit.command == Command.DISPATCH


def test_ready_full_executes_full():
    hit = react(RUNNING, Event.READY_FULL)
    assert hit.command == Command.EXECUTE_FULL
    assert hit.target_phase == VALIDATING


def test_cycle_done_returns_to_survey():
    hit = react(RUNNING, Event.CYCLE_DONE)
    assert hit.batch_state == DISCOVERY
    assert hit.command == Command.SURVEY
    assert hit.target_phase == TARGET_IDLE


def test_unknown_pair_keeps_state():
    hit = react(WAITING_READY, Event.CYCLE_DONE)
    assert hit.batch_state == WAITING_READY
    assert hit.command == Command.NONE
    assert hit.target_phase == TARGET_IDLE
