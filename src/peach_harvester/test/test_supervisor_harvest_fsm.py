"""Zero-ROS tests for harvest_fsm.react."""
from peach_harvester.supervisor.harvest_fsm import (
    apply_event,
    apply_resume,
    CMD_PAUSE,
    CMD_RESUME,
    Command,
    DISCOVERY,
    Event,
    EventHold,
    MODE_PAUSED,
    OBSERVING,
    PAUSE_PENDING,
    permissions_for,
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


def test_pause_pending_table_does_not_dispatch():
    """PAUSE_PENDING 不是作业 phase；不得靠给暂停态加表来修好（F02）."""
    for event in (Event.READY_FULL, Event.FULL_SUCCEEDED, Event.CYCLE_DONE):
        hit = react(PAUSE_PENDING, event)
        assert hit.command == Command.NONE
        assert hit.batch_state == PAUSE_PENDING
    assert apply_resume(PAUSE_PENDING) == PAUSE_PENDING


def test_apply_event_paused_suppresses_command():
    hit = apply_event(RUNNING, Event.READY_FULL, paused=True)
    assert hit.command == Command.NONE
    assert hit.batch_state == RUNNING
    assert hit.target_phase == VALIDATING
    assert hit.operation_mode == MODE_PAUSED
    live = apply_event(RUNNING, Event.READY_FULL, paused=False)
    assert live.command == Command.EXECUTE_FULL


def test_event_hold_ready_full_resumes_once():
    hold = EventHold()
    paused = hold.apply_event(RUNNING, Event.READY_FULL, paused=True)
    assert paused.command == Command.NONE
    again = hold.apply_event(RUNNING, Event.READY_FULL, paused=True)
    assert again.command == Command.NONE
    released = hold.release()
    assert released is not None
    assert released.command == Command.EXECUTE_FULL
    assert hold.release() is None


def test_event_hold_cancel_discards_without_dispatch():
    hold = EventHold()
    hold.apply_event(RUNNING, Event.READY_FULL, paused=True)
    assert hold.take_after_pause(cancel=True) is None
    assert hold.release() is None


def test_permissions_resume_when_paused_keeps_running_phase():
    allowed = permissions_for(RUNNING, recovery_required=False, paused=True)
    assert CMD_RESUME in allowed
    assert CMD_PAUSE not in allowed
    live = permissions_for(RUNNING, recovery_required=False, paused=False)
    assert CMD_PAUSE in live
    assert CMD_RESUME not in live


def test_blockers_for_vocabulary():
    """P1 软门：blockers 词表机读口径（HarvestState.blockers 填充源）."""
    from peach_harvester.supervisor.harvest_fsm import (
        BLOCKER_LEDGER_WRITE_FAILED,
        BLOCKER_MODE_MAINTENANCE,
        BLOCKER_MODE_PAUSED,
        BLOCKER_RECOVERY_REQUIRED,
        BLOCKER_STACK_NOT_READY,
        MODE_AUTO,
        MODE_MAINTENANCE,
        MODE_PAUSED,
        blockers_for,
    )
    assert blockers_for(
        operation_mode=MODE_AUTO, recovery_required=False) == []
    assert blockers_for(
        operation_mode=MODE_AUTO, recovery_required=False,
        stack_ready=False) == [BLOCKER_STACK_NOT_READY]
    assert blockers_for(
        operation_mode=MODE_AUTO, recovery_required=False,
        stack_ready=True) == []
    # stack_ready=None 表示不要求托管栈，不算阻塞
    assert blockers_for(
        operation_mode=MODE_AUTO, recovery_required=False,
        stack_ready=None) == []
    assert blockers_for(
        operation_mode=MODE_PAUSED, recovery_required=True) == [
        BLOCKER_RECOVERY_REQUIRED, BLOCKER_MODE_PAUSED]
    assert blockers_for(
        operation_mode=MODE_MAINTENANCE, recovery_required=False,
        ledger_write_failures=2) == [
        BLOCKER_MODE_MAINTENANCE, BLOCKER_LEDGER_WRITE_FAILED]
