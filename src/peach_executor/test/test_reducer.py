"""Zero-ROS 3D reducer: pause/resume/cancel/out-of-order."""
from peach_executor.domain.reducer import (
    BatchEvent,
    begin_session,
    OrchestratorState,
    reduce_event,
    set_paused,
)
from peach_executor.harvest_fsm import (
    Command,
    DISCOVERY,
    Event,
    MODE_PAUSED,
    RUNNING,
    WAITING_READY,
)


def test_pause_keeps_running_and_resume_dispatches_once():
    state = OrchestratorState(batch_state=RUNNING, target_phase=2, session_id='s')
    state = set_paused(state, True)
    assert state.batch_state == RUNNING
    assert state.operation_mode == MODE_PAUSED
    state, effects = reduce_event(
        state, BatchEvent(Event.READY_FULL, transaction_id='txn-1', session_id='s'))
    assert not effects
    assert state.batch_state == RUNNING
    state = set_paused(state, False)
    state, effects = reduce_event(
        state, BatchEvent(Event.READY_FULL, transaction_id='txn-1', session_id='s'))
    assert len(effects) == 1
    assert effects[0].command == Command.EXECUTE_FULL


def test_first_success_settles_once():
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s',
        transaction_id='txn-1')
    state, effects = reduce_event(
        state, BatchEvent(Event.FULL_SUCCEEDED, transaction_id='txn-1', session_id='s'))
    assert len(effects) == 1
    assert effects[0].command == Command.NONE
    assert effects[0].event_code == 'target_succeeded'
    assert state.settled_transaction == 'txn-1'


def test_duplicate_terminal_does_not_redispatch():
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s',
        settled_transaction='txn-1')
    state, effects = reduce_event(
        state, BatchEvent(Event.FULL_SUCCEEDED, transaction_id='txn-1', session_id='s'))
    assert effects == []


def test_cross_session_dropped():
    state = begin_session(
        OrchestratorState(batch_state=DISCOVERY, target_phase=0), 'sess-a')
    state, effects = reduce_event(
        state, BatchEvent(Event.TARGET_SELECTED, session_id='sess-b'))
    assert effects == []
    assert state.batch_state == DISCOVERY


def test_late_generation_dropped():
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, generation=5, session_id='s')
    state, effects = reduce_event(
        state, BatchEvent(Event.READY_FULL, generation=3, session_id='s'))
    assert effects == []


def test_cancel_interrupts():
    state = OrchestratorState(batch_state=RUNNING, target_phase=2)
    state, effects = reduce_event(state, BatchEvent(Event.CANCEL))
    assert state.batch_state == 8
    assert effects[0].command == Command.INTERRUPT


def test_waiting_ready_run_requested():
    state = OrchestratorState(batch_state=WAITING_READY, target_phase=0)
    state, effects = reduce_event(state, BatchEvent(Event.RUN_REQUESTED))
    assert state.batch_state == DISCOVERY
    assert effects[0].command == Command.NAVIGATE
