"""Zero-ROS 3D reducer: pause/resume/cancel/out-of-order."""
from peach_harvester.supervisor.domain.reducer import (
    BatchEvent,
    OrchestratorState,
    reduce_event,
)
from peach_harvester.supervisor.harvest_fsm import (
    Command,
    DISCOVERY,
    Event,
    MODE_PAUSED,
    RUNNING,
    WAITING_READY,
)


def test_pause_keeps_running_and_resume_dispatches_once():
    # set_paused/set_recovery/begin_session 是节点未调用的死动词（W6-B
    # 删除）；暂停投影由 OrchestratorState.operation_mode 直接表达。
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s',
        operation_mode=MODE_PAUSED)
    assert state.batch_state == RUNNING
    assert state.operation_mode == MODE_PAUSED
    state, effects = reduce_event(
        state, BatchEvent(Event.READY_FULL, transaction_id='txn-1', session_id='s'))
    assert not effects
    assert state.batch_state == RUNNING
    state = OrchestratorState(
        batch_state=state.batch_state, target_phase=state.target_phase,
        session_id='s', settled_transaction=state.settled_transaction)
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
    state = OrchestratorState(
        batch_state=DISCOVERY, target_phase=0, session_id='sess-a')
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


# ---- W2/S1 回归：事件级事务 id 的两条不变量 ----
# 节点侧约定（executor_node._react）：每个事件携带 f'{generation}:{event}'
# 事务 id。若节点传常驻 txn，终局结案后其余事件全部被 _stale 第三条款
# 丢弃，主循环 NONE 分支无限重发 CYCLE_DONE（终局活锁）。


def test_event_scoped_txn_progresses_after_terminal():
    """终局结案后，不同事件（不同 txn）必须照常推进，不得被丢弃."""
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s',
        transaction_id='4:READY_FULL')
    state, effects = reduce_event(
        state,
        BatchEvent(
            Event.FULL_SUCCEEDED, transaction_id='4:FULL_SUCCEEDED',
            generation=4, session_id='s'))
    assert effects[0].event_code == 'target_succeeded'
    assert state.settled_transaction == '4:FULL_SUCCEEDED'
    # 主循环 NONE 分支随后发 CYCLE_DONE（同世代、不同事件 → 不同 txn）：
    # 不得命中 settled 去重而丢失。
    state, effects = reduce_event(
        state,
        BatchEvent(
            Event.CYCLE_DONE, transaction_id='4:CYCLE_DONE',
            generation=4, session_id='s'))
    assert state.settled_transaction == '4:FULL_SUCCEEDED'
    if effects:
        assert effects[0].command != ''


def test_event_scoped_txn_still_dedups_same_event_redelivery():
    """同一世代同一事件的重复投递（迟到结果回调）仍被结案去重."""
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s',
        settled_transaction='4:FULL_SUCCEEDED')
    state, effects = reduce_event(
        state,
        BatchEvent(
            Event.FULL_SUCCEEDED, transaction_id='4:FULL_SUCCEEDED',
            generation=4, session_id='s'))
    assert effects == []
    assert state.batch_state == RUNNING


def test_stale_drop_counted():
    """迟到/重复事件丢弃必须计数可见（P0 观测性）."""
    state = OrchestratorState(
        batch_state=RUNNING, target_phase=2, session_id='s')
    state, effects = reduce_event(
        state,
        BatchEvent(
            Event.FULL_SUCCEEDED, transaction_id='1:FULL_SUCCEEDED',
            generation=1, session_id='other'))
    assert effects == []
    assert state.dropped_stale_events == 1
    state, effects = reduce_event(
        state,
        BatchEvent(
            Event.FULL_SUCCEEDED, transaction_id='1:FULL_SUCCEEDED',
            generation=1, session_id='other'))
    assert state.dropped_stale_events == 2


def test_fresh_event_keeps_zero_counter():
    state = OrchestratorState(
        batch_state=DISCOVERY, target_phase=1, session_id='s')
    state, effects = reduce_event(
        state, BatchEvent(Event.NAV_OK, session_id='s'))
    assert effects
    assert state.dropped_stale_events == 0
