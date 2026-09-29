"""三维批次 reducer：target_phase × operation_mode × recovery_latch."""
from __future__ import annotations

from dataclasses import dataclass, replace

from peach_harvester.supervisor.harvest_fsm import (
    apply_event,
    Command,
    Event,
    INTERRUPTED,
    MODE_AUTO,
    MODE_PAUSED,
    PAUSE_PENDING,
    react,
    Reaction,
)

# PAUSE_PENDING 留在 IDL 兼容层；reducer 永不写入该批次态。


@dataclass(frozen=True)
class OrchestratorState:
    """与 HarvestState 正交三维对齐的纯核状态."""

    batch_state: int
    target_phase: int
    operation_mode: int = MODE_AUTO
    recovery_latch: bool = False
    generation: int = 0
    transaction_id: str = ''
    session_id: str = ''
    settled_transaction: str = ''
    dropped_stale_events: int = 0
    """迟到/重复/跨世代事件丢弃计数（观测性：丢弃不可静默）."""


@dataclass(frozen=True)
class BatchEvent:
    """带事务世代的事件；迟到/重复/跨会话丢弃."""

    name: str
    transaction_id: str = ''
    generation: int = 0
    session_id: str = ''


@dataclass(frozen=True)
class Effect:
    """有限副作用：派发命令或结案."""

    command: str
    event_code: str = ''
    message: str = ''


def _stale(state: OrchestratorState, event: BatchEvent) -> bool:
    if event.session_id and state.session_id and event.session_id != state.session_id:
        return True
    if event.generation and state.generation and event.generation < state.generation:
        return True
    if (event.transaction_id and state.settled_transaction
            and event.transaction_id == state.settled_transaction):
        return True
    return False


def reduce_event(
        state: OrchestratorState, event: BatchEvent) -> tuple[OrchestratorState, list]:
    """单一入口：event → state + 有限 effects。禁止 NONE busy-loop 语义在此产生."""
    if event.name == Event.CANCEL:
        hit = react(state.batch_state, Event.CANCEL)
        nxt = replace(
            state,
            batch_state=INTERRUPTED,
            target_phase=hit.target_phase,
            generation=state.generation + 1,
            transaction_id='',
        )
        return nxt, [Effect(hit.command, hit.event_code, hit.message)]
    if _stale(state, event):
        # 迟到/重复/跨世代事件丢弃必须可观测：计数进状态，节点侧投影诊断
        return replace(
            state,
            dropped_stale_events=state.dropped_stale_events + 1), []
    paused = state.operation_mode == MODE_PAUSED
    hit = apply_event(state.batch_state, event.name, paused=paused)
    if hit.batch_state == PAUSE_PENDING:
        hit = Reaction(
            state.batch_state, hit.target_phase, Command.NONE,
            hit.event_code, hit.message, MODE_PAUSED)
    nxt_generation = state.generation
    nxt_txn = state.transaction_id
    settled = state.settled_transaction
    effects = []
    if hit.command not in (Command.NONE, '') and not paused:
        nxt_generation = state.generation + 1
        nxt_txn = event.transaction_id or f'{nxt_generation}:{event.name}'
        effects.append(Effect(hit.command, hit.event_code, hit.message))
    elif not paused and hit.event_code:
        # 终局 command=NONE 仍须带 event_code，供账本/审计结案一次。
        effects.append(Effect(Command.NONE, hit.event_code, hit.message))
    elif hit.command not in (Command.NONE, '') and paused:
        # 暂停：推进 phase，命令入 hold（由调用方 EventHold 处理）
        pass
    if hit.event_code in (
            'target_succeeded', 'target_failed', 'target_skipped',
            'target_canceled', 'target_operator_skipped'):
        settled = event.transaction_id or nxt_txn
    nxt = replace(
        state,
        batch_state=hit.batch_state,
        target_phase=hit.target_phase,
        operation_mode=hit.operation_mode if paused else state.operation_mode,
        generation=nxt_generation,
        transaction_id=nxt_txn,
        settled_transaction=settled,
    )
    return nxt, effects
