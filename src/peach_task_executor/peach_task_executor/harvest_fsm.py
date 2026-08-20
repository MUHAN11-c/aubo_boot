"""
批次编排纯核：事件到动作映射表（对齐 MoveIt hybrid_planning PlannerLogic）.

零 ROS import。节点把动作结果翻译成 Event，再执行 Reaction.command。
batch_state / target_phase 只由本表推导，禁止节点手写枚举。
"""
from __future__ import annotations

from dataclasses import dataclass

# HarvestState.msg 批次枚举（数值必须与 IDL 一致）
WAITING_READY = 0
DISCOVERY = 1
RUNNING = 2
PAUSE_PENDING = 3
PAUSED = 4
MAINTENANCE = 5
COMPLETED = 6
RECOVERY_REQUIRED = 7
INTERRUPTED = 8

TARGET_IDLE = 0
SELECTING = 1
OBSERVING = 2
FINALIZING = 3
VALIDATING = 4
APPROACHING = 5
TOOL_ACTION = 6
RETREATING = 7
COMPLETING = 8
TARGET_SUCCEEDED = 9
TARGET_SKIPPED = 10
TARGET_FAILED = 11

MODE_AUTO = 0
MODE_PAUSED = 1
MODE_MAINTENANCE = 2

CMD_PAUSE = 0
CMD_RESUME = 1
CMD_ENTER_MAINTENANCE = 2
CMD_EXIT_MAINTENANCE = 3
CMD_CANCEL_NOW = 4
CMD_SKIP_TARGET = 5
CMD_ACKNOWLEDGE_RECOVERY = 6

BATCH_NAMES = {
    WAITING_READY: 'waiting_ready',
    DISCOVERY: 'discovery',
    RUNNING: 'running',
    PAUSE_PENDING: 'pause_pending',
    PAUSED: 'paused',
    MAINTENANCE: 'maintenance',
    COMPLETED: 'completed',
    RECOVERY_REQUIRED: 'recovery_required',
    INTERRUPTED: 'interrupted',
}


class Event:
    """Harvest orchestration event names."""

    RUN_REQUESTED = 'run_requested'
    BEGIN_OK = 'begin_ok'
    BEGIN_FAILED = 'begin_failed'
    SURVEY_DONE = 'survey_done'
    SURVEY_ONLY = 'survey_only'
    NO_TARGET = 'no_target'
    EMPTY_LIMIT = 'empty_limit'
    TARGET_SELECTED = 'target_selected'
    EXECUTION_DISABLED = 'execution_disabled'
    SKIP = 'skip'
    OBSERVE_FAILED = 'observe_failed'
    BUILD_FAILED = 'build_failed'
    READY_FULL = 'ready_full'
    FULL_SUCCEEDED = 'full_succeeded'
    FULL_FAILED = 'full_failed'
    FULL_CANCELED = 'full_canceled'
    FULL_SKIPPED = 'full_skipped'
    CYCLE_DONE = 'cycle_done'
    CANCEL = 'cancel'


class Command:
    """Next ROS I/O command for the executor node."""

    BEGIN_SCENE = 'begin_scene'
    SURVEY = 'survey'
    SELECT = 'select'
    DISPATCH = 'dispatch'
    EXECUTE_FULL = 'execute_full'
    RECORD_DISABLED = 'record_disabled'
    SETTLE = 'settle'
    ABORT = 'abort'
    INTERRUPT = 'interrupt'
    NONE = 'none'


@dataclass(frozen=True)
class Reaction:
    """一次 react() 的结果."""

    batch_state: int
    target_phase: int
    command: str
    event_code: str
    message: str
    operation_mode: int = MODE_AUTO


_TABLE: dict[tuple, Reaction] = {
    (WAITING_READY, Event.RUN_REQUESTED): Reaction(
        DISCOVERY, TARGET_IDLE, Command.BEGIN_SCENE, '', 'preparing'),
    (DISCOVERY, Event.BEGIN_FAILED): Reaction(
        INTERRUPTED, TARGET_IDLE, Command.ABORT, '', 'begin_scene_failed'),
    (DISCOVERY, Event.BEGIN_OK): Reaction(
        DISCOVERY, TARGET_IDLE, Command.SURVEY, '', 'discovery'),
    (DISCOVERY, Event.SURVEY_DONE): Reaction(
        DISCOVERY, SELECTING, Command.SELECT, 'round_locked', 'discovery'),
    (DISCOVERY, Event.SURVEY_ONLY): Reaction(
        COMPLETED, TARGET_IDLE, Command.SETTLE, '', 'completed'),
    (DISCOVERY, Event.NO_TARGET): Reaction(
        DISCOVERY, TARGET_IDLE, Command.SURVEY, '', 'discovery'),
    (DISCOVERY, Event.EMPTY_LIMIT): Reaction(
        COMPLETED, TARGET_IDLE, Command.SETTLE, '', 'completed'),
    (DISCOVERY, Event.TARGET_SELECTED): Reaction(
        RUNNING, OBSERVING, Command.DISPATCH, 'target_dispatched',
        'running'),
    (DISCOVERY, Event.EXECUTION_DISABLED): Reaction(
        COMPLETED, TARGET_SKIPPED, Command.RECORD_DISABLED, '',
        'completed'),
    (RUNNING, Event.SKIP): Reaction(
        RUNNING, TARGET_SKIPPED, Command.NONE, 'target_operator_skipped',
        'running'),
    (RUNNING, Event.OBSERVE_FAILED): Reaction(
        RUNNING, TARGET_SKIPPED, Command.NONE, 'target_skipped', 'running'),
    (RUNNING, Event.BUILD_FAILED): Reaction(
        RUNNING, TARGET_SKIPPED, Command.NONE, 'target_skipped', 'running'),
    (RUNNING, Event.READY_FULL): Reaction(
        RUNNING, VALIDATING, Command.EXECUTE_FULL, '', 'running'),
    (RUNNING, Event.FULL_SUCCEEDED): Reaction(
        RUNNING, TARGET_SUCCEEDED, Command.NONE, 'target_succeeded',
        'running'),
    (RUNNING, Event.FULL_FAILED): Reaction(
        RUNNING, TARGET_FAILED, Command.NONE, 'target_failed', 'running'),
    (RUNNING, Event.FULL_CANCELED): Reaction(
        RUNNING, TARGET_IDLE, Command.NONE, 'target_canceled', 'running'),
    (RUNNING, Event.FULL_SKIPPED): Reaction(
        RUNNING, TARGET_SKIPPED, Command.NONE, 'target_skipped', 'running'),
    (RUNNING, Event.CYCLE_DONE): Reaction(
        DISCOVERY, SELECTING, Command.SELECT, '', 'discovery'),
}


def react(batch_state: int, event: str) -> Reaction:
    """查表：当前批次态 + 事件 → 下一态与命令。未知组合保持原态."""
    if event == Event.CANCEL:
        return Reaction(
            INTERRUPTED, TARGET_IDLE, Command.INTERRUPT, '', 'interrupted')
    hit = _TABLE.get((batch_state, event))
    if hit is not None:
        return hit
    return Reaction(
        batch_state, TARGET_IDLE, Command.NONE, '',
        BATCH_NAMES.get(batch_state, 'unknown'))


def permissions_for(batch_state: int, recovery_required: bool) -> list:
    """当前允许的 ControlTask 命令列表."""
    allowed = []
    if batch_state in (DISCOVERY, RUNNING, PAUSE_PENDING):
        allowed = [CMD_PAUSE, CMD_CANCEL_NOW, CMD_ENTER_MAINTENANCE]
        if batch_state == RUNNING:
            allowed.append(CMD_SKIP_TARGET)
    elif batch_state == PAUSED:
        allowed = [CMD_RESUME, CMD_CANCEL_NOW, CMD_ENTER_MAINTENANCE]
    elif batch_state == MAINTENANCE:
        allowed = [CMD_EXIT_MAINTENANCE, CMD_CANCEL_NOW]
    elif batch_state == RECOVERY_REQUIRED:
        allowed = [CMD_CANCEL_NOW]
    # 技能接触锁与批次态无关：空闲时也必须能 ACK，否则下一颗 ExecuteTarget 会被拒
    if recovery_required and CMD_ACKNOWLEDGE_RECOVERY not in allowed:
        allowed.append(CMD_ACKNOWLEDGE_RECOVERY)
    return allowed


def event_for_outcome(outcome: int, operator_skip: bool = False) -> str:
    """Map a TargetOutcome code to an orchestration Event."""
    if operator_skip:
        return Event.SKIP
    mapping = {
        0: Event.FULL_SUCCEEDED,
        1: Event.FULL_SKIPPED,
        2: Event.FULL_SKIPPED,
        3: Event.FULL_FAILED,
        4: Event.FULL_CANCELED,
    }
    return mapping.get(int(outcome), Event.FULL_FAILED)


def canonical_code_for_outcome(
        outcome: int, operator_skip: bool = False) -> str:
    """记录器 TERMINAL_TARGET_CODES 词表."""
    if operator_skip:
        return 'target_operator_skipped'
    mapping = {
        0: 'target_succeeded',
        1: 'target_skipped',
        2: 'target_skipped',
        3: 'target_failed',
        4: 'target_canceled',
    }
    return mapping.get(int(outcome), 'target_failed')
