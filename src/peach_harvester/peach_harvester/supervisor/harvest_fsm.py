"""
批次编排纯核：事件到动作映射表（对齐 MoveIt hybrid_planning PlannerLogic）.

零 ROS import。节点把动作结果翻译成 Event，再执行 Reaction.command。
batch_state / target_phase 只由本表推导，禁止节点手写枚举；表外迁移
（暂停/恢复/恢复等待/终局）同样只经本模块纯函数：
enter_pause / apply_pause_pending / apply_resume / enter_recovery /
apply_recovery_ack / settle_terminal。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

# HarvestState.msg 批次枚举（数值必须与 IDL 一致）
WAITING_READY = 0
"""栈未就绪或未开批；RunHarvest 尚未推进."""
DISCOVERY = 1
"""发现段：导航/BeginScene/收齐窗口，尚未派接触."""
RUNNING = 2
"""已锁定并正在对当前目标执行观察或接触."""
PAUSE_PENDING = 3
"""已请求暂停，等在途动作收口后再进 PAUSED."""
PAUSED = 4
"""批次暂停；须 CMD_RESUME 才继续."""
MAINTENANCE = 5
"""维护态（IDL 对齐保留；现行未接线）."""
COMPLETED = 6
"""批次正常结算."""
RECOVERY_REQUIRED = 7
"""接触故障，须人确认后 ACK 才能再派."""
INTERRUPTED = 8
"""导航/扫场等不可恢复失败，批次中断."""

TARGET_IDLE = 0
"""当前无目标阶段."""
SELECTING = 1
"""从锁定集选下一颗."""
OBSERVING = 2
"""补视/积分观察."""
FINALIZING = 3
"""收口几何与质量门（含验证）."""
VALIDATING = 4
"""抓取前再确认（新鲜观测+锚点漂移）."""
APPROACHING = 5
"""接近/套入段."""
TOOL_ACTION = 6
"""刀具 IO（剪切）."""
RETREATING = 7
"""沿轴撤退."""
COMPLETING = 8
"""plan-only 圆满收尾."""
TARGET_SUCCEEDED = 9
"""本目标成功."""
TARGET_SKIPPED = 10
"""本目标跳过（时限/采收率/人工）."""
TARGET_FAILED = 11
"""本目标失败."""

MODE_AUTO = 0
"""自动推进."""
MODE_PAUSED = 1
"""暂停投影（操作台）."""
MODE_MAINTENANCE = 2
"""维护投影（IDL 对齐）."""

CMD_PAUSE = 0
"""ControlTask：请求暂停."""
CMD_RESUME = 1
"""ControlTask：从暂停恢复."""
CMD_ENTER_MAINTENANCE = 2
"""ControlTask：进维护（未接线）."""
CMD_EXIT_MAINTENANCE = 3
"""ControlTask：出维护（IDL 对齐保留）."""
CMD_CANCEL_NOW = 4
"""ControlTask：立即取消批次."""
CMD_SKIP_TARGET = 5
"""ControlTask：跳过当前目标."""
CMD_ACKNOWLEDGE_RECOVERY = 6
"""ControlTask：人确认现场后解除恢复等待."""

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
    """节点把动作/服务结果翻译成这些名字后交给 ``react()``."""

    RUN_REQUESTED = 'run_requested'
    """RunHarvest goal 被接受."""
    NAV_OK = 'nav_ok'
    """固定座无导航或导航成功."""
    NAV_FAILED = 'nav_failed'
    """导航失败 → 批次 INTERRUPTED."""
    BEGIN_OK = 'begin_ok'
    """BeginScene 成功，进入收齐窗口."""
    BEGIN_FAILED = 'begin_failed'
    """BeginScene 失败."""
    SURVEY_AT_POSE = 'survey_at_pose'
    """SurveyScene 已到拍照位."""
    SURVEY_FAILED = 'survey_failed'
    """SurveyScene 失败."""
    SURVEY_DONE = 'survey_done'
    """SurveyScene 正常结束."""
    LOCK_READY = 'lock_ready'
    """感知收齐窗口关闭，锁定集可用."""
    SURVEY_ONLY = 'survey_only'
    """本批只扫不摘，直接结算."""
    NO_TARGET = 'no_target'
    """锁定集空或选果窗无候选，再扫一轮."""
    EMPTY_LIMIT = 'empty_limit'
    """连续空扫达上限，结算."""
    TARGET_SELECTED = 'target_selected'
    """已选出下一颗，进入 RUNNING."""
    EXECUTION_DISABLED = 'execution_disabled'
    """execution_enabled=false，不派接触."""
    SKIP = 'skip'
    """时限/采收率/人工跳过当前目标."""
    OBSERVE_FAILED = 'observe_failed'
    """观察周期失败."""
    BUILD_FAILED = 'build_failed'
    """BuildTargetModel 失败."""
    READY_FULL = 'ready_full'
    """观察+重建就绪，可派 FULL/PREGRASP."""
    FULL_SUCCEEDED = 'full_succeeded'
    """ExecuteTarget 成功."""
    FULL_FAILED = 'full_failed'
    """ExecuteTarget 失败."""
    FULL_CANCELED = 'full_canceled'
    """ExecuteTarget 被取消."""
    FULL_SKIPPED = 'full_skipped'
    """ExecuteTarget 跳过."""
    CYCLE_DONE = 'cycle_done'
    """本目标周期收口，回到选果."""
    CANCEL = 'cancel'
    """整批取消."""


class Command:
    """``Reaction.command``：节点下一步该发的 ROS I/O（禁止手写 batch_state）."""

    BEGIN_SCENE = 'begin_scene'
    """调感知 BeginScene."""
    NAVIGATE = 'navigate'
    """固定座为空操作成功；有底盘时导航到位."""
    SURVEY = 'survey'
    """发 SurveyScene."""
    WAIT_LOCK = 'wait_lock'
    """等收齐窗口关闭（survey_wait_s）."""
    SELECT = 'select'
    """从锁定集选下一颗."""
    DISPATCH = 'dispatch'
    """派观察周期（OBSERVE_ONLY）."""
    EXECUTE_FULL = 'execute_full'
    """派 ExecuteTarget FULL 或 PREGRASP_ONLY."""
    RECORD_DISABLED = 'record_disabled'
    """使能关：记账后结算，不动臂."""
    SETTLE = 'settle'
    """写账本、关批."""
    ABORT = 'abort'
    """失败中止."""
    INTERRUPT = 'interrupt'
    """不可恢复中断."""
    NONE = 'none'
    """无 I/O（表内等待）."""


@dataclass(frozen=True)
class Reaction:
    """一次 ``react()`` 的结果；节点只执行 ``command``，禁止手写 batch_state."""

    batch_state: int
    """HarvestState.batch_state（WAITING_READY…INTERRUPTED）."""
    target_phase: int
    """HarvestState.target_phase（TARGET_IDLE…TARGET_FAILED）."""
    command: str
    """Command.* 下一步 ROS I/O."""
    event_code: str
    """账本/话题 CanonicalEvent.code；空串=不另发事件."""
    message: str
    """人读短句，写入 HarvestState.message."""
    operation_mode: int = MODE_AUTO
    """MODE_AUTO / PAUSED / MAINTENANCE 投影."""


_TABLE: dict[tuple, Reaction] = {
    (WAITING_READY, Event.RUN_REQUESTED): Reaction(
        DISCOVERY, TARGET_IDLE, Command.NAVIGATE, '', 'preparing'),
    (DISCOVERY, Event.NAV_FAILED): Reaction(
        INTERRUPTED, TARGET_IDLE, Command.ABORT, 'navigate_failed',
        'navigate_failed'),
    (DISCOVERY, Event.NAV_OK): Reaction(
        DISCOVERY, TARGET_IDLE, Command.SURVEY, '', 'surveying'),
    (DISCOVERY, Event.SURVEY_FAILED): Reaction(
        INTERRUPTED, TARGET_IDLE, Command.ABORT, 'survey_failed',
        'survey_failed'),
    (DISCOVERY, Event.SURVEY_AT_POSE): Reaction(
        DISCOVERY, TARGET_IDLE, Command.BEGIN_SCENE, '', 'preparing'),
    (DISCOVERY, Event.BEGIN_FAILED): Reaction(
        INTERRUPTED, TARGET_IDLE, Command.ABORT, 'begin_scene_failed',
        'begin_scene_failed'),
    (DISCOVERY, Event.BEGIN_OK): Reaction(
        DISCOVERY, TARGET_IDLE, Command.WAIT_LOCK, '', 'collecting'),
    (DISCOVERY, Event.LOCK_READY): Reaction(
        DISCOVERY, SELECTING, Command.SELECT, 'round_locked', 'discovery'),
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
        DISCOVERY, TARGET_IDLE, Command.SURVEY, '', 'discovery'),
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


def apply_event(phase: int, event: str, paused: bool) -> Reaction:
    """
    Apply react() to the workflow phase; pause keeps command NONE.

    Pause must not overwrite workflow state: callers pass RUNNING/DISCOVERY
    rather than PAUSE_PENDING. EventHold stores dispatch while paused.
    """
    hit = react(phase, event)
    if not paused:
        return hit
    return Reaction(
        hit.batch_state,
        hit.target_phase,
        Command.NONE,
        hit.event_code,
        hit.message,
        MODE_PAUSED,
    )


@dataclass
class EventHold:
    """暂停期间暂存第一条非 NONE 命令，恢复时只释放一次."""

    pending: Optional[Reaction] = None
    pending_event: Optional[str] = None

    def apply_event(self, phase: int, event: str, paused: bool) -> Reaction:
        """Paused hold records the first non-NONE command once."""
        hit = react(phase, event)
        if paused:
            if (hit.command != Command.NONE
                    and self.pending_event != event):
                self.pending = hit
                self.pending_event = event
            return Reaction(
                hit.batch_state,
                hit.target_phase,
                Command.NONE,
                hit.event_code,
                hit.message,
                MODE_PAUSED,
            )
        return hit

    def release(self) -> Optional[Reaction]:
        """恢复时取出暂存命令；无暂存返回 None."""
        held = self.pending
        self.pending = None
        self.pending_event = None
        return held

    def take_after_pause(self, cancel: bool) -> Optional[Reaction]:
        """Resume dispatches the held command once; cancel discards it."""
        held = self.release()
        if cancel:
            return None
        return held


# 终局批次态：进入后暂停/恢复类迁移一律无意义
_TERMINAL_BATCH_STATES = (COMPLETED, INTERRUPTED)


def enter_pause(state: int) -> int:
    """
    请求暂停后的去向：非终局且非 WAITING_READY → PAUSE_PENDING.

    终局态或 WAITING_READY 下暂停无意义，保持原态；节点按返回值与
    当前态是否相等决定要不要发布，避免重复广播。
    """
    if state in _TERMINAL_BATCH_STATES or state == WAITING_READY:
        return state
    return PAUSE_PENDING


def apply_pause_pending(state: int) -> int:
    """暂停确认：PAUSE_PENDING → PAUSED，其余保持原态."""
    if state == PAUSE_PENDING:
        return PAUSED
    return state


def apply_resume(saved: int) -> int:
    """恢复：回到暂停前保存的批次态；saved 非法批次态回 DISCOVERY."""
    if saved in BATCH_NAMES:
        return saved
    return DISCOVERY


def enter_recovery(state: int) -> int:
    """
    进入恢复等待：非终局且非 WAITING_READY → RECOVERY_REQUIRED.

    终局态或 WAITING_READY 下无批可恢复，保持原态。
    """
    if state in _TERMINAL_BATCH_STATES or state == WAITING_READY:
        return state
    return RECOVERY_REQUIRED


def apply_recovery_ack(saved: int) -> int:
    """恢复确认：回到进恢复等待前保存的批次态；非法回 DISCOVERY."""
    if saved in BATCH_NAMES:
        return saved
    return DISCOVERY


def settle_terminal() -> int:
    """终局统一入口：正常收口一律返回 COMPLETED，节点不得裸写终局枚举."""
    return COMPLETED


def permissions_for(batch_state: int, recovery_required: bool,
                    paused: bool = False) -> list:
    """
    List ControlTask commands allowed in this batch/pause state.

    ``paused`` is orthogonal to batch_state: while paused the workflow
    stays RUNNING/DISCOVERY and RESUME is allowed instead of PAUSE.
    """
    allowed = []
    if paused and batch_state not in _TERMINAL_BATCH_STATES:
        allowed = [CMD_RESUME, CMD_CANCEL_NOW, CMD_ENTER_MAINTENANCE]
    elif batch_state in (DISCOVERY, RUNNING, PAUSE_PENDING):
        allowed = [CMD_PAUSE, CMD_CANCEL_NOW, CMD_ENTER_MAINTENANCE]
        if batch_state == RUNNING:
            allowed.append(CMD_SKIP_TARGET)
    elif batch_state == PAUSED:
        allowed = [CMD_RESUME, CMD_CANCEL_NOW, CMD_ENTER_MAINTENANCE]
    elif batch_state == MAINTENANCE:
        # 预留：MAINTENANCE 批次态与 EXIT_MAINTENANCE 命令未接线
        allowed = [CMD_EXIT_MAINTENANCE, CMD_CANCEL_NOW]
    elif batch_state == RECOVERY_REQUIRED:
        allowed = [CMD_CANCEL_NOW]
    # 技能接触锁与批次态无关：空闲时也必须能 ACK，否则下一颗 ExecuteTarget 会被拒
    if recovery_required and CMD_ACKNOWLEDGE_RECOVERY not in allowed:
        allowed.append(CMD_ACKNOWLEDGE_RECOVERY)
    return allowed


# blockers 词表（HarvestState.blockers，软门机读口径；写进 docs/io.md）：
BLOCKER_STACK_NOT_READY = 'stack_not_ready'
BLOCKER_RECOVERY_REQUIRED = 'recovery_required'
BLOCKER_MODE_PAUSED = 'mode_paused'
BLOCKER_MODE_MAINTENANCE = 'mode_maintenance'
BLOCKER_LEDGER_WRITE_FAILED = 'ledger_write_failed'


def blockers_for(*, operation_mode: int, recovery_required: bool,
                 stack_ready: bool | None = None,
                 ledger_write_failures: int = 0) -> list:
    """
    批次阻塞原因列表（P1 软门：msg.blockers 从恒空到机读可判）.

    ``stack_ready`` 传 None 表示本部署不要求托管栈（require_managed_stack
    =false），不作为阻塞项。词表见上方 BLOCKER_* 常量。
    """
    blockers = []
    if stack_ready is False:
        blockers.append(BLOCKER_STACK_NOT_READY)
    if recovery_required:
        blockers.append(BLOCKER_RECOVERY_REQUIRED)
    if operation_mode == MODE_PAUSED:
        blockers.append(BLOCKER_MODE_PAUSED)
    elif operation_mode == MODE_MAINTENANCE:
        blockers.append(BLOCKER_MODE_MAINTENANCE)
    if ledger_write_failures > 0:
        blockers.append(BLOCKER_LEDGER_WRITE_FAILED)
    return blockers


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
