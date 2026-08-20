"""
采摘任务执行器：显式 RunHarvest，launch 从不自动开批.

Lifecycle Active 后才接受动作。选择与账本在本节点；感知不再提供
reset/complete/clear/reopen。批次态由 harvest_fsm.react 唯一推导。
"""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

from peach_interfaces.action import (
    BuildTargetModel, ExecuteTarget, RunHarvest, SurveyScene)
from peach_interfaces.msg import (
    CanonicalEvent,
    HarvestState,
    JobIntent,
    PeachTargetObservationArray,
    SceneSnapshot,
    TargetOutcome,
)
from peach_interfaces.srv import BeginScene, ControlTask
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .control import apply_control
from .harvest_fsm import (
    BATCH_NAMES, canonical_code_for_outcome, Command, COMPLETED, Event,
    event_for_outcome, MODE_AUTO, MODE_MAINTENANCE, MODE_PAUSED, PAUSE_PENDING,
    PAUSED, permissions_for, react, RECOVERY_REQUIRED, WAITING_READY)
from .ledger import default_ledger_root, ledger_file, load_ledger, save_ledger
from .select import next_target_id
from .summary import build_summary


class PeachTaskExecutor(LifecycleNode):
    """批次唯一所有者：RunHarvest / ControlTask；SurveyScene 在技能节点."""

    def __init__(self):
        super().__init__('peach_task_executor')
        self._lock = threading.Lock()
        self._state_seq = 0
        self._paused = False
        self._cancel = False
        self._skip_target = False
        self._active = False
        self._run_id = ''
        self._cycle_id = ''
        self._current_target_id = ''
        self._scene_epoch = 0
        self._outcomes = []
        self._batch_state = WAITING_READY
        self._target_phase = 0
        self._operation_mode = MODE_AUTO
        self._action_active = False
        self._recovery_required = False
        self._grasp_enabled = False
        self._tool_enabled = False
        self._progress = 0.0
        self._blockers = []
        self._paused_batch = WAITING_READY
        self._recovery_batch = WAITING_READY
        self._in_flight = []
        self._run_goal_handle = None
        self._harvest_busy = False
        self._run_started = 0.0
        self._discovered = 0
        self._ledger_loaded = False
        self._observations: Optional[PeachTargetObservationArray] = None
        self._stack_ready = False
        self._cb = ReentrantCallbackGroup()
        from peach_task_executor.executor_parameters import executor_node
        self._param_listener = executor_node.ParamListener(self)

    def on_configure(self, state):
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_state = self.create_lifecycle_publisher(
            HarvestState, '~/state', latched)
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_event = self.create_lifecycle_publisher(
            CanonicalEvent, '~/events', event_qos)
        self._pub_snapshot = self.create_lifecycle_publisher(
            SceneSnapshot, '~/scene_snapshot', latched)
        self.create_subscription(
            PeachTargetObservationArray,
            '/peach/perception/target_observations',
            self._on_obs, 10, callback_group=self._cb)
        self._begin = self.create_client(
            BeginScene, self.get_parameter('begin_scene_service').value,
            callback_group=self._cb)
        self._survey = ActionClient(
            self, SurveyScene,
            self.get_parameter('survey_scene_action').value,
            callback_group=self._cb)
        self._exec = ActionClient(
            self, ExecuteTarget,
            self.get_parameter('execute_target_action').value,
            callback_group=self._cb)
        self._build = ActionClient(
            self, BuildTargetModel,
            self.get_parameter('build_target_model_action').value,
            callback_group=self._cb)
        self._ack_recovery = self.create_client(
            Trigger,
            '/peach_manipulation_skills_node/acknowledge_recovery',
            callback_group=self._cb)
        ActionServer(
            self, RunHarvest, '~/run_harvest',
            execute_callback=self._run_harvest,
            goal_callback=self._goal_if_active,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb)
        self.create_service(
            ControlTask, '~/control', self._on_control,
            callback_group=self._cb)
        self.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_stack_ready, latched, callback_group=self._cb)
        self.get_logger().info(
            'task executor configured; will not auto-start harvest')
        return super().on_configure(state)

    def on_activate(self, state):
        result = super().on_activate(state)
        self._active = True
        self._batch_state = WAITING_READY
        self._publish_state()
        return result

    def on_deactivate(self, state):
        self._active = False
        return super().on_deactivate(state)

    def _on_stack_ready(self, msg: Bool) -> None:
        self._stack_ready = bool(msg.data)

    def _goal_if_active(self, goal_request):
        del goal_request
        with self._lock:
            if not self._active:
                return GoalResponse.REJECT
            if (bool(self.get_parameter('require_managed_stack').value)
                    and not self._stack_ready):
                self.get_logger().warning('RunHarvest 拒绝：生命周期栈未就绪')
                return GoalResponse.REJECT
            if self._harvest_busy:
                self.get_logger().warning('RunHarvest 拒绝：已有批次在跑')
                return GoalResponse.REJECT
            self._harvest_busy = True
            return GoalResponse.ACCEPT

    def _accept_cancel(self, cancel_request):
        del cancel_request
        return CancelResponse.ACCEPT

    def _on_obs(self, msg):
        self._observations = msg
        if msg is not None:
            self._discovered = max(
                self._discovered, len(msg.observations))

    def _on_control(self, request, response):
        cmd = int(request.command)
        if cmd == 6:
            return self._on_ack_recovery(request, response)
        with self._lock:
            allowed = permissions_for(
                self._batch_state, self._recovery_required)
            ok, seq, paused, cancel, skip = apply_control(
                self._state_seq, int(request.expected_state_seq),
                cmd, self._paused, allowed=allowed)
            if ok:
                self._state_seq = seq
                self._paused = paused
                if cmd == 2:
                    self._operation_mode = MODE_MAINTENANCE
                elif cmd in (1, 3):
                    self._operation_mode = MODE_AUTO
                if cancel:
                    self._cancel = True
                if skip:
                    self._skip_target = True
            response.accepted = ok
            response.state_seq = self._state_seq
            if ok:
                response.message = 'ok'
            elif int(request.expected_state_seq) not in (0, self._state_seq):
                response.message = 'expected_state_seq mismatch'
            else:
                response.message = 'command not permitted'
        if response.accepted:
            response.state = self._publish_state(bump=False)
        else:
            response.state = self._make_state()
        return response

    def _on_ack_recovery(self, request, response):
        """ACK：技能确认成功后才加 state_seq，失败不消耗序号."""
        expected = int(request.expected_state_seq)
        with self._lock:
            allowed = permissions_for(
                self._batch_state, self._recovery_required)
            if expected != 0 and expected != self._state_seq:
                response.accepted = False
                response.message = 'expected_state_seq mismatch'
                response.state_seq = self._state_seq
                response.state = self._make_state()
                return response
            if 6 not in allowed:
                response.accepted = False
                response.message = 'command not permitted'
                response.state_seq = self._state_seq
                response.state = self._make_state()
                return response
        ack_ok, ack_msg = self._acknowledge_recovery()
        if not ack_ok:
            response.accepted = False
            response.message = ack_msg
            response.state_seq = self._state_seq
            response.state = self._make_state()
            return response
        with self._lock:
            self._state_seq += 1
        response.accepted = True
        response.message = ack_msg
        response.state_seq = self._state_seq
        response.state = self._publish_state(bump=False)
        return response

    def _acknowledge_recovery(self) -> tuple:
        """转发技能 ~/acknowledge_recovery，成功才清批次恢复旗标."""
        timeout = min(
            5.0, float(self.get_parameter('service_timeout_s').value))
        if not self._ack_recovery.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('acknowledge_recovery 不可用')
            return False, 'acknowledge_recovery unavailable'
        resp = self._await_future(
            self._ack_recovery.call_async(Trigger.Request()), timeout)
        if resp is None or not bool(getattr(resp, 'success', False)):
            message = str(getattr(resp, 'message', 'ack failed'))
            self.get_logger().error(f'acknowledge_recovery 失败: {message}')
            return False, message
        with self._lock:
            self._recovery_required = False
        return True, str(getattr(resp, 'message', 'ok'))

    def _apply(self, reaction, request_id: str, target_id: str = '') -> None:
        self._batch_state = reaction.batch_state
        self._target_phase = reaction.target_phase
        if reaction.operation_mode != MODE_AUTO:
            self._operation_mode = reaction.operation_mode
        self._publish_state()
        if reaction.event_code:
            self._emit(reaction.event_code, request_id, target_id)

    def _run_harvest(self, goal_handle):
        goal = goal_handle.request
        self._run_goal_handle = goal_handle
        self._run_started = time.monotonic()
        with self._lock:
            self._cancel = False
            self._skip_target = False
            self._run_id = goal.request_id or 'harvest'
            self._cycle_id = self._run_id
            self._outcomes = []
            self._scene_epoch = 0
            self._discovered = 0
            self._ledger_loaded = False
            self._in_flight = []
            self._batch_state = WAITING_READY
            self._operation_mode = MODE_AUTO
        try:
            return self._run_harvest_body(goal_handle, goal)
        finally:
            self._current_target_id = ''
            self._action_active = False
            self._run_goal_handle = None
            with self._lock:
                self._harvest_busy = False
            self._publish_state()

    def _run_harvest_body(self, goal_handle, goal):
        """批次命令循环；busy 旗标由 _run_harvest 的 finally 清掉."""
        result = RunHarvest.Result()
        reaction = react(self._batch_state, Event.RUN_REQUESTED)
        self._apply(reaction, goal.request_id)
        claimed = set()
        empty_limit = max(1, int(self.get_parameter('empty_survey_limit').value))
        empty_rounds = 0
        enabled = bool(self.get_parameter('execution_enabled').value)
        survey_only = int(goal.intent) == int(JobIntent.SURVEY_ONLY)
        survey_goal = SurveyScene.Goal()
        survey_goal.request_id = goal.request_id
        survey_goal.scene_key = goal.scene_key
        terminal = {Command.SETTLE, Command.ABORT, Command.INTERRUPT}
        while reaction.command not in terminal:
            if goal_handle.is_cancel_requested:
                self._cancel = True
            if self._cancel:
                reaction = react(self._batch_state, Event.CANCEL)
                self._apply(reaction, goal.request_id, self._current_target_id)
                self._cancel_inflight()
                break
            self._wait_pause()
            if self._cancel:
                continue
            cmd = reaction.command
            if cmd in (
                    Command.SELECT, Command.DISPATCH, Command.EXECUTE_FULL,
                    Command.NONE):
                self._wait_recovery()
                if self._cancel:
                    continue
            if cmd == Command.BEGIN_SCENE:
                reaction = self._cmd_begin(goal)
            elif cmd == Command.SURVEY:
                self._survey_body(survey_goal)
                if not self._ledger_loaded:
                    claimed, restored = self._restore_ledger(self._run_id)
                    if restored:
                        self._outcomes = restored
                    self._ledger_loaded = True
                if survey_only:
                    reaction = react(self._batch_state, Event.SURVEY_ONLY)
                elif not enabled:
                    reaction = react(
                        self._batch_state, Event.EXECUTION_DISABLED)
                else:
                    reaction = react(self._batch_state, Event.SURVEY_DONE)
                self._apply(reaction, goal.request_id)
            elif cmd == Command.SELECT:
                target_id = next_target_id(
                    self._observations, claimed, goal.target_ids)
                if not target_id:
                    empty_rounds += 1
                    event = (
                        Event.EMPTY_LIMIT if empty_rounds >= empty_limit
                        else Event.NO_TARGET)
                    reaction = react(self._batch_state, event)
                    self._apply(reaction, goal.request_id)
                    continue
                empty_rounds = 0
                claimed.add(target_id)
                self._current_target_id = target_id
                self._cycle_id = f'{self._run_id}:{target_id}'
                reaction = react(self._batch_state, Event.TARGET_SELECTED)
                self._apply(reaction, goal.request_id, target_id)
            elif cmd == Command.DISPATCH:
                reaction = self._cmd_dispatch(goal.request_id)
            elif cmd == Command.EXECUTE_FULL:
                reaction = self._cmd_full(goal.request_id)
            elif cmd == Command.RECORD_DISABLED:
                break
            elif cmd == Command.NONE:
                reaction = react(self._batch_state, Event.CYCLE_DONE)
                self._current_target_id = ''
                self._persist_ledger(claimed)
                self._apply(reaction, goal.request_id)
            else:
                break
            self._persist_ledger(claimed)
        aborted = reaction.command == Command.ABORT
        interrupted = (
            reaction.command == Command.INTERRUPT or self._cancel)
        result.success = not aborted and not interrupted
        if aborted:
            result.termination_reason = 'begin_scene_failed'
        elif interrupted:
            result.termination_reason = 'canceled'
        else:
            result.termination_reason = 'completed'
            self._batch_state = COMPLETED
        result.summary = build_summary(
            self._run_id, self._outcomes, self._discovered,
            time.monotonic() - self._run_started)
        if interrupted:
            goal_handle.canceled()
        elif aborted:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        self._publish_state()
        self._persist_ledger(claimed)
        return result

    def _cmd_begin(self, goal):
        begin = BeginScene.Request()
        begin.request_id = goal.request_id
        begin.scene_key = goal.scene_key
        resp = self._call_service(self._begin, begin)
        if resp is None or not bool(getattr(resp, 'accepted', False)):
            reaction = react(self._batch_state, Event.BEGIN_FAILED)
            self._apply(reaction, goal.request_id)
            return reaction
        self._scene_epoch = int(getattr(resp, 'scene_epoch', 0) or 0)
        reaction = react(self._batch_state, Event.BEGIN_OK)
        self._apply(reaction, goal.request_id)
        return reaction

    def _cmd_dispatch(self, request_id: str):
        timeout = float(self.get_parameter('action_timeout_s').value)
        target_id = self._current_target_id
        if self._take_skip():
            reaction = react(self._batch_state, Event.SKIP)
            self._record_skip(target_id, TargetOutcome.CANCELED, 'skip_target')
            self._apply(reaction, request_id, target_id)
            return reaction
        build_goal = BuildTargetModel.Goal()
        build_goal.request_id = request_id
        build_goal.target_id = target_id
        build_goal.scene_epoch = self._scene_epoch
        build_handle = self._send_goal(self._build, build_goal, timeout)
        if build_handle is None:
            reaction = react(self._batch_state, Event.BUILD_FAILED)
            self._record_skip(
                target_id, TargetOutcome.SKIPPED_QUALITY,
                'build_target_model rejected')
            self._apply(reaction, request_id, target_id)
            return reaction
        self._in_flight.append(build_handle)
        observe = ExecuteTarget.Goal()
        observe.request_id = request_id
        observe.target_id = target_id
        observe.mode = ExecuteTarget.Goal.OBSERVE_ONLY
        observed = self._send_action(
            self._exec, observe, timeout, feedback=True)
        if self._cancel or self._peek_skip() or observed is None:
            self._take_skip()
            self._cancel_handle(build_handle)
            self._wait_result(build_handle, min(timeout, 10.0))
            if observed is None and not self._cancel:
                reaction = react(self._batch_state, Event.OBSERVE_FAILED)
                self._record_skip(
                    target_id, TargetOutcome.SKIPPED_QUALITY,
                    'observe_only failed')
            else:
                reaction = react(self._batch_state, Event.SKIP)
                self._record_skip(
                    target_id, TargetOutcome.CANCELED, 'canceled_or_skipped')
            self._apply(reaction, request_id, target_id)
            return reaction
        built = self._wait_result(build_handle, timeout)
        self._forget_handle(build_handle)
        if self._cancel or self._take_skip():
            reaction = react(self._batch_state, Event.SKIP)
            self._record_skip(
                target_id, TargetOutcome.CANCELED, 'canceled_or_skipped')
            self._apply(reaction, request_id, target_id)
            return reaction
        if built is None or not bool(getattr(built, 'success', False)):
            reaction = react(self._batch_state, Event.BUILD_FAILED)
            self._record_skip(
                target_id, TargetOutcome.SKIPPED_QUALITY,
                str(getattr(built, 'message', 'build_target_model failed')))
            self._apply(reaction, request_id, target_id)
            return reaction
        reaction = react(self._batch_state, Event.READY_FULL)
        self._apply(reaction, request_id, target_id)
        return reaction

    def _cmd_full(self, request_id: str):
        timeout = float(self.get_parameter('action_timeout_s').value)
        target_id = self._current_target_id
        full = ExecuteTarget.Goal()
        full.request_id = request_id
        full.target_id = target_id
        full.mode = ExecuteTarget.Goal.FULL
        full.skip_observation = True
        executed = self._send_action(self._exec, full, timeout, feedback=True)
        operator_skip = self._take_skip()
        outcome = TargetOutcome()
        outcome.target_id = target_id
        if executed is None:
            outcome.outcome = TargetOutcome.FAILED
            outcome.reason = (
                'skip_target' if operator_skip else 'execute_target failed')
        else:
            record = getattr(executed, 'outcome_record', None)
            if record is not None and getattr(record, 'target_id', ''):
                outcome = record
            else:
                outcome.outcome = int(getattr(
                    executed, 'outcome', TargetOutcome.FAILED))
                outcome.reason = str(getattr(executed, 'reason', ''))
            deposit = getattr(executed, 'deposit', None)
            if deposit is not None and not bool(
                    getattr(deposit, 'deposited', False)):
                extra = str(getattr(deposit, 'reason', ''))
                if extra and extra not in outcome.reason:
                    outcome.reason = (
                        outcome.reason + '; ' + extra).strip('; ')
            self._recovery_required = self._recovery_required or bool(
                getattr(executed, 'recovery_required', False))
        self._outcomes.append(outcome)
        event = event_for_outcome(outcome.outcome, operator_skip)
        reaction = react(self._batch_state, event)
        self._apply(reaction, request_id, target_id)
        return reaction

    def _record_skip(self, target_id: str, code: int, reason: str) -> None:
        outcome = TargetOutcome()
        outcome.target_id = target_id
        outcome.outcome = code
        outcome.reason = reason
        self._outcomes.append(outcome)

    def _survey_body(self, goal):
        timeout = float(self.get_parameter('action_timeout_s').value)
        self._action_active = True
        self._publish_state()
        while not self._cancel:
            self._send_action(
                self._survey, goal, timeout, interrupt_on_pause=True)
            if self._paused and not self._cancel:
                self._wait_pause()
                continue
            break
        wait_s = float(self.get_parameter('survey_wait_s').value)
        deadline = time.monotonic() + max(wait_s, 0.0)
        while time.monotonic() < deadline and not self._cancel:
            self._wait_pause()
            obs = self._observations
            if obs is not None and obs.target_set_locked:
                break
            time.sleep(0.1)
        snap = SceneSnapshot()
        snap.scene_epoch = self._scene_epoch
        snap.scene_key = goal.scene_key
        snap.message = 'survey'
        if self._observations is not None:
            snap.snapshot_id = str(self._observations.snapshot_id)
            snap.degraded = not self._observations.target_set_locked
            snap.observation_count = len(self._observations.observations)
            snap.target_ids = [
                item.target_id for item in self._observations.observations
                if item.target_id]
            self._discovered = max(self._discovered, snap.observation_count)
        else:
            snap.degraded = True
        if hasattr(self, '_pub_snapshot'):
            self._pub_snapshot.publish(snap)
        self._action_active = False

    def _send_goal(self, client, goal_msg, timeout_s: float,
                   feedback: bool = False):
        if not client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().warning('action server not ready')
            return None
        kwargs = {}
        if feedback:
            kwargs['feedback_callback'] = self._on_exec_feedback
        send_fut = client.send_goal_async(goal_msg, **kwargs)
        handle = self._await_future(send_fut, timeout_s)
        if handle is None or not handle.accepted:
            self.get_logger().warning('action goal rejected')
            return None
        self._action_active = True
        return handle

    def _on_exec_feedback(self, feedback_msg) -> None:
        fb = getattr(feedback_msg, 'feedback', feedback_msg)
        state = getattr(fb, 'state', None)
        if state is None:
            return
        phase = int(getattr(state, 'target_phase', self._target_phase) or 0)
        if phase:
            self._target_phase = phase
        cycle = str(getattr(state, 'cycle_id', '') or '')
        if cycle:
            self._cycle_id = cycle
        if bool(getattr(state, 'recovery_required', False)):
            self._recovery_required = True
        self._grasp_enabled = bool(getattr(state, 'grasp_enabled', False))
        self._tool_enabled = bool(getattr(state, 'tool_enabled', False))
        self._action_active = True
        self._publish_state()

    def _wait_result(self, handle, timeout_s: float,
                     interrupt_on_pause: bool = False):
        if handle is None:
            return None
        result_fut = handle.get_result_async()
        deadline = time.monotonic() + max(timeout_s, 0.0)
        while not result_fut.done():
            if time.monotonic() >= deadline:
                self.get_logger().warning('action result timeout')
                self._cancel_handle(handle)
                return None
            if self._paused and self._batch_state not in (
                    PAUSED, PAUSE_PENDING):
                self._batch_state = PAUSE_PENDING
                self._publish_state()
            if interrupt_on_pause and self._paused:
                self._cancel_handle(handle)
            if self._cancel or self._peek_skip():
                self._cancel_handle(handle)
            time.sleep(0.05)
        self._action_active = False
        try:
            wrapped = result_fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'action result failed: {exc}')
            return None
        return getattr(wrapped, 'result', wrapped)

    def _cancel_handle(self, handle) -> None:
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'cancel goal failed: {exc}')

    def _cancel_inflight(self) -> None:
        for handle in list(self._in_flight):
            self._cancel_handle(handle)
        self._in_flight = []

    def _forget_handle(self, handle) -> None:
        self._in_flight = [item for item in self._in_flight if item is not handle]

    def _take_skip(self) -> bool:
        with self._lock:
            skip = self._skip_target
            self._skip_target = False
            return skip

    def _peek_skip(self) -> bool:
        with self._lock:
            return self._skip_target

    def _ledger_path(self):
        return ledger_file(default_ledger_root(), self._run_id)

    def _restore_ledger(self, run_id: str):
        if not bool(self.get_parameter('persist_ledger').value):
            return set(), []
        claimed, outcomes = load_ledger(
            ledger_file(default_ledger_root(), run_id))
        if claimed:
            self.get_logger().info(
                f'resume ledger {run_id}: {len(claimed)} claimed')
        return claimed, outcomes

    def _persist_ledger(self, claimed) -> None:
        if not bool(self.get_parameter('persist_ledger').value):
            return
        try:
            save_ledger(self._ledger_path(), claimed, self._outcomes)
        except OSError as exc:
            self.get_logger().warning(f'ledger write failed: {exc}')

    def _call_service(self, client, request):
        timeout = float(self.get_parameter('service_timeout_s').value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning('service not ready')
            return None
        fut = client.call_async(request)
        return self._await_future(fut, timeout)

    def _await_future(self, fut, timeout_s: float):
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if fut.done() or done.wait(timeout=max(timeout_s, 0.0)):
            try:
                return fut.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'future failed: {exc}')
                return None
        self.get_logger().warning('future timeout')
        return None

    def _send_action(self, client, goal_msg, timeout_s: float,
                     feedback: bool = False, interrupt_on_pause: bool = False):
        handle = self._send_goal(
            client, goal_msg, timeout_s, feedback=feedback)
        if handle is not None:
            self._in_flight.append(handle)
        result = self._wait_result(
            handle, timeout_s, interrupt_on_pause=interrupt_on_pause)
        self._forget_handle(handle)
        return result

    def _wait_pause(self) -> None:
        entered = False
        while True:
            with self._lock:
                paused = self._paused
                cancel = self._cancel
                maintenance = self._operation_mode == MODE_MAINTENANCE
            if cancel or not paused:
                if entered:
                    self._batch_state = self._paused_batch
                    if not maintenance:
                        self._operation_mode = MODE_AUTO
                    self._publish_state()
                return
            if not entered:
                entered = True
                self._paused_batch = self._batch_state
                self._batch_state = PAUSED
                self._operation_mode = (
                    MODE_MAINTENANCE if maintenance else MODE_PAUSED)
                self._publish_state()
            time.sleep(0.1)

    def _wait_recovery(self) -> None:
        """技能接触锁未 ACK 时不派下一颗；CANCEL 可打断."""
        entered = False
        while True:
            with self._lock:
                recovery = self._recovery_required
                cancel = self._cancel
            if cancel or not recovery:
                if entered:
                    self._batch_state = self._recovery_batch
                    self._publish_state()
                return
            if not entered:
                entered = True
                self._recovery_batch = self._batch_state
                self._batch_state = RECOVERY_REQUIRED
                self._publish_state()
            time.sleep(0.1)

    def _make_state(self) -> HarvestState:
        msg = HarvestState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        seq = self._state_seq
        msg.revision = seq
        msg.state_seq = seq
        msg.run_id = self._run_id
        msg.cycle_id = self._cycle_id
        msg.target_id = self._current_target_id
        msg.operation_mode = self._operation_mode
        msg.batch_state = self._batch_state
        msg.target_phase = self._target_phase
        msg.action_active = self._action_active
        msg.auto_start_enabled = False
        msg.execution_enabled = bool(
            self.get_parameter('execution_enabled').value)
        msg.grasp_enabled = self._grasp_enabled
        msg.tool_enabled = self._tool_enabled
        msg.recovery_required = self._recovery_required
        attempted = len(self._outcomes)
        total = max(self._discovered, attempted, 1)
        msg.progress = float(attempted) / float(total)
        msg.message = BATCH_NAMES.get(self._batch_state, '')
        msg.blockers = list(self._blockers)
        msg.permissions = permissions_for(
            self._batch_state, self._recovery_required)
        return msg

    def _publish_state(self, bump: bool = True):
        if bump:
            with self._lock:
                self._state_seq += 1
        if not hasattr(self, '_pub_state'):
            return self._make_state()
        state = self._make_state()
        self._pub_state.publish(state)
        handle = self._run_goal_handle
        is_active = getattr(handle, 'is_active', False) if handle else False
        if callable(is_active):
            is_active = is_active()
        if handle is not None and is_active:
            try:
                feedback = RunHarvest.Feedback()
                feedback.state = state
                handle.publish_feedback(feedback)
            except Exception:  # noqa: BLE001
                pass
        return state

    def _emit(self, code: str, request_id: str, target_id: str = '') -> None:
        if code in {
            'target_succeeded', 'target_skipped', 'target_failed',
            'target_canceled', 'target_operator_skipped',
        }:
            # Prefer canonical mapping if last outcome exists.
            if self._outcomes:
                code = canonical_code_for_outcome(
                    self._outcomes[-1].outcome,
                    code == 'target_operator_skipped')
        ev = CanonicalEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.code = code
        ev.request_id = request_id
        ev.run_id = self._run_id
        ev.target_id = target_id
        ev.state_seq = self._state_seq
        ev.message = json.dumps({'code': code}, ensure_ascii=False)
        if hasattr(self, '_pub_event'):
            self._pub_event.publish(ev)


def main(args=None):
    """生命周期执行器入口."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = PeachTaskExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
