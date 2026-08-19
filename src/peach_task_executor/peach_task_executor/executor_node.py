"""
采摘任务执行器：显式 RunHarvest，launch 从不自动开批.

Lifecycle Active 后才接受动作。选择与账本在本节点；感知不再提供
reset/complete/clear/reopen。
"""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

from peach_interfaces.action import ExecuteTarget, RunHarvest, SurveyScene
from peach_interfaces.msg import (
    CanonicalEvent,
    HarvestState,
    HarvestSummary,
    PeachTargetObservationArray,
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

from .control import apply_control
from .select import next_target_id


class PeachTaskExecutor(LifecycleNode):
    """批次唯一所有者：RunHarvest / ControlTask；SurveyScene 在技能节点."""

    def __init__(self):
        super().__init__('peach_task_executor')
        self._lock = threading.Lock()
        self._state_seq = 0
        self._paused = False
        self._cancel = False
        self._active = False
        self._run_id = ''
        self._current_target_id = ''
        self._scene_epoch = 0
        self._outcomes = []
        self._observations: Optional[PeachTargetObservationArray] = None
        self._cb = ReentrantCallbackGroup()
        self.declare_parameter('execution_enabled', False)
        self.declare_parameter(
            'begin_scene_service', '/peach_scene_perception_node/begin_scene')
        self.declare_parameter(
            'survey_scene_action',
            '/peach_manipulation_skills_node/survey_scene')
        self.declare_parameter(
            'execute_target_action',
            '/peach_manipulation_skills_node/execute_target')
        self.declare_parameter('survey_wait_s', 8.0)
        self.declare_parameter('service_timeout_s', 30.0)
        self.declare_parameter('action_timeout_s', 180.0)

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
        ActionServer(
            self, RunHarvest, '~/run_harvest',
            execute_callback=self._run_harvest,
            goal_callback=self._goal_if_active,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb)
        self.create_service(
            ControlTask, '~/control', self._on_control,
            callback_group=self._cb)
        self.get_logger().info(
            'task executor configured; will not auto-start harvest')
        return super().on_configure(state)

    def on_activate(self, state):
        result = super().on_activate(state)
        self._active = True
        self._publish_state('idle')
        return result

    def on_deactivate(self, state):
        self._active = False
        return super().on_deactivate(state)

    def _goal_if_active(self, goal_request):
        del goal_request
        if not self._active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_cancel(self, cancel_request):
        del cancel_request
        return CancelResponse.ACCEPT

    def _on_obs(self, msg):
        self._observations = msg

    def _on_control(self, request, response):
        with self._lock:
            ok, seq, paused, cancel = apply_control(
                self._state_seq, int(request.expected_state_seq),
                int(request.command), self._paused)
            if ok:
                self._state_seq = seq
                self._paused = paused
                if cancel:
                    self._cancel = True
            response.accepted = ok
            response.state_seq = self._state_seq
            response.message = (
                'ok' if ok else 'expected_state_seq mismatch')
            response.state = self._make_state(
                'paused' if self._paused else 'running')
        return response

    def _run_harvest(self, goal_handle):
        goal = goal_handle.request
        result = RunHarvest.Result()
        with self._lock:
            self._cancel = False
            self._run_id = goal.request_id or 'harvest'
            self._outcomes = []
            self._scene_epoch = 0
        self._emit('harvest_start', goal.request_id)
        self._publish_state('preparing')
        begin = BeginScene.Request()
        begin.request_id = goal.request_id
        begin.scene_key = goal.scene_key
        resp = self._call_service(self._begin, begin)
        if resp is None or not bool(getattr(resp, 'accepted', False)):
            result.success = False
            result.termination_reason = 'begin_scene_failed'
            self._emit('harvest_failed', goal.request_id)
            self._publish_state('failed')
            goal_handle.abort()
            return result
        self._scene_epoch = int(getattr(resp, 'scene_epoch', 0) or 0)
        survey_goal = SurveyScene.Goal()
        survey_goal.request_id = goal.request_id
        survey_goal.scene_key = goal.scene_key
        self._survey_body(survey_goal)
        claimed = set()
        enabled = bool(self.get_parameter('execution_enabled').value)
        while not self._cancel:
            if goal_handle.is_cancel_requested:
                self._cancel = True
                break
            self._wait_pause()
            if self._cancel:
                break
            target_id = next_target_id(
                self._observations, claimed, goal.target_ids)
            if not target_id:
                break
            claimed.add(target_id)
            if enabled:
                self._current_target_id = target_id
                self._publish_state('running')
                self._execute_one(goal.request_id, target_id)
                self._current_target_id = ''
            else:
                outcome = TargetOutcome()
                outcome.target_id = target_id
                outcome.outcome = TargetOutcome.SKIPPED_QUALITY
                outcome.reason = 'execution_enabled=false'
                self._outcomes.append(outcome)
                break
        result.success = not self._cancel
        result.termination_reason = (
            'canceled' if self._cancel else 'completed')
        summary = HarvestSummary()
        summary.run_id = self._run_id
        summary.outcomes = list(self._outcomes)
        result.summary = summary
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        self._current_target_id = ''
        self._publish_state(
            'interrupted' if self._cancel else 'completed')
        return result

    def _survey_body(self, goal):
        result = SurveyScene.Result()
        self._publish_state('discovery')
        timeout = float(self.get_parameter('action_timeout_s').value)
        surveyed = self._send_action(self._survey, goal, timeout)
        if surveyed is not None:
            result = surveyed
        wait_s = float(self.get_parameter('survey_wait_s').value)
        deadline = time.monotonic() + max(wait_s, 0.0)
        while time.monotonic() < deadline and not self._cancel:
            self._wait_pause()
            obs = self._observations
            if obs is not None and obs.target_set_locked:
                break
            time.sleep(0.1)
        if self._observations is not None:
            result.snapshot_id = str(self._observations.snapshot_id)
            result.degraded = not self._observations.target_set_locked
        else:
            result.degraded = True
        result.scene_epoch = self._scene_epoch
        self._emit('survey', goal.request_id)
        return result

    def _execute_one(self, request_id: str, target_id: str) -> None:
        timeout = float(self.get_parameter('action_timeout_s').value)
        outcome = TargetOutcome()
        outcome.target_id = target_id
        eg = ExecuteTarget.Goal()
        eg.request_id = request_id
        eg.target_id = target_id
        eg.mode = ExecuteTarget.Goal.FULL
        executed = self._send_action(self._exec, eg, timeout)
        if executed is None:
            outcome.outcome = TargetOutcome.FAILED
            outcome.reason = 'execute_target failed'
        else:
            record = getattr(executed, 'outcome_record', None)
            if record is not None and getattr(record, 'target_id', ''):
                outcome = record
            else:
                outcome.outcome = int(getattr(
                    executed, 'outcome', TargetOutcome.FAILED))
                outcome.reason = str(getattr(executed, 'reason', ''))
        self._outcomes.append(outcome)
        self._emit('execute', request_id, target_id)

    def _call_service(self, client, request):
        timeout = float(self.get_parameter('service_timeout_s').value)
        if not client.wait_for_server(timeout_sec=timeout):
            self.get_logger().warning('service not ready')
            return None
        fut = client.call_async(request)
        try:
            return fut.result(timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'service call failed: {exc}')
            return None

    def _send_action(self, client, goal_msg, timeout_s: float):
        if not client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().warning('action server not ready')
            return None
        send_fut = client.send_goal_async(goal_msg)
        try:
            handle = send_fut.result(timeout=timeout_s)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'send_goal failed: {exc}')
            return None
        if handle is None or not handle.accepted:
            self.get_logger().warning('action goal rejected')
            return None
        result_fut = handle.get_result_async()
        try:
            wrapped = result_fut.result(timeout=timeout_s)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'action result failed: {exc}')
            return None
        return getattr(wrapped, 'result', wrapped)

    def _wait_pause(self) -> None:
        while True:
            with self._lock:
                paused = self._paused
                cancel = self._cancel
            if cancel or not paused:
                return
            time.sleep(0.1)

    def _make_state(self, message: str) -> HarvestState:
        msg = HarvestState()
        msg.revision = self._state_seq
        msg.state_seq = self._state_seq
        msg.run_id = self._run_id
        msg.target_id = self._current_target_id
        msg.auto_start_enabled = False
        msg.execution_enabled = bool(
            self.get_parameter('execution_enabled').value)
        msg.message = message
        return msg

    def _publish_state(self, message: str) -> None:
        if hasattr(self, '_pub_state'):
            self._pub_state.publish(self._make_state(message))

    def _emit(self, code: str, request_id: str, target_id: str = '') -> None:
        ev = CanonicalEvent()
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
