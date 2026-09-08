"""
手动调试桥：Web 调试操作 → 既有 ROS 动作/服务（不做任何新控制路径）.

只做「转发 + 结果摘要」：目标全部是调度/感知/重建/技能已存在的入口，
技能侧 `ExecutionAuthority`、调度使能与重建门照常生效。运动类操作由
`is_motion` 分类、`motion_enabled` 参数放行（默认 false 一律 423）。

客户端在 on_configure 的 ROS 线程一次性建立（`open_all`）；HTTP 线程
只做 `call_async` + 带超时等待。动作结果等待上限 `action_timeout_s`，
期间可通过 `cancel` 语义取消在途目标（保留最近句柄）。
"""
from __future__ import annotations

from collections import deque
import threading
import time
from typing import Callable, Mapping

from geometry_msgs.msg import PoseStamped
from peach_interfaces.action import (
    BuildTargetModel,
    ExecuteTarget,
    RunHarvest,
    SurveyScene,
)
from peach_interfaces.msg import JobIntent
from peach_interfaces.srv import (
    BeginScene,
    CheckReachability,
    ControlTask,
    ManageLifecycleNodes,
)
from rclpy.action import ActionClient
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger

# JobIntent 常量名 → 值（与 msg 定义对齐；不在图上重复魔数）
_INTENT = {
    'PICK_ALL': JobIntent.PICK_ALL,
    'PICK_SELECTED': JobIntent.PICK_SELECTED,
    'SURVEY_ONLY': JobIntent.SURVEY_ONLY,
}
_CONTROL = {
    'PAUSE': ControlTask.Request.PAUSE,
    'RESUME': ControlTask.Request.RESUME,
    'ENTER_MAINTENANCE': ControlTask.Request.ENTER_MAINTENANCE,
    'EXIT_MAINTENANCE': ControlTask.Request.EXIT_MAINTENANCE,
    'CANCEL_NOW': ControlTask.Request.CANCEL_NOW,
    'SKIP_TARGET': ControlTask.Request.SKIP_TARGET,
    'ACKNOWLEDGE_RECOVERY': ControlTask.Request.ACKNOWLEDGE_RECOVERY,
}
_LIFECYCLE = {
    'STARTUP': ManageLifecycleNodes.Request.STARTUP,
    'PAUSE': ManageLifecycleNodes.Request.PAUSE,
    'RESUME': ManageLifecycleNodes.Request.RESUME,
    'RESET': ManageLifecycleNodes.Request.RESET,
    'SHUTDOWN': ManageLifecycleNodes.Request.SHUTDOWN,
}
_EXECUTE_MODE = {
    'PREVIEW': ExecuteTarget.Goal.PREVIEW,
    'OBSERVE_ONLY': ExecuteTarget.Goal.OBSERVE_ONLY,
    'FULL': ExecuteTarget.Goal.FULL,
    'PREGRASP_ONLY': ExecuteTarget.Goal.PREGRASP_ONLY,
}

# 服务端点 → (srv 类型, 是否 Trigger 形)
_SERVICE_TYPES = {
    'control_service': (ControlTask, False),
    'begin_scene_service': (BeginScene, False),
    'check_reachability_service': (CheckReachability, False),
    'photo_pose_service': (Trigger, True),
    'preview_approach_service': (Trigger, True),
    'preview_full_service': (Trigger, True),
    'ack_recovery_service': (Trigger, True),
    'arm_service': (SetBool, False),
    'skill_cancel_service': (Trigger, True),
    'recon_save_session_service': (Trigger, True),
    'recon_reset_service': (Trigger, True),
    'recon_finalize_service': (Trigger, True),
    'recon_query_service': (Trigger, True),
    'manage_nodes_service': (ManageLifecycleNodes, False),
}
# 动作端点 → 动作类型
_ACTION_TYPES = {
    'run_harvest_action': RunHarvest,
    'survey_action': SurveyScene,
    'execute_action': ExecuteTarget,
    'build_action': BuildTargetModel,
}

# payload 键白名单（防误传字段；缺省语义见各 IDL）
_STR_KEYS = ('request_id', 'run_id', 'cycle_id', 'target_id', 'scene_key',
             'model_revision', 'tool_profile_id', 'reason')


def is_motion(action: str, payload: dict) -> bool:
    """
    分类一次调试操作是否会引发臂/机位运动.

    规则（与各 IDL/阶段文档对齐）：
      - run_harvest：intent != SURVEY_ONLY（会 Survey/Execute）
      - survey / photo_pose / arm / skill 端 start_cycle：本身运动
      - execute：mode != PREVIEW（OBSERVE_ONLY 有 0.15 m 级观察短移）
      - control：RESUME / EXIT_MAINTENANCE（恢复自动流会继续派动作）；
        PAUSE / CANCEL_NOW / SKIP / ACK 是降低风险操作，不拦
      - 其余（重建 Trigger、check_reachability、begin_scene、lifecycle）
        不动臂

    Args:
        action: 端点键（ACTIONS 注册表键）.
        payload: 已解析的 JSON 请求体.

    Returns
    -------
        True = 属运动类，须 motion_enabled 放行.

    """
    if action in ('survey_action', 'photo_pose_service', 'arm_service'):
        return True
    if action == 'run_harvest_action':
        intent = str(payload.get('intent', 'PICK_ALL')).upper()
        return _INTENT.get(intent, JobIntent.PICK_ALL) != JobIntent.SURVEY_ONLY
    if action == 'execute_action':
        mode = str(payload.get('mode', 'PREVIEW')).upper()
        return _EXECUTE_MODE.get(
            mode, ExecuteTarget.Goal.PREVIEW) != ExecuteTarget.Goal.PREVIEW
    if action == 'control_service':
        command = _CONTROL.get(str(payload.get('command', '')).upper())
        return command in (
            ControlTask.Request.RESUME,
            ControlTask.Request.EXIT_MAINTENANCE)
    return False


def _validate_payload(action: str, payload: dict) -> str | None:
    """枚举字段合法性；非法返回错误文案，合法返回 None."""
    if action == 'execute_action':
        mode = str(payload.get('mode', 'PREVIEW')).upper()
        if mode not in _EXECUTE_MODE:
            return f'未知 ExecuteTarget mode: {mode}'
    if action == 'run_harvest_action':
        intent = str(payload.get('intent', 'PICK_ALL')).upper()
        if intent not in _INTENT:
            return f'未知 RunHarvest intent: {intent}'
    if action == 'control_service':
        command = str(payload.get('command', '')).upper()
        if command not in _CONTROL:
            return f'未知 ControlTask command: {command}'
    if action == 'manage_nodes_service':
        command = str(payload.get('command', '')).upper()
        if command not in _LIFECYCLE:
            return f'未知 ManageNodes command: {command}'
    return None


def _fill_str(request, payload: dict) -> None:
    """按白名单拷贝字符串字段（仅当请求侧真有该字段）."""
    for key in _STR_KEYS:
        if key in payload and hasattr(request, key):
            setattr(request, key, str(payload[key] or ''))


def _goal_for(action: str, payload: dict):
    """Payload → goal/request（显式映射，不反射魔法）."""
    if action == 'run_harvest_action':
        goal = RunHarvest.Goal()
        _fill_str(goal, payload)
        goal.intent = _INTENT.get(
            str(payload.get('intent', 'PICK_ALL')).upper(),
            JobIntent.PICK_ALL)
        goal.selection_mode = RunHarvest.Goal.MANUAL if payload.get(
            'target_ids') else RunHarvest.Goal.AUTO
        goal.target_ids = [str(t) for t in payload.get('target_ids') or []]
        return goal
    if action == 'survey_action':
        goal = SurveyScene.Goal()
        _fill_str(goal, payload)
        return goal
    if action == 'execute_action':
        goal = ExecuteTarget.Goal()
        _fill_str(goal, payload)
        goal.mode = _EXECUTE_MODE.get(
            str(payload.get('mode', 'PREVIEW')).upper(),
            ExecuteTarget.Goal.PREVIEW)
        goal.skip_observation = bool(payload.get('skip_observation', False))
        return goal
    if action == 'build_action':
        goal = BuildTargetModel.Goal()
        _fill_str(goal, payload)
        goal.scene_epoch = int(payload.get('scene_epoch', 0) or 0)
        return goal
    if action == 'control_service':
        request = ControlTask.Request()
        _fill_str(request, payload)
        request.command = _CONTROL.get(
            str(payload.get('command', '')).upper(), 255)
        request.expected_state_seq = int(
            payload.get('expected_state_seq', 0) or 0)
        return request
    if action == 'begin_scene_service':
        request = BeginScene.Request()
        _fill_str(request, payload)
        return request
    if action == 'check_reachability_service':
        request = CheckReachability.Request()
        request.timeout_s = float(payload.get('timeout_s', 0.0) or 0.0)
        for item in payload.get('tcp_poses') or []:
            pose = PoseStamped()
            position = item.get('position') or {}
            pose.pose.position.x = float(position.get('x', 0.0) or 0.0)
            pose.pose.position.y = float(position.get('y', 0.0) or 0.0)
            pose.pose.position.z = float(position.get('z', 0.0) or 0.0)
            orientation = item.get('orientation') or {}
            pose.pose.orientation.x = float(orientation.get('x', 0.0) or 0.0)
            pose.pose.orientation.y = float(orientation.get('y', 0.0) or 0.0)
            pose.pose.orientation.z = float(orientation.get('z', 0.0) or 0.0)
            pose.pose.orientation.w = float(orientation.get('w', 1.0) or 1.0)
            pose.header.frame_id = str(
                item.get('frame_id', 'base_link') or 'base_link')
            request.tcp_poses.append(pose)
        return request
    if action == 'arm_service':
        request = SetBool.Request()
        request.data = bool(payload.get('data', False))
        return request
    if action == 'manage_nodes_service':
        request = ManageLifecycleNodes.Request()
        request.command = _LIFECYCLE.get(
            str(payload.get('command', '')).upper(), 255)
        return request
    # 其余全部 Trigger 形（无字段）
    return Trigger.Request()


def _summarize_service(response) -> dict:
    """服务响应 → 精简 dict（字段存在才收录，防类型差异）."""
    out = {}
    for key in ('success', 'accepted', 'message', 'scene_epoch',
                'state_seq', 'reachable'):
        if hasattr(response, key):
            value = getattr(response, key)
            out[key] = list(value) if isinstance(value, list) else value
    return out


def _summarize_result(action: str, result) -> dict:
    """动作结果 → 精简 dict（各动作挑关键字段，全量走 /api/state 镜像）."""
    out: dict = {}
    for key in ('success', 'message', 'termination_reason', 'degraded',
                'snapshot_id', 'outcome', 'completion_level', 'failure_code',
                'reason', 'recovery_required', 'quality_level', 'view_count'):
        if hasattr(result, key):
            out[key] = getattr(result, key)
    if action == 'run_harvest_action' and hasattr(result, 'summary'):
        summary = result.summary
        # HarvestSummary 无 harvested 字段（成功口径是 succeeded）
        for key in ('attempted', 'succeeded', 'failed'):
            if hasattr(summary, key):
                out[f'summary_{key}'] = getattr(summary, key)
    return out


class DebugBridge:
    """调试操作桥：端点客户端注册表 + 带超时的调用/取消 + 最近结果环."""

    def __init__(self, node, endpoints: Mapping[str, str], timeout_s: float,
                 motion_enabled: bool, log_warning: Callable[[str], None]):
        """建桥；客户端等 open_all() 在 ROS 线程建立，端点表来自 GPL 快照."""
        self._node = node
        self._endpoints = dict(endpoints)
        self._timeout_s = float(timeout_s)
        self._motion_enabled = bool(motion_enabled)
        self._log_warning = log_warning
        self._clients: dict = {}
        self._action_clients: dict = {}
        self._goal_handles: dict = {}
        self._recent: deque = deque(maxlen=20)
        self._lock = threading.Lock()

    def open_all(self) -> None:
        """在 on_configure 的 ROS 线程建全部客户端（含 lazy 目标等待）."""
        for key, (srv_type, _trigger) in _SERVICE_TYPES.items():
            name = self._endpoints.get(key)
            if name:
                self._clients[key] = self._node.create_client(srv_type, name)
        for key, action_type in _ACTION_TYPES.items():
            name = self._endpoints.get(key)
            if name:
                self._action_clients[key] = ActionClient(
                    self._node, action_type, name)

    def close(self) -> None:
        """销毁全部客户端（cleanup 期，ROS 线程）."""
        for client in list(self._clients.values()):
            self._node.destroy_client(client)
        self._clients.clear()
        for client in list(self._action_clients.values()):
            client.destroy()
        self._action_clients.clear()
        with self._lock:
            self._goal_handles.clear()

    def state(self) -> dict:
        """/api/state 的 debug 段（不含令牌本身）."""
        with self._lock:
            recent = list(self._recent)
        return {
            'motion_enabled': self._motion_enabled,
            'recent': recent,
        }

    def command(self, action: str, payload: dict) -> tuple:
        """
        执行一次调试操作.

        Args:
            action: 端点键.
            payload: 已解析 JSON 请求体.

        Returns
        -------
            (http_status, 响应 dict)；响应体 ``accepted`` = 操作是否被受理
            （HTTP 层），目标端点的原始响应字段全部收在 ``result`` 子字典
            （避免与服务响应自身的 accepted 字段撞键）；结果同时进最近
            结果环.

        """
        accepted, summary = self._dispatch(action, payload)
        status = int(summary.pop('_status', 200 if accepted else 502))
        with self._lock:
            self._recent.appendleft({
                'ts': time.time(), 'action': action,
                'accepted': accepted, 'status': status, 'result': summary})
        return status, {'accepted': accepted, 'result': summary}

    def cancel(self, target: str) -> tuple:
        """
        取消在途调试动作目标（execute/survey/build/run_harvest）.

        Args:
            target: 动作端点键.

        Returns
        -------
            (http_status, 响应 dict).

        """
        with self._lock:
            handle = self._goal_handles.get(target)
        if handle is None:
            return 404, {'accepted': False, 'message': '无在途目标'}
        future = handle.cancel_goal_async()
        self._wait(future, 10.0)
        done = future.done() and future.result() is not None
        return (200, {'accepted': done, 'message': '已请求取消'}) if done \
            else (504, {'accepted': False, 'message': '取消请求超时'})

    # ------------------------------------------------------------------
    def _dispatch(self, action: str, payload: dict) -> tuple:
        """路由到动作或服务；返回 (accepted, summary[_status]=HTTP 码)."""
        if action == 'cancel':
            status, body = self.cancel(str(payload.get('target', '')))
            summary = dict(body)
            summary['_status'] = status
            return bool(body.get('accepted')), summary
        error = _validate_payload(action, payload)
        if error:
            return False, {'_status': 400, 'message': error}
        if action in _ACTION_TYPES:
            return self._call_action(action, payload)
        if action in _SERVICE_TYPES:
            return self._call_service(action, payload)
        return False, {'_status': 404, 'message': f'未知调试端点: {action}'}

    def _call_service(self, key: str, payload: dict) -> tuple:
        client = self._clients.get(key)
        if client is None:
            return False, {'_status': 404, 'message': '端点未配置'}
        if not client.wait_for_service(timeout_sec=2.0):
            return False, {'_status': 503, 'message': '目标服务不可用'}
        try:
            request = _goal_for(key, payload)
        except (TypeError, ValueError, AttributeError, OverflowError) as exc:
            return False, {'_status': 400, 'message': f'请求体无法映射: {exc}'}
        future = client.call_async(request)
        if not self._wait(future, 10.0):
            return False, {'_status': 504, 'message': '服务响应超时'}
        response = future.result()
        if response is None:
            return False, {'_status': 502, 'message': '服务调用失败'}
        return True, _summarize_service(response)

    def _call_action(self, key: str, payload: dict) -> tuple:
        client = self._action_clients.get(key)
        if client is None:
            return False, {'_status': 404, 'message': '端点未配置'}
        if not client.wait_for_server(timeout_sec=2.0):
            return False, {'_status': 503, 'message': '目标动作服务端不可用'}
        try:
            goal = _goal_for(key, payload)
        except (TypeError, ValueError, AttributeError, OverflowError) as exc:
            return False, {'_status': 400, 'message': f'请求体无法映射: {exc}'}
        goal_future = client.send_goal_async(goal)
        if not self._wait(goal_future, 10.0):
            return False, {'_status': 504, 'message': '目标受理超时'}
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            return False, {'_status': 409, 'message': '目标被服务端拒绝'}
        with self._lock:
            self._goal_handles[key] = handle
        result_future = handle.get_result_async()
        if not self._wait(result_future, self._timeout_s):
            return False, {
                '_status': 202,
                'message': '仍在执行（结果等待超时，可稍后查询状态或取消）',
                'goal_handle_kept': True}
        wrapped = result_future.result()
        if wrapped is None:
            return False, {'_status': 502, 'message': '动作结果不可得'}
        status = int(getattr(wrapped, 'status', 0) or 0)
        result = wrapped.result
        summary = _summarize_result(key, result)
        summary['action_status'] = status
        with self._lock:
            self._goal_handles.pop(key, None)
        return True, summary

    @staticmethod
    def _wait(future: Future, timeout_s: float) -> bool:
        """HTTP 线程内的有界等待（ROS 侧由 MultiThreadedExecutor 驱动）."""
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True
