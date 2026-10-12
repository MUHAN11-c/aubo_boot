#!/usr/bin/env python3
"""Drive one frozen-camera FULL dry grasp on an already-running v1 stack.

Waits for lifecycle + RGB-D replay + JTC, enables execution+grasp (tool stays
false: mock has no aubo_io_controller), sends RunHarvest PICK_ALL, and
cancels the action after the first terminal target so the batch is one
photo-pose view then one grasp.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from peach_interfaces.action import RunHarvest
from peach_interfaces.msg import HarvestState
from peach_interfaces.srv import ControlTask, SetEnables
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)
CONTROLLERS = ('joint_state_broadcaster', 'joint_trajectory_controller')
BATCH_DONE = (
    HarvestState.COMPLETED,
    HarvestState.INTERRUPTED,
    HarvestState.RECOVERY_REQUIRED,
)


def _wait(predicate, timeout_s: float, period_s: float = 0.2) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


class Driver(Node):
    """Operator stand-in: parameters, SetEnables, RunHarvest, cancel/ACK."""

    def __init__(self) -> None:
        super().__init__('drive_frozen_grasp')
        self._group = ReentrantCallbackGroup()
        self.activated = False
        self.images = 0
        self.joints: JointState | None = None
        self.state: HarvestState | None = None
        self.replay_paused = False
        self._canceled = False
        self.create_subscription(
            Bool, '/peach/lifecycle/managed_nodes_activated',
            self._on_flag, LATCHED, callback_group=self._group)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, RELIABLE,
            callback_group=self._group)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, RELIABLE,
            callback_group=self._group)
        self.create_subscription(
            HarvestState, '/peach_supervisor/state', self._on_state, LATCHED,
            callback_group=self._group)
        self._enables = self.create_client(
            SetEnables, '/peach_supervisor/set_enables',
            callback_group=self._group)
        self._control = self.create_client(
            ControlTask, '/peach_supervisor/control',
            callback_group=self._group)
        self._params = self.create_client(
            SetParameters, '/peach_supervisor/set_parameters',
            callback_group=self._group)
        self._arm_params = self.create_client(
            SetParameters, '/peach_arm/set_parameters',
            callback_group=self._group)
        self._obstacle_params = self.create_client(
            SetParameters, '/peach_scene_obstacles/set_parameters',
            callback_group=self._group)
        self._list = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._group)
        self._run = ActionClient(
            self, RunHarvest, '/peach_supervisor/run_harvest',
            callback_group=self._group)
        self._replay_pause = self.create_client(
            SetBool, '/frozen_camera_replay/pause', callback_group=self._group)

    def _on_flag(self, msg: Bool) -> None:
        self.activated = bool(msg.data)

    def _on_image(self, _msg: Image) -> None:
        self.images += 1

    def _on_joints(self, msg: JointState) -> None:
        self.joints = msg

    def _on_state(self, msg: HarvestState) -> None:
        self.state = msg

    def wait_ready(self, timeout_s: float) -> bool:
        ok = _wait(lambda: self.activated and self.images > 0, timeout_s)
        print(
            f'ready activated={self.activated} images={self.images}',
            flush=True)
        return ok

    def wait_controllers(self, timeout_s: float) -> bool:
        def active() -> bool:
            response = self._call(self._list, ListControllers.Request(), 2.0)
            if response is None:
                return False
            names = {c.name for c in response.controller if c.state == 'active'}
            return set(CONTROLLERS) <= names

        ok = _wait(active, timeout_s, 0.5)
        print(f'controllers ready={ok}', flush=True)
        return ok

    def _call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        return future.result() if future.done() else None

    def set_bool(self, name: str, value: bool, client=None) -> bool:
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=value)
        req.parameters = [param]
        target = client if client is not None else self._params
        response = self._call(target, req)
        ok = response is not None and all(r.successful for r in response.results)
        print(f'param {name}={value}: {"OK" if ok else "FAIL"}', flush=True)
        return ok

    def set_int(self, name: str, value: int) -> bool:
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=int(value))
        req.parameters = [param]
        response = self._call(self._params, req)
        ok = response is not None and all(r.successful for r in response.results)
        print(f'param {name}={value}: {"OK" if ok else "FAIL"}', flush=True)
        return ok

    def set_double(self, name: str, value: float, client=None) -> bool:
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [param]
        target = client if client is not None else self._params
        response = self._call(target, req)
        ok = response is not None and all(r.successful for r in response.results)
        print(f'param {name}={value}: {"OK" if ok else "FAIL"}', flush=True)
        return ok

    def set_enables(self, grasp: bool) -> bool:
        req = SetEnables.Request()
        req.execution = True
        req.grasp = grasp
        req.tool = False
        req.reason = 'peach_system_tests frozen_camera one-view FULL dry'
        response = self._call(self._enables, req)
        ok = response is not None
        print(
            f'enables(execution=True grasp={grasp} tool=False): '
            f'{"OK" if ok else "FAIL"}',
            flush=True)
        return ok

    def cancel_now(self, handle, reason: str) -> None:
        if self._canceled:
            return
        self._canceled = True
        if handle is not None:
            handle.cancel_goal_async()
        state = self.state
        req = ControlTask.Request()
        req.request_id = '' if state is None else state.run_id
        req.expected_state_seq = 0
        req.command = ControlTask.Request.CANCEL_NOW
        req.reason = reason
        response = self._call(self._control, req, 5.0)
        print(
            f'cancel {reason!r} CANCEL_NOW accepted='
            f'{getattr(response, "accepted", None)} '
            f'{getattr(response, "message", "")}',
            flush=True)

    def maybe_stop_after_first(self, handle) -> None:
        state = self.state
        if state is None or self._canceled:
            return
        # ExecuteTarget feedback can latch recovery_required at pregrasp
        # while FULL is still planning sleeve. ACK/cancel then aborts the cut.
        if state.action_active:
            return
        if state.recovery_required:
            allowed = list(state.permissions)
            if ControlTask.Request.ACKNOWLEDGE_RECOVERY in allowed:
                req = ControlTask.Request()
                req.request_id = state.run_id
                req.expected_state_seq = int(state.state_seq)
                req.command = ControlTask.Request.ACKNOWLEDGE_RECOVERY
                req.reason = 'peach_system_tests frozen_camera ACK'
                response = self._call(self._control, req, 5.0)
                print(
                    f'ACK seq={state.state_seq} accepted='
                    f'{getattr(response, "accepted", None)} '
                    f'{getattr(response, "message", "")}',
                    flush=True)
            self.cancel_now(handle, 'frozen_camera one target (recovery)')
            return
        if state.target_phase in (
                HarvestState.TARGET_SUCCEEDED, HarvestState.TARGET_FAILED
        ) and state.target_id:
            self.cancel_now(handle, 'frozen_camera one target')
            return
        if state.batch_state in BATCH_DONE:
            return

    def watch_motion_freeze(self, tol_rad: float = 2e-3) -> None:
        """Pause RGB-D and the point cloud once the arm leaves the photo pose.

        The frozen frame is a wrist camera at global_photo_pose. Republishing
        it with stamp=now() while the joints move would drag the fruit along
        the arm. Observation stays live; the first motion at APPROACHING or
        later stops the publishers.
        """
        def watch() -> None:
            baseline = None
            deadline = time.monotonic() + 180.0
            while time.monotonic() < deadline:
                js = self.joints
                if baseline is None and js is not None and js.position:
                    baseline = list(js.position)
                state = self.state
                if (baseline is not None and state is not None and js is not None
                        and js.position
                        and state.target_phase >= HarvestState.APPROACHING):
                    dev = max(
                        (abs(a - b) for a, b in zip(js.position, baseline)),
                        default=0.0)
                    if dev > tol_rad:
                        self._pause_replay(True)
                        print(
                            f'photo: replay paused on harvest motion '
                            f'(joint dev {dev:.4f} rad)',
                            flush=True)
                        # #region agent log
                        try:
                            import json
                            _payload = {
                                'sessionId': 'd5915a', 'runId': 'compare',
                                'hypothesisId': 'H4',
                                'location': 'v1/drive_frozen_grasp.py:pause',
                                'message': 'v1 replay paused',
                                'timestamp': int(time.time() * 1000),
                                'data': {
                                    'joint_dev_rad': round(dev, 4),
                                    'phase': int(state.target_phase),
                                    'joints': [round(float(x), 4) for x in js.position],
                                },
                            }
                            with open(
                                    '/home/mu/Desktop/aubo_e5_jazzy_ws/.cursor/'
                                    'debug-d5915a.log', 'a') as _fh:
                                _fh.write(json.dumps(_payload) + '\n')
                        except Exception:
                            pass
                        # #endregion
                        return
                time.sleep(0.05)
            print('photo: motion watch expired (replay left streaming)',
                  file=sys.stderr, flush=True)

        threading.Thread(target=watch, daemon=True).start()

    def _pause_replay(self, paused: bool) -> None:
        if not self._replay_pause.wait_for_service(timeout_sec=2.0):
            print('replay pause service unavailable', file=sys.stderr, flush=True)
            return
        req = SetBool.Request()
        req.data = paused
        response = self._call(self._replay_pause, req, 5.0)
        ok = response is not None and bool(response.success)
        if ok and paused:
            self.replay_paused = True
        print(
            f'replay pause={paused} ok={ok} '
            f'{getattr(response, "message", "")}',
            flush=True)

    def log_state(self) -> None:
        state = self.state
        if state is None:
            return
        print(
            f'state batch={state.batch_state} phase={state.target_phase} '
            f'target={state.target_id!r} recovery={state.recovery_required}',
            flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-id', default='',
                        help='batch id (default frozen_v1_<unix>)')
    parser.add_argument('--scene-key', default='lab')
    parser.add_argument('--profile', default='default')
    parser.add_argument('--ready-timeout', type=float, default=120.0,
                        help='stack boot budget; not part of the grasp clock')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='seconds from goal acceptance until the grasp must finish')
    args = parser.parse_args()
    rid = args.request_id.strip() or f'frozen_v1_{int(time.time())}'

    rclpy.init()
    node = Driver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not node.wait_ready(args.ready_timeout):
            print('stack not ready (lifecycle or frozen camera)', file=sys.stderr)
            return 2
        if not node.wait_controllers(12.0):
            print('controllers not active', file=sys.stderr)
            return 2
        time.sleep(5.0)
        if not node.set_bool('execute_pregrasp_only', False):
            return 1
        node.set_bool('reconstruct_in_trajectory', True)
        node.set_int('reconstruction_min_views', 1)
        # Frozen RGB-D stays at the photo pose; live TF after leaving it
        # would look like the fruit drifted. Skip ReconfirmTarget.
        if not node.set_bool(
                'grasp.allow_stale_anchor', True, client=node._arm_params):
            return 1
        # Frozen photo-pose cloud becomes 769 planning boxes; sleeve LIN
        # then fails camera×obstacle. Product switch: idle-only, before harvest.
        if not node.set_bool(
                'moveit.obstacle_guard_enabled', False,
                client=node._arm_params):
            return 1
        # One-view bag reconstructs ~0.10 m bag at ~1.04 m TCP; +0.10 m along
        # the neck axis leaves the E5 workspace (CartesianPath 0/1). Product
        # clamp stays 0.02–0.20; dry FULL still sleeves, just a short hop.
        if not node.set_double(
                'grasp.maximum_travel_m', 0.02, client=node._arm_params):
            return 1
        # Frozen photo-pose cloud is not a job-time map. Shrink the snapshot
        # so 769 boxes cannot occupy the sleeve corridor.
        node.set_double(
            'workspace_radius_m', 0.05, client=node._obstacle_params)
        if not node.set_enables(True):
            return 1
        node.watch_motion_freeze()
        if not node._run.wait_for_server(timeout_sec=10.0):
            print('run_harvest ACTION_UNAVAILABLE', file=sys.stderr)
            return 1
        goal = RunHarvest.Goal()
        goal.request_id = rid
        goal.scene_key = args.scene_key
        goal.profile_id = args.profile
        goal.intent = RunHarvest.Goal.INTENT_PICK_ALL
        print(
            f'goal: request_id={rid} intent=PICK_ALL '
            f'(one-view overlay, cancel after first target)',
            flush=True)
        send = node._run.send_goal_async(goal)
        if not _wait(send.done, 15.0, 0.05):
            print('goal send TIMEOUT', file=sys.stderr)
            return 1
        handle = send.result()
        if handle is None or not handle.accepted:
            print('goal REJECTED', file=sys.stderr)
            return 1
        accepted_at = time.monotonic()
        result_future = handle.get_result_async()
        deadline = accepted_at + args.timeout
        last_log = 0.0
        while time.monotonic() < deadline and not result_future.done():
            node.maybe_stop_after_first(handle)
            now = time.monotonic()
            if now - last_log >= 5.0:
                node.log_state()
                last_log = now
            time.sleep(0.3)
        if not result_future.done():
            print(f'goal TIMEOUT(>={args.timeout}s) — inspect', file=sys.stderr)
            node.cancel_now(handle, 'frozen_camera wall timeout')
            _wait(result_future.done, 8.0, 0.1)
            return 2
        wrapper = result_future.result()
        status_names = {
            GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
            GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
            GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
        }
        status = status_names.get(wrapper.status, str(wrapper.status))
        result = wrapper.result
        summary = result.summary
        elapsed = time.monotonic() - accepted_at
        summary_elapsed = summary.elapsed.sec + summary.elapsed.nanosec * 1e-9
        print(
            f'RESULT status={status} success={result.success} '
            f'termination={result.termination_reason} '
            f'discovered={summary.discovered} attempted={summary.attempted} '
            f'succeeded={summary.succeeded} failed={summary.failed} '
            f'grasp_s={elapsed:.1f} summary_elapsed={summary_elapsed:.1f}s '
            f'replay_paused={node.replay_paused}',
            flush=True)
        # #region agent log
        try:
            import json
            _js = node.joints.position if node.joints is not None else []
            _payload = {
                'sessionId': 'd5915a', 'runId': 'compare', 'hypothesisId': 'H4',
                'location': 'v1/drive_frozen_grasp.py:result',
                'message': 'v1 grasp result',
                'timestamp': int(time.time() * 1000),
                'data': {
                    'status': status,
                    'succeeded': int(summary.succeeded),
                    'grasp_s': round(elapsed, 2),
                    'replay_paused': bool(node.replay_paused),
                    'reasons': [str(item.reason) for item in summary.outcomes],
                    'joints': [round(float(x), 4) for x in _js],
                },
            }
            with open(
                    '/home/mu/Desktop/aubo_e5_jazzy_ws/.cursor/debug-d5915a.log',
                    'a') as _fh:
                _fh.write(json.dumps(_payload) + '\n')
        except Exception:
            pass
        # #endregion
        for item in summary.outcomes:
            print(
                f'  target={item.target_id} outcome={item.outcome} '
                f'reason={item.reason!r} quality={item.quality_score}',
                flush=True)
        attempted = int(summary.attempted) + len(summary.outcomes)
        if attempted < 1:
            print('no target attempted (perception empty?) — inspect',
                  file=sys.stderr)
            return 3
        if int(summary.succeeded) < 1:
            print('no target succeeded — inspect', file=sys.stderr)
            return 3
        if elapsed > args.timeout + 1.0:
            print(f'grasp exceeded {args.timeout:.0f}s — inspect', file=sys.stderr)
            return 2
        if not node.replay_paused:
            print('grasp finished while replay was still streaming — inspect',
                  file=sys.stderr)
            return 3
        return 0
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
