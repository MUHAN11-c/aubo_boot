#!/usr/bin/env python3
"""
Drive one frozen-camera FULL harvest on an already-running v2 mock stack.

Waits for lifecycle + RGB-D replay + JTC, enables execution+grasp+tool
(mock IO backend, no real SetIO), moves the arm to a calibrated PHOTO POSE
(the real flow photographs at a wrist pose; at the mock stow pose the frozen
bag lands ~1.0 m out and the reach-to-neck pose has no IK — see
plans/冻结相机全流程测试/测试报告.md §5.7/§5.8), then sends RunBatch and
ACKs if the batch parks in WAITING_ACK.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from peach2_interfaces.action import RunBatch
from peach2_interfaces.msg import BatchState, HarvestResult, TargetModelArray
from peach2_interfaces.srv import SetEnables
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import SetBool, Trigger


LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)
MANAGED = (
    'peach2_perception', 'peach2_target_model', 'peach2_scene',
    'peach2_manipulation', 'peach2_task',
)
CONTROLLERS = ('joint_state_broadcaster', 'joint_trajectory_controller')
REACHED = {
    HarvestResult.REACHED_NONE: 'NONE',
    HarvestResult.REACHED_PREGRASP: 'PREGRASP',
    HarvestResult.REACHED_INSERTED: 'INSERTED',
    HarvestResult.REACHED_CUT_CONFIRMED: 'CUT_CONFIRMED',
    HarvestResult.REACHED_RETREATED: 'RETREATED',
    HarvestResult.REACHED_RELEASED: 'RELEASED',
}


def _wait(predicate, timeout_s: float, period_s: float = 0.2) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


def _dist(a, b) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


class Driver(Node):
    """Operator stand-in: SetEnables, RunBatch, ACK."""

    def __init__(self) -> None:
        super().__init__('drive_frozen_grasp')
        self._group = ReentrantCallbackGroup()
        self.images = 0
        self.batch_state: BatchState | None = None
        self.model: TargetModelArray | None = None
        self.joints: JointState | None = None
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, RELIABLE,
            callback_group=self._group)
        self.create_subscription(
            JointState, '/joint_states', self._on_joints, RELIABLE,
            callback_group=self._group)
        self.create_subscription(
            BatchState, '/peach/task/state', self._on_batch, LATCHED,
            callback_group=self._group)
        self.create_subscription(
            TargetModelArray, '/peach/target_model/models', self._on_models,
            LATCHED, callback_group=self._group)
        self._state_clients: dict[str, object] = {}
        self._list = self.create_client(
            ListControllers, '/controller_manager/list_controllers',
            callback_group=self._group)
        self._enables = self.create_client(
            SetEnables, '/peach/task/set_enables', callback_group=self._group)
        self._ack = self.create_client(
            Trigger, '/peach/task/acknowledge_recovery',
            callback_group=self._group)
        self._run = ActionClient(
            self, RunBatch, '/peach/task/run_batch',
            callback_group=self._group)
        self._replay_pause = self.create_client(
            SetBool, '/frozen_camera_replay/pause', callback_group=self._group)

    def _on_image(self, _msg: Image) -> None:
        self.images += 1

    def _on_joints(self, msg: JointState) -> None:
        self.joints = msg

    def _on_batch(self, msg: BatchState) -> None:
        self.batch_state = msg

    def _on_models(self, msg: TargetModelArray) -> None:
        if msg.models:
            self.model = msg
            for m in msg.models:
                b, n = m.bottom.position, m.neck.position
                print(
                    f'model {m.target_id} rev={m.model_revision} '
                    f'bottom=({b.x:+.3f},{b.y:+.3f},{b.z:+.3f}) '
                    f'neck=({n.x:+.3f},{n.y:+.3f},{n.z:+.3f}) '
                    f'len={m.length_m:.3f} d95={m.d95_m:.3f} '
                    f'conv={int(m.converged)} n_views={m.n_views}',
                    flush=True)

    def _call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(request)
        if not _wait(future.done, timeout_s, 0.05):
            return None
        return future.result()

    def lifecycle_state(self, name: str) -> int | None:
        client = self._state_clients.get(name)
        if client is None:
            client = self.create_client(
                GetState, f'/{name}/get_state', callback_group=self._group)
            self._state_clients[name] = client
        response = self._call(client, GetState.Request(), 3.0)
        return None if response is None else response.current_state.id

    def wait_ready(self, timeout_s: float, require_images: bool = True) -> bool:
        def ready() -> bool:
            if require_images and self.images <= 0:
                return False
            for name in MANAGED:
                if self.lifecycle_state(name) != LifecycleState.PRIMARY_STATE_ACTIVE:
                    return False
            return True

        ok = _wait(ready, timeout_s, 1.0)
        states = {n: self.lifecycle_state(n) for n in MANAGED}
        print(f'ready images={self.images} states={states}', flush=True)
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

    def set_enables(self) -> bool:
        req = SetEnables.Request()
        req.execution = True
        req.grasp = True
        req.tool = True
        response = self._call(self._enables, req)
        ok = response is not None and bool(response.accepted)
        print(
            f'enables(execution+grasp+tool): '
            f'{"OK" if ok else "FAIL"} {getattr(response, "message", "")}',
            flush=True)
        return ok

    def ack_if_needed(self) -> None:
        state = self.batch_state
        if state is None or state.phase != BatchState.WAITING_ACK:
            return
        if not state.recovery_required:
            return
        response = self._call(self._ack, Trigger.Request(), 5.0)
        print(
            f'ACK accepted={getattr(response, "success", None)} '
            f'{getattr(response, "message", "")}',
            flush=True)

    def watch_motion_freeze(self, tol_rad: float = 2e-3) -> None:
        """Daemon thread: once the batch leaves the survey phase (BeginScene
        has cleared the stale tracks and fresh anchors exist), pause the
        replay on the first harvest motion (>2 mm joint deviation from the
        phase-entry baseline). Frozen frame × live wrist TF would otherwise
        drift the fresh models along the arm during staging/insert. Survey-
        phase motion is intentionally NOT frozen: BeginScene clears whatever
        drifted there and re-anchors at the (stationary) survey pose."""
        def watch() -> None:
            baseline = None
            deadline = time.monotonic() + 180.0
            while time.monotonic() < deadline:
                state = self.batch_state
                if state is not None and state.phase >= 3:  # OBSERVING..ABORTED
                    js = self.joints
                    if js is not None:
                        if baseline is None:
                            baseline = list(js.position)
                        else:
                            dev = max((abs(a - b)
                                       for a, b in zip(js.position, baseline)),
                                      default=0.0)
                            if dev > tol_rad:
                                self._pause_replay(True)
                                print(f'photo: replay paused on harvest motion '
                                      f'(joint dev {dev:.4f} rad) — world frozen',
                                      flush=True)
                                return
                time.sleep(0.05)
            print('photo: motion watch expired (replay left streaming)',
                  file=sys.stderr, flush=True)

        threading.Thread(target=watch, daemon=True).start()

    def _pause_replay(self, paused: bool) -> None:
        if self._replay_pause.wait_for_service(timeout_sec=2.0):
            req = SetBool.Request()
            req.data = paused
            self._call(self._replay_pause, req, 5.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-id', default='',
                        help='batch id (default frozen_v2_<unix>)')
    parser.add_argument('--tool-id', default='bite_shear_v1')
    parser.add_argument('--intent', default='FULL', choices=('FULL', 'PREGRASP_ONLY'),
                        help='FULL needs the blade goal inside the pose reach sphere; '
                             'PREGRASP_ONLY completes at the pregrasp stop')
    parser.add_argument('--ready-timeout', type=float, default=45.0)
    parser.add_argument('--timeout', type=float, default=100.0,
                        help='RunBatch wall budget; the batch now also owns '
                             'the ~16 s survey transit to the photo pose')
    args = parser.parse_args()
    rid = args.request_id.strip() or f'frozen_v2_{int(time.time())}'

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
        if not node.set_enables():
            return 1
        # The mock spawns at the true photo pose (SRDF global_photo_pose via
        # mock_initial_positions): no transit exists, the world stays live and
        # no frame ever spans motion until the harvest freeze watcher.
        node.watch_motion_freeze()
        print('photo: arm spawns at the photo pose — survey anchors in place',
              flush=True)
        if not node._run.wait_for_server(timeout_sec=10.0):
            print('run_batch ACTION_UNAVAILABLE', file=sys.stderr)
            return 1
        goal = RunBatch.Goal()
        goal.request_id = rid
        goal.intent = (RunBatch.Goal.INTENT_FULL if args.intent == 'FULL'
                       else RunBatch.Goal.INTENT_PREGRASP_ONLY)
        goal.tool_id = args.tool_id
        goal.max_targets = 1
        goal.per_target_timeout_s = 70.0
        print(f'goal: request_id={rid} INTENT_{args.intent} tool={args.tool_id}',
              flush=True)
        send = node._run.send_goal_async(goal)
        if not _wait(send.done, 15.0, 0.05):
            print('goal send TIMEOUT', file=sys.stderr)
            return 1
        handle = send.result()
        if handle is None or not handle.accepted:
            print('goal REJECTED', file=sys.stderr)
            return 1
        result_future = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not result_future.done():
            node.ack_if_needed()
            time.sleep(0.5)
        if not result_future.done():
            print(f'goal TIMEOUT(>={args.timeout}s) — inspect', file=sys.stderr)
            handle.cancel_goal_async()
            _wait(result_future.done, 8.0, 0.1)
            return 2
        wrapper = result_future.result()
        result = wrapper.result
        print(
            f'RESULT termination={result.termination_reason} '
            f'succeeded={result.succeeded} failed={result.failed} '
            f'skipped={result.skipped}',
            flush=True)
        for item in result.results:
            print(
                f'  target={item.target_id} outcome={item.outcome} '
                f'code={item.failure_code} reached='
                f'{REACHED.get(item.reached, item.reached)} '
                f'reason={item.reason!r}',
                flush=True)
        n = int(result.succeeded) + int(result.failed) + int(result.skipped)
        if n < 1:
            print('no target attempted (perception empty?) — inspect',
                  file=sys.stderr)
            return 3
        if int(result.succeeded) < 1:
            print('no target succeeded — inspect', file=sys.stderr)
            return 3
        return 0
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
