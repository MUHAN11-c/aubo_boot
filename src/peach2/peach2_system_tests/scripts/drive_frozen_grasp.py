#!/usr/bin/env python3
"""
Drive one frozen-camera FULL harvest on an already-running v2 mock stack.

Waits for lifecycle + RGB-D replay + JTC, enables execution+grasp+tool
(mock IO backend, no real SetIO), sends RunBatch INTENT_FULL, and ACKs if
the batch parks in WAITING_ACK.
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
from peach2_interfaces.msg import BatchState, HarvestResult
from peach2_interfaces.srv import SetEnables
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

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


class Driver(Node):
    """Operator stand-in: SetEnables, RunBatch, ACK."""

    def __init__(self) -> None:
        super().__init__('drive_frozen_grasp')
        self._group = ReentrantCallbackGroup()
        self.images = 0
        self.batch_state: BatchState | None = None
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, RELIABLE,
            callback_group=self._group)
        self.create_subscription(
            BatchState, '/peach/task/state', self._on_batch, LATCHED,
            callback_group=self._group)
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

    def _on_image(self, _msg: Image) -> None:
        self.images += 1

    def _on_batch(self, msg: BatchState) -> None:
        self.batch_state = msg

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

    def wait_ready(self, timeout_s: float) -> bool:
        def ready() -> bool:
            if self.images <= 0:
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-id', default='',
                        help='batch id (default frozen_v2_<unix>)')
    parser.add_argument('--tool-id', default='bite_shear_v1')
    parser.add_argument('--ready-timeout', type=float, default=45.0)
    parser.add_argument('--timeout', type=float, default=80.0)
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
        if not node._run.wait_for_server(timeout_sec=10.0):
            print('run_batch ACTION_UNAVAILABLE', file=sys.stderr)
            return 1
        goal = RunBatch.Goal()
        goal.request_id = rid
        goal.intent = RunBatch.Goal.INTENT_FULL
        goal.tool_id = args.tool_id
        goal.max_targets = 1
        goal.per_target_timeout_s = 70.0
        print(f'goal: request_id={rid} INTENT_FULL tool={args.tool_id}',
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
