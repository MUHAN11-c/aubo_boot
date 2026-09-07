"""有序拉起 / 拆除 peach 生命周期节点；不自动 RunHarvest."""
from __future__ import annotations

import threading

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from peach_executor.lifecycle_manager_parameters import peach_lifecycle_manager
from peach_interfaces.srv import ManageLifecycleNodes
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _wait_future(future, timeout: float) -> bool:
    """在工作线程里等服务 future，避免自旋占用 executor."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout=timeout)


class LifecycleManagerNode(Node):
    """按 node_names 先 configure 再 activate；拆除逆序；manage_nodes 可再入."""

    def __init__(self):
        super().__init__('peach_lifecycle_manager')
        # GPL 声明/默认值/校验单一事实源：config/lifecycle_manager_parameters.yaml
        self._param_listener = peach_lifecycle_manager.ParamListener(self)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(
            Bool, '/peach/lifecycle/managed_nodes_activated', latched)
        self._pub.publish(Bool(data=False))
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._change_clients = {}
        self._state_clients = {}
        self.create_service(
            ManageLifecycleNodes, '~/manage_nodes', self._on_manage,
            callback_group=self._cb)
        self._timer = self.create_timer(0.2, self._kick, callback_group=self._cb)

    def _snapshot(self):
        """当前 GPL 参数快照（名单与超时）."""
        return self._param_listener.get_params()

    def _kick(self):
        # 离开定时器回调再阻塞 RPC，避免卡住默认 executor
        self.destroy_timer(self._timer)
        threading.Thread(target=self._startup, daemon=True).start()

    def _on_manage(self, request, response):
        """整栈生命周期命令；与批次 ControlTask 不是同一层."""
        timeout = float(self._snapshot().startup_timeout_s)
        ok, message = self._run_command(int(request.command), timeout)
        response.success = ok
        response.message = message
        return response

    def _startup(self):
        timeout = float(self._snapshot().startup_timeout_s)
        ok, message = self._run_command(
            ManageLifecycleNodes.Request.STARTUP, timeout)
        if ok:
            self.get_logger().info(
                'managed nodes Active（仍须显式 RunHarvest）')
        else:
            self.get_logger().error(f'lifecycle 启动失败: {message}')

    def _run_command(self, command: int, timeout: float) -> tuple[bool, str]:
        names = list(self._snapshot().node_names)
        reverse = list(reversed(names))
        with self._lock:
            if command == ManageLifecycleNodes.Request.STARTUP:
                ok = self._configure_then_activate(names, timeout)
                self._pub.publish(Bool(data=ok))
                return ok, 'active' if ok else 'startup failed'
            if command == ManageLifecycleNodes.Request.PAUSE:
                ok = self._transition_all(
                    reverse, Transition.TRANSITION_DEACTIVATE, timeout)
                self._pub.publish(Bool(data=False))
                return ok, 'inactive' if ok else 'pause failed'
            if command == ManageLifecycleNodes.Request.RESUME:
                ok = self._transition_all(
                    names, Transition.TRANSITION_ACTIVATE, timeout)
                self._pub.publish(Bool(data=ok))
                return ok, 'active' if ok else 'resume failed'
            if command == ManageLifecycleNodes.Request.RESET:
                self._pub.publish(Bool(data=False))
                if not self._transition_all(
                        reverse, Transition.TRANSITION_DEACTIVATE, timeout):
                    return False, 'reset deactivate failed'
                if not self._transition_all(
                        reverse, Transition.TRANSITION_CLEANUP, timeout):
                    return False, 'reset cleanup failed'
                ok = self._configure_then_activate(names, timeout)
                self._pub.publish(Bool(data=ok))
                return ok, 'active' if ok else 'reset startup failed'
            if command == ManageLifecycleNodes.Request.SHUTDOWN:
                ok = self._teardown_locked(reverse, timeout)
                self._pub.publish(Bool(data=False))
                return ok, 'unconfigured' if ok else 'shutdown failed'
            return False, f'unknown command {command}'

    def _configure_then_activate(self, names, timeout: float) -> bool:
        if not self._transition_all(
                names, Transition.TRANSITION_CONFIGURE, timeout):
            return False
        return self._transition_all(
            names, Transition.TRANSITION_ACTIVATE, timeout)

    def _teardown_locked(self, reverse, timeout: float) -> bool:
        ok = self._transition_all(
            reverse, Transition.TRANSITION_DEACTIVATE, timeout)
        ok = self._transition_all(
            reverse, Transition.TRANSITION_CLEANUP, timeout) and ok
        return ok

    def _transition_all(self, names, transition_id: int, timeout: float) -> bool:
        ok = True
        for name in names:
            if not self._change(name, transition_id, timeout):
                ok = False
        return ok

    def _change(self, name: str, transition_id: int, timeout: float) -> bool:
        current = self._get_state(name, timeout)
        if current is None:
            self.get_logger().error(f'{name} get_state 不可用')
            return False
        if self._already_done(current, transition_id):
            return True
        if not self._can_apply(current, transition_id):
            self.get_logger().error(
                f'{name} 状态 {current} 不能做 transition {transition_id}')
            return False
        client = self._client(
            self._change_clients, ChangeState, f'/{name}/change_state')
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'{name} change_state 不可用')
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        if not _wait_future(future, timeout):
            self.get_logger().error(f'{name} change_state 超时')
            return False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'{name} change_state 失败: {exc}')
            return False
        if not response.success:
            self.get_logger().error(
                f'{name} 拒绝 transition {transition_id}')
            return False
        return True

    def _already_done(self, current: int, transition_id: int) -> bool:
        if transition_id == Transition.TRANSITION_CONFIGURE:
            return current in (
                State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE)
        if transition_id == Transition.TRANSITION_ACTIVATE:
            return current == State.PRIMARY_STATE_ACTIVE
        if transition_id == Transition.TRANSITION_DEACTIVATE:
            return current != State.PRIMARY_STATE_ACTIVE
        if transition_id == Transition.TRANSITION_CLEANUP:
            return current == State.PRIMARY_STATE_UNCONFIGURED
        return False

    def _can_apply(self, current: int, transition_id: int) -> bool:
        if transition_id == Transition.TRANSITION_CONFIGURE:
            return current == State.PRIMARY_STATE_UNCONFIGURED
        if transition_id == Transition.TRANSITION_ACTIVATE:
            return current == State.PRIMARY_STATE_INACTIVE
        if transition_id == Transition.TRANSITION_DEACTIVATE:
            return current == State.PRIMARY_STATE_ACTIVE
        if transition_id == Transition.TRANSITION_CLEANUP:
            return current == State.PRIMARY_STATE_INACTIVE
        return False

    def _get_state(self, name: str, timeout: float):
        client = self._client(
            self._state_clients, GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
            return None
        future = client.call_async(GetState.Request())
        if not _wait_future(future, min(5.0, timeout)):
            return None
        try:
            return future.result().current_state.id
        except Exception:  # noqa: BLE001
            return None

    def _client(self, cache: dict, srv_type, service_name: str):
        client = cache.get(service_name)
        if client is None:
            client = self.create_client(
                srv_type, service_name, callback_group=self._cb)
            cache[service_name] = client
        return client


def main(args=None):
    """节点入口；双线程以免 manage_nodes 与 change_state 互锁."""
    rclpy.init(args=args)
    node = LifecycleManagerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
