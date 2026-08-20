"""有序拉起 peach 生命周期节点；不自动 RunHarvest."""
from __future__ import annotations

import threading

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _wait_future(future, timeout: float) -> bool:
    """在工作线程里等服务 future，避免自旋占用 executor."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    return done.wait(timeout=timeout)


class PeachLifecycleManager(Node):
    """按 node_names 先全部 configure，再全部 activate；旗标闩锁发布."""

    def __init__(self):
        super().__init__('peach_lifecycle_manager')
        # 感知 → 重建 → 技能 → 执行器；执行器 require_managed_stack 读下方旗标
        self.declare_parameter(
            'node_names', [
                'peach_scene_perception_node',
                'peach_target_reconstruction_node',
                'peach_manipulation_skills_node',
                'peach_task_executor',
            ])
        self.declare_parameter('startup_timeout_s', 60.0)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(
            Bool, '/peach/lifecycle/managed_nodes_activated', latched)
        self._pub.publish(Bool(data=False))
        self._timer = self.create_timer(0.2, self._kick)

    def _kick(self):
        # 离开定时器回调再阻塞 RPC，避免卡住默认 executor
        self.destroy_timer(self._timer)
        threading.Thread(target=self._startup, daemon=True).start()

    def _startup(self):
        names = list(self.get_parameter('node_names').value)
        timeout = float(self.get_parameter('startup_timeout_s').value)
        ok = True
        for name in names:
            if not self._change(name, Transition.TRANSITION_CONFIGURE, timeout):
                ok = False
                break
        if ok:
            for name in names:
                if not self._change(name, Transition.TRANSITION_ACTIVATE, timeout):
                    ok = False
                    break
        self._pub.publish(Bool(data=ok))
        if ok:
            self.get_logger().info('managed nodes Active（仍须显式 RunHarvest）')
        else:
            self.get_logger().error('lifecycle startup failed')

    def _change(self, name: str, transition_id: int, timeout: float) -> bool:
        if self._is_active(name, timeout):
            return True
        client = self.create_client(ChangeState, f'/{name}/change_state')
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
            self.get_logger().error(f'{name} 拒绝 transition {transition_id}')
            return False
        return True

    def _is_active(self, name: str, timeout: float) -> bool:
        client = self.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=min(5.0, timeout)):
            return False
        future = client.call_async(GetState.Request())
        if not _wait_future(future, min(5.0, timeout)):
            return False
        try:
            return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
        except Exception:  # noqa: BLE001
            return False


def main(args=None):
    """节点入口."""
    rclpy.init(args=args)
    node = PeachLifecycleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
