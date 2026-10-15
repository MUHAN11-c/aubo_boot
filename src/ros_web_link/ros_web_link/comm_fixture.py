"""rwl_comm_fixture — Web↔ROS 通信矩阵演示/验证节点.

在 `/rwl_demo` 命名空间下提供 ROS 生态各通信原语的最小被测端，供
ros_web_link 的 E2E 测试（test/test_e2e_comm_matrix.py）与 console.html
诊断页自检使用——等价 rosbridge_suite 自带 examples 的定位：

  话题  /rwl_demo/chatter      std_msgs/String        5 Hz 发布（订阅面）
  话题  /rwl_demo/echo         std_msgs/String        订阅→回发 /rwl_demo/echo_reply
                                                    （客户端发布面 roundtrip）
  服务  /rwl_demo/add_two_ints example_interfaces/srv/AddTwoInts
  动作  /rwl_demo/fibonacci    example_interfaces/action/Fibonacci
        每 step_step_s 秒一条 feedback；支持取消（cancel → CANCELED）
  参数  comm_fixture.rwl.answer (int, 默认 42)   经标准参数服务读写
  TF    /tf  rwl_demo_base → rwl_demo_marker    1 Hz

启动：
  ros2 run ros_web_link rwl_comm_fixture
  # 或 launch: ros2 launch ros_web_link transport.launch.xml fixture:=true
"""
from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class CommFixture(Node):

    def __init__(self) -> None:
        super().__init__('comm_fixture')
        self.declare_parameter('rwl.answer', 42)
        self.declare_parameter('rwl.step_s', 0.15)

        cb_group = ReentrantCallbackGroup()

        # ── 话题：发布面（chatter）───────────────────────────────────────
        self._chatter_pub = self.create_publisher(String, '/rwl_demo/chatter', 10)
        self._count = 0
        self._chatter_timer = self.create_timer(0.2, self._on_chatter_timer)

        # ── 话题：客户端发布面（echo → echo_reply roundtrip）───────────
        # echo 同时保留一个 1Hz 存活发布者：浏览器端 foxglove 客户端发布要求
        # 话题已在图上（schema 取自 channel），subscriber-only 话题无 channel
        self._echo_pub = self.create_publisher(String, '/rwl_demo/echo', 10)
        self._echo_reply_pub = self.create_publisher(String, '/rwl_demo/echo_reply', 10)
        self._echo_liveness_timer = self.create_timer(1.0, self._on_echo_liveness)
        self._echo_sub = self.create_subscription(
            String, '/rwl_demo/echo', self._on_echo, 10)

        # ── 服务 ─────────────────────────────────────────────────────────
        self._add_srv = self.create_service(
            AddTwoInts, '/rwl_demo/add_two_ints', self._on_add_two_ints,
            callback_group=cb_group)

        # ── 动作（feedback + 取消语义）──────────────────────────────────
        self._fib_server = ActionServer(
            self, Fibonacci, '/rwl_demo/fibonacci',
            execute_callback=self._on_fib_execute,
            goal_callback=self._on_fib_goal,
            cancel_callback=self._on_fib_cancel,
            callback_group=cb_group)

        # ── TF ───────────────────────────────────────────────────────────
        # TRANSIENT_LOCAL：晚到的 web 订阅者（诊断页懒连接）也能拿到最后一帧，
        # 与 /tf_static 生态语义一致；Reliability 保持 RELIABLE。
        self._tf_broadcaster = TransformBroadcaster(
            self, qos=QoSProfile(
                depth=10,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self._tf_timer = self.create_timer(1.0, self._on_tf_timer)

        self.get_logger().info(
            'comm_fixture 就绪: /rwl_demo/{chatter,echo,echo_reply,add_two_ints,fibonacci} + TF + 参数 rwl.answer')

    # ── chatter ────────────────────────────────────────────────────────────
    def _on_chatter_timer(self) -> None:
        self._count += 1
        msg = String()
        msg.data = f'rwl hello #{self._count}'
        self._chatter_pub.publish(msg)

    # ── echo roundtrip ─────────────────────────────────────────────────────
    def _on_echo_liveness(self) -> None:
        """echo 存活心跳（让浏览器端发布前图上有该话题的 schema）."""
        msg = String()
        msg.data = 'rwl echo channel alive'
        self._echo_pub.publish(msg)

    def _on_echo(self, msg: String) -> None:
        if msg.data.startswith('rwl echo channel alive'):
            return  # 自己的心跳不回发
        reply = String()
        reply.data = f'echo:{msg.data}'
        self._echo_reply_pub.publish(reply)

    # ── 服务 ───────────────────────────────────────────────────────────────
    def _on_add_two_ints(self, request: AddTwoInts.Request,
                         response: AddTwoInts.Response) -> AddTwoInts.Response:
        response.sum = int(request.a) + int(request.b)
        self.get_logger().debug(f'add_two_ints({request.a}, {request.b}) = {response.sum}')
        return response

    # ── fibonacci 动作 ─────────────────────────────────────────────────────
    def _on_fib_goal(self, goal) -> int:
        """接受全部 goal（order 上限防呆）。rclpy 回调参数是 goal 消息本体."""
        if goal.order < 0 or goal.order > 1000:
            self.get_logger().warn(f'拒绝 order={goal.order}（越界）')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_fib_cancel(self, goal_handle) -> int:
        return CancelResponse.ACCEPT

    def _on_fib_execute(self, goal_handle):
        step_s = float(self.get_parameter('rwl.step_s').value)
        request = goal_handle.request
        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()
        seq = [0, 1]

        self.get_logger().info(f'fibonacci goal 开始 order={request.order}')
        for i in range(1, request.order + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.sequence = seq
                self.get_logger().info('fibonacci goal 取消')
                return result
            seq.append(seq[i] + seq[i - 1])
            feedback.sequence = [int(x) for x in seq]
            goal_handle.publish_feedback(feedback)
            time.sleep(step_s)

        goal_handle.succeed()
        result.sequence = [int(x) for x in seq]
        self.get_logger().info(f'fibonacci goal 完成（{len(seq)} 项）')
        return result

    # ── TF ─────────────────────────────────────────────────────────────────
    def _on_tf_timer(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'rwl_demo_base'
        t.child_frame_id = 'rwl_demo_marker'
        t.transform.translation.x = 0.5
        t.transform.translation.z = 0.3
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommFixture()
    # execute 回调里 time.sleep 长跑：单线程 executor 会把 cancel_goal 服务
    # 堵到 goal 终结（rclpy 文档：长 execute 必须配 MultiThreadedExecutor）
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
