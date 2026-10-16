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

import math
import struct
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float64

from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


class CommFixture(Node):

    def __init__(self) -> None:
        super().__init__('comm_fixture')
        self.declare_parameter('rwl.answer', 42)
        self.declare_parameter('rwl.step_s', 0.15)
        self.declare_parameter('rwl.bigdata', False)

        cb_group = ReentrantCallbackGroup()

        # ── 话题：发布面（chatter）───────────────────────────────────────
        self._chatter_pub = self.create_publisher(String, '/rwl_demo/chatter', 10)
        self._count = 0
        self._chatter_timer = self.create_timer(0.2, self._on_chatter_timer)

        # ── 大数据源（rwl.bigdata:=true 时启用；大图/点云稳定传输压测被测端）──
        # Image 640x480 rgb8 @30Hz ≈ 0.92MB/帧 ≈ 28MB/s（滚动波纹动画，帧间可见运动）；
        # PointCloud2 50k 点 ×16B @10Hz ≈ 0.8MB/帧（绕 z 轴旋转动画）；
        # Float64 正弦波 @10Hz（rqt_plot 字段绘图演示）。
        self._big_seq = 0
        if bool(self.get_parameter('rwl.bigdata').value):
            self._img_pub = self.create_publisher(Image, '/rwl_demo/big_image', 2)
            self._cloud_pub = self.create_publisher(
                PointCloud2, '/rwl_demo/big_cloud', 2)
            self._wave_pub = self.create_publisher(Float64, '/rwl_demo/wave', 10)
            self._img_variants = self._make_image_variants(640, 480, steps=30)
            self._cloud_variants = self._make_cloud_variants(50_000, steps=24)
            self._img_timer = self.create_timer(1.0 / 30.0, self._on_big_image)
            self._cloud_timer = self.create_timer(0.1, self._on_big_cloud)
            self._wave_timer = self.create_timer(0.1, self._on_wave)

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

    # ── 大数据源 ───────────────────────────────────────────────────────────
    @staticmethod
    def _make_image_variants(width: int, height: int, steps: int) -> list[Image]:
        """预生成 steps 帧滚动波纹动画（对角彩色波带 + 左上角帧号计数块），
        运行时按序发布——30Hz 全帧 Python 重绘不可行，变体轮播等效动画.
        行缓存：行内容只依赖 (x+wy)，预计算 width+height 条行，变体按行拼接."""
        digit_rows = {  # 0..9 的 4 行位图（每行 4bit）
            '0': (0b0110, 0b1001, 0b1001, 0b0110), '1': (0b0010, 0b0110, 0b0010, 0b0111),
            '2': (0b1110, 0b0011, 0b1100, 0b1111), '3': (0b1110, 0b0011, 0b0011, 0b1110),
            '4': (0b1001, 0b1001, 0b1111, 0b0001), '5': (0b1111, 0b1100, 0b0011, 0b1100),
            '6': (0b0111, 0b1100, 0b1001, 0b0110), '7': (0b1111, 0b0010, 0b0100, 0b0100),
            '8': (0b0110, 0b1111, 0b1001, 0b0110), '9': (0b0110, 0b1001, 0b0111, 0b0001),
        }

        def draw_counter(data, seq):
            text = f'{seq % 10000:04d}'
            for i, ch in enumerate(text):
                bits = digit_rows[ch]
                for ry, row in enumerate(bits):
                    for rx in range(4):
                        if row & (0b1000 >> rx):
                            for dy in range(2):
                                for dx in range(2):
                                    base = ((ry * 2 + dy) * width
                                            + i * 10 + rx * 2 + 2) * 3
                                    data[base] = data[base + 1] = data[base + 2] = 255

        rows = []
        for wy in range(width + height):
            row = bytearray(width * 3)
            band = math.sin(wy * 0.11) * 0.5 + 0.5
            for x in range(width):
                wave = math.sin((x + wy) * 0.02) * 0.5 + 0.5
                base = x * 3
                row[base] = int(30 + 90 * wave)
                row[base + 1] = int(60 + 180 * band * wave)
                row[base + 2] = int(40 + 120 * band)
            rows.append(bytes(row))

        imgs = []
        for k in range(steps):
            shift = k * width // steps
            data = bytearray(b''.join(rows[shift + y] for y in range(height)))
            draw_counter(data, k)
            img = Image()
            img.width, img.height = width, height
            img.encoding = 'rgb8'
            img.is_bigendian = 0
            img.step = width * 3
            img.data = bytes(data)
            imgs.append(img)
        return imgs

    @staticmethod
    def _make_cloud_variants(n_points: int, steps: int) -> list[PointCloud2]:
        """预生成 steps 帧绕 z 轴旋转的正弦环点云（运行时轮播 = 3D 旋转动画）."""
        clouds = []
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        for k in range(steps):
            ang = 2.0 * math.pi * k / steps
            ca, sa = math.cos(ang), math.sin(ang)
            cloud = PointCloud2()
            cloud.height = 1
            cloud.width = n_points
            cloud.fields = fields
            cloud.is_bigendian = False
            cloud.point_step = 16
            cloud.row_step = n_points * 16
            buf = bytearray(n_points * 16)
            for i in range(n_points):
                u = i / n_points
                bx = 4.0 * u - 2.0
                by = math.sin(6.2832 * u) * 1.5
                bz = math.cos(12.566 * u) * 0.3 + 0.5
                struct.pack_into('<fffI', buf, i * 16,
                                 bx * ca - by * sa, bx * sa + by * ca, bz,
                                 (i * 7919) & 0xffffff)
            cloud.data = bytes(buf)
            cloud.is_dense = True
            clouds.append(cloud)
        return clouds

    def _on_big_image(self) -> None:
        self._big_seq += 1
        img = self._img_variants[self._big_seq % len(self._img_variants)]
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'rwl_demo_base'
        self._img_pub.publish(img)

    def _on_big_cloud(self) -> None:
        cloud = self._cloud_variants[(self._big_seq // 3) % len(self._cloud_variants)]
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = 'rwl_demo_base'
        self._cloud_pub.publish(cloud)

    def _on_wave(self) -> None:
        msg = Float64()
        msg.data = math.sin(self._big_seq * 0.08) * 2.0
        self._wave_pub.publish(msg)

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
