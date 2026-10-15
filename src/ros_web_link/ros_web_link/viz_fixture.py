"""rwl_viz_fixture — 常用可视化消息发布节点（Web↔ROS 可视化链路独立被测端）.

在 `/rwl_viz` 命名空间下按 1 Hz 发布 RViz/Foxglove 常用可视化消息，覆盖
CDR 解码的难点形态（定长数组 float64[36]、嵌套结构、二进制 blob、变长数组）：

  sensor_msgs/Image            单色 8UC1 棋盘 64×48（blob）
  sensor_msgs/CompressedImage  同图的 JPEG（真实可解码，两相位交替；bytes 数组）
  sensor_msgs/CameraInfo       定长数组 distortion/内参矩阵
  sensor_msgs/PointCloud2      XYZ float32 ×128 点（嵌套 PointField[] + data blob）
  sensor_msgs/LaserScan        ranges/intensities float32[]（含 inf 语义）
  sensor_msgs/Range            超声/雷达标量
  nav_msgs/Odometry            嵌套 Pose+Twist + float64[36]×2 协方差（圆周轨迹）
  nav_msgs/Path                PoseStamped[] 折线
  geometry_msgs/PoseArray      Pose[]
  geometry_msgs/PoseStamped    单位姿
  geometry_msgs/PoseWithCovarianceStamped  位姿 + 旋转协方差（95% 椭圆显示）
  geometry_msgs/WrenchStamped  力/力矩
  visualization_msgs/Marker    SPHERE_LIST（points+colors 嵌套数组）
  visualization_msgs/MarkerArray
  diagnostic_msgs/DiagnosticArray  嵌套 status/key_value
  sensor_msgs/JointState       变长 name/position/velocity 并行数组（机械臂 6 关节）

另每 5s 打一条 INFO 日志（/rosout，web 端 rqt_console 面板数据源）。

启动：ros2 run ros_web_link rwl_viz_fixture
或 ros2 launch ros_web_link transport.launch.xml viz_fixture:=true
"""
from __future__ import annotations

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (PointStamped, Pose, PoseArray, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped,
                               WrenchStamped)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, JointState,
                             LaserScan, PointCloud2, PointField, Range)
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformBroadcaster

from ._jpeg_frames import JPEG_FRAMES as _JPEG_FRAMES


def _now(node: Node) -> Header:
    h = Header()
    h.stamp = node.get_clock().now().to_msg()
    h.frame_id = 'rwl_viz'
    return h


def _link(idx: int, size: str, origin: str) -> str:
    return f'''
  <link name="link_{idx}">
    <visual>
      <origin xyz="{origin}" rpy="0 0 0"/>
      <geometry><box size="{size}"/></geometry>
      <material name="m_link_{idx}"><color rgba="0.45 0.55 0.75 1"/></material>
    </visual>
  </link>'''


def _joint(idx: int, axis: str, origin: str) -> str:
    parent = 'base_link' if idx == 0 else f'link_{idx - 1}'
    return f'''
  <joint name="joint_{idx}" type="revolute">
    <parent link="{parent}"/>
    <child link="link_{idx}"/>
    <origin xyz="{origin}" rpy="0 0 0"/>
    <axis xyz="{axis}"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1.0"/>
  </joint>'''


# 演示 URDF：6 转动关节箱链（joint_0..5 与 /rwl_viz/joint_states 同名驱动）
_DEMO_URDF = ('<?xml version="1.0"?><robot name="rwl_viz_bot">'
              + '<link name="base_link"><visual><geometry>'
              + '<box size="0.24 0.24 0.04"/></geometry>'
              + '<material name="m_base"><color rgba="0.3 0.35 0.45 1"/></material>'
              + '</visual></link>'
              + _link(0, '0.09 0.09 0.30', '0 0 0.17')
              + _link(1, '0.08 0.08 0.26', '0 0 0.15')
              + _link(2, '0.07 0.07 0.22', '0 0 0.13')
              + _link(3, '0.06 0.06 0.18', '0 0 0.11')
              + _link(4, '0.05 0.05 0.14', '0 0 0.09')
              + _link(5, '0.05 0.05 0.05', '0 0 0.05')
              + _joint(0, '0 0 1', '0 0 0.02')
              + _joint(1, '0 1 0', '0 0 0.32')
              + _joint(2, '0 1 0', '0 0 0.30')
              + _joint(3, '0 0 1', '0 0 0.24')
              + _joint(4, '0 1 0', '0 0 0.20')
              + _joint(5, '0 1 0', '0 0 0.16')
              + '</robot>')


class VizFixture(Node):

    def __init__(self) -> None:
        super().__init__('rwl_viz_fixture')
        self._seq = 0
        # TRANSIENT_LOCAL：晚连接的 web 订阅者（诊断页懒连接）也能拿到最后一帧
        viz_qos = QoSProfile(depth=5,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._pubs = {
            'image': self.create_publisher(Image, '/rwl_viz/image', viz_qos),
            'image_compressed': self.create_publisher(CompressedImage, '/rwl_viz/image_compressed', viz_qos),
            'camera_info': self.create_publisher(CameraInfo, '/rwl_viz/camera_info', viz_qos),
            'points': self.create_publisher(PointCloud2, '/rwl_viz/points', viz_qos),
            'scan': self.create_publisher(LaserScan, '/rwl_viz/scan', viz_qos),
            'range': self.create_publisher(Range, '/rwl_viz/range', viz_qos),
            'odom': self.create_publisher(Odometry, '/rwl_viz/odom', viz_qos),
            'path': self.create_publisher(Path, '/rwl_viz/path', viz_qos),
            'pose_array': self.create_publisher(PoseArray, '/rwl_viz/pose_array', viz_qos),
            'pose': self.create_publisher(PoseStamped, '/rwl_viz/pose', viz_qos),
            'pose_cov': self.create_publisher(PoseWithCovarianceStamped, '/rwl_viz/pose_cov', viz_qos),
            'wrench': self.create_publisher(WrenchStamped, '/rwl_viz/wrench', viz_qos),
            'marker': self.create_publisher(Marker, '/rwl_viz/marker', viz_qos),
            'markers': self.create_publisher(MarkerArray, '/rwl_viz/markers', viz_qos),
            'diagnostics': self.create_publisher(DiagnosticArray, '/rwl_viz/diagnostics', viz_qos),
            'joint_states': self.create_publisher(JointState, '/rwl_viz/joint_states', viz_qos),
        }
        self._tf = TransformBroadcaster(self, qos=viz_qos)
        # clicked_point 存活发布者：浏览器端 Publish Point 工具要求图上已有该话题的
        # channel/schema（foxglove 客户端发布约束；RViz 亦为持续 advertise 语义）
        self._click_pub = self.create_publisher(PointStamped, '/rwl_viz/clicked_point', viz_qos)
        # RobotModel（RViz 招牌显示）：/robot_description latched（joint_0..5 与
        # joint_states 同名，web 端 setJointValues 驱动动画）。
        # publish_urdf:=false 时让位——官方栈（robot_state_publisher 等）自带
        # /robot_description 的场景不应双发（latched 双源会令 web 端解析翻转）。
        self._publish_urdf = bool(self.declare_parameter('publish_urdf', True).value)
        from std_msgs.msg import String as StringMsg
        self._urdf_pub = self.create_publisher(StringMsg, '/robot_description', viz_qos)
        if self._publish_urdf:
            self._urdf_pub.publish(StringMsg(data=_DEMO_URDF))
        # 2D Goal Pose / Initial Pose（Nav2/RViz 交互工具的目标话题）存活发布。
        # /initialpose 用 PoseWithCovarianceStamped（RViz 标准语义，web 端
        # 「2D 初始位姿」工具发布的同型 schema 前置）
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', viz_qos)
        self._initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', viz_qos)
        # 关节命令闭环（Rebot-Arm 镜像模式最小版）：web 滑块 → /joint_cmd →
        # fixture 3s 内跟随命令值 → joint_states → URDF 模型实时响应。
        # liveness 发布者是浏览器端发布的 schema 前置。
        self._joint_cmd_pub = self.create_publisher(JointState, '/rwl_viz/joint_cmd', viz_qos)
        self._joint_cmd = None       # 最近命令（msg + 单调时刻）
        self._joint_cmd_at = 0.0
        self._joint_cmd_seq = 0
        self.create_subscription(JointState, '/rwl_viz/joint_cmd', self._on_joint_cmd, 10)
        self._timer = self.create_timer(1.0, self._on_timer)

    # ── 各消息构造 ─────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        self._seq += 1
        s = self._seq
        self._pubs['image'].publish(self._image(s))
        self._pubs['image_compressed'].publish(self._compressed(s))
        self._pubs['camera_info'].publish(self._camera_info())
        self._pubs['points'].publish(self._cloud(s))
        self._pubs['scan'].publish(self._scan(s))
        self._pubs['range'].publish(self._range(s))
        self._pubs['odom'].publish(self._odom(s))
        self._pubs['path'].publish(self._path(s))
        self._pubs['pose_array'].publish(self._pose_array(s))
        self._pubs['pose'].publish(self._pose_stamped(s))
        self._pubs['pose_cov'].publish(self._pose_cov(s))
        self._pubs['wrench'].publish(self._wrench(s))
        self._pubs['marker'].publish(self._marker(s))
        self._pubs['markers'].publish(self._markers(s))
        self._pubs['diagnostics'].publish(self._diagnostics(s))
        self._pubs['joint_states'].publish(self._joint_states(s))
        cp = PointStamped()
        cp.header = _now(self)
        self._click_pub.publish(cp)  # 存活占位帧（真实点击由 web 端发布）
        gp = PoseStamped()
        gp.header = _now(self)
        self._goal_pub.publish(gp)
        ip = PoseWithCovarianceStamped()
        ip.header = _now(self)
        self._initial_pub.publish(ip)
        jc = JointState()  # joint_cmd 存活占位（全 0，订阅端忽略）
        jc.header = _now(self)
        jc.name = [f'joint_{i}' for i in range(6)]
        jc.position = [0.0] * 6
        self._joint_cmd_pub.publish(jc)
        self._tf.sendTransform(self._tf_msg(s))
        # /rosout 周期日志：web 端 rqt_console 面板的真数据源（0.2Hz 足够演示）
        if s % 5 == 0:
            self.get_logger().info(f'viz fixture tick {s}')

    def _image(self, s: int) -> Image:
        m = Image()
        m.header = _now(self)
        m.height, m.width = 48, 64
        m.encoding = 'mono8'
        m.is_bigendian = 0
        m.step = m.width
        # 棋盘纹 + 帧序号写入首行首像素（解码侧可校验内容一致性）
        px = bytearray(m.height * m.width)
        for y in range(m.height):
            for x in range(m.width):
                px[y * m.width + x] = (255 if (x // 8 + y // 8 + s) % 2 == 0 else 0)
        px[0] = s & 0xFF
        m.data = bytes(px)
        return m

    def _compressed(self, s: int) -> CompressedImage:
        m = CompressedImage()
        m.header = _now(self)
        m.format = 'jpeg'
        # 真实可解码棋盘 JPEG（两相位交替动起来；web 端 createImageBitmap 实解）
        m.data = _JPEG_FRAMES[s % 2]
        return m

    def _camera_info(self) -> CameraInfo:
        m = CameraInfo()
        m.header = _now(self)
        m.height, m.width = 480, 640
        m.distortion_model = 'plumb_bob'
        m.d = [0.1, -0.2, 0.001, 0.0002, 0.0]
        m.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def _cloud(self, s: int) -> PointCloud2:
        m = PointCloud2()
        m.header = _now(self)
        m.height, m.width = 1, 128
        m.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        m.is_bigendian = False
        m.point_step = 12
        m.is_dense = True
        pts = bytearray()
        for i in range(128):
            a = i * 0.1 + s * 0.01
            pts += struct.pack('<fff', math.cos(a) * 2.0, math.sin(a) * 2.0, 0.5)
        m.data = bytes(pts)
        m.row_step = m.point_step * m.width
        return m

    def _scan(self, s: int) -> LaserScan:
        m = LaserScan()
        m.header = _now(self)
        m.angle_min, m.angle_max = -math.pi / 2, math.pi / 2
        m.angle_increment = math.pi / 180.0
        m.time_increment, m.scan_time = 0.0, 0.1
        m.range_min, m.range_max = 0.2, 10.0
        m.ranges = [float('inf') if i % 17 == 0 else 1.0 + (i % 10) * 0.5 + s * 0.01
                    for i in range(181)]
        m.intensities = [float(i % 100) for i in range(181)]
        return m

    def _range(self, s: int) -> Range:
        m = Range()
        m.header = _now(self)
        m.radiation_type = Range.ULTRASOUND
        m.field_of_view = 0.5
        m.min_range, m.max_range = 0.1, 5.0
        m.range = 1.5 + (s % 10) * 0.1
        return m

    def _odom(self, s: int) -> Odometry:
        # 圆周运动（半径 2m，0.15 rad/s：每秒弦长 ~0.3m，超过显示端 0.1m 轨迹采样阈）
        m = Odometry()
        m.header = _now(self)
        m.child_frame_id = 'rwl_viz_base'
        m.pose.pose = Pose()
        a = 0.15 * s
        m.pose.pose.position.x = 2.0 * math.cos(a)
        m.pose.pose.position.y = 2.0 * math.sin(a)
        m.pose.pose.orientation.z = math.sin(a / 2 + math.pi / 4)
        m.pose.pose.orientation.w = math.cos(a / 2 + math.pi / 4)
        cov = [0.0] * 36
        cov[0] = 0.01
        m.pose.covariance = cov
        m.twist.twist.linear.x = 0.5
        m.twist.twist.angular.z = 0.1
        tcov = [0.0] * 36
        tcov[35] = 0.02
        m.twist.covariance = tcov
        return m

    def _path(self, s: int) -> Path:
        m = Path()
        m.header = _now(self)
        for i in range(5):
            ps = PoseStamped()
            ps.header = _now(self)
            ps.pose.position.x = 0.5 * i
            ps.pose.position.y = 0.1 * s
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        return m

    def _pose_array(self, s: int) -> PoseArray:
        m = PoseArray()
        m.header = _now(self)
        for i in range(4):
            p = Pose()
            p.position.z = 0.1 * (i + s % 5)
            p.orientation.w = 1.0
            m.poses.append(p)
        return m

    def _pose_stamped(self, s: int) -> PoseStamped:
        m = PoseStamped()
        m.header = _now(self)
        m.pose.position.x = 0.3
        m.pose.position.y = 0.4
        m.pose.position.z = 0.5 + (s % 7) * 0.01
        m.pose.orientation.w = 1.0
        return m

    def _pose_cov(self, s: int) -> PoseWithCovarianceStamped:
        # 旋转椭圆协方差（主轴随 s 交替摆动，web 端 95% 置信椭圆可视化）
        m = PoseWithCovarianceStamped()
        m.header = _now(self)
        m.pose.pose.position.x = -0.6
        m.pose.pose.position.y = 0.8
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cxx, cyy = 0.04, 0.01
        cxy = 0.015 * math.sin(0.2 * s)
        cov[0], cov[1], cov[7] = cxx, cxy, cyy
        m.pose.covariance = cov
        return m

    def _wrench(self, s: int) -> WrenchStamped:
        m = WrenchStamped()
        m.header = _now(self)
        m.wrench.force.x = 1.5
        m.wrench.torque.z = 0.25 + s * 0.001
        return m

    def _marker(self, s: int) -> Marker:
        m = Marker()
        m.header = _now(self)
        m.ns = 'rwl_viz'
        m.id = s
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.05
        for i in range(6):
            p = Pose()
            p.position.x = 0.2 * i
            p.position.y = 0.1 * (s % 5)
            p.orientation.w = 1.0
            m.points.append(p.position)
            m.colors.append(_color(i / 6.0, 0.8))
        m.pose.orientation.w = 1.0
        return m

    def _markers(self, s: int) -> MarkerArray:
        arr = MarkerArray()
        mk = self._marker(s)
        mk.type = Marker.CUBE
        mk.pose.position.x = 1.0
        arr.markers.append(mk)
        text = Marker()
        text.header = _now(self)
        text.ns, text.id, text.type = 'rwl_viz_text', s, Marker.TEXT_VIEW_FACING
        text.text = f'rwl_viz #{s}'
        text.pose.position.z = 2.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color = _color(1.0, 1.0)
        arr.markers.append(text)
        return arr

    def _diagnostics(self, s: int) -> DiagnosticArray:
        m = DiagnosticArray()
        m.header = _now(self)
        st = DiagnosticStatus()
        st.name = 'rwl_viz/fixture'
        st.level = DiagnosticStatus.OK
        st.message = 'nominal'
        kv = KeyValue()
        kv.key, kv.value = 'seq', str(s)
        st.values.append(kv)
        m.status.append(st)
        return m

    def _on_joint_cmd(self, msg: JointState) -> None:
        """记录 web 端滑块命令（忽略自己的 liveness 占位帧）."""
        if msg.position and any(abs(p) > 1e-9 for p in msg.position):
            self._joint_cmd = msg
            self._joint_cmd_at = self.get_clock().now().nanoseconds / 1e9
            self._joint_cmd_seq += 1

    def _joint_states(self, s: int) -> JointState:
        m = JointState()
        m.header = _now(self)
        m.name = [f'joint_{i}' for i in range(6)]
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self._joint_cmd is not None and (now_s - self._joint_cmd_at) < 3.0:
            # 命令模式：跟随 web 滑块（按关节名对齐，缺省回落演示值）
            demo = [0.1 * ((i + s) % 10) - 0.5 for i in range(6)]
            cmd = {n: float(p) for n, p in zip(self._joint_cmd.name, self._joint_cmd.position)}
            m.position = [cmd.get(f'joint_{i}', demo[i]) for i in range(6)]
            m.velocity = [0.0] * 6
            m.effort = [10.0 + i for i in range(6)]
            return m
        m.position = [0.1 * ((i + s) % 10) - 0.5 for i in range(6)]
        m.velocity = [0.01 * i for i in range(6)]
        m.effort = [10.0 + i for i in range(6)]
        return m

    def _tf_msg(self, s: int):
        t = TransformStamped()
        t.header = _now(self)
        t.child_frame_id = 'rwl_viz_marker'
        t.transform.translation.x = 0.1 * (s % 5)
        t.transform.rotation.w = 1.0
        return t


def _color(h: float, a: float):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = h, 1.0 - h, 0.5, a
    return c


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VizFixture()
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
