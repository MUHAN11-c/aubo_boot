"""Isolated launch_testing: peach_scene_obstacles 快照链（离线，无真机）.

验证（2026-09-29 快照式建图）：合成点云 + 假 refined_pose + 触发 →
节点经 /apply_planning_scene 写入障碍对象——对象存在、障碍点成 box、
目标胶囊邻域无 box、贴机器人表面点被自身滤除、工作空间外点被裁剪、
第二个目标精化后同帧重写（首写后 REMOVE+ADD 原子替换）。
"""
import os
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import SetEnvironmentVariable
import launch_ros.actions
import launch_testing
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, \
    ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Empty, String
import tf2_ros

from moveit_msgs.srv import ApplyPlanningScene
from peach_interfaces.msg import BagGraspCandidate, BagGraspCandidateArray
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

_XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

CLOUD_FRAME = 'camera_depth_optical_frame'
BASE_FRAME = 'base_link'
# 测试自造静态 TF：base_link ← camera_depth_optical_frame 平移 (0.3,0,0.5)
# （相机系点 + (0.3,0,0.5) = base 系点）
CAM_IN_BASE = (0.3, 0.0, 0.5)

# 目标 1 胶囊（base 系）：竖直袋 (0.6,0,0.9)→(0.6,0,1.1)，直径 0.10
CAPSULE_1_BOTTOM = (0.6, 0.0, 0.9)
CAPSULE_1_NECK = (0.6, 0.0, 1.1)
# 目标 2 胶囊（base 系）：(0.0,0.4,0.9)→(0.0,0.4,1.1)
CAPSULE_2_BOTTOM = (0.0, 0.4, 0.9)
CAPSULE_2_NECK = (0.0, 0.4, 1.1)

# 合成点（base 系语义，下面转相机系发）：
OBSTACLE_POINT = (-0.5, 0.3, 0.9)     # 远障碍：应成 box（离 t1/t2 胶囊
#                                       膨胀半径均 >0.4m，不被邻域滤除）
INSIDE_CAPSULE_1 = (0.6, 0.0, 1.0)    # 目标1 轴心：应被胶囊滤除
ON_ROBOT_SURFACE = (0.05, 0.05, 0.05)  # 底座内部：应被自身滤除
OUT_OF_WORKSPACE = (1.4, 0.0, 1.4)    # 半径 1.5 外（3D 距离≈1.98）：应被裁剪

_MUST_JOINTS = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint')

_LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

# 点云发布端对齐真实前端（09-29 勘定：percipio 与 peach_stereo 发布端均
# RELIABLE；scene_obstacles 订阅亦 RELIABLE——BE 发布会被 QoS 匹配拒绝）。
_CLOUD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)


def _to_cloud(base_point):
    return tuple(b - c for b, c in zip(base_point, CAM_IN_BASE))


def _generate_urdf() -> str:
    description = os.path.join(
        get_package_share_directory('aubo_description'),
        'urdf', 'aubo_e5.urdf.xacro')
    result = subprocess.run(
        ['xacro', description, 'hardware_mode:=mock'],
        capture_output=True, text=True, timeout=60, check=True)
    return result.stdout


def _candidate(target_id, bottom, neck):
    item = BagGraspCandidate()
    item.target_id = target_id
    item.bag_bottom.x, item.bag_bottom.y, item.bag_bottom.z = bottom
    item.bag_neck.x, item.bag_neck.y, item.bag_neck.z = neck
    item.bag_diameter_upper_m = 0.10
    return item


@pytest.mark.launch_test
def generate_test_description():
    """Start only peach_scene_obstacles（假数据全由测试进程喂）."""
    return launch.LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'),
        launch_ros.actions.Node(
            package='peach_harvester',
            executable='peach_scene_obstacles',
            output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSceneObstacles(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('scene_obstacles_test_harness')
        cls.requests = []
        cls.request_event = threading.Event()
        cls.stop_stream = threading.Event()
        cls.stream_thread = None
        cls.urdf_sent = False
        cls.srv = cls.node.create_service(
            ApplyPlanningScene, '/apply_planning_scene',
            cls._on_apply)
        cls.static_tf = tf2_ros.StaticTransformBroadcaster(cls.node)
        transform = tf2_ros.TransformStamped()
        transform.header.frame_id = BASE_FRAME
        transform.child_frame_id = CLOUD_FRAME
        transform.transform.translation.x = CAM_IN_BASE[0]
        transform.transform.translation.y = CAM_IN_BASE[1]
        transform.transform.translation.z = CAM_IN_BASE[2]
        transform.transform.rotation.w = 1.0
        cls.static_tf.sendTransform(transform)
        cls.pub_cloud = cls.node.create_publisher(
            PointCloud2, '/camera/depth_registered/points',
            _CLOUD_QOS)
        cls.pub_refined = cls.node.create_publisher(
            BagGraspCandidateArray, '/peach/reconstruction/refined_pose',
            _LATCHED)
        cls.pub_refresh = cls.node.create_publisher(
            Empty, '/peach/scene/obstacles_refresh', _LATCHED)
        cls.pub_description = cls.node.create_publisher(
            String, '/robot_description', _LATCHED)
        cls.pub_joints = cls.node.create_publisher(
            JointState, '/joint_states', 10)

    @classmethod
    def _on_apply(cls, request, response):
        cls.requests.append(request.scene)
        cls.request_event.set()
        response.success = True
        return response

    @classmethod
    def tearDownClass(cls):
        cls.stop_stream.set()
        if cls.stream_thread is not None:
            cls.stream_thread.join(timeout=3.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _start_streaming(self, candidates):
        """发 latched 三件套并确保后台流线程在跑（贴近真实相机流，
        跨过 DDS 建联延迟；URDF 只发一次，refined 每次按最新目标集发）."""
        if not self.urdf_sent:
            self.urdf_sent = True
            description = String()
            description.data = _generate_urdf()
            self.pub_description.publish(description)
        refined = BagGraspCandidateArray()
        refined.candidates = list(candidates)
        self.pub_refined.publish(refined)
        if self.stream_thread is not None and self.stream_thread.is_alive():
            return

        def _stream():
            while not self.stop_stream.is_set():
                joints = JointState()
                joints.name = list(_MUST_JOINTS)
                joints.position = [0.0] * len(_MUST_JOINTS)
                self.pub_joints.publish(joints)
                header = Header()
                header.frame_id = CLOUD_FRAME
                header.stamp = self.node.get_clock().now().to_msg()
                points = [
                    _to_cloud(OBSTACLE_POINT), _to_cloud(INSIDE_CAPSULE_1),
                    _to_cloud(ON_ROBOT_SURFACE), _to_cloud(OUT_OF_WORKSPACE)]
                self.pub_cloud.publish(
                    pc2.create_cloud(header, _XYZ_FIELDS, points))
                time.sleep(0.1)
        self.stream_thread = threading.Thread(target=_stream, daemon=True)
        self.stream_thread.start()

    def _wait_request(self, timeout_s: float = 30.0):
        """发触发并轮询服务请求；5s 未达重发一次（节点对未就绪触发丢弃）."""
        self.request_event.clear()
        self.pub_refresh.publish(Empty())
        started = time.monotonic()
        resent = False
        deadline = started + timeout_s
        while time.monotonic() < deadline and not self.request_event.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if not resent and time.monotonic() > started + 5.0:
                self.pub_refresh.publish(Empty())
                resent = True
        self.assertTrue(self.request_event.is_set(), '等待快照写入超时')
        return self.requests[-1]

    def test_01_snapshot_contents(self):
        """首写：障碍点成 box；胶囊内/贴机器人/超半径点全部不进图."""
        self._start_streaming(
            [_candidate('t1', CAPSULE_1_BOTTOM, CAPSULE_1_NECK)])
        scene = self._wait_request()
        objects = scene.world.collision_objects
        self.assertEqual(len(objects), 1)
        obj = objects[0]
        self.assertEqual(obj.id, 'peach_scene_obstacles')
        self.assertEqual(obj.operation, obj.ADD)
        self.assertEqual(obj.header.frame_id, BASE_FRAME)
        centers = [(p.position.x, p.position.y, p.position.z)
                   for p in obj.primitive_poses]
        # 障碍点成 box（0.06 体素内）
        self.assertTrue(any(
            abs(c[0] - OBSTACLE_POINT[0]) < 0.06 and
            abs(c[1] - OBSTACLE_POINT[1]) < 0.06 and
            abs(c[2] - OBSTACLE_POINT[2]) < 0.06 for c in centers),
            f'障碍点未成 box: {centers}')
        # 其余三点均不得出现在任何 box
        for forbidden in (INSIDE_CAPSULE_1, ON_ROBOT_SURFACE,
                          OUT_OF_WORKSPACE):
            self.assertFalse(any(
                abs(c[0] - forbidden[0]) < 0.06 and
                abs(c[1] - forbidden[1]) < 0.06 and
                abs(c[2] - forbidden[2]) < 0.06 for c in centers),
                f'{forbidden} 不应成 box: {centers}')

    def test_02_second_target_rewrites_atomically(self):
        """新目标精化后同帧重写：REMOVE+ADD 原子替换、新邻域无 box."""
        self._start_streaming([
            _candidate('t1', CAPSULE_1_BOTTOM, CAPSULE_1_NECK),
            _candidate('t2', CAPSULE_2_BOTTOM, CAPSULE_2_NECK)])
        scene = self._wait_request()
        objects = scene.world.collision_objects
        self.assertEqual(len(objects), 2, '首写后应 REMOVE+ADD 两条目')
        self.assertEqual(objects[0].operation, objects[0].REMOVE)
        self.assertEqual(objects[1].operation, objects[1].ADD)
        centers = [(p.position.x, p.position.y, p.position.z)
                   for p in objects[1].primitive_poses]
        # 目标 2 轴心不得成 box（滤除集合扩大）
        forbidden = (0.0, 0.4, 1.0)
        self.assertFalse(any(
            abs(c[0] - forbidden[0]) < 0.06 and
            abs(c[1] - forbidden[1]) < 0.06 and
            abs(c[2] - forbidden[2]) < 0.06 for c in centers),
            f'目标2邻域不应成 box: {centers}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """节点进程须干净退出（无未捕获异常）."""

    def test_exit_codes(self, proc_info):
        items = launch_testing.util.resolveProcesses(
            info_obj=proc_info, process=None, cmd_args=None,
            strict_proc_matching=True)
        self.assertTrue(items, '未找到已跟踪进程')
        for item in items:
            info = proc_info[item]
            self.assertIn(
                info.returncode, (0, -2, -9, -15),
                '%s exited %s' % (info.process_name, info.returncode))
