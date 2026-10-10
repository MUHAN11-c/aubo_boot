"""
peach_scene_obstacles 节点（薄壳）：订阅/TF/服务客户端，逻辑全在 core.

数据流：缓存最近一帧点云 + 累积已精化目标胶囊；Survey 完成触发
（/peach/scene/obstacles_refresh，supervisor 发）后在工作线程一次性跑
滤除链并经 ApplyPlanningScene 原子写入快照；新目标精化后同帧重写
（滤除集合扩大、源帧不变=快照稳定）。作业期图冻结，新批次重建。

写域约定（interface_manifest 双写者分域）：本节点只写 world.collision_
objects；peach_arm 只写 ACM diff。两者互不覆盖。
"""
from __future__ import annotations

import threading
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
from peach_interfaces.msg import BagGraspCandidateArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, \
    QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String
import tf2_ros

from .core import (
    build_snapshot,
    CapsuleSpec,
    collect_self_triangles,
    fk_link_transforms,
    ObstacleSnapshot,
    parse_urdf,
    SnapshotParams,
)
from .params import SceneObstaclesParams


def _quaternion_to_matrix(
        x: float, y: float, z: float, w: float) -> np.ndarray:
    """四元数 → 3x3（tf_transformations 未装；标准展开）."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SceneObstaclesNode(Node):
    """场景障碍快照服务（无状态转发，不进 lifecycle 名单）."""

    def __init__(self) -> None:
        """订阅/客户端/缓存初始化；触发经 0.5s 泵转工作线程."""
        super().__init__('peach_scene_obstacles')
        self.params = SceneObstaclesParams.attach(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cloud: PointCloud2 | None = None
        self._capsules: dict[str, CapsuleSpec] = {}
        self._joint_positions: dict[str, float] = {}
        self._urdf_xml: str | None = None
        self._mesh_cache: dict[str, np.ndarray] = {}
        self._first_write = False
        self._rebuild = threading.Event()
        self._worker: threading.Thread | None = None
        self._worker_lock = threading.Lock()
        # 最近一次快照结局（诊断投影；此前只在日志）
        self._last_snapshot_ok: bool | None = None
        self._last_snapshot_mono = 0.0
        self._last_snapshot_detail = ''

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 点云对齐发布端 RELIABLE（percipio default 档；感知链同惯例）：
        # BEST_EFFORT 订户在 FastDDS 大帧实测大量丢包（RViz 正常、BE 探针
        # 0 帧的已知现象，09-29 真相机轮实锤）。快照只取最近一帧，depth 5。
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            PointCloud2, '/camera/depth_registered/points',
            self._on_cloud, cloud_qos)
        self.create_subscription(
            BagGraspCandidateArray, '/peach/reconstruction/refined_pose',
            self._on_refined, latched)
        self.create_subscription(
            Empty, '/peach/scene/obstacles_refresh', self._on_refresh, latched)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched)
        self._apply_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self.create_timer(0.5, self._pump)
        # /diagnostics：快照服务结局此前只在日志（P1 补 L3 通道）
        self._diag = diagnostic_updater.Updater(self, period=5.0)
        self._diag.setHardwareID(self.get_name())
        self._diag.add('obstacle_snapshot', self._diag_snapshot)
        self.get_logger().info(
            'scene obstacles snapshot node ready (waits refresh trigger)')

    def _diag_snapshot(self, stat):
        """快照健康投影：从未触发/最近成败/距今."""
        if self._last_snapshot_ok is None:
            stat.summary(DiagnosticStatus.OK, '尚未触发快照（等待 Survey）')
        elif self._last_snapshot_ok:
            stat.summary(DiagnosticStatus.OK, f'最近快照 {self._last_snapshot_detail}')
        else:
            stat.summary(
                DiagnosticStatus.ERROR,
                f'最近快照失败: {self._last_snapshot_detail}')
        age = (time.monotonic() - self._last_snapshot_mono
               if self._last_snapshot_mono else None)
        stat.add(
            'last_snapshot_age_s', '-' if age is None else f'{age:.1f}')
        stat.add('capsules', str(len(self._capsules)))
        stat.add('cloud_cached', str(self._cloud is not None))
        return stat

    # ------------------------------------------------------------ 缓存回调

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._cloud = msg

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_positions.update(dict(zip(msg.name, msg.position)))

    def _on_robot_description(self, msg: String) -> None:
        self._urdf_xml = msg.data

    def _on_refined(self, msg: BagGraspCandidateArray) -> None:
        """累积已精化目标胶囊；新目标且已有快照 → 同帧重写（滤除扩大）."""
        changed = False
        for item in msg.candidates:
            if not item.target_id or item.bag_diameter_upper_m <= 1.0e-6:
                continue
            capsule = CapsuleSpec(
                bottom=(item.bag_bottom.x, item.bag_bottom.y,
                        item.bag_bottom.z),
                neck=(item.bag_neck.x, item.bag_neck.y, item.bag_neck.z),
                radius=0.5 * float(item.bag_diameter_upper_m))
            if item.target_id not in self._capsules:
                changed = True
            self._capsules[item.target_id] = capsule
        if changed and self._first_write:
            self._rebuild.set()

    def _on_refresh(self, _msg: Empty) -> None:
        self._rebuild.set()

    # ------------------------------------------------------------ 工作线程

    def _pump(self) -> None:
        """0.5s 泵：触发 → 单飞工作线程（Open3D 突发不占 executor）."""
        if not self._rebuild.is_set():
            return
        with self._worker_lock:
            if self._worker is not None and self._worker.is_alive():
                return
            self._rebuild.clear()
            self._worker = threading.Thread(
                target=self._rebuild_worker, daemon=True)
            self._worker.start()

    def _rebuild_worker(self) -> None:
        try:
            self._rebuild_once()
        except Exception as error:  # noqa: BLE001（工作线程兜底：不炸进程）
            import traceback
            self._last_snapshot_ok = False
            self._last_snapshot_detail = f'异常: {error}'
            self.get_logger().error(
                f'障碍快照重建失败: {error}\n{traceback.format_exc()}')

    def _rebuild_once(self) -> None:
        cloud = self._cloud
        urdf_xml = self._urdf_xml
        if cloud is None or urdf_xml is None:
            self.get_logger().warning(
                '障碍快照触发但点云/URDF 未就绪，跳过（等待下次触发）')
            return
        cloud_age = (
            self.get_clock().now() - Time.from_msg(cloud.header.stamp)
        ).nanoseconds * 1.0e-9
        if cloud_age > float(self.params.cloud_max_age_s):
            self.get_logger().warning(
                f'缓存点云帧龄 {cloud_age:.2f}s 超过 '
                f'{self.params.cloud_max_age_s}s，仍按快照使用')
        try:
            xyz = pc2.read_points_numpy(
                cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'点云解码失败: {error}')
            return
        if xyz.size == 0:
            self.get_logger().warning('点云为空，跳过快照')
            return
        base_xyz = self._transform_to_base(
            xyz.reshape(-1, 3), cloud.header.frame_id)
        if base_xyz is None:
            return

        joints, links = parse_urdf(urdf_xml)
        transforms, root_link = fk_link_transforms(
            joints, dict(self._joint_positions))
        self_triangles = collect_self_triangles(
            links, transforms, root_link, str(self.params.base_frame),
            lambda package: get_package_share_directory(package),
            cache=self._mesh_cache)
        if self_triangles.shape[0] == 0:
            self.get_logger().warning(
                'URDF 无可用 collision mesh，快照将不含自身滤除（有假障碍风险）')

        tuning = self.params.tuning()
        snapshot = build_snapshot(
            base_xyz, list(self._capsules.values()), self_triangles,
            SnapshotParams(
                voxel_size_m=tuning.voxel_size_m,
                self_filter_margin_m=tuning.self_filter_margin_m,
                capsule_radial_margin_m=tuning.capsule_radial_margin_m,
                capsule_axial_margin_m=tuning.capsule_axial_margin_m,
                workspace_radius_m=tuning.workspace_radius_m,
                max_boxes=tuning.max_boxes))
        self._apply_snapshot(snapshot)

    def _transform_to_base(
        self, xyz: np.ndarray, cloud_frame: str
    ) -> np.ndarray | None:
        """点云换到 base 系（静态链，零时刻+2s 超时；失败放弃本帧."""
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.params.base_frame), cloud_frame, Time(),
                Duration(seconds=2.0))
        except tf2_ros.TransformException as error:
            self.get_logger().warning(
                f'TF {self.params.base_frame}<-{cloud_frame} 不可用: {error}')
            return None
        tr = transform.transform
        rotation = _quaternion_to_matrix(
            tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
        return xyz @ rotation.T + np.array(
            [tr.translation.x, tr.translation.y, tr.translation.z])

    # ------------------------------------------------------------ 场景写入

    def _apply_snapshot(self, snapshot: ObstacleSnapshot) -> None:
        objects = []
        object_id = str(self.params.object_id)
        base_frame = str(self.params.base_frame)
        if self._first_write:
            # 原子替换：同请求先 REMOVE 再 ADD（不依赖未确认的重复 ADD 语义）
            remove = CollisionObject()
            remove.id = object_id
            remove.header.frame_id = base_frame
            remove.operation = CollisionObject.REMOVE
            objects.append(remove)
        if snapshot.centers.shape[0] > 0:
            add = CollisionObject()
            add.id = object_id
            add.header.frame_id = base_frame
            add.operation = CollisionObject.ADD
            size = float(snapshot.voxel_size_m)
            for center in snapshot.centers:
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [size, size, size]
                pose = Pose()
                pose.orientation.w = 1.0
                pose.position.x = float(center[0])
                pose.position.y = float(center[1])
                pose.position.z = float(center[2])
                add.primitives.append(primitive)
                add.primitive_poses.append(pose)
            objects.append(add)
        if not objects:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        if not self._apply_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                'apply_planning_scene 不可用，本次快照丢弃（下次触发重试）')
            return
        future = self._apply_client.call_async(request)
        # rclpy Future.result() 无 timeout 参数（非 concurrent.futures）：
        # 工作线程里 done() 轮询实现有界等待。
        deadline = time.monotonic() + float(self.params.apply_service_timeout_s)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            self.get_logger().error(
                f'障碍快照写入超时（boxes={snapshot.centers.shape[0]}）')
            return
        response = future.result()
        if response is None or not response.success:
            self._last_snapshot_ok = False
            self._last_snapshot_detail = f'apply 失败（boxes={snapshot.centers.shape[0]}）'
            self.get_logger().error(
                f'障碍快照写入失败（boxes={snapshot.centers.shape[0]}）')
            return
        self._first_write = True
        self._last_snapshot_ok = True
        self._last_snapshot_mono = time.monotonic()
        self._last_snapshot_detail = f'boxes={snapshot.centers.shape[0]}'
        message = (
            f'障碍快照已写入: boxes={snapshot.centers.shape[0]} '
            f'源点={snapshot.source_points} 体素存活={snapshot.kept_points}')
        if snapshot.truncated:
            self.get_logger().warning(
                message + f' 截断于 {self.params.max_boxes} 上限')
        else:
            self.get_logger().info(message)


def main(argv=None) -> None:
    """独立进程入口（console_script peach_scene_obstacles）."""
    rclpy.init(args=argv)
    node = SceneObstaclesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
