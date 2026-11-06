"""
peach_scene_obstacles 节点：订阅、快照调度、TF 与场景写入；几何滤除在 core.

数据流：缓存最近一帧点云 + 累积已精化目标胶囊；Survey 完成触发
（/peach/scene/obstacles_refresh，supervisor 发）后在工作线程一次性跑
滤除链并经 ApplyPlanningScene 原子写入快照；新目标精化后同帧重写
（滤除集合扩大、源帧不变=快照稳定）。作业期图冻结，新批次重建。

写域约定（interface_manifest 双写者分域）：本节点只写 world.collision_
objects；peach_arm 只写 ACM diff。两者互不覆盖。
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import numpy as np
from peach_interfaces.msg import BagGraspCandidateArray
from peach_vision_core.geometry import transform_msg_to_matrix
from peach_vision_core.geometry import transform_points
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from std_msgs.msg import String
import tf2_ros

from .core import CapsuleSpec
from .core import collect_self_triangles
from .core import filter_survey
from .core import ObstacleSnapshot
from .core import parse_urdf
from .core import SnapshotParams
from .core import survey_voxels
from .params import SceneObstaclesParams


@dataclass
class _SurveyFrame:
    """Survey 触发时冻结源帧与参数；后台只需计算一次未截断的体素."""

    cloud: PointCloud2 | None
    base_frame: str
    params: SnapshotParams
    started_ns: int
    voxels: ObstacleSnapshot | None = None


class SceneObstaclesNode(Node):
    """持有 Survey 体素和目标胶囊的场景障碍节点，不进 lifecycle 名单."""

    def __init__(self) -> None:
        """订阅/客户端/缓存初始化；触发经 0.5s 泵转工作线程."""
        super().__init__('peach_scene_obstacles')
        self.params = SceneObstaclesParams.attach(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._cloud: PointCloud2 | None = None
        self._survey: _SurveyFrame | None = None
        self._capsules: dict[str, CapsuleSpec] = {}
        self._urdf_xml: str | None = None
        self._mesh_cache: dict[str, np.ndarray] = {}
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
        self._cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._cloud_sub = None
        self._sync_cloud_subscription()
        self.create_subscription(
            BagGraspCandidateArray, '/peach/reconstruction/refined_pose',
            self._on_refined, latched)
        self.create_subscription(
            Empty, '/peach/scene/obstacles_refresh', self._on_refresh, latched)
        self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched)
        self._apply_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self._get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
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

    def _sync_cloud_subscription(self) -> None:
        """禁用时释放大点云订阅，使相机无消费者时可跳过构云."""
        if self.params.enabled:
            if self._cloud_sub is None:
                self._cloud_sub = self.create_subscription(
                    PointCloud2, '/camera/depth_registered/points',
                    self._on_cloud, self._cloud_qos)
        elif self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)
            self._cloud_sub = None
            self._cloud = None
            self._survey = None
            self._capsules.clear()
            self._rebuild.set()

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._cloud = msg
        survey = self._survey
        if survey is not None and survey.voxels is None and survey.cloud is None:
            survey.cloud = msg

    def _on_robot_description(self, msg: String) -> None:
        self._urdf_xml = msg.data

    def _on_refined(self, msg: BagGraspCandidateArray) -> None:
        """本轮目标的几何改变时重新滤除同一帧."""
        survey = self._survey
        if survey is None or Time.from_msg(msg.header.stamp).nanoseconds < survey.started_ns:
            return
        changed = False
        for item in msg.candidates:
            if (not item.target_id or not item.bag_diameter_upper_m > 1.0e-6
                    or (item.header.frame_id or msg.header.frame_id) != survey.base_frame):
                continue
            capsule = CapsuleSpec(
                bottom=(item.bag_bottom.x, item.bag_bottom.y,
                        item.bag_bottom.z),
                neck=(item.bag_neck.x, item.bag_neck.y, item.bag_neck.z),
                radius=0.5 * float(item.bag_diameter_upper_m))
            if (np.isfinite((*capsule.bottom, *capsule.neck, capsule.radius)).all()
                    and capsule != self._capsules.get(item.target_id)):
                changed = True
                self._capsules[item.target_id] = capsule
        if changed:
            self._rebuild.set()

    def _on_refresh(self, _msg: Empty) -> None:
        self._capsules.clear()
        self._survey = _SurveyFrame(
            self._cloud, str(self.params.base_frame), self.params.tuning(),
            self.get_clock().now().nanoseconds) if self.params.enabled else None
        self._rebuild.set()

    # ------------------------------------------------------------ 工作线程

    def _pump(self) -> None:
        """0.5s 泵：触发 → 单飞工作线程（Open3D 突发不占 executor）."""
        self._sync_cloud_subscription()
        if not self._rebuild.is_set():
            return
        # 刚启用的订阅尚未收到首帧时保留刷新请求，不吞掉本轮 Survey。
        survey = self._survey
        if self.params.enabled:
            if survey is None:
                return
            if survey.voxels is None and (survey.cloud is None or self._urdf_xml is None):
                return
        with self._worker_lock:
            if self._worker is not None and self._worker.is_alive():
                return
            self._rebuild.clear()
            self._worker = threading.Thread(
                target=self._rebuild_worker,
                args=(survey, self._urdf_xml, tuple(self._capsules.values())), daemon=True)
            self._worker.start()

    def _rebuild_worker(self, survey, urdf_xml, capsules) -> None:
        try:
            if survey is None:
                self._remove_obstacles()
                return
            if survey.voxels is None:
                survey.voxels = self._prepare_survey(survey, urdf_xml)
                survey.cloud = None
            snapshot = filter_survey(survey.voxels, capsules, survey.params)
            if self._survey is survey:
                self._apply_snapshot(snapshot, survey.base_frame)
        except Exception as error:  # noqa: BLE001（工作线程兜底：不炸进程）
            self._last_snapshot_ok = False
            self._last_snapshot_detail = str(error)
            self.get_logger().error(f'障碍快照失败: {error}')
        finally:
            self._last_snapshot_mono = time.monotonic()

    def _prepare_survey(self, survey: _SurveyFrame, urdf_xml: str) -> ObstacleSnapshot:
        cloud = survey.cloud
        stamp = Time.from_msg(cloud.header.stamp)
        if stamp.nanoseconds == 0 or not cloud.header.frame_id:
            raise ValueError('点云缺少曝光时间或坐标系')
        cloud_age = (survey.started_ns - stamp.nanoseconds) * 1.0e-9
        if cloud_age > float(self.params.cloud_max_age_s):
            self.get_logger().warning(
                f'缓存点云帧龄 {cloud_age:.2f}s 超过 '
                f'{self.params.cloud_max_age_s}s，仍按快照使用')
        xyz = pc2.read_points_numpy(
            cloud, field_names=['x', 'y', 'z'], skip_nans=True)
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.all():
            xyz = xyz[finite]
        if xyz.size == 0:
            raise ValueError('点云为空')

        def base_transform(frame):
            if frame == survey.base_frame:
                return np.eye(4)
            tf = self._tf_buffer.lookup_transform(
                survey.base_frame, frame, stamp, Duration(seconds=2.0))
            return transform_msg_to_matrix(tf.transform)

        base_xyz = transform_points(xyz.reshape(-1, 3), base_transform(cloud.header.frame_id))
        _, links = parse_urdf(urdf_xml)
        transforms = {link.name: base_transform(link.name) for link in links}
        self_triangles = collect_self_triangles(
            links, transforms, survey.base_frame, survey.base_frame, get_package_share_directory,
            cache=self._mesh_cache)
        if self_triangles.shape[0] == 0:
            self.get_logger().warning(
                'URDF 无可用 collision mesh，快照将不含自身滤除（有假障碍风险）')

        return survey_voxels(base_xyz, self_triangles, survey.params)

    # ------------------------------------------------------------ 场景写入

    def _remove_obstacles(self) -> None:
        """关掉枝叶快照时清掉已写入的碰撞对象，避免旧方块继续挡路."""
        self._write_obstacle(str(self.params.base_frame))
        self._last_snapshot_ok = True
        self._last_snapshot_detail = 'disabled'
        self.get_logger().info('场景障碍已关闭，不把枝叶写入碰撞场景')

    def _apply_snapshot(self, snapshot: ObstacleSnapshot, base_frame: str) -> None:
        self._write_obstacle(base_frame, snapshot)
        self._last_snapshot_ok = True
        self._last_snapshot_detail = f'boxes={snapshot.centers.shape[0]}'
        message = (
            f'障碍快照已写入: boxes={snapshot.centers.shape[0]} '
            f'源点={snapshot.source_points} 体素存活={snapshot.kept_points}')
        if snapshot.truncated:
            self.get_logger().warning(
                message + f' 截断于 {self.params.max_boxes} 上限')
        else:
            self.get_logger().info(message)

    def _world_has_object(self, object_id: str) -> bool:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        response = self._call_scene_service(self._get_scene_client, request)
        return any(obj.id == object_id for obj in response.scene.world.collision_objects)

    def _write_obstacle(self, base_frame: str, snapshot: ObstacleSnapshot | None = None) -> None:
        """ADD 替换同名对象；清场以 MoveIt 世界中实际存在的 id 为准."""
        obj = CollisionObject()
        obj.id = str(self.params.object_id)
        if not obj.id.strip():  # MoveIt 的空 id REMOVE 会清掉整个世界。
            raise ValueError('障碍对象 id 不能为空')
        obj.header.frame_id = base_frame
        if snapshot is None or not snapshot.centers.size:
            if not self._world_has_object(obj.id):
                return
            obj.operation = CollisionObject.REMOVE
        else:
            obj.operation = CollisionObject.ADD
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
                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose)
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call_scene_service(self._apply_client, request)
        if not response.success:
            if obj.operation == CollisionObject.REMOVE and not self._world_has_object(obj.id):
                return
            raise RuntimeError('apply_planning_scene 未确认写入')

    def _call_scene_service(self, client, request):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'{client.srv_name} 不可用')
        future = client.call_async(request)
        deadline = time.monotonic() + float(self.params.apply_service_timeout_s)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            future.cancel()
            raise TimeoutError(f'{client.srv_name} 回执超时')
        response = future.result()
        if response is None:
            raise RuntimeError(f'{client.srv_name} 缺少回执')
        return response


def main(argv=None) -> None:
    """独立进程入口（console_script peach_scene_obstacles）."""
    rclpy.init(args=argv)
    node = SceneObstaclesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGINT may invalidate take_message while spin unwinds. Runtime
        # faults with a live context must still propagate and fail the node.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
