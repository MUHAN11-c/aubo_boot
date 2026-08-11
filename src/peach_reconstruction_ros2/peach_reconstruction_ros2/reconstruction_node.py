"""
连续运动局部重建节点（精确时间 FK + 有界 ICP + 在线 TSDF）.

默认工作流（零服务）：IDLE 时 /peach/perception/initial_pose 有候选即自动
绑定开始；COLLECTING 时每个唯一 RGB-D 时间戳均进入质量门，合格即在线
积分 TSDF；finalize 只提取最终网格并做几何 refit。6 个 Trigger 服务保留
（capture_frame 补拍、remove_last 回滚等语义不变）；capture.auto_mode=false
时使用纯手动服务流。

每帧只按 depth.header.stamp 查 base←camera TF；失败跳帧，禁止运动中使用
latest TF。当前帧先由 FK 变到 base 系，再与已有 TSDF 表面做有界 ICP；
ICP 只修正小刚性误差，越界或低质量帧不进入不可回滚的 TSDF。
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import time
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv_bridge
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Vector3Stamped
import message_filters
import numpy as np
from peach_pose_msgs.msg import (
    BagFitting,
    BagFittingArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
    PeachTargetObservationArray,
)
from peach_pose_ros2.harvest_data import HarvestDataStore
from peach_pose_ros2.peach_pose.depth_geometry import normalize_depth_to_uint16_mm
from peach_reconstruction_ros2.candidate_contract import (
    select_reconstruction_candidate,
    TargetKindMemory,
)
from peach_reconstruction_ros2.captured_frame import CapturedFrame
from peach_reconstruction_ros2.cloud_builder import (
    apply_target_mask,
    CLOUD_BUILDERS,
    pack_rgb_bgr,
)
from peach_reconstruction_ros2.frame_collector import (
    CollectorConfig,
    FRAME_STORES,
    STATE_COLLECTING,
    STATE_IDLE,
    STATE_READY,
)
from peach_reconstruction_ros2.geometry_refiner import (
    GEOMETRY_REFINERS,
    RefitConfig,
    STATUS_ACCEPT,
    STATUS_REJECT,
)
from peach_reconstruction_ros2.icp_refiner import (
    IcpConfig,
    POSE_REFINERS,
    transform_points,
)
from peach_reconstruction_ros2.overlap import (
    assembly_overlap_metrics,
    summarize_pairs_mm,
)
from peach_reconstruction_ros2.params import ReconstructionParams
from peach_reconstruction_ros2.session_io import save_session
from peach_reconstruction_ros2.tf_utils import transform_msg_to_matrix
from peach_reconstruction_ros2.tsdf_volume import LocalTsdf, VOLUME_FUSIONS
from peach_reconstruction_ros2.visualization import (
    build_camera_markers,
    build_mesh_marker,
    build_refined_marker,
)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray


def _xyzrgb_to_cloud_msg(xyz: np.ndarray, colors_bgr,
                         header: Header) -> PointCloud2:
    """
    (N, 3) 点 [m] + (N, 3) uint8 BGR → PointCloud2（xyz + 位打包 rgb 字段）.

    布局与 peach_pose_node._xyzrgb_to_cloud 一致：x/y/z/rgb 各一个 FLOAT32
    （offset 0/4/8/12，point_step=16），rgb 位内容为 0xRRGGBB（RViz RGB8
    上色约定）；colors_bgr 为 None 或长度不符时 rgb 字段补零（黑色），
    空云/启动首发同样保持该字段布局。消息组装用官方
    sensor_msgs_py.point_cloud2.create_cloud（numpy 结构化数组快速路径，
    逐字节布局与旧手写 tobytes 版一致）；唯一覆写 is_dense=True——
    create_cloud 硬编码 False，而本云已剔无效深度恒 dense。
    pack_rgb_bgr 无官方等价物（RViz float32 位打包），保留。

    Args:
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常 [m]）；空给空云.
        colors_bgr: (N, 3) uint8 BGR 颜色（OpenCV 排列）；None 补零.
        header: 输出消息头（frame_id 决定点云坐标系解释）.

    Returns
    -------
        sensor_msgs/PointCloud2（is_dense=True，反投影已剔除无效深度）.

    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = int(pts.shape[0])
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = pts
    if colors_bgr is not None and len(colors_bgr) == n:
        arr[:, 3] = pack_rgb_bgr(colors_bgr)
    msg = create_cloud(header, fields, arr)
    msg.is_dense = True  # create_cloud 硬编码 False；本云恒 dense（无效已剔）
    return msg


class PeachReconstructionNode(Node):
    """连续运动局部重建节点：默认自动收帧，Trigger 服务作人工控制."""

    def __init__(self):
        """建节点：参数层一行装载 → 数据持有者/算法（注册表）→ ROS 接线."""
        super().__init__('peach_reconstruction_node')
        self.bridge = cv_bridge.CvBridge()
        # 参数层：declare + 装载（全部 47 参数在 params.py）
        ReconstructionParams.declare(self)
        self.params = ReconstructionParams.from_node(self)
        p = self.params
        # 派生量（ROS 类型/容器形态转换，非参数副本）
        self.tf_timeout = Duration(seconds=p.tf_timeout_sec)
        self.local_volume = (
            p.local_volume.size_x, p.local_volume.size_y, p.local_volume.size_z)
        self.tsdf_params = {
            'voxel_length': p.tsdf.voxel_length,
            'sdf_trunc': p.tsdf.sdf_trunc,
            'depth_trunc': p.tsdf.depth_trunc,
        }
        self.refit_config = RefitConfig(
            cylinder_inlier_min=p.refit.cylinder_inlier_min,
            rmse_max_m=p.refit.rmse_max_m,
            entry_standoff_m=p.refit.entry_standoff_m)
        self.icp_config = IcpConfig(
            min_points=p.icp.min_points,
            coarse_voxel=p.icp.coarse_voxel,
            fine_voxel=p.icp.fine_voxel,
            coarse_correspondence=p.icp.coarse_correspondence,
            fine_correspondence=p.icp.fine_correspondence,
            coarse_iterations=p.icp.coarse_iterations,
            fine_iterations=p.icp.fine_iterations,
            min_fitness=p.icp.min_fitness,
            max_rmse=p.icp.max_rmse,
            max_translation=p.icp.max_translation,
            max_rotation_deg=p.icp.max_rotation_deg)

        # 数据持有者与算法实现（显式注册表按名实例化，设计文档 §2.2/§2.4）
        self.collector = FRAME_STORES['default'](config=CollectorConfig(
            min_views=p.capture.min_views,
            recommended_views=p.capture.recommended_views,
            max_views=p.capture.max_views,
            min_translation=p.view_filter.min_translation,
            max_translation=p.view_filter.max_translation,
            min_rotation_deg=p.view_filter.min_rotation_deg,
            max_rotation_deg=p.view_filter.max_rotation_deg,
            allow_duplicate_views=p.view_filter.allow_duplicate_views,
            auto_mode=p.capture.auto_mode,
            auto_finalize_at_max=p.capture.auto_finalize_at_max,
            auto_min_interval_s=p.capture.auto_min_interval_s,
        ))
        self._cloud_builder = CLOUD_BUILDERS['open3d']()
        self._geometry_refiner = GEOMETRY_REFINERS['ransac']()
        self._icp_refiner = POSE_REFINERS['open3d_bounded'](self.icp_config)

        # 最新一帧同步 RGB-D 缓存：(rgb, depth_mm, K, stamp_msg, stamp_sec,
        # cam_frame)。只缓存、不直接累积；手动/自动门禁通过后才会入帧栈
        self._latest_frame: Optional[tuple] = None
        self._last_captured_stamp_sec = -1.0  # [s] 上次成功采帧的图像时间戳
        self._latest_candidates: Optional[BagGraspCandidateArray] = None
        self._preferred_target_id = ''
        self._harvest_run_id = ''
        self._target_observation_seen = False
        self._target_masks = {}
        self._harvest_data = HarvestDataStore()
        self._joint_states_seen = False
        self._max_joint_vel = 0.0  # [rad/s] 最近 /joint_states 的最大关节速度
        self._last_tf_latency_ms: Optional[float] = None  # 最近一次 TF 查询墙钟耗时
        # finalize 时计算的 overlap 指标缓存（帧栈变动即失效置 None）
        self._overlap_cache: Optional[dict] = None
        # TSDF 云缓存：(xyz, colors_bgr) 或 None；finalize 时重建，帧栈变动失效
        self._tsdf_cloud_cache: Optional[tuple] = None
        self._tsdf_info: Optional[dict] = None  # diagnostics 的 tsdf 键内容
        self._tsdf_volume = None  # 每轮 session 持续在线积分
        self._mesh_cache: Optional[dict] = None
        self._registration_history = []
        # refit：感知 diagnostics 的 target_id→target_kind 映射；
        # _refined_result=refine_geometry 结果（None=未跑/失败），
        # _refined_info=diagnostics JSON 的 refined 键内容
        self._target_kind_memory = TargetKindMemory()
        self._refined_result: Optional[dict] = None
        self._refined_info: Optional[dict] = None

        # ---- 发布者（/peach/reconstruction/* 固定命名）----
        # 状态类话题用 transient_local 闩锁（depth=1）：后启动的订阅者
        # （验证记录器 / RViz）也能拿到最后一次发布；发布频率低，闩锁代价可忽略
        latched_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/peach/reconstruction/local_cloud', latched_qos)
        self.pub_status = self.create_publisher(
            String, '/peach/reconstruction/status', latched_qos)
        self.pub_diag = self.create_publisher(
            String, '/peach/reconstruction/diagnostics', latched_qos)
        self.pub_grasp_decision = self.create_publisher(
            String, '/peach/reconstruction/grasp_decision', latched_qos)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/peach/reconstruction/markers', latched_qos)
        self.pub_tsdf_cloud = self.create_publisher(
            PointCloud2, '/peach/reconstruction/tsdf_cloud', latched_qos)
        # refit 输出三件套（同为 transient_local 闩锁 + base_frame）
        self.pub_refined_pose = self.create_publisher(
            BagGraspCandidateArray, '/peach/reconstruction/refined_pose',
            latched_qos)
        self.pub_refined_axis = self.create_publisher(
            Vector3Stamped, '/peach/reconstruction/refined_axis', latched_qos)
        self.pub_refined_diag = self.create_publisher(
            BagFittingArray, '/peach/reconstruction/refined_diagnostics',
            latched_qos)

        # ---- 订阅：RGB-D 三件套（RELIABLE，与回放/驱动对齐）----
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        sub_rgb = message_filters.Subscriber(
            self, Image, self.params.camera.color_topic, qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, self.params.camera.depth_topic, qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, self.params.camera.camera_info_topic, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=self.params.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)
        self.create_subscription(
            BagGraspCandidateArray, '/peach/perception/initial_pose',
            self._on_initial_pose, 10)
        self.create_subscription(
            PeachTargetObservationArray,
            '/peach/perception/target_observations',
            self._on_target_observations, 10)
        # 感知诊断（target_id→target_kind）：refit 选圆柱/球拟合线的依据
        self.create_subscription(
            BagFittingArray, '/peach/perception/diagnostics',
            self._on_perception_diagnostics, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        # ---- 服务（std_srvs/Trigger，节点相对名）----
        self.create_service(
            Trigger, '~/start_reconstruction', self._on_start)
        self.create_service(Trigger, '~/capture_frame', self._on_capture)
        self.create_service(
            Trigger, '~/remove_last_frame', self._on_remove_last)
        self.create_service(
            Trigger, '~/reset_reconstruction', self._on_reset)
        self.create_service(
            Trigger, '~/finalize_reconstruction', self._on_finalize)
        self.create_service(Trigger, '~/save_session', self._on_save_session)
        self.create_service(
            Trigger, '~/query_reconstruction_state',
            self._on_query_reconstruction_state)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 启动即首发一次（IDLE + 空云），闩锁话题让后启动的订阅者立即可读
        self._publish_all()

        self.get_logger().info(
            f'peach_reconstruction_node ready: '
            f'base={self.params.frames.base_frame} '
            f'color={self.params.camera.color_topic} '
            f'depth={self.params.camera.depth_topic} '
            f'slop={self.params.sync_slop_s}s '
            f'depth_scale_unit={self.params.depth_scale_unit} '
            f'views(min/rec/max)={self.params.capture.min_views}/'
            f'{self.params.capture.recommended_views}/'
            f'{self.params.capture.max_views} '
            f'require_static={self.params.capture.require_robot_static} '
            f'auto_mode={self.params.capture.auto_mode} '
            f'session_root={self._session_root()}')

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """
        同步回调：归一化深度后**只缓存最新一帧**（绝不自动累积）.

        Args:
            rgb_msg: 彩色图（bgr8）.
            depth_msg: 深度图（uint16 原始值或 32FC1 米制）.
            info: 彩色相机内参.

        Returns
        -------
            无返回值（None）；缓存写 self._latest_frame.

        """
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'图像解码失败，丢帧: {exc}')
            return
        # uint16：raw × depth_scale_unit = 毫米；32FC1：米 ×1000 = 毫米
        try:
            depth_mm = normalize_depth_to_uint16_mm(depth_raw, self.params.depth_scale_unit)
        except ValueError as exc:
            self.get_logger().warning(f'深度归一化失败，丢帧: {exc}')
            return
        if rgb.shape[:2] != depth_mm.shape[:2]:
            self.get_logger().warning(
                f'RGB/深度分辨率不一致 {rgb.shape[:2]} vs {depth_mm.shape[:2]}，丢帧')
            return
        K = {
            'fx': float(info.k[0]), 'fy': float(info.k[4]),
            'cx': float(info.k[2]), 'cy': float(info.k[5]),
            'width': int(depth_mm.shape[1]), 'height': int(depth_mm.shape[0]),
        }
        intrinsic_values = np.array(
            [K['fx'], K['fy'], K['cx'], K['cy']], dtype=np.float64)
        if (not np.all(np.isfinite(intrinsic_values))
                or K['fx'] <= 0.0 or K['fy'] <= 0.0):
            self.get_logger().warning('相机内参含非有限值或 fx/fy≤0，丢帧')
            return
        # TF 查询按深度图时间戳（相机 HW 时间戳，常超前机器人 TF）
        stamp_msg = depth_msg.header.stamp
        stamp_sec = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        cam_frame = depth_msg.header.frame_id or rgb_msg.header.frame_id
        self._latest_frame = (rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame)
        # 自动模式：每个新同步帧驱动一次（自动开始/采帧/完成）；
        # auto_mode=false 时不走这里，改用纯手动 Trigger 服务流
        if self.params.capture.auto_mode:
            self._auto_drive()

    def _on_initial_pose(self, msg: BagGraspCandidateArray):
        """缓存最新感知候选（启动重建时绑定最优目标用）."""
        self._latest_candidates = msg

    def _on_target_observations(
            self, msg: PeachTargetObservationArray) -> None:
        """缓存全局计划选中 ID 的精确深度时刻掩膜与当前中心."""
        self._target_observation_seen = True
        if msg.harvest_run_id != self._harvest_run_id:
            self._harvest_run_id = msg.harvest_run_id
            self._harvest_data.attach(self._harvest_run_id)
            self._harvest_data.append_event({
                'source': 'reconstruction',
                'event': 'reconstruction_linked'})
        requested_target_id = msg.selected_target_id
        if (self._preferred_target_id
                and requested_target_id != self._preferred_target_id):
            if self.collector.state == STATE_READY:
                self.collector.reset()
                self._target_kind_memory.reset()
                self._last_captured_stamp_sec = -1.0
                self._reset_products(create_volume=False)
                self._target_masks.clear()
            elif self.collector.state != STATE_IDLE:
                self.get_logger().warning(
                    '重建尚未 READY，拒绝切换 selected_target_id')
                return
        self._preferred_target_id = requested_target_id
        selected = next((item for item in msg.observations
                         if item.target_id == msg.selected_target_id), None)
        if selected is None or selected.tracking_status != selected.OBSERVED:
            return
        if not selected.mask.data:
            return
        try:
            mask = self.bridge.imgmsg_to_cv2(
                selected.mask, desired_encoding='mono8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'目标掩膜解码失败: {exc}')
            return
        stamp = selected.mask.header.stamp
        stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
        bottom = np.array([
            selected.candidate.bag_bottom.x,
            selected.candidate.bag_bottom.y,
            selected.candidate.bag_bottom.z], dtype=np.float64)
        neck = np.array([
            selected.candidate.bag_neck.x,
            selected.candidate.bag_neck.y,
            selected.candidate.bag_neck.z], dtype=np.float64)
        center = 0.5 * (bottom + neck)
        if not np.all(np.isfinite(center)) or not np.any(center):
            center = None
        self._target_masks[stamp_ns] = (
            np.asarray(mask, dtype=np.uint8), center)
        while len(self._target_masks) > 30:
            self._target_masks.pop(next(iter(self._target_masks)))
        if self.params.capture.auto_mode:
            self._auto_drive()

    def _target_mask_for_frame(self, stamp_msg, depth_mm):
        """取严格同时间戳掩膜，并执行小目标、深度空洞和风动质量门."""
        if not self.params.capture.require_target_mask:
            return None, ''
        stamp_ns = int(stamp_msg.sec) * 1000000000 + int(stamp_msg.nanosec)
        entry = self._target_masks.get(stamp_ns)
        if entry is None:
            return None, '缺少所选 target_id 的同时间戳掩膜'
        mask, center = entry
        pixels = int(np.count_nonzero(mask))
        if pixels < self.params.capture.min_mask_pixels:
            return None, (
                f'目标掩膜仅 {pixels} 像素 < '
                f'{self.params.capture.min_mask_pixels}')
        try:
            _masked, ratio = apply_target_mask(depth_mm, mask)
        except ValueError as exc:
            return None, str(exc)
        if ratio < self.params.capture.min_mask_depth_ratio:
            return None, (
                f'掩膜内有效深度占比 {ratio:.2f} < '
                f'{self.params.capture.min_mask_depth_ratio:.2f}')
        bound = self.collector.target_center
        if center is not None and bound is not None:
            drift = float(np.linalg.norm(center - np.asarray(bound)))
            if drift > self.params.capture.max_target_drift_m:
                return None, (
                    f'目标漂移 {drift * 1000.0:.1f} mm > '
                    f'{self.params.capture.max_target_drift_m * 1000.0:.1f} mm')
        return mask, ''

    def _on_query_reconstruction_state(self, request, response):
        """~/query_reconstruction_state：返回当前重建和数据关联 JSON."""
        del request
        response.success = True
        response.message = json.dumps(
            self._diagnostics(), ensure_ascii=False)
        return response

    def _on_perception_diagnostics(self, msg: BagFittingArray):
        """缓存 target_id→target_kind 映射（refit 选圆柱/球拟合线用）."""
        self._target_kind_memory.update(msg.fittings)

    def _resolve_target_kind(self) -> Tuple[str, bool]:
        """
        按已绑定 target_id 查感知 diagnostics 的 target_kind.

        Returns
        -------
            (kind, defaulted)：kind ∈ {'bag', 'fruit'}（'fruit' 以外一律按
            'bag' 圆柱线，与感知包 `target_kind or 'bag'` 语义一致）；
            defaulted=True 表示未查到，缺省袋桃（结果记
            target_kind_defaulted 标记）.

        """
        return self._target_kind_memory.resolve()

    def _on_joint_states(self, msg: JointState):
        """缓存最大关节速度幅值 [rad/s]（require_robot_static 判定用）."""
        self._joint_states_seen = True
        self._max_joint_vel = max((abs(float(v)) for v in msg.velocity), default=0.0)

    # ------------------------------------------------------------------
    # TF
    # ------------------------------------------------------------------
    def _lookup_T_base_camera(self, cam_frame: str,
                              stamp) -> Tuple[Optional[np.ndarray], str]:
        """
        按图像时间戳精确查询 base←camera 4×4 矩阵.

        timeout 只用于等待相应时刻的机器人 TF 到达。连续运动中禁止回退
        latest TF，因为错时位姿会在 TSDF 中形成不可回滚的双层表面。

        Args:
            cam_frame: 相机光学系 frame_id（取深度图 header.frame_id）.
            stamp: 查询时刻（builtin Time 消息）.

        Returns
        -------
            (T, status)：status ∈ {'ok', 'unavailable'}；
            副作用：刷新 _last_tf_latency_ms（查询墙钟耗时 [ms]）.

        """
        if not cam_frame or cam_frame == self.params.frames.base_frame:
            return np.eye(4), 'ok'
        t0 = time.perf_counter()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.params.frames.base_frame, cam_frame, Time.from_msg(stamp),
                timeout=self.tf_timeout)
            self._last_tf_latency_ms = (time.perf_counter() - t0) * 1000.0
            return transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException as ex:
            self._last_tf_latency_ms = (time.perf_counter() - t0) * 1000.0
            self.get_logger().warning(
                f'TF {self.params.frames.base_frame}←{cam_frame} 在图像时刻'
                f'不可用，本帧跳过: {ex}')
            return None, 'unavailable'

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------
    def _reset_products(self, create_volume: bool) -> None:
        """清空本轮派生结果；开始新轮时同时创建一个空在线 TSDF."""
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        self._mesh_cache = None
        self._registration_history = []
        self._refined_result = None
        self._refined_info = None
        self._tsdf_volume = None
        if create_volume and self.params.tsdf.enable:
            self._tsdf_volume = VOLUME_FUSIONS['open3d_scalable'](
                **self.tsdf_params)

    def _roi_center(self):
        """返回局部体素盒中心；候选缺失时退到首帧局部云质心."""
        center = self.collector.target_center
        if center is None and self.collector.frames:
            first = self.collector.frames[0].cloud_base
            if first is not None and len(first):
                center = np.asarray(first).mean(axis=0)
        return center

    def _refresh_tsdf_outputs(self, extract_mesh: bool = False) -> None:
        """从当前在线体积刷新局部点云；finalize 时额外提取网格."""
        if self._tsdf_volume is None:
            self._tsdf_cloud_cache = None
            self._mesh_cache = None
            return
        xyz, colors = self._tsdf_volume.extract_cloud()
        center = self._roi_center()
        if center is not None and xyz.size:
            xyz, colors = LocalTsdf.crop_to_box(
                xyz, colors, center, self.local_volume)
        if self.params.cloud_filter.voxel_size > 0.0:
            xyz, colors = LocalTsdf.voxel_downsample(
                xyz, colors, self.params.cloud_filter.voxel_size)
        if self.params.cloud_filter.enable_statistical_filter:
            xyz, colors = LocalTsdf.statistical_filter(xyz, colors)
        self._tsdf_cloud_cache = (xyz, colors)
        if extract_mesh:
            self._mesh_cache = self._tsdf_volume.extract_mesh(
                center=center, size_xyz=self.local_volume)
        self._tsdf_info = {
            'points': int(xyz.shape[0]),
            'integrated_frames': len(self.collector.frames),
            'integrate_time_s': float(self._tsdf_volume.integrate_time_s),
            'voxel_length': self.tsdf_params['voxel_length'],
            'sdf_trunc': self.tsdf_params['sdf_trunc'],
            'mesh_vertices': int(
                0 if self._mesh_cache is None
                else len(self._mesh_cache['vertices'])),
            'roi_center': (None if center is None
                           else [float(v) for v in center]),
        }

    def _deny(self, response, message: str, count_reject: bool = True):
        """
        统一拒帧/拒绝出口：写响应、计数、刷新状态话题.

        Args:
            response: Trigger 响应（被原地填写）.
            message: 拒绝原因（中文，回给调用方）.
            count_reject: 是否计 rejected_views（TF 失败单独计 tf_failures）.

        Returns
        -------
            填写后的 response.

        """
        response.success = False
        response.message = message
        self.get_logger().warning(f'拒绝：{message}')
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_rejected',
            'target_id': self.collector.target_id, 'reason': message,
        })
        if count_reject:
            self.collector.rejected_views += 1
        self._publish_all()
        return response

    def _best_candidate(self) -> Tuple[str, Optional[np.ndarray]]:
        """取全局计划选中且坐标系与 TF 诊断均安全的候选."""
        if (self.params.capture.require_target_mask
                and not self._preferred_target_id):
            return '', None
        return select_reconstruction_candidate(
            self._latest_candidates, self.params.frames.base_frame,
            self._preferred_target_id)

    def _on_start(self, request, response):
        """~/start_reconstruction：清空帧栈，绑定当前最优候选，→ COLLECTING."""
        del request
        target_id, center = self._best_candidate()
        response.message = self.collector.start(target_id, center)
        self._target_kind_memory.bind(target_id)
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=True)
        response.success = True
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    def _on_capture(self, request, response):
        """~/capture_frame：过全部门禁后把当前缓存帧采入帧栈并重发累加云."""
        del request
        if self.collector.state != STATE_COLLECTING:
            return self._deny(
                response,
                f'当前状态 {self.collector.state}，先 ~/start_reconstruction',
                count_reject=False)
        if len(self.collector.frames) >= self.params.capture.max_views:
            return self._deny(
                response,
                f'已达 max_views={self.params.capture.max_views}，请 finalize 或 remove_last')
        cached = self._latest_frame
        if cached is None:
            return self._deny(response, '尚无同步 RGB-D 帧（确认相机/回放在线）')
        rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame = cached
        target_mask, mask_reason = self._target_mask_for_frame(
            stamp_msg, depth_mm)
        if mask_reason:
            return self._deny(response, mask_reason)
        if stamp_sec <= self._last_captured_stamp_sec:
            return self._deny(response, '缓存帧未更新（与上次采帧同帧），请等下一帧')
        age_s = self.get_clock().now().nanoseconds / 1e9 - stamp_sec
        if age_s > self.params.capture.max_frame_age_s:
            return self._deny(
                response,
                f'缓存帧龄期 {age_s:.2f} s > max_frame_age_s='
                f'{self.params.capture.max_frame_age_s}（陈帧拒采）')
        if self.params.capture.require_robot_static:
            if not self._joint_states_seen:
                return self._deny(
                    response, 'require_robot_static=true 但未收到 /joint_states')
            if self._max_joint_vel > self.params.capture.static_joint_vel_thresh:
                return self._deny(
                    response,
                    f'机器人未静止：最大关节速度 {self._max_joint_vel:.4f} rad/s '
                    f'> {self.params.capture.static_joint_vel_thresh}')
        if not cam_frame:
            return self._deny(
                response, '深度图 header.frame_id 为空，无法查 TF', count_reject=False)
        T_base_camera, tf_status = self._lookup_T_base_camera(cam_frame, stamp_msg)
        if T_base_camera is None:
            self.collector.tf_failures += 1
            return self._deny(
                response,
                f'TF {self.params.frames.base_frame}←{cam_frame} 查询失败（已计 tf_failures）',
                count_reject=False)
        ok, reason, trans, rot = self.collector.check_view(T_base_camera)
        if not ok:
            return self._deny(response, reason)
        if reason == 'duplicate_allowed':
            self.get_logger().warning(
                f'重复视角仍采帧（allow_duplicate_views=true）：'
                f'平移 {trans * 1000.0:.1f} mm / 旋转 {rot:.1f} deg')
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask=target_mask)
        if not accepted:
            return self._deny(response, message)
        response.success = True
        response.message = message
        return response

    def _accept_frame(self, rgb, depth_mm, K, stamp_sec: float,
                      T_base_camera, tf_status: str,
                      target_mask=None) -> Tuple[bool, str]:
        """
        FK 构云 → 有界帧到模型 ICP → 入库并立即积分 TSDF.

        T_base_camera 是精确图像时刻的 FK/手眼位姿；ICP 只估计相对它的
        小修正。ICP 与 FK 预对齐都不合格时拒帧，避免污染在线体积。

        Args:
            rgb: (H, W, 3) uint8 BGR 彩图.
            depth_mm: (H, W) uint16 深度 [mm].
            K: 内参 dict.
            stamp_sec: 图像时间戳 [s].
            T_base_camera: (4, 4) 精确时间戳的 base←camera FK 位姿.
            tf_status: TF 查询状态；本实现只接受 'ok'.

        Returns
        -------
            (accepted, message).

        """
        try:
            cloud_fk, cloud_rgb, ratio = self._cloud_builder.build(
                depth_mm, rgb, K, T_base_camera,
                target_mask=target_mask)
        except (RuntimeError, ValueError) as exc:
            return False, f'点云构建失败: {exc}'
        if tf_status != 'ok':
            return False, '非精确时间 TF 帧禁止进入 TSDF'
        masked_depth, _mask_ratio = apply_target_mask(
            depth_mm, target_mask)

        # ICP 只看目标局部表面，避免桌面/枝叶等背景主导刚体修正。
        center = self._roi_center()
        if center is None:
            center = self.collector.target_center
        if center is not None and cloud_fk.size:
            cloud_fk, cloud_rgb = LocalTsdf.crop_to_box(
                cloud_fk, cloud_rgb, center, self.local_volume)
        target = (np.zeros((0, 3), dtype=np.float64)
                  if self._tsdf_cloud_cache is None
                  else self._tsdf_cloud_cache[0])

        if self.params.icp.enable:
            registration = self._icp_refiner.refine(cloud_fk, target)
            if not registration.accepted:
                return False, (
                    f'配准拒帧：{registration.reason}，'
                    f'fitness={registration.fitness:.3f} '
                    f'rmse={registration.rmse * 1000.0:.1f}mm，'
                    f'修正={registration.translation_m * 1000.0:.1f}mm/'
                    f'{registration.rotation_deg:.2f}deg')
            correction = registration.correction
            mode = registration.mode
            reg_info = {
                'mode': mode,
                'reason': registration.reason,
                'fitness': float(registration.fitness),
                'rmse_m': float(registration.rmse),
                'translation_m': float(registration.translation_m),
                'rotation_deg': float(registration.rotation_deg),
            }
        else:
            correction = np.eye(4, dtype=np.float64)
            mode = 'fk'
            reg_info = {
                'mode': mode, 'reason': 'icp_disabled',
                'fitness': -1.0, 'rmse_m': -1.0,
                'translation_m': 0.0, 'rotation_deg': 0.0,
            }
        T_used = correction @ np.asarray(T_base_camera, dtype=np.float64)
        cloud_base = transform_points(cloud_fk, correction)
        flags = [f'pose_{mode}']
        if reg_info['reason'] not in ('accepted', 'model_warmup',
                                      'icp_disabled'):
            flags.append(reg_info['reason'])
        frame = CapturedFrame(
            rgb=rgb, depth_mm=masked_depth, camera_K=K, stamp=stamp_sec,
            T_base_camera=T_used, T_base_camera_fk=T_base_camera,
            target_id=self.collector.target_id,
            valid_depth_ratio=ratio, cloud_base=cloud_base,
            cloud_rgb=cloud_rgb, diagnostic_flags=flags,
            registration=reg_info)
        if not self.collector.add_frame(frame):
            return False, f'已达 max_views={self.params.capture.max_views}'

        if self.params.tsdf.enable:
            try:
                if self._tsdf_volume is None:
                    self._tsdf_volume = VOLUME_FUSIONS['open3d_scalable'](
                        **self.tsdf_params)
                self._tsdf_volume.integrate_frame(
                    rgb, masked_depth, K, T_used)
                self._refresh_tsdf_outputs()
            except Exception as exc:  # noqa: BLE001
                # ScalableTSDF 不支持移除单帧；重建空体积并重放此前已确认帧，
                # 确保失败帧绝不残留。
                self.collector.remove_last()
                self._tsdf_volume = VOLUME_FUSIONS['open3d_scalable'](
                    **self.tsdf_params)
                for old in self.collector.frames:
                    self._tsdf_volume.integrate_frame(
                        old.rgb, old.depth_mm, old.camera_K,
                        old.T_base_camera)
                self._refresh_tsdf_outputs()
                return False, f'TSDF 在线积分失败: {exc}'

        self._last_captured_stamp_sec = stamp_sec
        self._registration_history.append(reg_info)
        # 新帧使 finalize 指标与几何精化失效；在线 TSDF 缓存已在上面刷新。
        self._overlap_cache = None
        self._mesh_cache = None
        self._refined_result = None
        self._refined_info = None
        n = len(self.collector.frames)
        message = (
            f'已采第 {n}/{self.params.capture.recommended_views} 视角，'
            f'本帧 {cloud_base.shape[0]} 点，有效深度占比 {ratio:.2f}，'
            f'位姿={mode}')
        self.get_logger().info(message)
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_accepted',
            'target_id': self.collector.target_id,
            'stamp_ns': int(round(stamp_sec * 1000000000.0)),
            'view_index': n, 'mask_depth_ratio': float(ratio),
            'registration': reg_info,
        })
        self._publish_all()
        return True, message

    def _on_remove_last(self, request, response):
        """~/remove_last_frame：弹帧后重放剩余帧，保证在线 TSDF 一致."""
        del request
        removed = self.collector.remove_last()
        if removed is None:
            response.success = False
            response.message = '帧栈为空，无可移除帧'
            self.get_logger().warning(response.message)
        else:
            response.success = True
            response.message = (
                f'已移除最后一帧，剩余 {len(self.collector.frames)} 视角')
            self._overlap_cache = None
            if self.params.tsdf.enable:
                self._tsdf_volume = VOLUME_FUSIONS['open3d_scalable'](
                    **self.tsdf_params)
                for old in self.collector.frames:
                    self._tsdf_volume.integrate_frame(
                        old.rgb, old.depth_mm, old.camera_K,
                        old.T_base_camera)
                self._refresh_tsdf_outputs()
            else:
                self._tsdf_cloud_cache = None
                self._tsdf_info = None
            self._mesh_cache = None
            self._registration_history = [
                dict(f.registration) for f in self.collector.frames]
            self._refined_result = None
            self._refined_info = None
            self.get_logger().info(response.message)
        self._publish_all()
        return response

    def _on_reset(self, request, response):
        """~/reset_reconstruction：清空帧栈与绑定目标，回 IDLE."""
        del request
        self.collector.reset()
        self._target_kind_memory.reset()
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=False)
        response.success = True
        response.message = '已清空，回 IDLE'
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    def _finalize_now(self) -> Tuple[bool, str]:
        """
        共享 finalize：状态迁移、重叠指标、最终网格与几何精化.

        Returns
        -------
            (ok, message)；ok=False 时保持 COLLECTING.

        """
        ok, message, _cloud = self.collector.finalize()
        if ok:
            # 刚性对齐量化指标：相邻帧最近邻统计 + 质心（并入诊断 JSON）
            self._overlap_cache = assembly_overlap_metrics(self.collector.frames)
            summary = summarize_pairs_mm(self._overlap_cache['pairs'])
            if summary is None:
                message += '；重叠指标需 ≥2 帧，本批次不可用'
            else:
                message += (f'；重叠 mean={summary["mean_mm"]:.1f}mm '
                            f'p95={summary["p95_mm"]:.1f}mm')
            if self.params.tsdf.enable:
                message += self._run_tsdf()
                # 在线 TSDF 最终提取后接几何精化。
                if self.params.refit.enable:
                    message += self._run_refit()
            self.get_logger().info(message)
            self._harvest_data.append_event({
                'source': 'reconstruction', 'event': 'reconstruction_finalized',
                'target_id': self.collector.target_id,
                'captured_views': len(self.collector.frames),
                'refined': self._refined_info,
                'grasp_decision': self._grasp_decision(),
            })
        else:
            self._overlap_cache = None
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self._refined_result = None
            self._refined_info = None
            self.get_logger().warning(message)
        # 累加云由 _publish_all 统一重发（frame_id=base_frame）
        self._publish_all()
        return ok, message

    def _run_tsdf(self) -> str:
        """
        从在线 TSDF 提取最终点云和三角网格.

        每帧已在 _accept_frame 中完成积分；此处禁止再次批量积分，只做
        ROI 点云后处理与 Open3D marching-cubes 网格提取。

        Returns
        -------
            追加到 finalize message 的片段（如 '；TSDF 123456 点'）.

        """
        try:
            self._refresh_tsdf_outputs(extract_mesh=True)
            xyz = self._tsdf_cloud_cache[0]
            mesh_vertices = (
                0 if self._mesh_cache is None
                else len(self._mesh_cache['vertices']))
        except Exception as exc:  # noqa: BLE001
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self._mesh_cache = None
            self.get_logger().error(f'TSDF 最终提取失败（不影响 finalize）: {exc}')
            return f'；TSDF 提取失败（{exc}）'
        return (f'；TSDF {xyz.shape[0]} 点'
                f' / mesh {mesh_vertices} 顶点'
                f'（累计积分 {self._tsdf_volume.integrate_time_s:.2f}s）')

    def _run_refit(self) -> str:
        """
        REFINING 阶段：对 TSDF 云做几何二次拟合（finalize 成功后调用一次）.

        链：tsdf_cloud（xyz，base_frame，米）→ 按绑定 target_id 查感知
        diagnostics 得 target_kind（查不到缺省袋桃并记
        target_kind_defaulted）→ refine_geometry（圆柱/球 RANSAC +
        bottom→neck 消歧 + ACCEPT/REOBSERVE 门控）。结果缓存供
        refined_pose/refined_axis/refined_diagnostics 闩锁重发与
        save_session 摘要；失败只告警不污染 finalize（diagnostics 记
        refined.ok=false 与原因，refined_pose 发空数组覆盖闩锁防陈旧）。

        Returns
        -------
            追加到 finalize message 的片段（如 '；refit ACCEPT（...）'）.

        """
        if self._tsdf_cloud_cache is None or not self._tsdf_cloud_cache[0].size:
            self._refined_result = None
            self._refined_info = {'ok': False, 'reason': 'no_tsdf_cloud'}
            self.get_logger().warning('REFINING：无 TSDF 云，refit 跳过')
            return '；refit 跳过（无 TSDF 云）'
        xyz = self._tsdf_cloud_cache[0]
        kind, defaulted = self._resolve_target_kind()
        self.get_logger().info(
            f'REFINING：几何二次拟合开始（kind={kind}，{xyz.shape[0]} 点）')
        try:
            result = self._geometry_refiner.refine(xyz, kind, self.refit_config)
        except Exception as exc:  # noqa: BLE001
            self._refined_result = None
            self._refined_info = {'ok': False, 'reason': f'exception:{exc}'}
            self.get_logger().warning(f'refit 异常（不影响 finalize）: {exc}')
            return f'；refit 失败（{exc}）'
        if defaulted:
            result['flags'].append('target_kind_defaulted')
        if not result['ok']:
            self._refined_result = None
            self._refined_info = {
                'ok': False, 'reason': result['reason'],
                'kind': kind, 'n_points': result['n_points']}
            self.get_logger().warning(
                f'refit 失败（不影响 finalize）：{result["reason"]}')
            return f'；refit 失败（{result["reason"]}）'
        self._refined_result = result
        self._refined_info = self._refined_diag_dict(result)
        status_text = ('ACCEPT' if result['status'] == STATUS_ACCEPT
                       else 'REOBSERVE')
        self.get_logger().info(
            f"REFINING 完成：{result['kind']} status={status_text} "
            f"axis={np.round(result['axis'], 4).tolist()} "
            f"diameter={result['diameter'] * 1000.0:.1f}mm "
            f"rmse={result['rmse'] * 1000.0:.2f}mm "
            f"inlier={result['inlier_ratio']:.2f}")
        return (f"；refit {status_text}（{result['kind']}，"
                f"rmse {result['rmse'] * 1000.0:.1f}mm，"
                f"inlier {result['inlier_ratio']:.2f}）")

    @staticmethod
    def _refined_diag_dict(result: dict) -> dict:
        """
        组装 diagnostics JSON 的 refined 键（refit 成功结果，numpy→原生类型）.

        Args:
            result: refine_geometry 的 ok=True 结果.

        Returns
        -------
            JSON 可序列化 dict（kind/center/axis/diameter/rmse/
            inlier_ratio/ok 等）.

        """
        return {
            'ok': True,
            'kind': result['kind'],
            'status': int(result['status']),
            'center': [float(v) for v in result['center']],
            'axis': [float(v) for v in result['axis']],
            'bottom': [float(v) for v in result['bottom']],
            'neck': [float(v) for v in result['neck']],
            'diameter': float(result['diameter']),
            'span_m': float(result['span_m']),
            'rmse': float(result['rmse']),
            'inlier_ratio': float(result['inlier_ratio']),
            'n_points': int(result['n_points']),
            'flags': list(result['flags']),
        }

    def _on_finalize(self, request, response):
        """~/finalize_reconstruction：视角数达标则拼接全部帧发 local_cloud."""
        del request
        ok, message = self._finalize_now()
        response.success = ok
        response.message = message
        return response

    def _on_save_session(self, request, response):
        """~/save_session：全部已采帧落盘 session_<时间戳>/（含参数快照）."""
        del request
        frames = self.collector.frames
        if not frames:
            response.success = False
            response.message = '无已采帧，未落盘'
            self.get_logger().warning(response.message)
            self._publish_all()
            return response
        try:
            session_dir = save_session(
                self._session_root(), frames, self._session_metadata(),
                tsdf_cloud=self._tsdf_cloud_cache,
                tsdf_mesh=self._mesh_cache)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'落盘失败: {exc}'
            self.get_logger().error(response.message)
            self._publish_all()
            return response
        response.success = True
        response.message = f'已保存 {len(frames)} 帧到 {session_dir}'
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'session_saved',
            'target_id': self.collector.target_id,
            'session_dir': str(session_dir),
        })
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    # ------------------------------------------------------------------
    # 自动模式（决策纯逻辑在 FrameCollector，这里只做 TF/订阅接线）
    # ------------------------------------------------------------------
    def _auto_drive(self):
        """
        自动模式驱动：每个新同步帧回调末尾调用一次.

        流程：IDLE 且有候选 → 自动开始；COLLECTING → 满 max_views 自动
        finalize，否则尝试自动采帧；READY/FAILED 停采，等 reset/start 进
        下一轮。所有"不行"都只对当前帧跳过/告警，不打断流程。
        """
        if self.collector.state == STATE_IDLE and \
                self.collector.should_auto_start():
            self._auto_start()
        if self.collector.state == STATE_COLLECTING:
            if self.collector.should_auto_finalize():
                ok, message = self._finalize_now()
                if ok:
                    self.get_logger().info(f'自动完成：{message}')
            else:
                self._try_auto_capture()

    def _auto_start(self):
        """自动开始：绑定当前最优候选进 COLLECTING；无候选静默等待."""
        target_id, center = self._best_candidate()
        if not target_id:
            return  # 无候选：静默等待（initial_pose 到位后自然触发）
        message = self.collector.start(target_id, center)
        self._target_kind_memory.bind(target_id)
        self._last_captured_stamp_sec = -1.0
        self._reset_products(create_volume=True)
        self.get_logger().info(f'自动开始：{message}')
        self._publish_all()

    def _try_auto_capture(self):
        """自动采帧：帧新鲜度/静止/TF/视角决策全过才建云入库."""
        if len(self.collector.frames) >= self.params.capture.max_views:
            return  # 满栈后静默等待 finalize，避免每帧重复构云/ICP和刷屏
        cached = self._latest_frame
        if cached is None:
            return
        rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame = cached
        target_mask, mask_reason = self._target_mask_for_frame(
            stamp_msg, depth_mm)
        if mask_reason:
            self.get_logger().debug(f'自动采帧跳过：{mask_reason}')
            return
        if stamp_sec <= self._last_captured_stamp_sec:
            return  # 本帧已采过（等下一帧回调）
        age_s = self.get_clock().now().nanoseconds / 1e9 - stamp_sec
        if age_s > self.params.capture.max_frame_age_s:
            return  # 陈帧跳过（自动模式不记 rejected_views）
        if self.params.capture.require_robot_static:
            if not self._joint_states_seen:
                return
            if self._max_joint_vel > self.params.capture.static_joint_vel_thresh:
                self.get_logger().debug(
                    f'自动采帧跳过：关节速度 {self._max_joint_vel:.4f} rad/s 超阈')
                return
        if not cam_frame:
            return
        T_base_camera, tf_status = self._lookup_T_base_camera(
            cam_frame, stamp_msg)
        if T_base_camera is None:
            self.collector.tf_failures += 1
            return  # TF 失败跳过本帧，不打断流程
        if self._last_captured_stamp_sec > 0.0:
            since_last = stamp_sec - self._last_captured_stamp_sec
        else:
            since_last = float('inf')  # 首帧不受间隔门限制
        action, reason = self.collector.auto_capture_decision(
            T_base_camera, since_last)
        if action == 'skip':
            self.get_logger().debug(f'自动采帧跳过：{reason}')
            return
        if action == 'warn_capture':
            self.get_logger().warning(f'自动采帧：{reason}')
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask=target_mask)
        if not accepted:
            self.collector.rejected_views += 1
            self.get_logger().warning(f'自动采帧未入库：{message}')

    # ------------------------------------------------------------------
    # 发布与诊断
    # ------------------------------------------------------------------
    def _publish_all(self):
        """状态变化后统一重发：累加云 + 状态 + 诊断 JSON + 相机轨迹 Marker."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.params.frames.base_frame
        cloud = self.collector.accumulated_cloud()
        self.pub_cloud.publish(_xyzrgb_to_cloud_msg(
            cloud, self.collector.accumulated_rgb(), header))
        # TSDF 云仅 finalize 后非空；无缓存发空云（字段布局保持一致）
        if self._tsdf_cloud_cache is not None:
            tsdf_xyz, tsdf_rgb = self._tsdf_cloud_cache
        else:
            tsdf_xyz, tsdf_rgb = np.zeros((0, 3)), None
        self.pub_tsdf_cloud.publish(
            _xyzrgb_to_cloud_msg(tsdf_xyz, tsdf_rgb, header))
        self.pub_status.publish(String(data=self.collector.state))
        self.pub_diag.publish(
            String(data=json.dumps(self._diagnostics(), ensure_ascii=False)))
        self.pub_grasp_decision.publish(String(
            data=json.dumps(self._grasp_decision(), ensure_ascii=False)))
        markers = build_camera_markers(header, self.collector.frames)
        refined_arrow = build_refined_marker(header, self._refined_result)
        if refined_arrow is not None:
            markers.markers.append(refined_arrow)
        mesh_marker = build_mesh_marker(header, self._mesh_cache)
        if mesh_marker is not None:
            markers.markers.append(mesh_marker)
        self.pub_markers.publish(markers)
        # refit 三件套：闩锁话题每次重发（无结果发空消息，防陈旧数据）
        pose_arr, axis_msg, fit_arr = self._refined_messages(header)
        self.pub_refined_pose.publish(pose_arr)
        self.pub_refined_axis.publish(axis_msg)
        self.pub_refined_diag.publish(fit_arr)

    def _refined_messages(self, header: Header):
        """
        由 refit 缓存组 refined 三话题消息（闩锁重发/清空共用）.

        无结果（未跑 finalize/refit 关闭）→ 全空消息；拟合成功 →
        pose/axis/diagnostics 按结果填充（status=ACCEPT/REOBSERVE 照常
        发布）；拟合失败（REJECT）→ pose 发空数组、axis 发零向量（无效
        占位），diagnostics 发 status=REJECT 单条记录（标量 -1）——闩锁
        话题必须发消息覆盖，防后启动订阅者读到上一轮陈旧结果。

        Args:
            header: 输出头（frame_id=base_frame）.

        Returns
        -------
            (BagGraspCandidateArray, Vector3Stamped, BagFittingArray).

        """
        pose_arr = BagGraspCandidateArray()
        pose_arr.header = header
        axis_msg = Vector3Stamped()
        axis_msg.header = header
        fit_arr = BagFittingArray()
        fit_arr.header = header
        result = self._refined_result
        if result is not None and result['ok']:
            cand = BagGraspCandidate()
            cand.header = header
            cand.target_id = self.collector.target_id
            # entry = bottom − axis×standoff（几何在 refiner 内算好）；
            # 姿态未用（接近方向由 translation_direction 给出），置单位四元数
            cand.entry_pose = Pose(
                position=Point(x=float(result['entry'][0]),
                               y=float(result['entry'][1]),
                               z=float(result['entry'][2])),
                orientation=Quaternion(w=1.0))
            cand.bag_bottom = Point(x=float(result['bottom'][0]),
                                    y=float(result['bottom'][1]),
                                    z=float(result['bottom'][2]))
            cand.bag_neck = Point(x=float(result['neck'][0]),
                                  y=float(result['neck'][1]),
                                  z=float(result['neck'][2]))
            # 剪切行进方向 = refined 轴（bottom→neck 单位向量）
            cand.translation_direction = Vector3(x=float(result['axis'][0]),
                                                 y=float(result['axis'][1]),
                                                 z=float(result['axis'][2]))
            # 圆柱为袋径、球为果径（均 = 2r）
            cand.bag_diameter_upper_m = float(result['diameter'])
            cand.suggested_travel_m = float(result['span_m'])
            cand.confidence = float(result['inlier_ratio'])
            cand.status = int(result['status'])
            cand.diagnostic_flags = list(result['flags'])
            cand.strategy_id = f"reconstruction_refit_{result['kind']}"
            pose_arr.candidates.append(cand)
            axis_msg.vector = Vector3(x=float(result['axis'][0]),
                                      y=float(result['axis'][1]),
                                      z=float(result['axis'][2]))
        fit = self._refined_fitting_msg(header, result)
        if fit is not None:
            fit_arr.fittings.append(fit)
        return pose_arr, axis_msg, fit_arr

    def _refined_fitting_msg(self, header: Header,
                             result: Optional[dict]) -> Optional[BagFitting]:
        """
        由 refit 结果组 BagFitting（无效标量 -1，语义对齐感知包 _to_fitting）.

        Args:
            header: 输出头.
            result: refine_geometry 结果；None 表示未跑或失败（失败时
                由 _refined_info 取原因，发 REJECT 记录）.

        Returns
        -------
            peach_pose_msgs/BagFitting；从未跑过 refit 给 None.

        """
        info = self._refined_info
        if result is None and not info:
            return None
        m = BagFitting()
        m.header = header
        m.target_id = self.collector.target_id
        m.axis_source = 'reconstruction_refit'
        # 全部标量先置 -1（无效约定），再按拟合线逐项覆盖有效字段
        for attr in ('axis_confidence', 'axis_disagreement_deg', 'theta_err_deg',
                     'error_budget_mm', 'radial_clearance_mm', 'valid_depth_ratio',
                     'foreground_ratio', 'boundary_touch_ratio', 'bag_length_m',
                     'bag_diameter_upper_m', 'travel_m', 'cylinder_rms_m',
                     'cylinder_inlier_ratio', 'fruit_radius_m', 'sphere_rms_m',
                     'sphere_inlier_ratio', 'cavity_dip_mm'):
            setattr(m, attr, -1.0)
        m.boundary_sides_touched = -1
        m.n_points = -1
        if result is None or not result['ok']:
            info = info or {}
            m.target_kind = str(info.get('kind', ''))
            m.status = STATUS_REJECT  # 拟合失败不发 pose/axis，仅留诊断记录
            m.diagnostic_flags = ['refit_failed',
                                  str(info.get('reason', 'unknown'))]
            return m
        m.target_kind = 'fruit' if result['kind'] == 'sphere' else 'bag'
        m.n_points = int(result['n_points'])
        m.bag_diameter_upper_m = float(result['diameter'])
        m.travel_m = float(result['span_m'])
        if result['kind'] == 'cylinder':
            m.bag_length_m = float(result['span_m'])
            m.cylinder_rms_m = float(result['rmse'])
            m.cylinder_inlier_ratio = float(result['inlier_ratio'])
        else:
            m.fruit_radius_m = float(result['radius'])
            m.sphere_rms_m = float(result['rmse'])
            m.sphere_inlier_ratio = float(result['inlier_ratio'])
        m.status = int(result['status'])
        m.diagnostic_flags = list(result['flags'])
        return m

    def _grasp_decision(self) -> dict:
        """把最终精化质量归一成只读抓取许可，不发送运动指令."""
        decision = {
            'harvest_run_id': self._harvest_run_id,
            'target_id': self.collector.target_id,
            'allowed': False,
            'reason': 'reconstruction_not_ready',
        }
        if self.collector.state != 'READY':
            return decision
        result = self._refined_result
        if result is None or not result.get('ok'):
            decision['reason'] = 'refined_geometry_unavailable'
            return decision
        if int(result.get('status', STATUS_REJECT)) != STATUS_ACCEPT:
            decision['reason'] = 'refined_quality_requires_reobserve'
            return decision
        decision.update({
            'allowed': True,
            'reason': 'refined_geometry_accept',
            'entry': [float(v) for v in result['entry']],
            'axis': [float(v) for v in result['axis']],
            'diameter_m': float(result['diameter']),
            'rmse_m': float(result['rmse']),
            'inlier_ratio': float(result['inlier_ratio']),
        })
        return decision

    def _diagnostics(self) -> dict:
        """组装诊断 dict（随 /peach/reconstruction/diagnostics 以 JSON 发出）."""
        c = self.collector
        cloud = c.accumulated_cloud()
        last_ratio = c.frames[-1].valid_depth_ratio if c.frames else None
        return {
            'harvest_run_id': self._harvest_run_id,
            'selected_target_id': self._preferred_target_id,
            'target_mask_cache_size': len(self._target_masks),
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'captured_views': len(c.frames),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'tf_latency_ms': self._last_tf_latency_ms,
            'valid_depth_ratio': last_ratio,
            'cloud_points': int(cloud.shape[0]),
            'last_rel_translation_m': c.last_rel_translation_m,
            'last_rel_rotation_deg': c.last_rel_rotation_deg,
            # finalize 时的重叠度指标（pairs/质心）；未 finalize 或帧栈已变为 None
            'overlap': self._overlap_cache,
            # finalize 时的 TSDF 摘要（points/integrate_time_s/roi_center 等）
            'tsdf': self._tsdf_info,
            'registration': {
                'accepted': len(self._registration_history),
                'latest': (None if not self._registration_history
                           else self._registration_history[-1]),
            },
            # refit 摘要（kind/center/axis/diameter/rmse/inlier_ratio/ok）；
            # 未跑为 None，失败为 {'ok': False, 'reason': ...}
            'refined': self._refined_info,
            'grasp_decision': self._grasp_decision(),
        }

    def _session_root(self) -> Path:
        """解析 session 落盘根目录：参数显式给优先，缺省 <工作区>/peach_sessions."""
        if self.params.session.root_dir:
            return Path(self.params.session.root_dir)
        # install/<pkg>/share/<pkg> → parents[3] 即工作区根
        share = Path(get_package_share_directory('peach_reconstruction_ros2'))
        return share.parents[3] / 'peach_sessions'

    def _session_metadata(self) -> dict:
        """参数快照与帧级摘要（随 metadata.yaml 落盘，供离线复现）."""
        c = self.collector
        return {
            'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'node': 'peach_reconstruction_node',
            'pipeline': 'exact-time FK + bounded ICP + online TSDF + refit',
            'harvest_run_id': self._harvest_run_id,
            'selected_target_id': self._preferred_target_id,
            'target_mask_cache_size': len(self._target_masks),
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'captured_views': len(c.frames),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'parameters': {
                'frames.base_frame': self.params.frames.base_frame,
                'sync_slop_s': self.params.sync_slop_s,
                'tf_timeout_sec': self.params.tf_timeout_sec,
                'depth_scale_unit': self.params.depth_scale_unit,
                'capture.min_views': self.params.capture.min_views,
                'capture.recommended_views': self.params.capture.recommended_views,
                'capture.max_views': self.params.capture.max_views,
                'capture.require_robot_static': self.params.capture.require_robot_static,
                'capture.static_joint_vel_thresh': self.params.capture.static_joint_vel_thresh,
                'capture.max_frame_age_s': self.params.capture.max_frame_age_s,
                'view_filter.min_translation': self.params.view_filter.min_translation,
                'view_filter.max_translation': self.params.view_filter.max_translation,
                'view_filter.min_rotation_deg': self.params.view_filter.min_rotation_deg,
                'view_filter.max_rotation_deg': self.params.view_filter.max_rotation_deg,
                'view_filter.allow_duplicate_views': self.params.view_filter.allow_duplicate_views,
                'icp.enable': self.params.icp.enable,
                'icp.min_points': self.icp_config.min_points,
                'icp.coarse_voxel': self.icp_config.coarse_voxel,
                'icp.fine_voxel': self.icp_config.fine_voxel,
                'icp.coarse_correspondence':
                    self.icp_config.coarse_correspondence,
                'icp.fine_correspondence':
                    self.icp_config.fine_correspondence,
                'icp.min_fitness': self.icp_config.min_fitness,
                'icp.max_rmse': self.icp_config.max_rmse,
                'icp.max_translation': self.icp_config.max_translation,
                'icp.max_rotation_deg': self.icp_config.max_rotation_deg,
                'local_volume.size_x': self.local_volume[0],
                'local_volume.size_y': self.local_volume[1],
                'local_volume.size_z': self.local_volume[2],
                'tsdf.enable': self.params.tsdf.enable,
                'tsdf.voxel_length': self.tsdf_params['voxel_length'],
                'tsdf.sdf_trunc': self.tsdf_params['sdf_trunc'],
                'tsdf.depth_trunc': self.tsdf_params['depth_trunc'],
                'cloud_filter.voxel_size': self.params.cloud_filter.voxel_size,
                'cloud_filter.enable_statistical_filter':
                    self.params.cloud_filter.enable_statistical_filter,
                'refit.enable': self.params.refit.enable,
                'refit.cylinder_inlier_min':
                    self.refit_config.cylinder_inlier_min,
                'refit.rmse_max_m': self.refit_config.rmse_max_m,
                'refit.entry_standoff_m': self.refit_config.entry_standoff_m,
            },
            'tsdf_result': self._tsdf_info,
            'refined_result': self._refined_info,
            'frames': [{
                'index': i,
                'stamp_sec': float(f.stamp),
                'valid_depth_ratio': float(f.valid_depth_ratio),
                'camera_position_base': [float(v)
                                         for v in f.camera_position_base],
                'cloud_points': int(0 if f.cloud_base is None
                                    else f.cloud_base.shape[0]),
                'diagnostic_flags': list(f.diagnostic_flags),
                'T_base_camera_fk': np.asarray(
                    f.T_base_camera_fk, dtype=np.float64).tolist(),
                'T_base_camera_used': np.asarray(
                    f.T_base_camera, dtype=np.float64).tolist(),
                'registration': dict(f.registration),
            } for i, f in enumerate(c.frames)],
        }


def main(args=None):
    """
    节点入口：rclpy 初始化 → PeachReconstructionNode spin → 干净收尾.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = PeachReconstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
