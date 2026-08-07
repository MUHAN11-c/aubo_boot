"""
多视角局部重建节点（自动模式默认开；无 TSDF、无 refit）.

默认工作流（零服务）：IDLE 时 /peach/perception/initial_pose 有候选即自动
绑定开始；COLLECTING 时随视角变化（平移/旋转达阈）自动采帧；采满
max_views 自动 finalize 出 overlap 并置 READY；reset/start 进入下一轮。
6 个 Trigger 服务全部保留作手动备用（capture_frame 补拍、remove_last
回滚等语义不变）；capture.auto_mode=false 回 Phase 2 纯手动服务流。

每帧按 depth.header.stamp 查 base←camera TF（相机 HW 时间戳常超前机器人
TF，按 stamp 失败回退一次最新 TF 并打 tf_stale 标记，彻底失败在自动模式
下只跳过本帧）。TSDF 融合与几何 refit 属 Phase 4/5，本阶段刻意不做。
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import time
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv_bridge
import message_filters
import numpy as np
from peach_pose_msgs.msg import BagGraspCandidateArray
from peach_pose_ros2.peach_pose.depth_geometry import normalize_depth_to_uint16_mm
from peach_reconstruction_ros2.captured_frame import CapturedFrame
from peach_reconstruction_ros2.cloud_builder import (
    build_cloud_base,
    pack_rgb_bgr,
)
from peach_reconstruction_ros2.frame_collector import (
    CollectorConfig,
    FrameCollector,
    STATE_COLLECTING,
    STATE_IDLE,
)
from peach_reconstruction_ros2.overlap import (
    assembly_overlap_metrics,
    summarize_pairs_mm,
)
from peach_reconstruction_ros2.session_io import save_session
from peach_reconstruction_ros2.tf_utils import transform_msg_to_matrix
from peach_reconstruction_ros2.tsdf_volume import LocalTsdf
from peach_reconstruction_ros2.visualization import build_camera_markers
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

# /peach/perception/initial_pose 候选消息的状态枚举（与 peach_pose_msgs 一致）
_STATUS_ACCEPT = 0


def _xyzrgb_to_cloud_msg(xyz: np.ndarray, colors_bgr,
                         header: Header) -> PointCloud2:
    """
    (N, 3) 点 [m] + (N, 3) uint8 BGR → PointCloud2（xyz + 位打包 rgb 字段）.

    布局与 peach_pose_node._xyzrgb_to_cloud 一致：x/y/z/rgb 各一个 FLOAT32
    （offset 0/4/8/12，point_step=16），rgb 位内容为 0xRRGGBB（RViz RGB8
    上色约定）；colors_bgr 为 None 或长度不符时 rgb 字段补零（黑色），
    空云/启动首发同样保持该字段布局。走 numpy tobytes 快速路径：
    不逐点建 python 列表，几十万点也是 ms 级。

    Args:
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常 [m]）；空给空云.
        colors_bgr: (N, 3) uint8 BGR 颜色（OpenCV 排列）；None 补零.
        header: 输出消息头（frame_id 决定点云坐标系解释）.

    Returns
    -------
        sensor_msgs/PointCloud2（is_dense=True，反投影已剔除无效深度）.

    """
    msg = PointCloud2()
    msg.header = header
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.is_dense = True
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = int(pts.shape[0])
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = pts
    if colors_bgr is not None and len(colors_bgr) == n:
        arr[:, 3] = pack_rgb_bgr(colors_bgr)
    msg.height = 1
    msg.width = n
    msg.row_step = msg.point_step * msg.width
    msg.data = arr.tobytes()
    return msg


class PeachReconstructionNode(Node):
    """多视角局部重建节点：默认全自动（零服务），Trigger 服务留作手动备用."""

    def __init__(self):
        """建节点：参数 → 发布/订阅/服务 → TF 监听."""
        super().__init__('peach_reconstruction_node')
        self.bridge = cv_bridge.CvBridge()
        self._declare_params()
        self._load_params()

        self.collector = FrameCollector(config=CollectorConfig(
            min_views=self.cfg_min_views,
            recommended_views=self.cfg_recommended_views,
            max_views=self.cfg_max_views,
            min_translation=self.vf_min_translation,
            max_translation=self.vf_max_translation,
            min_rotation_deg=self.vf_min_rotation_deg,
            max_rotation_deg=self.vf_max_rotation_deg,
            allow_duplicate_views=self.vf_allow_duplicate,
            auto_mode=self.cfg_auto_mode,
            auto_finalize_at_max=self.cfg_auto_finalize_at_max,
            auto_min_interval_s=self.cfg_auto_min_interval_s,
        ))

        # 最新一帧同步 RGB-D 缓存：(rgb, depth_mm, K, stamp_msg, stamp_sec,
        # cam_frame)。只缓存、绝不自动累积；累积只发生在 ~/capture_frame 里
        self._latest_frame: Optional[tuple] = None
        self._last_captured_stamp_sec = -1.0  # [s] 上次成功采帧的图像时间戳
        self._latest_candidates: Optional[BagGraspCandidateArray] = None
        self._joint_states_seen = False
        self._max_joint_vel = 0.0  # [rad/s] 最近 /joint_states 的最大关节速度
        self._last_tf_latency_ms: Optional[float] = None  # 最近一次 TF 查询墙钟耗时
        self._tf_warned = False
        # finalize 时计算的 overlap 指标缓存（帧栈变动即失效置 None）
        self._overlap_cache: Optional[dict] = None
        # TSDF 云缓存：(xyz, colors_bgr) 或 None；finalize 时重建，帧栈变动失效
        self._tsdf_cloud_cache: Optional[tuple] = None
        self._tsdf_info: Optional[dict] = None  # diagnostics 的 tsdf 键内容

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
        self.pub_markers = self.create_publisher(
            MarkerArray, '/peach/reconstruction/markers', latched_qos)
        self.pub_tsdf_cloud = self.create_publisher(
            PointCloud2, '/peach/reconstruction/tsdf_cloud', latched_qos)

        # ---- 订阅：RGB-D 三件套（RELIABLE，与回放/驱动对齐）----
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        sub_rgb = message_filters.Subscriber(
            self, Image, self.color_topic, qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, self.depth_topic, qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic, qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=self.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)
        self.create_subscription(
            BagGraspCandidateArray, '/peach/perception/initial_pose',
            self._on_initial_pose, 10)
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 启动即首发一次（IDLE + 空云），闩锁话题让后启动的订阅者立即可读
        self._publish_all()

        self.get_logger().info(
            f'peach_reconstruction_node ready: base={self.base_frame} '
            f'color={self.color_topic} depth={self.depth_topic} '
            f'slop={self.sync_slop_s}s depth_scale_unit={self.depth_scale_unit} '
            f'views(min/rec/max)={self.cfg_min_views}/'
            f'{self.cfg_recommended_views}/{self.cfg_max_views} '
            f'require_static={self.cfg_require_static} '
            f'auto_mode={self.cfg_auto_mode} '
            f'session_root={self._session_root()}')

    def _declare_params(self):
        """声明 ROS 参数默认值（与 config/reconstruction.yaml 逐项对齐）."""
        defaults = {
            'frames.base_frame': 'base_link',
            'camera.color_topic': '/camera/color/image_raw',
            'camera.depth_topic': '/camera/depth/image_raw',
            'camera.camera_info_topic': '/camera/color/camera_info',
            'sync_slop_s': 0.05,
            'tf_timeout_sec': 0.5,
            # Percipio 原始深度常需 ×0.25 才是毫米量级；32FC1 米制不生效
            'depth_scale_unit': 0.25,
            'capture.min_views': 4,
            'capture.recommended_views': 5,
            'capture.max_views': 8,
            'capture.require_robot_static': False,
            'capture.static_joint_vel_thresh': 0.01,
            'capture.max_frame_age_s': 2.0,
            # 自动模式：默认开，零服务全自动；false 回 Phase 2 纯手动
            'capture.auto_mode': True,
            'capture.auto_finalize_at_max': True,
            'capture.auto_min_interval_s': 2.0,
            'view_filter.min_translation': 0.020,
            'view_filter.max_translation': 0.080,
            'view_filter.min_rotation_deg': 5.0,
            'view_filter.max_rotation_deg': 25.0,
            'view_filter.allow_duplicate_views': False,
            'local_volume.size_x': 0.30,
            'local_volume.size_y': 0.30,
            'local_volume.size_z': 0.40,
            # TSDF（Phase 4 启用：finalize 时批量积分 + 后处理）
            'tsdf.enable': True,
            'tsdf.voxel_length': 0.003,
            'tsdf.sdf_trunc': 0.012,
            'tsdf.depth_trunc': 1.5,
            # 提取云后处理：体素降采样 + 统计离群剔除
            'cloud_filter.voxel_size': 0.003,
            'cloud_filter.enable_statistical_filter': True,
            'session.root_dir': '',
        }
        descriptions = {
            'frames.base_frame': '重建输出坐标系（点云/Marker 的 frame_id）',
            'camera.color_topic': '彩色图话题（bgr8）',
            'camera.depth_topic': '深度图话题（uint16 或 32FC1，须与彩图对齐）',
            'camera.camera_info_topic': '彩色相机内参话题',
            'sync_slop_s': 'RGB-D 近似同步允差 (s)',
            'tf_timeout_sec': '单次 TF 查询超时 (s)；按 depth.header.stamp 查，'
                              '失败回退一次最新 TF（打 tf_stale），再失败拒帧',
            'depth_scale_unit': '深度比例因子（仅 uint16 原始深度生效）：raw × 本值 = '
                                '毫米（Percipio 常见 0.25）；数据集回放设 1.0；'
                                '32FC1 浮点深度按「米」×1000 转毫米，本参数不生效',
            'capture.min_views': 'finalize 所需最少视角数',
            'capture.recommended_views': '推荐视角数（不足仅提示，不阻塞 finalize）',
            'capture.max_views': '帧栈上限（达到后拒采，先 remove_last 或 finalize）',
            'capture.require_robot_static': '采帧是否要求机器人静止（查 /joint_states）',
            'capture.static_joint_vel_thresh': '静止判定：最大关节速度阈值 [rad/s]',
            'capture.max_frame_age_s': '缓存帧龄期上限 (s)，超过视为陈帧拒采',
            'capture.auto_mode': '自动模式总开关：true=有候选自动开始、随视角'
                                 '变化自动采帧、采满自动完成（零服务）；'
                                 'false=纯手动 Trigger 服务流（Phase 2 行为）',
            'capture.auto_finalize_at_max': '采满 max_views 自动 finalize'
                                            '（仅 auto_mode=true 时生效）',
            'capture.auto_min_interval_s': '两次自动采帧最小间隔 (s)，'
                                           '防低帧率下抖动重采',
            'view_filter.min_translation': '与上一已采帧的最小平移 [m]（过近=重复视角）',
            'view_filter.max_translation': '与上一已采帧的最大平移 [m]（过远=跳变）',
            'view_filter.min_rotation_deg': '与上一已采帧的最小旋转 [deg]',
            'view_filter.max_rotation_deg': '与上一已采帧的最大旋转 [deg]',
            'view_filter.allow_duplicate_views': 'true 时重复视角仅告警仍采帧',
            'local_volume.size_x': '局部体素盒 X 尺寸 [m]（TSDF 云 ROI 裁剪）',
            'local_volume.size_y': '局部体素盒 Y 尺寸 [m]（TSDF 云 ROI 裁剪）',
            'local_volume.size_z': '局部体素盒 Z 尺寸 [m]（TSDF 云 ROI 裁剪）',
            'tsdf.enable': 'TSDF 融合开关：true=finalize 时批量积分全部已采帧'
                           '并发布 /peach/reconstruction/tsdf_cloud',
            'tsdf.voxel_length': 'TSDF 体素边长 [m]',
            'tsdf.sdf_trunc': 'TSDF 截断距离 [m]',
            'tsdf.depth_trunc': 'TSDF 深度截断 [m]（更远的深度不积分）',
            'cloud_filter.voxel_size': 'TSDF 提取云体素降采样边长 [m]（≤0 不降）',
            'cloud_filter.enable_statistical_filter': 'TSDF 提取云统计离群剔除'
                                                      '（20 邻域 2σ）',
            'session.root_dir': 'session 落盘根目录；空 = <工作区>/peach_sessions'
                                '（按包 share 路径反推工作区根）',
        }
        for k, v in defaults.items():
            self.declare_parameter(
                k, v, ParameterDescriptor(description=descriptions[k]))

    def _load_params(self):
        """从参数服务器读出并缓存为实例属性."""
        g = self.get_parameter
        self.base_frame = g(
            'frames.base_frame').get_parameter_value().string_value.strip()
        self.color_topic = g('camera.color_topic').get_parameter_value().string_value
        self.depth_topic = g('camera.depth_topic').get_parameter_value().string_value
        self.camera_info_topic = g(
            'camera.camera_info_topic').get_parameter_value().string_value
        self.sync_slop_s = float(g('sync_slop_s').value)
        self.tf_timeout = Duration(seconds=float(g('tf_timeout_sec').value))
        self.depth_scale_unit = float(g('depth_scale_unit').value)
        self.cfg_min_views = int(g('capture.min_views').value)
        self.cfg_recommended_views = int(g('capture.recommended_views').value)
        self.cfg_max_views = int(g('capture.max_views').value)
        self.cfg_require_static = bool(g('capture.require_robot_static').value)
        self.cfg_static_vel_thresh = float(g('capture.static_joint_vel_thresh').value)
        self.cfg_max_frame_age_s = float(g('capture.max_frame_age_s').value)
        self.cfg_auto_mode = bool(g('capture.auto_mode').value)
        self.cfg_auto_finalize_at_max = bool(
            g('capture.auto_finalize_at_max').value)
        self.cfg_auto_min_interval_s = float(
            g('capture.auto_min_interval_s').value)
        self.vf_min_translation = float(g('view_filter.min_translation').value)
        self.vf_max_translation = float(g('view_filter.max_translation').value)
        self.vf_min_rotation_deg = float(g('view_filter.min_rotation_deg').value)
        self.vf_max_rotation_deg = float(g('view_filter.max_rotation_deg').value)
        self.vf_allow_duplicate = bool(g('view_filter.allow_duplicate_views').value)
        self.local_volume = (
            float(g('local_volume.size_x').value),
            float(g('local_volume.size_y').value),
            float(g('local_volume.size_z').value))
        self.cfg_tsdf_enable = bool(g('tsdf.enable').value)
        self.tsdf_params = {
            'voxel_length': float(g('tsdf.voxel_length').value),
            'sdf_trunc': float(g('tsdf.sdf_trunc').value),
            'depth_trunc': float(g('tsdf.depth_trunc').value),
        }
        self.cfg_cloud_voxel_size = float(g('cloud_filter.voxel_size').value)
        self.cfg_cloud_stat_filter = bool(
            g('cloud_filter.enable_statistical_filter').value)
        self.session_root_dir = g(
            'session.root_dir').get_parameter_value().string_value.strip()

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
            depth_mm = normalize_depth_to_uint16_mm(depth_raw, self.depth_scale_unit)
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
        # TF 查询按深度图时间戳（相机 HW 时间戳，常超前机器人 TF）
        stamp_msg = depth_msg.header.stamp
        stamp_sec = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        cam_frame = depth_msg.header.frame_id or rgb_msg.header.frame_id
        self._latest_frame = (rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame)
        # 自动模式：每个新同步帧驱动一次（自动开始/采帧/完成）；
        # auto_mode=false 时不走这里，行为与 Phase 2 纯手动服务流一致
        if self.cfg_auto_mode:
            self._auto_drive()

    def _on_initial_pose(self, msg: BagGraspCandidateArray):
        """缓存最新感知候选（启动重建时绑定最优目标用）."""
        self._latest_candidates = msg

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
        查 base←camera 4×4 矩阵（按图像 stamp；失败回退一次最新 TF）.

        相机 HW 时间戳常超前机器人 TF：按 stamp 失败时给一次 latest 回退，
        命中返回 'stale'（采帧结果打 tf_stale 标记，与 peach_pose 语义一致）；
        彻底失败返回 (None, 'unavailable')，由调用方拒帧并计 tf_failures。

        Args:
            cam_frame: 相机光学系 frame_id（取深度图 header.frame_id）.
            stamp: 查询时刻（builtin Time 消息）.

        Returns
        -------
            (T, status)：status ∈ {'ok', 'stale', 'unavailable'}；
            副作用：刷新 _last_tf_latency_ms（查询墙钟耗时 [ms]）.

        """
        if not cam_frame or cam_frame == self.base_frame:
            return np.eye(4), 'ok'
        t0 = time.perf_counter()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cam_frame, Time.from_msg(stamp),
                timeout=self.tf_timeout)
            self._last_tf_latency_ms = (time.perf_counter() - t0) * 1000.0
            return transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, cam_frame, Time(), timeout=self.tf_timeout)
                self._last_tf_latency_ms = (time.perf_counter() - t0) * 1000.0
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.base_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF 回退（本帧打 tf_stale 标记）')
                    self._tf_warned = True
                return transform_msg_to_matrix(tf.transform), 'stale'
            except TransformException as ex:
                self._last_tf_latency_ms = (time.perf_counter() - t0) * 1000.0
                self.get_logger().warning(
                    f'TF {self.base_frame}←{cam_frame} 查询失败: {ex}')
                return None, 'unavailable'

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------
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
        if count_reject:
            self.collector.rejected_views += 1
        self._publish_all()
        return response

    def _best_candidate(self) -> Tuple[str, Optional[np.ndarray]]:
        """
        最新候选中取最优（第一个 ACCEPT，否则第一个）.

        Returns
        -------
            (target_id, center_base|None)：center 取袋底（果心侧，[m]），
            零点回退袋颈；无候选给 ('', None).

        """
        msg = self._latest_candidates
        if msg is None or not msg.candidates:
            return '', None
        best = None
        for cand in msg.candidates:
            if cand.status == _STATUS_ACCEPT:
                best = cand
                break
        if best is None:
            best = msg.candidates[0]
        center = np.array([best.bag_bottom.x, best.bag_bottom.y,
                           best.bag_bottom.z])
        if not np.any(center):
            center = np.array([best.bag_neck.x, best.bag_neck.y,
                               best.bag_neck.z])
        if not np.any(center):
            center = None
        return best.target_id, center

    def _on_start(self, request, response):
        """~/start_reconstruction：清空帧栈，绑定当前最优候选，→ COLLECTING."""
        del request
        target_id, center = self._best_candidate()
        response.message = self.collector.start(target_id, center)
        self._last_captured_stamp_sec = -1.0
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
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
        if len(self.collector.frames) >= self.cfg_max_views:
            return self._deny(
                response,
                f'已达 max_views={self.cfg_max_views}，请 finalize 或 remove_last')
        cached = self._latest_frame
        if cached is None:
            return self._deny(response, '尚无同步 RGB-D 帧（确认相机/回放在线）')
        rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame = cached
        if stamp_sec <= self._last_captured_stamp_sec:
            return self._deny(response, '缓存帧未更新（与上次采帧同帧），请等下一帧')
        age_s = self.get_clock().now().nanoseconds / 1e9 - stamp_sec
        if age_s > self.cfg_max_frame_age_s:
            return self._deny(
                response,
                f'缓存帧龄期 {age_s:.2f} s > max_frame_age_s='
                f'{self.cfg_max_frame_age_s}（陈帧拒采）')
        if self.cfg_require_static:
            if not self._joint_states_seen:
                return self._deny(
                    response, 'require_robot_static=true 但未收到 /joint_states')
            if self._max_joint_vel > self.cfg_static_vel_thresh:
                return self._deny(
                    response,
                    f'机器人未静止：最大关节速度 {self._max_joint_vel:.4f} rad/s '
                    f'> {self.cfg_static_vel_thresh}')
        if not cam_frame:
            return self._deny(
                response, '深度图 header.frame_id 为空，无法查 TF', count_reject=False)
        T_base_camera, tf_status = self._lookup_T_base_camera(cam_frame, stamp_msg)
        if T_base_camera is None:
            self.collector.tf_failures += 1
            return self._deny(
                response,
                f'TF {self.base_frame}←{cam_frame} 查询失败（已计 tf_failures）',
                count_reject=False)
        ok, reason, trans, rot = self.collector.check_view(T_base_camera)
        if not ok:
            return self._deny(response, reason)
        if reason == 'duplicate_allowed':
            self.get_logger().warning(
                f'重复视角仍采帧（allow_duplicate_views=true）：'
                f'平移 {trans * 1000.0:.1f} mm / 旋转 {rot:.1f} deg')
        accepted, message = self._accept_frame(
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status)
        if not accepted:
            return self._deny(response, message)
        response.success = True
        response.message = message
        return response

    def _accept_frame(self, rgb, depth_mm, K, stamp_sec: float,
                      T_base_camera, tf_status: str) -> Tuple[bool, str]:
        """
        建云入库共享段（手动/自动采帧共用）.

        反投影建云（[mm]→[m]，颜色逐点 BGR 采样）→ 变到 base_frame →
        CapturedFrame 入栈 → 重发累加云/状态/诊断/Marker。

        Args:
            rgb: (H, W, 3) uint8 BGR 彩图.
            depth_mm: (H, W) uint16 深度 [mm].
            K: 内参 dict.
            stamp_sec: 图像时间戳 [s].
            T_base_camera: (4, 4) base←camera 位姿.
            tf_status: TF 查询状态（'stale' 打 tf_stale 标记）.

        Returns
        -------
            (accepted, message)：accepted=False 表示帧栈已满.

        """
        cloud_base, cloud_rgb, ratio = build_cloud_base(
            depth_mm, K, T_base_camera, rgb_bgr=rgb)
        flags = ['tf_stale'] if tf_status == 'stale' else []
        frame = CapturedFrame(
            rgb=rgb, depth_mm=depth_mm, camera_K=K, stamp=stamp_sec,
            T_base_camera=T_base_camera, target_id=self.collector.target_id,
            valid_depth_ratio=ratio, cloud_base=cloud_base,
            cloud_rgb=cloud_rgb, diagnostic_flags=flags)
        if not self.collector.add_frame(frame):
            return False, f'已达 max_views={self.cfg_max_views}'
        self._last_captured_stamp_sec = stamp_sec
        # 帧栈变了，旧 finalize/TSDF 缓存失效
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        n = len(self.collector.frames)
        message = (
            f'已采第 {n}/{self.cfg_recommended_views} 视角，'
            f'本帧 {cloud_base.shape[0]} 点，有效深度占比 {ratio:.2f}'
            + (' [tf_stale]' if flags else ''))
        self.get_logger().info(message)
        self._publish_all()
        return True, message

    def _on_remove_last(self, request, response):
        """~/remove_last_frame：弹出最后一帧并重发累加云."""
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
            self._tsdf_cloud_cache = None
            self._tsdf_info = None  # 帧栈变了，旧 finalize/TSDF 缓存失效
            self.get_logger().info(response.message)
        self._publish_all()
        return response

    def _on_reset(self, request, response):
        """~/reset_reconstruction：清空帧栈与绑定目标，回 IDLE."""
        del request
        self.collector.reset()
        self._last_captured_stamp_sec = -1.0
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        response.success = True
        response.message = '已清空，回 IDLE'
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    def _finalize_now(self) -> Tuple[bool, str]:
        """
        共享 finalize 逻辑（服务与自动模式共用）：装配 + overlap + 状态迁移.

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
            if self.cfg_tsdf_enable:
                message += self._run_tsdf()
            self.get_logger().info(message)
        else:
            self._overlap_cache = None
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self.get_logger().warning(message)
        # 累加云由 _publish_all 统一重发（frame_id=base_frame）
        self._publish_all()
        return ok, message

    def _run_tsdf(self) -> str:
        """
        TSDF 批量积分 + 后处理链（finalize 成功后在 READY 态调用一次）.

        链：逐帧 integrate（外参取逆，见 tsdf_volume 注释）→ extract →
        ROI 裁剪（local_volume；中心取绑定候选，无候选用首帧云质心）→
        体素降采样 → 统计离群剔除。结果缓存供 tsdf_cloud 话题重发与
        save_session 落盘；失败只告警不污染 finalize 结果。

        Returns
        -------
            追加到 finalize message 的片段（如 '；TSDF 123456 点'）.

        """
        try:
            volume = LocalTsdf(**self.tsdf_params)
            for f in self.collector.frames:
                volume.integrate_frame(f.rgb, f.depth_mm, f.camera_K,
                                       f.T_base_camera)
            xyz, colors = volume.extract_cloud()
            # ROI 中心：绑定候选中心优先，无候选用首帧云质心
            roi_center = self.collector.target_center
            if roi_center is None and self.collector.frames:
                first = self.collector.frames[0].cloud_base
                if first is not None and len(first):
                    roi_center = np.asarray(first).mean(axis=0)
            if roi_center is not None and xyz.size:
                xyz, colors = LocalTsdf.crop_to_box(
                    xyz, colors, roi_center, self.local_volume)
            if self.cfg_cloud_voxel_size > 0.0:
                xyz, colors = LocalTsdf.voxel_downsample(
                    xyz, colors, self.cfg_cloud_voxel_size)
            if self.cfg_cloud_stat_filter:
                xyz, colors = LocalTsdf.statistical_filter(xyz, colors)
        except Exception as exc:  # noqa: BLE001
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self.get_logger().error(f'TSDF 积分失败（不影响 finalize）: {exc}')
            return f'；TSDF 失败（{exc}）'
        self._tsdf_cloud_cache = (xyz, colors)
        self._tsdf_info = {
            'points': int(xyz.shape[0]),
            'integrate_time_s': float(volume.integrate_time_s),
            'voxel_length': self.tsdf_params['voxel_length'],
            'sdf_trunc': self.tsdf_params['sdf_trunc'],
            'roi_center': (None if roi_center is None
                           else [float(v) for v in roi_center]),
        }
        return (f'；TSDF {xyz.shape[0]} 点'
                f'（积分 {volume.integrate_time_s:.2f}s）')

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
                tsdf_cloud=self._tsdf_cloud_cache)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f'落盘失败: {exc}'
            self.get_logger().error(response.message)
            self._publish_all()
            return response
        response.success = True
        response.message = f'已保存 {len(frames)} 帧到 {session_dir}'
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
        self._last_captured_stamp_sec = -1.0
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        self.get_logger().info(f'自动开始：{message}')
        self._publish_all()

    def _try_auto_capture(self):
        """自动采帧：帧新鲜度/静止/TF/视角决策全过才建云入库."""
        cached = self._latest_frame
        if cached is None:
            return
        rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame = cached
        if stamp_sec <= self._last_captured_stamp_sec:
            return  # 本帧已采过（等下一帧回调）
        age_s = self.get_clock().now().nanoseconds / 1e9 - stamp_sec
        if age_s > self.cfg_max_frame_age_s:
            return  # 陈帧跳过（自动模式不记 rejected_views）
        if self.cfg_require_static:
            if not self._joint_states_seen:
                return
            if self._max_joint_vel > self.cfg_static_vel_thresh:
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
            rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status)
        if not accepted:
            self.get_logger().warning(f'自动采帧未入库：{message}')

    # ------------------------------------------------------------------
    # 发布与诊断
    # ------------------------------------------------------------------
    def _publish_all(self):
        """状态变化后统一重发：累加云 + 状态 + 诊断 JSON + 相机轨迹 Marker."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
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
        self.pub_markers.publish(
            build_camera_markers(header, self.collector.frames))

    def _diagnostics(self) -> dict:
        """组装诊断 dict（随 /peach/reconstruction/diagnostics 以 JSON 发出）."""
        c = self.collector
        cloud = c.accumulated_cloud()
        last_ratio = c.frames[-1].valid_depth_ratio if c.frames else None
        return {
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
        }

    def _session_root(self) -> Path:
        """解析 session 落盘根目录：参数显式给优先，缺省 <工作区>/peach_sessions."""
        if self.session_root_dir:
            return Path(self.session_root_dir)
        # install/<pkg>/share/<pkg> → parents[3] 即工作区根
        share = Path(get_package_share_directory('peach_reconstruction_ros2'))
        return share.parents[3] / 'peach_sessions'

    def _session_metadata(self) -> dict:
        """参数快照与帧级摘要（随 metadata.yaml 落盘，供离线复现）."""
        c = self.collector
        return {
            'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'node': 'peach_reconstruction_node',
            'phase': 'Phase 2（原始点云累加，无 TSDF/refit）',
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'captured_views': len(c.frames),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'parameters': {
                'frames.base_frame': self.base_frame,
                'sync_slop_s': self.sync_slop_s,
                'tf_timeout_sec': self.tf_timeout.nanoseconds / 1e9,
                'depth_scale_unit': self.depth_scale_unit,
                'capture.min_views': self.cfg_min_views,
                'capture.recommended_views': self.cfg_recommended_views,
                'capture.max_views': self.cfg_max_views,
                'capture.require_robot_static': self.cfg_require_static,
                'capture.static_joint_vel_thresh': self.cfg_static_vel_thresh,
                'capture.max_frame_age_s': self.cfg_max_frame_age_s,
                'view_filter.min_translation': self.vf_min_translation,
                'view_filter.max_translation': self.vf_max_translation,
                'view_filter.min_rotation_deg': self.vf_min_rotation_deg,
                'view_filter.max_rotation_deg': self.vf_max_rotation_deg,
                'view_filter.allow_duplicate_views': self.vf_allow_duplicate,
                'local_volume.size_x': self.local_volume[0],
                'local_volume.size_y': self.local_volume[1],
                'local_volume.size_z': self.local_volume[2],
                'tsdf.enable': self.cfg_tsdf_enable,
                'tsdf.voxel_length': self.tsdf_params['voxel_length'],
                'tsdf.sdf_trunc': self.tsdf_params['sdf_trunc'],
                'tsdf.depth_trunc': self.tsdf_params['depth_trunc'],
                'cloud_filter.voxel_size': self.cfg_cloud_voxel_size,
                'cloud_filter.enable_statistical_filter':
                    self.cfg_cloud_stat_filter,
            },
            'tsdf_result': self._tsdf_info,
            'frames': [{
                'index': i,
                'stamp_sec': float(f.stamp),
                'valid_depth_ratio': float(f.valid_depth_ratio),
                'camera_position_base': [float(v)
                                         for v in f.camera_position_base],
                'cloud_points': int(0 if f.cloud_base is None
                                    else f.cloud_base.shape[0]),
                'diagnostic_flags': list(f.diagnostic_flags),
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
