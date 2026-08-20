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
E4 效率项（协议 2.13-E4）：ICP target 经 icp_target_cache.IcpTargetCache
增量复用（每 k 帧自适应或关键事件才从 TSDF 全量 extract）；发布面
local_cloud/tsdf_cloud/markers 经 publish_throttle.PublishThrottle
on-change + 最小间隔节流（心跳/状态/诊断 1Hz 活性发布不动）。

线程模型：节点级 RLock 保护 collector/TSDF/产物（worker 与 executor 线程
双写收敛）；采帧门禁的阻塞式 TF 查询在锁外完成（_gated_capture_begin →
_query_tf → _finish 三段式），锁内按帧 stamp 复核收口，查询期间 Trigger
服务/目标观测回调/1Hz 心跳不被堵住（A7 起）。

模块边界（A14 拆分）：本文件为节点编排壳（参数声明、接口装配、订阅/
服务回调入口、采帧门禁、TSDF/refit 写路径、session 落盘、main）；
自动状态机驱动在 auto_controller.py（AutoControllerMixin），发布面
（心跳/状态三件套/点云/Marker/refit 消息与诊断组装）在
publishers.py（PublisherMixin），两者均为无 __init__ 的 mixin，
宿主契约见各自模块 docstring。
"""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import threading
import time
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv_bridge
from geometry_msgs.msg import Vector3Stamped
import message_filters
import numpy as np
from peach_common_py.bounded_worker import BoundedWorker
from peach_common_py.depth_geometry import normalize_depth_to_uint16_mm
from peach_common_py.harvest_data import HarvestDataStore
from peach_common_py.ros.clock_adapter import RclpyClockAdapter
from peach_common_py.tf_utils import transform_msg_to_matrix
from peach_interfaces.action import BuildTargetModel
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    GraspDecision,
    HarvestState,
    PeachTargetObservationArray,
    ReconstructionStatus,
    ShapeHypothesis,
    TargetModel,
    TargetQuality,
)
from peach_target_reconstruction.auto_controller import AutoControllerMixin
from peach_target_reconstruction.bind_holdoff import BindSwitchHoldoff
from peach_target_reconstruction.candidate_contract import (
    candidate_axis_hint,
    select_reconstruction_candidate,
    TargetKindMemory,
)
from peach_target_reconstruction.capture_gate import (
    capture_gate,
    GATE_ALLOW,
    GATE_DENY,
    GATE_NEED_TF,
    GATE_SKIP,
    GateDecision,
)
from peach_target_reconstruction.captured_frame import CapturedFrame
from peach_target_reconstruction.cloud_builder import apply_target_mask
from peach_target_reconstruction.frame_collector import (
    CollectorConfig,
    STATE_COLLECTING,
    STATE_IDLE,
)
from peach_target_reconstruction.geometry_refiner import (
    RefitConfig,
    select_refitter,
    STATUS_ACCEPT,
)
from peach_target_reconstruction.icp_refiner import (
    IcpConfig,
    transform_points,
)
from peach_target_reconstruction.icp_target_cache import (
    IcpTargetCache,
    IcpTargetRefreshConfig,
)
from peach_target_reconstruction.interfaces import (
    CLOUD_BUILDERS,
    FRAME_STORES,
    MASK_GATES,
    REFINERS,
    REFITTERS,
    VOLUMES,
)
from peach_target_reconstruction.mask_gate import MaskContext
from peach_target_reconstruction.overlap import (
    assembly_overlap_metrics,
    summarize_pairs_mm,
)
from peach_target_reconstruction.params import ReconstructionParams
from peach_target_reconstruction.publish_throttle import PublishThrottle
from peach_target_reconstruction.publishers import PublisherMixin
from peach_target_reconstruction.session_io import save_session
from peach_target_reconstruction.timing import TimingStats
from peach_target_reconstruction.tsdf_volume import LocalTsdf
from peach_target_reconstruction.view_coverage import summarize_view_coverage
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray


class PeachReconstructionNode(AutoControllerMixin, PublisherMixin, LifecycleNode):
    """连续运动局部重建 Lifecycle 节点：Active 后才积分与受理 BuildTargetModel."""

    def __init__(self):
        """建节点：参数层一行装载 → 数据持有者/算法（注册表）→ ROS 接线."""
        super().__init__('peach_target_reconstruction_node')
        self._lifecycle_active = False
        self.bridge = cv_bridge.CvBridge()
        # 协议 I3（时钟唯一）：节点时钟适配为纯核 Clock，一切计时走注入 now
        self._algo_clock = RclpyClockAdapter(self.get_clock())
        # 参数层：declare + 装载（全部 64 参数在 params.py）
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

        # 数据持有者与算法实现（2.14：ABC + Registry，构造期按 yaml
        # *.impl 注册名 create 注入，设计文档 §2.2/§2.4）
        self.collector = FRAME_STORES.create('default', config=CollectorConfig(
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
        self._cloud_builder = CLOUD_BUILDERS.create(p.cloud_builder.impl)
        self._refitters = {
            'cylinder': REFITTERS.create(p.refitter.cylinder_impl),
            'sphere': REFITTERS.create(p.refitter.sphere_impl),
        }
        self._icp_refiner = REFINERS.create(
            p.refiner.impl, config=self.icp_config)
        self._mask_gate = MASK_GATES.create(
            p.mask_gate.impl,
            require_target_mask=p.capture.require_target_mask,
            min_mask_pixels=p.capture.min_mask_pixels,
            min_mask_depth_ratio=p.capture.min_mask_depth_ratio,
            max_target_drift_m=p.capture.max_target_drift_m,
            min_neighbor_gap_m=p.capture.min_neighbor_gap_m)
        # E2 selected 切换防抖状态机（纯核 bind_holdoff.BindSwitchHoldoff，
        # 注入时钟 I3）：selected 变化须持续超过 bind.switch_holdoff_s 才
        # 放弃进行中会话重绑，holdoff 内切回原 ID 取消挂起
        self._switch_holdoff = BindSwitchHoldoff(
            holdoff_s=p.bind.switch_holdoff_s, now=self._algo_clock.now)
        # E4 ICP target 增量复用缓存（纯核 icp_target_cache.IcpTargetCache）：
        # 两次全量 extract 之间复用「上次全量+已采帧修正后云增量拼接」做
        # ICP target；全量刷新周期 k 按修正量 EMA 在上下限间自适应伸缩。
        # max_translation_m 注入 icp 配置（漂移阈值基准）；downsample_voxel
        # 注入 tsdf.voxel_length（增量拼接超限降采样与模型分辨率同尺度）
        self._icp_target_cache = IcpTargetCache(IcpTargetRefreshConfig(
            min_period=p.icp.target_refresh_min_period,
            max_period=p.icp.target_refresh_max_period,
            drift_ratio=p.icp.target_refresh_drift_ratio,
            max_translation_m=p.icp.max_translation,
            downsample_voxel=p.tsdf.voxel_length))
        # E4 发布节流（纯核 publish_throttle.PublishThrottle，注入时钟 I3）：
        # 点云/Marker 类大消息 on-change + 最小间隔；心跳/状态/诊断/refit
        # 三件套不经过本节流。on_change_only=false 时编排层整体绕过
        self._publish_throttle = PublishThrottle(
            min_interval_s=p.publish.min_interval_s, now=self._algo_clock.now)
        # 产物版本号（E4 发布节流的 on-change 判据）：_tsdf_cloud_version
        # 随 _tsdf_cloud_cache 每次写入递增；_products_version 随任一
        # 云/Marker 产物（含 refined/mesh）写入递增；清空类事件另置
        # _products_force_publish 强制下一轮 _publish_all 立即透传
        self._tsdf_cloud_version = 0
        self._products_version = 0
        self._products_force_publish = False

        # 并发收敛选型（方案 b，与 peach_scene_perception_node._plan_lock 同模式）：
        # collector/在线 TSDF/派生产物的竞态源是 worker 线程
        # （_process_rgbd→_auto_drive）与 executor 线程（订阅/服务回调）
        # 双写；方案 (a) 把命令类任务也挤进 BoundedWorker 不可行——
        # capacity=3 且 drop_oldest=False，满队列直接拒收，与「命令不丢」
        # 冲突，Trigger 服务又需同步应答难以异步化。故用节点级 RLock：
        # 服务/订阅/自动驱动入口持锁，帧栈、TSDF 积分与全部产物读写均在
        # 锁内（RLock 允许 _auto_drive→_finalize_now 等锁内嵌套调用）。
        # 唯一锁外段：采帧门禁的阻塞式精确时刻 TF 查询（_gated_capture_*
        # 三段式，最长 tf_timeout），查询期间锁空闲、Trigger 服务/观测回调/
        # 心跳不被堵；锁内 finish 按帧 stamp 复核后落地，杜绝旧帧位姿套新帧。
        # _latest_frame/_latest_candidates/_max_joint_vel 等单字段原子
        # 赋值不持锁（CPython 引用赋值原子，读者一次取引用后局部使用）。
        self._state_lock = threading.RLock()
        self._view_progress = threading.Event()

        # 最新一帧同步 RGB-D 缓存：(rgb, depth_mm, K, stamp_msg, stamp_sec,
        # cam_frame)。只缓存、不直接累积；手动/自动门禁通过后才会入帧栈
        self._latest_frame: Optional[tuple] = None
        self._last_captured_stamp_sec = -1.0  # [s] 上次成功采帧的图像时间戳
        self._latest_candidates: Optional[BagGraspCandidateArray] = None
        self._preferred_target_id = ''
        self._executor_target_id = ''
        self._executor_state_seen = False
        self._harvest_run_id = ''
        self._target_observation_seen = False
        self._target_masks = {}
        # 锁定集目标锚点缓存 {target_id: (3,) base 系中心 [m]}：E2 邻目标
        # 串扰门数据源（每条 target_observations 全量重建，未锁定恒空）
        self._locked_target_centers = {}
        self._harvest_data = HarvestDataStore()
        self._joint_states_seen = False
        self._max_joint_vel = 0.0  # [rad/s] 最近 /joint_states 的最大关节速度
        self._last_tf_latency_ms: Optional[float] = None  # 最近一次 TF 查询墙钟耗时
        # 耗时累计器（阶段 C 埋点）：ICP/TSDF 积分/帧总耗时 EMA + refit/
        # finalize last 值；计时打点用注入时钟（I3），快照进 diagnostics
        # JSON 的 timing 子对象与 session metadata
        self._timing = TimingStats()
        # finalize 时计算的 overlap 指标缓存（帧栈变动即失效置 None）
        self._overlap_cache: Optional[dict] = None
        # TSDF 云缓存：(xyz, colors_bgr) 或 None；finalize 时重建，帧栈变动失效
        self._tsdf_cloud_cache: Optional[tuple] = None
        self._tsdf_info: Optional[dict] = None  # diagnostics 的 tsdf 键内容
        self._tsdf_volume = None  # 每轮 session 持续在线积分
        self._mesh_cache: Optional[dict] = None
        # refit：感知 diagnostics 的 target_id→target_kind 映射；
        # _refined 为 refit 唯一缓存——refine_geometry 成功结果 dict 或
        # {'ok': False, 'reason': ...} 失败记录（None=未跑/已失效），
        # diagnostics JSON 的 refined 键由 _refined_info() 投影派生
        self._target_kind_memory = TargetKindMemory()
        self._refined: Optional[dict] = None
        # 球体 refit 无法独立恢复姿态轴；绑定时冻结感知侧果梗/凹陷方向先验。
        self._bound_axis_hint: Optional[np.ndarray] = None

        # ---- 发布者（/peach/reconstruction/* 固定命名）----
        # 状态类话题用 transient_local 闩锁（depth=1）：后启动的订阅者
        # （验证记录器 / RViz）也能拿到最后一次发布；发布频率低，闩锁代价可忽略
        latched_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # A10：diagnostics 叠加 DDS Deadline（offered 1.5s < 1Hz 心跳周期留 50%
        # 余量），DDS 层补强应用层 2s 新鲜度门——执行器卡死/心跳断供在 1.5s 内
        # 暴露为 offered-deadline-missed 事件（只记日志，不改行为）。兼容面：
        # 请求方 deadline ≥ 1.5s 或不设 deadline（编排器 2s / Web 与能力端不设）
        # 均兼容；offered 收紧只影响比 1.5s 更苛刻的未来订阅者。
        diag_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            deadline=Duration(seconds=1.5),
        )
        diag_pub_callbacks = PublisherEventCallbacks(
            deadline=lambda info: self.get_logger().warning(
                f'diagnostics 心跳超过 offered deadline 1.5s'
                f'（累计违约 {info.total_count} 次）：节点执行器疑似卡滞'),
            use_default_callbacks=False)
        self.pub_cloud = self.create_publisher(
            PointCloud2, '/peach/reconstruction/local_cloud', latched_qos)
        self.pub_status = self.create_publisher(
            String, '/peach/reconstruction/status', latched_qos)
        self.pub_diag = self.create_publisher(
            ReconstructionStatus, '/peach/reconstruction/diagnostics',
            diag_qos, event_callbacks=diag_pub_callbacks)
        # 完整调试明细（tsdf/registration/overlap/refined/逐机位）不进类型化
        # 消息，另发 String JSON 调试话题（同闩锁，仅供排查与 Web 明细镜像）
        self.pub_diag_debug = self.create_publisher(
            String, '/peach/reconstruction/diagnostics_debug', latched_qos)
        self.pub_grasp_decision = self.create_publisher(
            GraspDecision, '/peach/reconstruction/grasp_decision', latched_qos)
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
        self.pub_shape = self.create_publisher(
            ShapeHypothesis, '/peach/reconstruction/shape_hypothesis',
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
        self._frame_worker = BoundedWorker(
            self._process_rgbd, capacity=3, drop_oldest=False)
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
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cb = ReentrantCallbackGroup()
        self.create_subscription(
            HarvestState, '/peach_task_executor/state',
            self._on_executor_state, latched, callback_group=self._cb)

        # ---- 服务（std_srvs/Trigger，节点相对名；人工/BT 调试口）----
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
        ActionServer(
            self, BuildTargetModel, '~/build_target_model',
            execute_callback=self._on_build_target_model,
            goal_callback=self._on_build_goal,
            cancel_callback=self._on_build_cancel,
            callback_group=self._cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1Hz 活性心跳：状态/诊断/抓取许可三件套周期重发。_publish_all 只在
        # 状态变化时触发，IDLE 期无消息会让编排器重建就绪门（2s 新鲜度）永远
        # 不满足——拍照前置建立不了目标→无法锁定→无法绑定的死锁由此解开。
        self._heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)

        # 启动即首发一次（IDLE + 空云），闩锁话题让后启动的订阅者立即可读
        self._publish_all()

        self.get_logger().info(
            f'peach_target_reconstruction_node ready: '
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

    def on_configure(self, state):
        del state
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self._lifecycle_active = True
        self.get_logger().info('reconstruction Active：开始积分')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self._lifecycle_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        del state
        self._lifecycle_active = False
        return TransitionCallbackReturn.SUCCESS

    def _on_build_goal(self, goal_request):
        del goal_request
        if not self._lifecycle_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_build_cancel(self, cancel_request):
        del cancel_request
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """将同步帧交给 TSDF 单写者队列，满队列拒绝新帧."""
        if not self._lifecycle_active:
            return
        if not self._frame_worker.submit((rgb_msg, depth_msg, info)):
            self.get_logger().warning(
                '重建 worker 队列已满，拒绝新帧以保持积分顺序',
                throttle_duration_sec=1.0)

    def _process_rgbd(self, frame):
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
        rgb_msg, depth_msg, info = frame
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
        """缓存当前绑定目标 ID 的精确深度时刻掩膜与中心，并刷新锁定集锚点."""
        # 目标切换会 reset 帧栈并清空产物，须与 worker 线程互斥（持锁全程）
        with self._state_lock:
            self._target_observation_seen = True
            if msg.harvest_run_id != self._harvest_run_id:
                self._harvest_run_id = msg.harvest_run_id
                self._harvest_data.attach(self._harvest_run_id)
                self._harvest_data.append_event({
                    'source': 'reconstruction',
                    'event': 'reconstruction_linked'})
            if self._executor_state_seen:
                requested_target_id = self._executor_target_id
            else:
                requested_target_id = msg.selected_target_id
            # E2 切换防抖（bind_holdoff.BindSwitchHoldoff）：进行中会话的
            # 放弃重绑须新 selected 持续稳定超过 bind.switch_holdoff_s；
            # 挂起中（PEND/WAIT）保持旧绑定，旧会话照常采帧，瞬态抖动
            # （A→空→A、A→B→A）不再销毁正在积分的 TSDF 会话
            action = self._switch_holdoff.arbitrate(
                requested_target_id, self._preferred_target_id,
                session_active=self.collector.state != STATE_IDLE)
            if action == BindSwitchHoldoff.COMMIT:
                # 挂起到期执行放弃重绑（原无条件切换逻辑，语义不变）：
                # 感知计划推进（上一目标周期已终局）后必须跟随新 selected：
                # 旧目标未 READY 的半成品会话已随周期结束失效，继续绑定只会让
                # 身份门永远 mismatch。COLLECTING/READY 一律放弃旧会话并重发
                # 清空产物（RViz 同步刷新）。
                self.get_logger().info(
                    f'跟随计划切换重建目标: {self._preferred_target_id} -> '
                    f'{requested_target_id or "（空）"}'
                    f'（放弃 {self.collector.state} 会话）')
                self.collector.reset()
                self._target_kind_memory.reset()
                self._last_captured_stamp_sec = -1.0
                self._reset_products(create_volume=False)
                self._bound_axis_hint = None
                self._target_masks.clear()
                self._publish_all()
                self._preferred_target_id = requested_target_id
            elif action in (BindSwitchHoldoff.PEND, BindSwitchHoldoff.WAIT):
                # 挂起中：不更新 _preferred_target_id，下方掩膜缓存与邻目标
                # 锚点仍按旧绑定目标刷新（会话零扰动）
                pass
            else:
                # FOLLOW（无会话可毁/未偏离/未绑定）与 CANCEL（holdoff 内
                # 切回原 ID）均直通；requested==bound 时赋值幂等
                self._preferred_target_id = requested_target_id
            # 掩膜缓存按当前绑定目标（防抖期=旧目标）取观测；绑定目标本帧
            # 无观测/非 OBSERVED/无掩膜时本帧不更新缓存
            bound_obs = next((item for item in msg.observations
                              if item.target_id == self._preferred_target_id),
                             None)
            # E2 邻目标串扰门数据源：每条观测消息全量重建锁定集锚点缓存
            # （绑定目标自身在 _target_mask_for_frame 组 MaskContext 时剔除；
            # 未锁定时 observations 恒空，缓存随之为空）
            centers = {}
            for item in msg.observations:
                c = self._candidate_center(item.candidate)
                if c is not None:
                    centers[item.target_id] = c
            self._locked_target_centers = centers
            if (bound_obs is None
                    or bound_obs.tracking_status != bound_obs.OBSERVED):
                return
            if not bound_obs.mask.data:
                return
            try:
                mask = self.bridge.imgmsg_to_cv2(
                    bound_obs.mask, desired_encoding='mono8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'目标掩膜解码失败: {exc}')
                return
            stamp = bound_obs.mask.header.stamp
            stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            center = self._candidate_center(bound_obs.candidate)
            self._target_masks[stamp_ns] = (
                np.asarray(mask, dtype=np.uint8), center)
            while len(self._target_masks) > 30:
                self._target_masks.pop(next(iter(self._target_masks)))
            if self.params.capture.auto_mode:
                self._auto_drive()

    @staticmethod
    def _candidate_center(candidate):
        """候选几何袋底/袋颈中点（base 系 [m]）；非有限或全零时返回 None."""
        bottom = np.array([
            candidate.bag_bottom.x,
            candidate.bag_bottom.y,
            candidate.bag_bottom.z], dtype=np.float64)
        neck = np.array([
            candidate.bag_neck.x,
            candidate.bag_neck.y,
            candidate.bag_neck.z], dtype=np.float64)
        center = 0.5 * (bottom + neck)
        if not np.all(np.isfinite(center)) or not np.any(center):
            return None
        return center

    def _target_mask_for_frame(self, stamp_msg, depth_mm):
        """取严格同时间戳掩膜并过五道质量门（判定本体在 MaskGate 实现）."""
        stamp_ns = int(stamp_msg.sec) * 1000000000 + int(stamp_msg.nanosec)
        # 邻目标锚点=锁定集锚点缓存剔除绑定目标自身（E2 串扰门输入）
        neighbors = tuple(
            c for tid, c in self._locked_target_centers.items()
            if tid != self._preferred_target_id)
        result = self._mask_gate.check(MaskContext(
            stamp_ns=stamp_ns,
            depth_mm=depth_mm,
            masks=self._target_masks,
            bound_center=self.collector.target_center,
            neighbor_centers=neighbors))
        return result.mask, result.reason

    def _on_query_reconstruction_state(self, request, response):
        """~/query_reconstruction_state：返回当前重建和数据关联 JSON."""
        del request
        with self._state_lock:
            response.success = True
            response.message = json.dumps(
                self._diagnostics(), ensure_ascii=False)
            return response

    def _on_perception_diagnostics(self, msg: BagFittingArray):
        """缓存 target_id→target_kind 映射（refit 选圆柱/球拟合线用）."""
        # bind/reset 发生在持锁的开始/复位路径，update 同样入锁保持一致
        with self._state_lock:
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
        t0 = self._algo_clock.now()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.params.frames.base_frame, cam_frame, Time.from_msg(stamp),
                timeout=self.tf_timeout)
            self._last_tf_latency_ms = (self._algo_clock.now() - t0) * 1000.0
            return transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException as ex:
            self._last_tf_latency_ms = (self._algo_clock.now() - t0) * 1000.0
            self.get_logger().warning(
                f'TF {self.params.frames.base_frame}←{cam_frame} 在图像时刻'
                f'不可用，本帧跳过: {ex}')
            return None, 'unavailable'

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------
    def _create_volume(self):
        """按 yaml volume.impl 经 VOLUMES 注册表建一个空融合体积（I3 注入时钟）."""
        return VOLUMES.create(
            self.params.volume.impl, now=self._algo_clock.now, **self.tsdf_params)

    def _bump_products_version(self, tsdf_cloud: bool = False) -> None:
        """
        产物版本号递增（E4 发布节流的 on-change 判据；须持 _state_lock）.

        Args:
            tsdf_cloud: True 表示 _tsdf_cloud_cache 也被写入（同步递增
                tsdf_cloud 版本）.

        Returns
        -------
            无返回值（None）.

        """
        self._products_version += 1
        if tsdf_cloud:
            self._tsdf_cloud_version += 1

    def _reset_products(self, create_volume: bool) -> None:
        """清空本轮派生结果；开始新轮时同时创建一个空在线 TSDF."""
        self._overlap_cache = None
        self._tsdf_cloud_cache = None
        self._tsdf_info = None
        self._mesh_cache = None
        self._refined = None
        self._tsdf_volume = None
        # E4：模型已清空，ICP target 缓存作废（下次采帧强制全量刷新）；
        # 版本号递增 + force 标志使下一轮 _publish_all 立即透传空产物
        # （RViz 同步刷新不被 on-change/间隔门抑制）
        self._icp_target_cache.invalidate()
        self._bump_products_version(tsdf_cloud=True)
        self._products_force_publish = True
        if create_volume and self.params.tsdf.enable:
            self._tsdf_volume = self._create_volume()

    def _roi_center(self):
        """返回局部体素盒中心；候选缺失时退到首帧局部云质心."""
        center = self.collector.target_center
        if center is None and self.collector.frames:
            first = self.collector.frames[0].cloud_base
            if first is not None and len(first):
                center = np.asarray(first).mean(axis=0)
        return center

    def _refresh_tsdf_outputs(self, extract_mesh: bool = False) -> None:
        """
        从当前在线体积刷新局部点云；finalize 时额外提取网格.

        E4：本方法是 TSDF 全量 extract 的唯一入口（采帧路径受
        IcpTargetCache 节流，每 k 帧或关键事件才调用）；每次调用同步
        重置 ICP target 复用基线并递增 tsdf_cloud 产物版本号。
        """
        if self._tsdf_volume is None:
            self._tsdf_cloud_cache = None
            self._mesh_cache = None
            # 体积不存在：产物清空 + target 缓存作废（防御性路径，
            # 正常流程采帧/ finalize 前体积必已创建）
            self._icp_target_cache.invalidate()
            self._bump_products_version(tsdf_cloud=True)
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
        # E4：全量提取结果即 ICP target 复用基线（帧到模型语义不变，
        # 仅 extract 频率从逐帧降为每 k 帧）
        self._icp_target_cache.set_full(xyz)
        self._bump_products_version(tsdf_cloud=True)
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
        with self._state_lock:
            target_id, center = self._best_candidate()
            response.message = self.collector.start(target_id, center)
            self._target_kind_memory.bind(target_id)
            self._last_captured_stamp_sec = -1.0
            self._reset_products(create_volume=True)
            self._bound_axis_hint = candidate_axis_hint(
                self._latest_candidates, target_id)
            response.success = True
            self.get_logger().info(response.message)
            self._publish_all()
            return response

    def _collect_gate_values(self, automatic: bool
                             ) -> Tuple[dict, Optional[tuple]]:
        """
        采集采帧门禁判据快照（须持 _state_lock；只读共享状态，零副作用）.

        Args:
            automatic: True=自动模式 / False=手动服务（透传进判据）.

        Returns
        -------
            (gate_values, frame_snapshot)：frame_snapshot 为
            (rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame, target_mask)，
            无缓存帧时为 None.

        """
        cached = self._latest_frame
        rgb = depth_mm = K = stamp_msg = cam_frame = None
        stamp_sec = 0.0
        target_mask = None
        mask_reason = ''
        if cached is not None:
            rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame = cached
            target_mask, mask_reason = self._target_mask_for_frame(
                stamp_msg, depth_mm)
        gate_values = {
            'frame_available': cached is not None,
            'frame_count': len(self.collector.frames),
            'max_views': self.params.capture.max_views,
            'mask_reason': mask_reason,
            'stamp_sec': stamp_sec,
            'last_captured_stamp_sec': self._last_captured_stamp_sec,
            'frame_age_s': self._algo_clock.now() - stamp_sec,
            'max_frame_age_s': self.params.capture.max_frame_age_s,
            'require_robot_static': self.params.capture.require_robot_static,
            'joint_states_seen': self._joint_states_seen,
            'max_joint_vel': self._max_joint_vel,
            'static_joint_vel_thresh':
                self.params.capture.static_joint_vel_thresh,
            'cam_frame_ok': bool(cam_frame),
            'base_frame': self.params.frames.base_frame,
            'cam_frame': cam_frame or '',
            'automatic': automatic,
        }
        snapshot = None if cached is None else (
            rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame, target_mask)
        return gate_values, snapshot

    def _gated_capture_begin(self, automatic: bool
                             ) -> Tuple[GateDecision, Optional[tuple]]:
        """
        两路采帧公共门禁·锁内首段（须持 _state_lock）.

        前置门禁（满栈/无帧/掩膜/同帧/帧龄/静止/空 frame_id）任一不过即
        定案返回 (decision, None)；全过返回 (GateDecision(GATE_NEED_TF),
        tf_request)，调用方须释放 _state_lock 后用 tf_request 调
        _gated_capture_query_tf，再重新持锁调 _gated_capture_finish 收口。

        Args:
            automatic: True=自动模式（拒绝映射 skip），False=手动服务
                （映射 deny，由调用方写服务响应）.

        Returns
        -------
            (decision, tf_request)：tf_request 为
            (stamp_msg, stamp_sec, cam_frame) 或 None.

        """
        gate_values, snapshot = self._collect_gate_values(automatic)
        decision = capture_gate(tf_available=None, **gate_values)
        if decision.action != GATE_NEED_TF:
            return decision, None
        _rgb, _depth_mm, _K, stamp_msg, stamp_sec, cam_frame, _mask = snapshot
        return decision, (stamp_msg, stamp_sec, cam_frame)

    def _gated_capture_query_tf(self, tf_request
                                ) -> Tuple[Optional[np.ndarray], str]:
        """
        公共门禁·锁外段（不得持 _state_lock）：阻塞式精确时刻 TF 查询.

        最长阻塞 self.tf_timeout；查询期间 _state_lock 空闲，Trigger 服务/
        目标观测回调/心跳不被本查询堵住。查询按帧 stamp 进行，结果对当前
        缓存帧是否仍有效由锁内 _gated_capture_finish 按 stamp 复核。

        Args:
            tf_request: _gated_capture_begin 返回的
                (stamp_msg, stamp_sec, cam_frame).

        Returns
        -------
            (T_base_camera, tf_status)：同 _lookup_T_base_camera.

        """
        stamp_msg, _stamp_sec, cam_frame = tf_request
        return self._lookup_T_base_camera(cam_frame, stamp_msg)

    def _gated_capture_finish(self, automatic: bool, tf_request, tf_result
                              ) -> Tuple[GateDecision, Optional[tuple]]:
        """
        公共门禁·锁内收口段（须持 _state_lock）：复核帧有效性后以真实 TF 重评.

        竞态收口：TF 查询在锁外完成，期间缓存帧可能被新帧替换、帧栈可能被
        reset/收满/已采入同帧。本段重新采集判据快照，先比对 stamp_sec 与
        cam_frame——缓存帧已不再是发起查询时那一帧则丢弃本次尝试（自动
        =skip、手动=deny 提示重试），绝不把旧帧时刻的位姿套到新帧上；
        同帧已被采入/帧龄超期/静止破坏等也由重采判据经 capture_gate 重评
        兜住，随后才以 TF 结果走完第二阶段门禁。

        Args:
            automatic: 同 _gated_capture_begin.
            tf_request: _gated_capture_begin 返回的查询请求.
            tf_result: _gated_capture_query_tf 返回的 (T, status).

        Returns
        -------
            (decision, context)：action 为 GATE_ALLOW 时 context 为
            (rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
            target_mask)，否则为 None.

        """
        _stamp_msg, query_stamp_sec, query_cam_frame = tf_request
        T_base_camera, tf_status = tf_result
        gate_values, snapshot = self._collect_gate_values(automatic)
        frame_changed = (
            snapshot is None
            or snapshot[4] != query_stamp_sec
            or (snapshot[5] or '') != query_cam_frame)
        if frame_changed:
            return GateDecision(
                action=GATE_SKIP if automatic else GATE_DENY,
                reason='TF 查询期间缓存帧已更新，请等下一帧重试',
                count_reject=False), None
        decision = capture_gate(
            tf_available=T_base_camera is not None, **gate_values)
        if decision.action != GATE_ALLOW:
            return decision, None
        rgb, depth_mm, K, _sm, stamp_sec, _cf, target_mask = snapshot
        return decision, (rgb, depth_mm, K, stamp_sec, T_base_camera,
                          tf_status, target_mask)

    def _on_capture(self, request, response):
        """
        ~/capture_frame：过全部门禁后把当前缓存帧采入帧栈并重发累加云.

        两段式门禁：判据采集与 TF 重评在锁内（_gated_capture_begin/finish），
        阻塞式精确时刻 TF 查询在锁外（_gated_capture_query_tf，最长
        tf_timeout），查询期间 Trigger 服务/观测回调/心跳可正常取锁。
        """
        del request
        with self._state_lock:
            if self.collector.state != STATE_COLLECTING:
                return self._deny(
                    response,
                    f'当前状态 {self.collector.state}，先 ~/start_reconstruction',
                    count_reject=False)
            decision, tf_request = self._gated_capture_begin(automatic=False)
            if tf_request is None:
                return self._resolve_capture_decision(response, decision, None)
        # 锁外：阻塞式 TF 查询（不得持 _state_lock）。
        tf_result = self._gated_capture_query_tf(tf_request)
        with self._state_lock:
            if self.collector.state != STATE_COLLECTING:
                return self._deny(
                    response,
                    'TF 查询期间重建已离开 COLLECTING，请重试',
                    count_reject=False)
            decision, context = self._gated_capture_finish(
                automatic=False, tf_request=tf_request, tf_result=tf_result)
            return self._resolve_capture_decision(response, decision, context)

    def _resolve_capture_decision(self, response, decision, context):
        """手动采帧落地段（须持 _state_lock）：门禁结果 → 入库或统一拒绝."""
        if decision.action != GATE_ALLOW:
            if decision.count_tf_failure:
                self.collector.tf_failures += 1
            return self._deny(response, decision.reason,
                              count_reject=decision.count_reject)
        (rgb, depth_mm, K, stamp_sec,
         T_base_camera, tf_status, target_mask) = context
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

    def _crop_for_icp(self, cloud_fk, cloud_rgb):
        """把 ICP 输入裁到目标局部盒，避免背景主导刚体修正."""
        center = self._roi_center()
        if center is None:
            center = self.collector.target_center
        if center is not None and cloud_fk.size:
            return LocalTsdf.crop_to_box(
                cloud_fk, cloud_rgb, center, self.local_volume)
        return cloud_fk, cloud_rgb

    def _register_cloud(self, cloud_fk, target):
        """有界 ICP（或纯 FK）。返回 (ok, T_corr, mode, info, err)."""
        if not self.params.icp.enable:
            info = {
                'mode': 'fk', 'reason': 'icp_disabled',
                'fitness': -1.0, 'rmse_m': -1.0,
                'translation_m': 0.0, 'rotation_deg': 0.0,
            }
            return True, np.eye(4, dtype=np.float64), 'fk', info, ''
        t_icp0 = self._algo_clock.now()
        registration = self._icp_refiner.refine(cloud_fk, target)
        self._timing.record_icp((self._algo_clock.now() - t_icp0) * 1000.0)
        self._icp_target_cache.note_result(
            registration.mode, registration.translation_m)
        if not registration.accepted:
            err = (
                f'配准拒帧：{registration.reason}，'
                f'fitness={registration.fitness:.3f} '
                f'rmse={registration.rmse * 1000.0:.1f}mm，'
                f'修正={registration.translation_m * 1000.0:.1f}mm/'
                f'{registration.rotation_deg:.2f}deg')
            return False, None, '', {}, err
        info = {
            'mode': registration.mode,
            'reason': registration.reason,
            'fitness': float(registration.fitness),
            'rmse_m': float(registration.rmse),
            'translation_m': float(registration.translation_m),
            'rotation_deg': float(registration.rotation_deg),
        }
        return True, registration.correction, registration.mode, info, ''

    def _integrate_tsdf(self, rgb, masked_depth, K, T_used, cloud_base):
        """积分当前帧；失败回滚并返回错误串，成功返回 None."""
        if not self.params.tsdf.enable:
            return None
        try:
            if self._tsdf_volume is None:
                self._tsdf_volume = self._create_volume()
            t_tsdf0 = self._algo_clock.now()
            self._tsdf_volume.integrate_frame(
                rgb, masked_depth, K, T_used)
            if self._icp_target_cache.should_refresh():
                self._refresh_tsdf_outputs()
                if self.params.refit.enable:
                    self._run_refit(keep_last_good=True, mark_final=False)
            else:
                self._icp_target_cache.append_frame(cloud_base)
            self._timing.record_tsdf_integrate(
                (self._algo_clock.now() - t_tsdf0) * 1000.0)
            return None
        except Exception as exc:  # noqa: BLE001
            self.collector.remove_last()
            self._tsdf_volume = self._create_volume()
            for old in self.collector.frames:
                self._tsdf_volume.integrate_frame(
                    old.rgb, old.depth_mm, old.camera_K, old.T_base_camera)
            self._refresh_tsdf_outputs()
            return f'TSDF 在线积分失败: {exc}'

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
        # 帧总耗时起点（注入时钟，I3）：覆盖构云→ICP→入库→TSDF 积分全链，
        # 仅在成功收帧时计入 frame_total EMA（拒帧早退不污染基线）
        t_frame0 = self._algo_clock.now()
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

        cloud_fk, cloud_rgb = self._crop_for_icp(cloud_fk, cloud_rgb)
        cached_target = self._icp_target_cache.current_target()
        target = (np.zeros((0, 3), dtype=np.float64)
                  if cached_target is None else cached_target)
        ok, correction, mode, reg_info, err = self._register_cloud(
            cloud_fk, target)
        if not ok:
            return False, err
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
        tsdf_err = self._integrate_tsdf(
            rgb, masked_depth, K, T_used, cloud_base)
        if tsdf_err:
            return False, tsdf_err

        self._last_captured_stamp_sec = stamp_sec
        # 新帧使 overlap/mesh 失效；精化在 extract 后现场重拟（keep 上一帧
        # 成功结果），运动中 RViz 抓取示意连续更新而不是清屏。
        self._overlap_cache = None
        self._mesh_cache = None
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
        # 帧总耗时落账（成功路径终点；不含其后的统一重发，发布开销
        # 由心跳侧另计）
        self._timing.record_frame_total((self._algo_clock.now() - t_frame0) * 1000.0)
        self._publish_all()
        self._view_progress.set()
        return True, message

    def _on_remove_last(self, request, response):
        """~/remove_last_frame：弹帧后重放剩余帧，保证在线 TSDF 一致."""
        del request
        with self._state_lock:
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
                    self._tsdf_volume = self._create_volume()
                    for old in self.collector.frames:
                        self._tsdf_volume.integrate_frame(
                            old.rgb, old.depth_mm, old.camera_K,
                            old.T_base_camera)
                    self._refresh_tsdf_outputs()
                    if self.params.refit.enable:
                        self._run_refit(
                            keep_last_good=False, mark_final=False)
                else:
                    self._tsdf_cloud_cache = None
                    self._tsdf_info = None
                    # E4：TSDF 关闭路径的产物清空也须递增版本号并让
                    # ICP target 缓存作废（帧栈已变，模型语义已清）
                    self._icp_target_cache.invalidate()
                    self._bump_products_version(tsdf_cloud=True)
                    self._refined = None
                self._mesh_cache = None
                self.get_logger().info(response.message)
            self._publish_all()
            return response

    def _on_reset(self, request, response):
        """~/reset_reconstruction：清空帧栈与绑定目标，回 IDLE."""
        del request
        with self._state_lock:
            self.collector.reset()
            self._view_progress.clear()
            self._target_kind_memory.reset()
            self._last_captured_stamp_sec = -1.0
            self._reset_products(create_volume=False)
            self._bound_axis_hint = None
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
        # finalize 总耗时起点（含重叠指标、TSDF 最终提取、refit 全链；
        # 成功/失败路径都记 last 值）
        t_finalize0 = self._algo_clock.now()
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
                    message += self._run_refit(
                        keep_last_good=False, mark_final=True)
            self.get_logger().info(message)
            self._harvest_data.append_event({
                'source': 'reconstruction', 'event': 'reconstruction_finalized',
                'target_id': self.collector.target_id,
                'captured_views': len(self.collector.frames),
                'refined': self._refined_info(),
                'grasp_decision': self._grasp_decision(),
            })
        else:
            self._overlap_cache = None
            self._tsdf_cloud_cache = None
            self._tsdf_info = None
            self._refined = None
            # E4：finalize 失败清空产物同样递增版本号（闩锁话题需覆盖刷新）
            self._bump_products_version(tsdf_cloud=True)
            self.get_logger().warning(message)
        self._timing.record_finalize((self._algo_clock.now() - t_finalize0) * 1000.0)
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
            # E4：提取失败清空产物，版本号递增保证闩锁话题覆盖旧内容
            self._bump_products_version(tsdf_cloud=True)
            self.get_logger().error(f'TSDF 最终提取失败（不影响 finalize）: {exc}')
            return f'；TSDF 提取失败（{exc}）'
        return (f'；TSDF {xyz.shape[0]} 点'
                f' / mesh {mesh_vertices} 顶点'
                f'（累计积分 {self._tsdf_volume.integrate_time_s:.2f}s）')

    def _run_refit(self, keep_last_good: bool = False,
                   mark_final: bool = False) -> str:
        """
        对当前 TSDF 云做几何二次拟合.

        采帧路径在每次全量 extract 后调用（keep_last_good=True）：失败保留
        上一帧成功结果，RViz 抓取示意连续更新。finalize 调用
        keep_last_good=False、mark_final=True：定稿供技能端抓取。
        抓取许可仍要求 collector.state==READY，在线 ACCEPT 不会提前放行。

        Returns
        -------
            追加到 finalize/采帧 message 的片段.

        """
        previous = self._refined if keep_last_good else None
        if self._tsdf_cloud_cache is None or not self._tsdf_cloud_cache[0].size:
            if previous and previous.get('ok'):
                return '；refit 跳过（无新 TSDF 云，保留上一帧）'
            self._refined = {'ok': False, 'reason': 'no_tsdf_cloud'}
            self._bump_products_version()  # E4：refined 写入递增产物版本号
            self.get_logger().warning('REFINING：无 TSDF 云，refit 跳过')
            return '；refit 跳过（无 TSDF 云）'
        xyz = self._tsdf_cloud_cache[0]
        kind, defaulted = self._resolve_target_kind()
        self.get_logger().info(
            f'REFINING：几何二次拟合开始（kind={kind}，{xyz.shape[0]} 点）')
        # refit 耗时（last 值）：几何拟合主体调用；成功/异常路径都落账
        # （异常说明拟合已执行且开销真实发生），无云跳过路径保持上一次值
        t_refit0 = self._algo_clock.now()
        try:
            result = select_refitter(self._refitters, kind).refit(
                xyz, kind, self.refit_config, self._bound_axis_hint)
            self._timing.record_refit((self._algo_clock.now() - t_refit0) * 1000.0)
        except Exception as exc:  # noqa: BLE001
            self._timing.record_refit((self._algo_clock.now() - t_refit0) * 1000.0)
            if previous and previous.get('ok'):
                self.get_logger().warning(
                    f'refit 异常，保留上一帧抓取示意: {exc}')
                return f'；refit 异常，保留上一帧（{exc}）'
            self._refined = {'ok': False, 'reason': f'exception:{exc}'}
            self._bump_products_version()  # E4：refined 写入递增产物版本号
            self.get_logger().warning(f'refit 异常（不影响 finalize）: {exc}')
            return f'；refit 失败（{exc}）'
        if defaulted:
            result.setdefault('flags', []).append('target_kind_defaulted')
        if not result['ok']:
            if previous and previous.get('ok'):
                self.get_logger().warning(
                    f"refit 未收敛，保留上一帧：{result['reason']}")
                return f'；refit 未更新（{result["reason"]}）'
            self._refined = {
                'ok': False, 'reason': result['reason'],
                'kind': kind, 'n_points': result['n_points']}
            self._bump_products_version()  # E4：refined 写入递增产物版本号
            self.get_logger().warning(
                f'refit 失败（不影响 finalize）：{result["reason"]}')
            return f'；refit 失败（{result["reason"]}）'
        result['final'] = bool(mark_final)
        self._refined = result
        self._bump_products_version()  # E4：refined 写入递增产物版本号
        # 绑定当帧会先 force 发空 Marker；0.2s 间隔门会把紧随其后的
        # 抓取示意压掉，而静止位姿不再采帧就永远闩在 DELETEALL。
        self._products_force_publish = True
        status_text = ('ACCEPT' if result['status'] == STATUS_ACCEPT
                       else 'REOBSERVE')
        self.get_logger().info(
            f"REFINING 完成：{result['kind']} status={status_text} "
            f"final={result['final']} "
            f"axis={np.round(result['axis'], 4).tolist()} "
            f"diameter={result['diameter'] * 1000.0:.1f}mm "
            f"rmse={result['rmse'] * 1000.0:.2f}mm "
            f"inlier={result['inlier_ratio']:.2f}")
        return (f"；refit {status_text}（{result['kind']}，"
                f"rmse {result['rmse'] * 1000.0:.1f}mm，"
                f"inlier {result['inlier_ratio']:.2f}）")

    def _on_finalize(self, request, response):
        """~/finalize_reconstruction：视角数达标则拼接全部帧发 local_cloud."""
        del request
        with self._state_lock:
            ok, message = self._finalize_now()
            response.success = ok
            response.message = message
            return response

    def _on_executor_state(self, msg: HarvestState) -> None:
        """批次执行器当前目标：覆盖感知 selected，作为重建绑定权威."""
        with self._state_lock:
            self._executor_state_seen = True
            self._executor_target_id = str(msg.target_id or '')

    def _wait_min_views(self, goal_handle, target_id: str, timeout_s: float):
        """等 min_views（或取消/超时）。反馈只在视角数变化时发."""
        min_views = int(self.params.capture.min_views)
        deadline = time.monotonic() + timeout_s
        last_n = -1
        n_frames, bound = 0, ''
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return 'canceled', n_frames, bound
            with self._state_lock:
                n_frames = len(self.collector.frames)
                bound = self.collector.target_id
                state = self.collector.state
            if n_frames != last_n:
                last_n = n_frames
                feedback = BuildTargetModel.Feedback()
                feedback.view_count = n_frames
                feedback.status = state
                goal_handle.publish_feedback(feedback)
            if n_frames >= min_views and (
                    not target_id or bound == target_id):
                return 'ready', n_frames, bound
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._view_progress.wait(timeout=min(0.5, remaining))
            self._view_progress.clear()
        return 'timeout', n_frames, bound

    def _on_build_target_model(self, goal_handle):
        """BuildTargetModel：reset 后绑定 goal.target_id，等 min_views 再 finalize."""
        goal = goal_handle.request
        self._on_reset(Trigger.Request(), Trigger.Response())
        with self._state_lock:
            if goal.target_id:
                self._executor_state_seen = True
                self._executor_target_id = goal.target_id
                self._preferred_target_id = goal.target_id
        status, n_frames, _bound = self._wait_min_views(
            goal_handle, goal.target_id,
            timeout_s=float(self.params.capture.build_timeout_s))
        if status in ('canceled', 'timeout'):
            result = BuildTargetModel.Result()
            result.success = False
            result.message = status
            result.quality_level = TargetQuality.LOW
            model = TargetModel()
            model.target_id = goal.target_id
            model.scene_epoch = goal.scene_epoch
            model.accepted = False
            model.message = status
            model.quality.level = result.quality_level
            model.view_count = n_frames
            result.model = model
            if status == 'canceled':
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        with self._state_lock:
            ok, message = self._finalize_now()
        result = BuildTargetModel.Result()
        result.success = bool(ok)
        result.message = message
        result.quality_level = (
            TargetQuality.HIGH if ok else TargetQuality.LOW)
        model = TargetModel()
        model.target_id = goal.target_id
        model.scene_epoch = goal.scene_epoch
        model.accepted = bool(ok)
        model.message = message
        model.quality.level = result.quality_level
        model.view_count = n_frames
        result.model = model
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _on_save_session(self, request, response):
        """~/save_session：全部已采帧落盘 session_<时间戳>/（含参数快照）."""
        del request
        # 落盘期间持锁：帧栈/TSDF 缓存快照须一致，worker 短暂阻塞属预期
        with self._state_lock:
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
    def _session_root(self) -> Path:
        """解析 session 落盘根目录：参数显式给优先，缺省 <工作区>/peach_sessions."""
        if self.params.session.root_dir:
            return Path(self.params.session.root_dir)
        # install/<pkg>/share/<pkg> → parents[3] 即工作区根
        share = Path(get_package_share_directory('peach_target_reconstruction'))
        return share.parents[3] / 'peach_sessions'

    def _session_metadata(self) -> dict:
        """参数快照与帧级摘要（随 metadata.yaml 落盘，供离线复现）."""
        c = self.collector
        return {
            'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'node': 'peach_target_reconstruction_node',
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
            'view_coverage': summarize_view_coverage(
                c.frames, c.target_center),
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
                'icp.target_refresh_min_period':
                    self.params.icp.target_refresh_min_period,
                'icp.target_refresh_max_period':
                    self.params.icp.target_refresh_max_period,
                'icp.target_refresh_drift_ratio':
                    self.params.icp.target_refresh_drift_ratio,
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
            'refined_result': self._refined_info(),
            # 耗时基线（与 diagnostics JSON 的 timing 子对象同契约同实例）
            'timing': self._timing.snapshot(),
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

    def destroy_node(self):
        """停止重建单写者 worker 后销毁 ROS 节点."""
        self._frame_worker.close(drain=False)
        return super().destroy_node()


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
    executor = MultiThreadedExecutor(num_threads=4)
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


if __name__ == '__main__':
    main()
