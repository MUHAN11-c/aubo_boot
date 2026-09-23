"""
连续运动局部重建节点（精确时间 FK + 有界 ICP + 在线 TSDF）.

默认工作流：IDLE 时 /peach/perception/initial_pose 有候选即自动绑定开始
（或由 BuildTargetModel 直接绑定）；COLLECTING 时每个唯一 RGB-D 时间戳均
进入质量门，合格即在线积分 TSDF；finalize 只提取最终网格并做几何 refit。
手动 Trigger 保留 reset / finalize / save_session / query 四口（Web 调试面
用）；采集由 auto 状态机驱动，`capture.auto_mode=false` 时仅 Build 路径可
开新会话。

每帧只按 depth.header.stamp 查 base←camera TF；失败跳帧，禁止运动中使用
latest TF。当前帧先由 FK 变到 base 系，再与已有 TSDF 表面做有界 ICP；
ICP 只修正小刚性误差，越界或低质量帧不进入不可回滚的 TSDF。
E4 效率项（协议 2.13-E4）：ICP target 经 integrate.IcpTargetCache
增量复用（每 k 帧自适应或关键事件才从 TSDF 全量 extract）；发布面
local_cloud/tsdf_cloud/markers 经 publish.PublishThrottle
on-change + 最小间隔节流（心跳/状态/诊断 1Hz 活性发布不动）.

线程模型：节点级 RLock 保护 collector/TSDF/产物（worker 与 executor 线程
双写收敛）；采帧门禁的阻塞式 TF 查询在锁外完成（_gated_capture_begin →
_query_tf → _finish 三段式），锁内按帧 stamp 复核收口，查询期间 Trigger
服务/目标观测回调/1Hz 心跳不被堵住（A7 起）.

模块边界（W4）：本文件为节点编排壳（decode → `session.process` → 入环；
三段式门禁/构云/提交/TSDF 积分原位保留）。自动状态机、帧环、掩膜缓存与
发布面本体在 reconstruction_core.ReconstructionCore（Mixin 留过渡薄壳）；
refit/融合编排在 refit_orchestrator.RefitOrchestrator（本节点只保留
``_refined``/``_bag_model`` 成对写入与产物版本记账）；session/geometry
落盘在 session_recorder；TargetModel/PregraspVerification 组装在 publish
公开函数。`ReconstructionSession.from_params` 装配柱/球 refitter；映射表
只在 refine.py。
"""
from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Optional, Tuple

import cv_bridge
from geometry_msgs.msg import Vector3Stamped
import message_filters
import numpy as np
from peach_common.lifecycle import break_bond, create_bond
from peach_harvester.vision.common.geometry import (
    transform_msg_to_matrix,
    transform_points,
)
from peach_harvester.vision.common.ros.clock_adapter import RclpyClockAdapter
from peach_harvester.vision.common.runtime import (
    BoundedWorker,
    resolve_runs_root,
)
from peach_harvester.vision.common.tool_budget import ToolBudgetParams
from peach_harvester.vision.target_reconstruction.capture import (
    AutoControllerMixin,
    BindSwitchHoldoff,
    capture_gate,
    CapturedFrame,
    CollectorConfig,
    FrameCollector,
    FrameStoreMixin,
    GATE_ALLOW,
    GATE_DENY,
    GATE_NEED_TF,
    GATE_SKIP,
    GateDecision,
    STATE_IDLE,
    StrictMaskGate,
    TimingStats,
)
from peach_harvester.vision.target_reconstruction.integrate import (
    BoundedIcp,
    IcpConfig,
    IcpTargetCache,
    IcpTargetRefreshConfig,
    LocalTsdf,
    Open3dCloudBuilder,
    summarize_view_coverage,
)
from peach_harvester.vision.target_reconstruction.params import TargetReconstructionParams
from peach_harvester.vision.target_reconstruction.publish import (
    build_pregrasp_verification,
    fill_target_model,
    lookup_tool_frame,
    PublisherMixin,
    PublishThrottle,
)
from peach_harvester.vision.target_reconstruction.reconstruction_core import (
    ReconstructionCore,
)
from peach_harvester.vision.target_reconstruction.refine import (
    axis_from_vector3,
    candidate_axis_hint,
    RefitConfig,
    select_reconstruction_candidate,
    STATUS_ACCEPT,
    TargetKindMemory,
)
from peach_harvester.vision.target_reconstruction.refit_orchestrator import (
    RefitOrchestrator,
)
from peach_harvester.vision.target_reconstruction.session import (
    ReconstructionSession,
)
from peach_harvester.vision.target_reconstruction.session_recorder import (
    save_session,
    SessionRecorder,
)
from peach_interfaces.action import BuildTargetModel
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    GraspDecision,
    HarvestState,
    PeachTargetObservationArray,
    PregraspVerification,
    ReconstructionStatus,
    ShapeHypothesis,
    TargetModel,
    TargetQuality,
)
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


class TargetReconstructionNode(
        AutoControllerMixin, FrameStoreMixin, PublisherMixin,
        ReconstructionCore, LifecycleNode):
    """连续运动局部重建 Lifecycle 节点：Active 后才积分与受理 BuildTargetModel."""

    params: TargetReconstructionParams
    """config/target_reconstruction.yaml 快照."""
    session: ReconstructionSession
    """柱/球 refitter 装配；process() 过深度门."""
    _refitters: dict
    """{'cylinder': …, 'sphere': …}，与 session.refitters 同一引用."""
    tf_timeout: Duration
    """精确 stamp TF 查询超时."""
    collector: FrameCollector
    """采帧状态机 + 视角过滤；不查 TF（共享状态宿主见 ReconstructionCore）."""

    def __init__(self):
        """建节点：参数层一行装载 → 直接构造算法 → 组装 Core → ROS 接线."""
        # Mixin 薄壳与 Core 都不提供 LifecycleNode.__init__ 兼容签名，
        # 显式初始化 ROS 基类（W4；行为与旧 super().__init__ 一致）
        LifecycleNode.__init__(self, 'peach_target_reconstruction_node')
        self.bridge = cv_bridge.CvBridge()
        # W14：nav2_lm 进程死检心跳句柄（configure 建 / deactivate-cleanup 断）
        self._bond = None
        # 协议 I3（时钟唯一）：节点时钟适配为纯核 Clock，一切计时走注入 now
        self._algo_clock = RclpyClockAdapter(self.get_clock())
        self.params = TargetReconstructionParams.attach(self)
        self.session = ReconstructionSession.from_params(self.params)
        self._refitters = self.session.refitters
        p = self.params
        # 派生量（ROS 类型/容器形态转换，非参数副本）
        self.tf_timeout = Duration(seconds=p.tf_timeout_sec)
        self.refit_config = RefitConfig(
            cylinder_inlier_min=p.refit.cylinder_inlier_min,
            rmse_max_m=p.refit.rmse_max_m,
            max_axis_angle_deg=p.refit.max_axis_angle_deg)
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

        # 数据持有者与算法实现：唯一实现直接构造（原 *.impl 注册缝位已收回）；
        # 柱/球两条精化线保留 yaml refitter.cylinder_impl / sphere_impl 映射。
        self.collector = FrameCollector(
            config=CollectorConfig(
                min_views=p.capture.min_views,
                recommended_views=p.capture.recommended_views,
                max_views=p.capture.max_views,
                min_translation=p.view_filter.min_translation,
                min_rotation_deg=p.view_filter.min_rotation_deg,
                auto_mode=p.capture.auto_mode,
                auto_finalize_at_max=p.capture.auto_finalize_at_max,
                auto_min_interval_s=p.capture.auto_min_interval_s,
            ))
        self._cloud_builder = Open3dCloudBuilder()
        self._icp_refiner = BoundedIcp(config=self.icp_config)
        self._mask_gate = StrictMaskGate(
            require_target_mask=p.capture.require_target_mask,
            min_mask_pixels=p.capture.min_mask_pixels,
            min_mask_depth_ratio=p.capture.min_mask_depth_ratio,
            max_target_drift_m=p.capture.max_target_drift_m,
            min_neighbor_gap_m=p.capture.min_neighbor_gap_m,
            neighbor_gap_area_ratio=p.capture.neighbor_gap_area_ratio,
            mask_stamp_tolerance_s=p.capture.mask_stamp_tolerance_s)
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
        # 耗时累计器（阶段 C 埋点）：ICP/TSDF 积分/帧总耗时 EMA + refit/
        # finalize last 值；计时打点用注入时钟（I3），快照进 diagnostics
        # JSON 的 timing 子对象与 session metadata
        self._timing = TimingStats()

        # —— 组装 Core（W4：Mixin 方法宿主；§3.3 属性盘点显式注入，N12）——
        ReconstructionCore.__init__(
            self,
            collector=self.collector,
            mask_gate=self._mask_gate,
            kind_memory=TargetKindMemory(),
            params=self.params,
            algo_clock=self._algo_clock,
            logger=self.get_logger(),
            timing=self._timing,
            throttle=self._publish_throttle,
            icp_target_cache=self._icp_target_cache)

        # refit 编排纯核（_run_refit 计算本体；成对写入约定见 _run_refit）
        self._refit_orchestrator = RefitOrchestrator(
            self._refitters, self.refit_config, self._timing,
            self.get_logger(), now=self._algo_clock.now)
        # session/geometry 落盘纯核（锁外写盘，W2/R4；参数快照走 snapshot）
        self._recorder = SessionRecorder(
            root_resolver=lambda: resolve_runs_root(
                self.params.session.root_dir))

        # —— 节点专属状态（共享状态均在 Core；_harvest_run_id/_scene_epoch
        # 等 Core 已初始化的属性不重复置位）——
        self._view_progress = threading.Event()
        self._executor_target_id = ''
        self._executor_state_seen = False
        self._target_observation_seen = False
        self._joint_states_seen = False
        self._max_joint_vel = 0.0  # [rad/s] 最近 /joint_states 的最大关节速度
        # BuildTargetModel 单槽重入护栏（goal 回调置位，执行体 finally 清零）
        self._build_goal_active = False
        # ROS 实体（发布器/订阅/服务/ActionServer/TF/心跳）统一在 on_configure
        # 创建（官方 LifecycleNode 写法：Unconfigured 期零 ROS 接口，configure
        # 失败即 ERROR），on_cleanup 释放；见 _wire_ros / _unwire_ros。
        self._ros_entities_wired = False

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

    def _wire_ros(self) -> None:
        """在 on_configure 创建全部 ROS 实体（发布器/订阅/服务/动作/TF/心跳）."""
        if self._ros_entities_wired:
            return
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
                'diagnostics 心跳超过 offered deadline 1.5s'
                f'（累计违约 {info.total_count} 次）：节点执行器疑似卡滞'),
            use_default_callbacks=False)
        self.pub_cloud = self.create_lifecycle_publisher(
            PointCloud2, '/peach/reconstruction/local_cloud', latched_qos)
        self.pub_status = self.create_lifecycle_publisher(
            String, '/peach/reconstruction/status', latched_qos)
        self.pub_diag = self.create_lifecycle_publisher(
            ReconstructionStatus, '/peach/reconstruction/diagnostics',
            diag_qos, event_callbacks=diag_pub_callbacks)
        # 完整调试明细（tsdf/registration/overlap/refined/逐机位）不进类型化
        # 消息，另发 String JSON 调试话题（同闩锁，仅供排查与 Web 明细镜像）
        self.pub_diag_debug = self.create_lifecycle_publisher(
            String, '/peach/reconstruction/diagnostics_debug', latched_qos)
        self.pub_grasp_decision = self.create_lifecycle_publisher(
            GraspDecision, '/peach/reconstruction/grasp_decision', latched_qos)
        self.pub_pregrasp = self.create_lifecycle_publisher(
            PregraspVerification,
            '/peach/reconstruction/pregrasp_verification', latched_qos)
        self.pub_markers = self.create_lifecycle_publisher(
            MarkerArray, '/peach/reconstruction/markers', latched_qos)
        self.pub_tsdf_cloud = self.create_lifecycle_publisher(
            PointCloud2, '/peach/reconstruction/tsdf_cloud', latched_qos)
        # refit 输出三件套（同为 transient_local 闩锁 + base_frame）
        self.pub_refined_pose = self.create_lifecycle_publisher(
            BagGraspCandidateArray, '/peach/reconstruction/refined_pose',
            latched_qos)
        self.pub_refined_axis = self.create_lifecycle_publisher(
            Vector3Stamped, '/peach/reconstruction/refined_axis', latched_qos)
        self.pub_refined_diag = self.create_lifecycle_publisher(
            BagFittingArray, '/peach/reconstruction/refined_diagnostics',
            latched_qos)
        self.pub_shape = self.create_lifecycle_publisher(
            ShapeHypothesis, '/peach/reconstruction/shape_hypothesis',
            latched_qos)

        # ---- 订阅：RGB-D 三件套（RELIABLE，与回放/驱动对齐）----
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        self._sub_rgb = message_filters.Subscriber(
            self, Image, self.params.camera.color_topic, qos_profile=qos)
        self._sub_depth = message_filters.Subscriber(
            self, Image, self.params.camera.depth_topic, qos_profile=qos)
        self._sub_info = message_filters.Subscriber(
            self, CameraInfo, self.params.camera.camera_info_topic,
            qos_profile=qos)
        self._frame_worker = BoundedWorker(
            self._process_rgbd, capacity=3, drop_oldest=False)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth, self._sub_info],
            queue_size=10, slop=self.params.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)
        default_qos = rclpy.qos.QoSProfile(depth=10)
        self._sub_initial = self.create_subscription(
            BagGraspCandidateArray, '/peach/perception/initial_pose',
            self._on_initial_pose, latched_qos)
        self._sub_target_obs = self.create_subscription(
            PeachTargetObservationArray,
            '/peach/perception/target_observations',
            self._on_target_observations, default_qos)
        # 感知诊断（target_id→target_kind）：refit 选圆柱/球拟合线的依据
        self._sub_perc_diag = self.create_subscription(
            BagFittingArray, '/peach/perception/diagnostics',
            self._on_perception_diagnostics, default_qos)
        self._sub_joint = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, default_qos)
        self._cb = ReentrantCallbackGroup()
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub_exec_state = self.create_subscription(
            HarvestState, '/peach_supervisor/state',
            self._on_executor_state, latched, callback_group=self._cb)

        # ---- 服务（std_srvs/Trigger，节点相对名；人工/阶段执行器调试口）----
        # 会改状态的入口统一过 Active 门（非 Active 只读口拒绝驱动
        # 状态机；query 只读不加门）。
        self._svc_reset = self.create_service(
            Trigger, '~/reset_reconstruction',
            self._active_gate(self._on_reset))
        self._svc_finalize = self.create_service(
            Trigger, '~/finalize_reconstruction',
            self._active_gate(self._on_finalize))
        self._svc_save = self.create_service(
            Trigger, '~/save_session', self._active_gate(self._on_save_session))
        self._svc_query = self.create_service(
            Trigger, '~/query_reconstruction_state',
            self._on_query_reconstruction_state)
        self._action_server = ActionServer(
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
        self._heartbeat_timer = self.create_timer(
            1.0, self._publish_heartbeat, callback_group=self._cb)
        self._ros_entities_wired = True

    def _unwire_ros(self) -> None:
        """on_cleanup 释放全部 ROS 实体（与 _wire_ros 一一对应）."""
        if not self._ros_entities_wired:
            return
        try:
            self.destroy_timer(self._heartbeat_timer)
        except Exception:  # noqa: BLE001 已释放则忽略
            pass
        try:
            for svc in (
                    self._svc_reset, self._svc_finalize, self._svc_save,
                    self._svc_query):
                self.destroy_service(svc)
        except Exception:  # noqa: BLE001
            pass
        try:
            for sub in (
                    self._sub_initial, self._sub_target_obs,
                    self._sub_perc_diag, self._sub_joint, self._sub_exec_state,
                    self._sub_rgb.sub, self._sub_depth.sub, self._sub_info.sub):
                self.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
        try:
            for pub in (
                    self.pub_cloud, self.pub_status, self.pub_diag,
                    self.pub_diag_debug, self.pub_grasp_decision,
                    self.pub_pregrasp,
                    self.pub_markers, self.pub_tsdf_cloud,
                    self.pub_refined_pose, self.pub_refined_axis,
                    self.pub_refined_diag, self.pub_shape):
                self.destroy_lifecycle_publisher(pub)
        except Exception:  # noqa: BLE001
            pass
        try:
            self._action_server.destroy()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.destroy_subscription(self.tf_listener.tf_sub)
            self.destroy_subscription(self.tf_listener.tf_static_sub)
        except Exception:  # noqa: BLE001
            pass
        self._ros_entities_wired = False

    def on_configure(self, state):
        del state
        try:
            self._wire_ros()
        except Exception as exc:  # noqa: BLE001 接线失败（话题/参数非法）整包停走
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.ERROR
        # W14：nav2_lm 进程死检心跳（缺 ros-jazzy-bondpy 时守卫降级为 WARN）
        self._bond = create_bond(self, self.get_name())
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        result = LifecycleNode.on_activate(self, state)
        self._lifecycle_active = True
        if self._bond is None:
            self._bond = create_bond(self, self.get_name())  # deactivate 后重臂
        # 激活后首发一次（IDLE + 空云），闩锁话题让后启动的订阅者立即可读
        self._publish_all()
        self.get_logger().info('reconstruction Active：开始积分')
        return result

    def on_deactivate(self, state):
        self._lifecycle_active = False
        break_bond(self._bond)
        self._bond = None
        return LifecycleNode.on_deactivate(self, state)

    def on_cleanup(self, state):
        self._lifecycle_active = False
        break_bond(self._bond)
        self._bond = None
        self._unwire_ros()
        return LifecycleNode.on_cleanup(self, state)

    def _on_build_goal(self, goal_request):
        del goal_request
        if not self._lifecycle_active:
            return GoalResponse.REJECT
        # 单槽重入护栏：上一 Build 未结束（执行体重入/取消后立即重试）时
        # 拒绝第二个 goal——否则后到的 reset 会清掉前者的帧栈与绑定，
        # 双方都在错误绑定上等到 timeout/abort。
        if self._build_goal_active:
            self.get_logger().warning('BuildTargetModel 拒绝：已有 Build 在跑')
            return GoalResponse.REJECT
        self._build_goal_active = True
        return GoalResponse.ACCEPT

    def _on_build_cancel(self, cancel_request):
        del cancel_request
        self._view_progress.set()
        return CancelResponse.ACCEPT

    def _active_gate(self, handler):
        """Trigger 服务统一 Active 门：非 Active 拒绝驱动重建状态机."""
        def gated(request, response):
            if not self._lifecycle_active:
                response.success = False
                response.message = 'reconstruction 节点非 Active'
                return response
            return handler(request, response)
        return gated

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
        """Decode RGB-D, ingest, then push the frame ring."""
        rgb_msg, depth_msg, info = frame
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            rgb_stamp = rgb_msg.header.stamp
            depth_stamp = depth_msg.header.stamp
            self.get_logger().warning(
                f'图像解码失败，丢帧 rgb_frame={rgb_msg.header.frame_id} '
                f'rgb_stamp={rgb_stamp.sec}.{rgb_stamp.nanosec:09d} '
                f'depth_frame={depth_msg.header.frame_id} '
                f'depth_stamp={depth_stamp.sec}.{depth_stamp.nanosec:09d}: {exc}')
            return
        ingested = self.session.process(rgb, depth_raw, info.k)
        if ingested.frame is None:
            self.get_logger().warning(ingested.reason)
            return
        # TF 查询按深度图时间戳（相机 HW 时间戳，常超前机器人 TF）
        stamp_msg = depth_msg.header.stamp
        stamp_sec = float(stamp_msg.sec) + float(stamp_msg.nanosec) * 1e-9
        cam_frame = depth_msg.header.frame_id or rgb_msg.header.frame_id
        self._push_frame_ring(
            (ingested.frame.rgb, ingested.frame.depth_mm, ingested.frame.K,
             stamp_msg, stamp_sec, cam_frame))
        # 自动模式：每个新同步帧驱动一次（自动开始/采帧/完成）；
        # auto_mode=false 时不走这里，改用纯手动 Trigger 服务流
        if self.params.capture.auto_mode:
            self._auto_drive()

    def _on_initial_pose(self, msg: BagGraspCandidateArray):
        """缓存最新感知候选（启动重建时绑定最优目标用）."""
        self._latest_candidates = msg
        # 轴 hint 按 base 系解释；tf_unavailable 帧感知退相机系，混入会
        # 把错误坐标系的轴向喂给 refit/融合（绑定侧 select 已有同款门）。
        if msg.header.frame_id != self.params.frames.base_frame:
            return
        with self._state_lock:
            bound_id = self.collector.target_id
            hint = candidate_axis_hint(msg, bound_id)
            if hint is not None:
                self._bound_axis_hint = hint

    def _on_target_observations(
            self, msg: PeachTargetObservationArray) -> None:
        """缓存当前绑定目标 ID 的精确深度时刻掩膜与中心，并刷新锁定集锚点."""
        # 目标切换会 reset 帧栈并清空产物，须与 worker 线程互斥。
        # 2026-09-09 锁手术：锁域收窄为两段短临界（绑定切换/缓存写），
        # 会话文件 IO、_publish_all 组装序列化、掩膜解码与几何计算全部
        # 移出锁外——旧版持锁全程把本回调与心跳饿到分钟级滞后
        # （真机实测心跳违约 57 次、绑定延迟 73s）。
        publish_after = False
        link_event = False
        with self._state_lock:
            self._target_observation_seen = True
            if msg.harvest_run_id != self._harvest_run_id:
                self._harvest_run_id = msg.harvest_run_id
                self._harvest_data.attach(self._harvest_run_id)
                link_event = True
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
                self._preferred_target_id = requested_target_id
                publish_after = True
            elif action in (BindSwitchHoldoff.PEND, BindSwitchHoldoff.WAIT):
                # 挂起中：不更新 _preferred_target_id，下方掩膜缓存与邻目标
                # 锚点仍按旧绑定目标刷新（会话零扰动）
                pass
            else:
                # FOLLOW（无会话可毁/未偏离/未绑定）与 CANCEL（holdoff 内
                # 切回原 ID）均直通；requested==bound 时赋值幂等
                self._preferred_target_id = requested_target_id
            preferred = self._preferred_target_id
        if link_event:
            self._harvest_data.append_event({
                'source': 'reconstruction',
                'event': 'reconstruction_linked'})
        if publish_after:
            # 组装/序列化重活，锁外补发（COMMIT 清屏走 force 旁路）
            self._publish_all()
        # 几何缓存（轴 hint/邻目标锚点/掩膜中心）全部按 base 系解释：
        # 感知 tf_unavailable 帧退相机系，混入会让漂移门/串扰门按
        # 错误坐标系算距离（绑定侧 select_reconstruction_candidate
        # 已有同款 frame 门）。ID/会话切换与帧无关，不受此门影响。
        if msg.header.frame_id != self.params.frames.base_frame:
            return
        # 掩膜缓存按当前绑定目标（防抖期=旧目标）取观测；绑定目标本帧
        # 无观测/非 OBSERVED/无掩膜时本帧不更新缓存
        bound_obs = next((item for item in msg.observations
                          if item.target_id == preferred), None)
        hint = None
        if bound_obs is not None:
            hint = axis_from_vector3(
                bound_obs.candidate.translation_direction)
        # E2 邻目标串扰门数据源：每条观测消息全量重建锁定集锚点缓存
        # （绑定目标自身在 _target_mask_for_frame 组 MaskContext 时剔除；
        # 未锁定时 observations 恒空，缓存随之为空）。同步记录检测框
        # 面积（像素²）：串扰门对「远小于本目标的框」豁免——小框多为
        # 叶片遮挡残片/误检（09-01 现场 58.5 mm 近距即此类），大框先行。
        centers = {}
        areas = {}
        for item in msg.observations:
            c = self._candidate_center(item.candidate)
            if c is not None:
                centers[item.target_id] = c
            box = item.candidate_2d
            if box.bbox_w > 0 and box.bbox_h > 0:
                areas[item.target_id] = float(box.bbox_w) * float(box.bbox_h)
        mask_entry = None
        drive = bound_obs is not None
        if (bound_obs is None
                or bound_obs.tracking_status != bound_obs.OBSERVED):
            drive = False
        elif not bound_obs.mask.data:
            drive = False
        else:
            try:
                mask = self.bridge.imgmsg_to_cv2(
                    bound_obs.mask, desired_encoding='mono8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'目标掩膜解码失败: {exc}')
                drive = False
            else:
                stamp = bound_obs.mask.header.stamp
                mask_entry = (
                    self._stamp_ns(stamp),
                    np.asarray(mask, dtype=np.uint8),
                    self._candidate_center(bound_obs.candidate))
        with self._state_lock:
            if hint is not None:
                self._bound_axis_hint = hint
            self._locked_target_centers = centers
            self._locked_target_areas = areas
            if mask_entry is not None:
                stamp_ns, mask_arr, center = mask_entry
                self._target_masks[stamp_ns] = (mask_arr, center)
                while len(self._target_masks) > 30:
                    self._target_masks.pop(next(iter(self._target_masks)))
        if not drive:
            return
        # 自动驱动可能执行精确时刻 TF 查询、采帧与 TSDF/refit；必须在外层
        # 观测缓存锁释放后进入。否则 BuildTargetModel 的 reset/bind 会被饿死。
        if self.params.capture.auto_mode:
            self._auto_drive()

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
        latest TF，因为错时位姿会在 TSDF 中形成不可回滚的双层表面.

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
    def _best_candidate(self) -> Tuple[str, Optional[np.ndarray]]:
        """取全局计划选中且坐标系与 TF 诊断均安全的候选."""
        if (self.params.capture.require_target_mask
                and not self._preferred_target_id):
            return '', None
        return select_reconstruction_candidate(
            self._latest_candidates, self.params.frames.base_frame,
            self._preferred_target_id)

    def _collect_gate_values(self, automatic: bool,
                             prefer_stamp_sec=None,
                             prefer_cam_frame=None
                             ) -> Tuple[dict, Optional[tuple]]:
        """
        采集采帧门禁判据快照（须持 _state_lock；只读共享状态，零副作用）.

        Args:
            automatic: True=自动模式 / False=手动服务（透传进判据）.
            prefer_stamp_sec: 指定帧时间戳（TF 收口复核用）.
            prefer_cam_frame: 指定相机系名.

        Returns
        -------
            (gate_values, frame_snapshot)：frame_snapshot 为
            (rgb, depth_mm, K, stamp_msg, stamp_sec, cam_frame, target_mask)，
            无缓存帧时为 None.

        """
        cached = self._select_cached_frame(
            prefer_stamp_sec=prefer_stamp_sec,
            prefer_cam_frame=prefer_cam_frame)
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
        _gated_capture_query_tf，再重新持锁调 _gated_capture_finish 收口.

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
        缓存帧是否仍有效由锁内 _gated_capture_finish 按 stamp 复核.

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

        竞态收口：TF 查询在锁外完成，期间帧环可能追加新帧、帧栈可能被
        reset/收满/已采入同帧。本段按查询 stamp 从帧环取同一帧复核——
        查询中到达的更新帧不覆盖这次尝试；查询帧已滚出环则丢弃（自动
        =skip、手动=deny 提示重试）。绝不把旧帧时刻的位姿套到新帧上.

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
        gate_values, snapshot = self._collect_gate_values(
            automatic,
            prefer_stamp_sec=query_stamp_sec,
            prefer_cam_frame=query_cam_frame)
        if snapshot is None:
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

    def _crop_for_icp(self, cloud_fk, cloud_rgb):
        """把 ICP 输入裁到目标局部盒，避免背景主导刚体修正."""
        center = self._roi_center()
        if center is not None and cloud_fk.size:
            return LocalTsdf.crop_to_box(
                cloud_fk, cloud_rgb, center, self._local_volume)
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
        """积分当前帧；体积失败才回滚。袋融合失败保留体积，该帧仍算采入."""
        if not self.params.tsdf.enable:
            return None
        refreshed = False
        try:
            if self._tsdf_volume is None:
                self._tsdf_volume = self._create_volume()
            t_tsdf0 = self._algo_clock.now()
            self._tsdf_volume.integrate_frame(
                rgb, masked_depth, K, T_used)
            if self._icp_target_cache.should_refresh():
                self._refresh_tsdf_outputs()
                refreshed = True
            else:
                self._icp_target_cache.append_frame(cloud_base)
            self._timing.record_tsdf_integrate(
                (self._algo_clock.now() - t_tsdf0) * 1000.0)
        except Exception as exc:  # noqa: BLE001
            self.collector.remove_last()
            self._tsdf_volume = self._create_volume()
            # W2/R2：体积重建即模型重建，ICP target 复用缓存按其契约作废
            # （回放后由 _refresh_tsdf_outputs 的 set_full 重建基线），
            # 防止回放窗口内采帧路径取用 stale target。
            self._icp_target_cache.invalidate()
            for old in self.collector.frames:
                self._tsdf_volume.integrate_frame(
                    old.rgb, old.depth_mm, old.camera_K, old.T_base_camera)
            self._refresh_tsdf_outputs()
            return f'TSDF 在线积分失败: {exc}'
        if refreshed and self.params.refit.enable:
            try:
                self._run_refit(keep_last_good=True, mark_final=False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f'REFINING 失败（TSDF 体积已保留）: {exc}')
        return None

    def _prepare_frame(self, context, target_id):
        """
        采帧锁外纯计算段：FK 构云 → 有界帧到模型 ICP → CapturedFrame 组装.

        只读入参快照与 ICP 目标缓存，不触碰 collector/TSDF/计数等共享
        状态，因此不得持 _state_lock 调用（2026-09-09 锁手术：本段是
        每帧最重的 CPU 活，锁外执行解饿心跳/观测回调）。

        Args:
            context: _gated_capture_finish 的 ALLOW 上下文
                (rgb, depth_mm, K, stamp_sec, T_base_camera, tf_status,
                target_mask).
            target_id: finish 锁内快照的绑定目标，帧归属以此为准（提交段
                复核会话未变后入库）.

        Returns
        -------
            (payload, None) 或 (None, 拒帧原因)；payload =
            (frame, ratio, mode, reg_info)。

        """
        (rgb, depth_mm, K, stamp_sec,
         T_base_camera, tf_status, target_mask) = context
        try:
            # masked_depth 复用构云时已算好的掩膜结果（免全图级二次
            # apply_target_mask）；ratio 语义不变（掩膜内有效/掩膜像素）。
            cloud_fk, cloud_rgb, ratio, masked_depth = \
                self._cloud_builder.build(
                    depth_mm, rgb, K, T_base_camera,
                    target_mask=target_mask)
        except (RuntimeError, ValueError) as exc:
            return None, f'点云构建失败: {exc}'
        if tf_status != 'ok':
            return None, '非精确时间 TF 帧禁止进入 TSDF'

        cloud_fk, cloud_rgb = self._crop_for_icp(cloud_fk, cloud_rgb)
        cached_target = self._icp_target_cache.current_target()
        target = (np.zeros((0, 3), dtype=np.float64)
                  if cached_target is None else cached_target)
        ok, correction, mode, reg_info, err = self._register_cloud(
            cloud_fk, target)
        if not ok:
            return None, err
        T_used = correction @ np.asarray(T_base_camera, dtype=np.float64)
        cloud_base = transform_points(cloud_fk, correction)
        flags = [f'pose_{mode}']
        if reg_info['reason'] not in ('accepted', 'model_warmup',
                                      'icp_disabled'):
            flags.append(reg_info['reason'])
        frame = CapturedFrame(
            rgb=rgb, depth_mm=masked_depth, camera_K=K, stamp=stamp_sec,
            T_base_camera=T_used, T_base_camera_fk=T_base_camera,
            target_id=target_id,
            valid_depth_ratio=ratio, cloud_base=cloud_base,
            cloud_rgb=cloud_rgb, diagnostic_flags=flags,
            registration=reg_info)
        return (frame, ratio, mode, reg_info), None

    def _commit_prepared_frame(self, prepared, t_frame0):
        """
        采帧锁内提交段（须持 _state_lock）：入库 → TSDF 积分 → 计数落账.

        ICP 与 FK 预对齐都不合格的帧已在 _prepare_frame 拒掉；此处只为
        短临界（毫秒级），心跳/观测回调不再被构云/积分饿死。

        Args:
            prepared: _prepare_frame 的成功 payload。
            t_frame0: 帧总耗时起点（注入时钟，I3），在进入准备段前取；
                覆盖构云→ICP→入库→TSDF 积分全链，仅成功收帧时计入
                frame_total EMA（拒帧早退不污染基线）.

        Returns
        -------
            (accepted, message)。

        """
        frame, ratio, mode, reg_info = prepared
        if not self.collector.add_frame(frame):
            return False, f'已达 max_views={self.params.capture.max_views}'
        tsdf_err = self._integrate_tsdf(
            frame.rgb, frame.depth_mm, frame.camera_K, frame.T_base_camera,
            frame.cloud_base)
        if tsdf_err:
            return False, tsdf_err

        self._last_captured_stamp_sec = frame.stamp
        # 新帧使 overlap/mesh 失效；精化在 extract 后现场重拟（keep 上一帧
        # 成功结果），运动中 RViz 抓取示意连续更新而不是清屏。
        self._overlap_cache = None
        self._mesh_cache = None
        n = len(self.collector.frames)
        message = (
            f'已采第 {n}/{self.params.capture.recommended_views} 视角，'
            f'本帧 {frame.cloud_base.shape[0]} 点，有效深度占比 {ratio:.2f}，'
            f'位姿={mode}')
        self.get_logger().info(message)
        self._harvest_data.append_event({
            'source': 'reconstruction', 'event': 'frame_accepted',
            'target_id': self.collector.target_id,
            'stamp_ns': int(round(frame.stamp * 1000000000.0)),
            'view_index': n, 'mask_depth_ratio': float(ratio),
            'registration': reg_info,
        })
        # 帧总耗时落账（成功路径终点；发布开销由心跳侧另计）
        self._timing.record_frame_total(
            (self._algo_clock.now() - t_frame0) * 1000.0)
        self._view_progress.set()
        return True, message

    def _on_reset(self, request, response):
        """~/reset_reconstruction：清空帧栈与绑定目标，回 IDLE."""
        del request
        with self._state_lock:
            self.collector.reset()
            self._view_progress.clear()
            self._target_kind_memory.reset()
            self._last_captured_stamp_sec = -1.0
            self._clear_frame_ring()
            self._reset_products(create_volume=False)
            self._bound_axis_hint = None
            response.success = True
            response.message = '已清空，回 IDLE'
            self.get_logger().info(response.message)
            self._publish_all()
            return response

    def _run_refit(self, keep_last_good: bool = False,
                   mark_final: bool = False) -> str:
        """
        refit/袋融合编排壳（计算本体在 RefitOrchestrator.run）.

        本方法只保留缓存成对写入与产物版本记账。融合数学与门控语义见
        refit_orchestrator/refine docstring（W4 自本方法下沉，零改动）。

        Returns
        -------
            Suffix appended to the finalize/capture message.

        """
        previous = self._refined if keep_last_good else None
        kind, defaulted = self._resolve_target_kind()
        if (self._tsdf_cloud_cache is not None
                and self._tsdf_cloud_cache[0].size):
            tsdf_xyz = self._tsdf_cloud_cache[0]
        else:
            tsdf_xyz = None
        result, fused = self._refit_orchestrator.run(
            tsdf_xyz=tsdf_xyz,
            frames=list(self.collector.frames),
            target_center=self.collector.target_center,
            kind=kind,
            kind_defaulted=defaulted,
            bound_axis_hint=self._bound_axis_hint,
            target_id=self.collector.target_id,
            entry_standoff_m=float(self.params.refit.entry_standoff_m),
            pregrasp_standoff_m=float(self.params.refit.pregrasp_standoff_m),
            # 许可数学内径随当前工具档案（tool_profile launch 注入）
            budget_params=ToolBudgetParams(
                d_inner=float(self.params.tool.budget.d_inner)),
            mark_final=mark_final,
            previous=previous,
            on_view=self._on_view_geometry)
        if result.ok and result.budget:
            # _refined/_bag_model 须成对写入：GraspDecision 读 _refined、
            # PregraspVerification/TargetModel 读 _bag_model，只写一个会让
            # 同一时刻「许可与残差观测」互相矛盾。
            self._refined = result
            self._bag_model = fused
            self._log_geometry_row(result, fused)
            self._bump_products_version()
            self._products_force_publish = True
            status_text = (
                'ACCEPT' if result.status == STATUS_ACCEPT else 'REOBSERVE')
            return f'；refit {status_text}（袋模型 {fused.view_count} 视）'
        if (previous is not None and previous.ok and previous.budget):
            # keep_last_good：两个缓存都不动（_bag_model 由成对写入维护），
            # 避免旧 good _refined 配新失败 _bag_model 的矛盾对。
            return f'；refit 未更新（{result.reason}）'
        self._refined = result
        self._bag_model = fused
        self._bump_products_version()
        return f'；refit 失败（{result.reason}）'

    def _on_finalize(self, request, response):
        """~/finalize_reconstruction：机位与帧数达标则拼接全部帧发 local_cloud."""
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
            # 单根会话目录（R7）：批次 request_id 驱动 session/geometry 与
            # 事件库基目录（与感知 datastore 同一 runs/<request_id>/ 根）
            executor_run_id = str(msg.run_id or '')
            self._scene_epoch = int(getattr(msg, 'scene_epoch', 0) or 0)
            self._harvest_data.base_dir = (
                resolve_runs_root(None) / executor_run_id
                / 'perception_data'
                if executor_run_id else None)
            self._recorder.bind_executor_run_id(executor_run_id)

    def _wait_min_views(self, goal_handle, target_id: str, timeout_s: float):
        """等独立机位数与角基线同时达标（或取消/超时）."""
        capture = self.params.capture
        min_views = int(capture.min_views)
        # I3（时钟唯一）：等待/超时走注入 _algo_clock（旧 time.monotonic
        # 直取违反协议；Event 机制不动）
        deadline = self._algo_clock.now() + timeout_s
        last_poses = -1
        pose_count, bound = 0, ''
        while self._algo_clock.now() < deadline:
            if goal_handle.is_cancel_requested:
                return 'canceled', pose_count, bound
            with self._state_lock:
                frames = list(self.collector.frames)
                center = (None if self.collector.target_center is None
                          else self.collector.target_center.copy())
                bound = self.collector.target_id
                state = self.collector.state
            coverage = summarize_view_coverage(frames, center)
            pose_count = int(coverage.get('view_count') or 0)
            coverage_ready = (
                bool(coverage.get('valid'))
                and pose_count >= min_views
                and float(coverage.get('max_baseline_deg') or 0.0) >=
                float(capture.minimum_baseline_deg)
                and float(coverage.get('mean_nearest_baseline_deg') or 0.0) >=
                float(capture.minimum_mean_nearest_baseline_deg)
                and float(coverage.get('valid_depth_ratio_mean') or 0.0) >=
                float(capture.minimum_mean_depth_ratio)
            )
            if pose_count != last_poses:
                last_poses = pose_count
                feedback = BuildTargetModel.Feedback()
                feedback.view_count = pose_count
                feedback.status = state
                goal_handle.publish_feedback(feedback)
            if (target_id and bound == target_id and coverage_ready):
                return 'ready', pose_count, bound
            remaining = deadline - self._algo_clock.now()
            if remaining <= 0.0:
                break
            self._view_progress.wait(timeout=min(0.5, remaining))
            self._view_progress.clear()
        return 'timeout', pose_count, bound

    def _on_build_target_model(self, goal_handle):
        """BuildTargetModel：reset 后绑定 goal.target_id，等机位数再 finalize."""
        try:
            return self._build_target_model_body(goal_handle)
        finally:
            self._build_goal_active = False

    def _build_target_model_body(self, goal_handle):
        """Build 主体：reset→绑定→强制 COLLECTING→等机位数→finalize（槽位护栏见外层）."""
        goal = goal_handle.request
        if not goal.target_id:
            result = BuildTargetModel.Result()
            result.success = False
            result.message = 'empty_target_id'
            result.quality_level = TargetQuality.LOW
            model = TargetModel()
            model.accepted = False
            model.message = result.message
            result.model = model
            goal_handle.abort()
            return result
        self._on_reset(Trigger.Request(), Trigger.Response())
        with self._state_lock:
            if goal.target_id:
                self._executor_state_seen = True
                self._executor_target_id = goal.target_id
                self._preferred_target_id = goal.target_id
            # Build 刚 reset 回 IDLE；必须立刻进 COLLECTING，否则观察段
            # 走动时重建仍无绑定，质量门 reconstruction_unbound，Build 空等。
            self._auto_start()
            if self.collector.state == STATE_IDLE and goal.target_id:
                center = self._locked_target_centers.get(goal.target_id)
                message = self.collector.start(goal.target_id, center)
                self._target_kind_memory.bind(goal.target_id)
                self._last_captured_stamp_sec = -1.0
                self._reset_products(create_volume=True)
                self._bound_axis_hint = candidate_axis_hint(
                    self._latest_candidates, goal.target_id)
                self.get_logger().info(f'BuildTargetModel 强制开始：{message}')
                self._publish_all()
        with self._state_lock:
            bound_state = self.collector.state
        started = BuildTargetModel.Feedback()
        started.view_count = 0
        started.status = bound_state
        goal_handle.publish_feedback(started)
        status, pose_count, _bound = self._wait_min_views(
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
            model.view_count = pose_count
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
        model.view_count = pose_count
        self._fill_target_model(model)
        result.model = model
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _on_save_session(self, request, response):
        """~/save_session：全部已采帧落盘 session_<时间戳>/（含参数快照）."""
        del request
        # W2/R4：锁内只取一致快照，多帧 PNG/PLY 同步 IO 全部移出锁外——
        # 旧版持锁写盘秒级阻塞心跳（diag offered deadline 1.5s 违约风险）。
        with self._state_lock:
            frames = list(self.collector.frames)
            bound_target_id = str(self.collector.target_id or '')
            if not frames:
                response.success = False
                response.message = '无已采帧，未落盘'
                self.get_logger().warning(response.message)
                self._publish_all()
                return response
            metadata = self._session_metadata()
            tsdf_cloud = self._tsdf_cloud_cache
            tsdf_mesh = self._mesh_cache
            session_root = self._session_root()
        try:
            session_dir = save_session(
                session_root, frames, metadata,
                tsdf_cloud=tsdf_cloud,
                tsdf_mesh=tsdf_mesh)
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
            'target_id': bound_target_id,
            'session_dir': str(session_dir),
        })
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    # ------------------------------------------------------------------
    # 落盘薄壳（本体在 session_recorder；W4）
    # ------------------------------------------------------------------
    def _session_root(self) -> Path:
        """Session 根（本体 session_recorder.SessionRecorder；R7 单根 + W1 净化）."""
        return self._recorder.session_root()

    def _geometry_root(self) -> Path:
        """geometry.jsonl 根（本体 session_recorder.SessionRecorder；R7 批根）."""
        return self._recorder.geometry_root()

    def _session_metadata(self) -> dict:
        """参数快照与帧级摘要（组装在 SessionRecorder；快照走 params.snapshot）."""
        return self._recorder.session_metadata(
            collector=self.collector,
            parameters=self.params.snapshot(),
            harvest_run_id=self._harvest_run_id,
            selected_target_id=self._preferred_target_id,
            target_mask_cache_size=len(self._target_masks),
            tsdf_result=self._tsdf_info,
            refined_result=self._refined_info(),
            timing=self._timing.snapshot())

    def _log_geometry_row(self, result, fused) -> None:
        """追加 geometry.jsonl 融合行（本体 SessionRecorder.geometry_row）."""
        self._recorder.geometry_row(result, fused, self.collector.target_id)

    def _on_view_geometry(self, landmarks, frame) -> None:
        """逐视角 geometry.jsonl 行（RefitOrchestrator on_view 回调）."""
        self._recorder.view_row(landmarks, frame, self.collector.target_id)

    # ------------------------------------------------------------------
    # 消息组装薄壳（本体在 publish 公开函数；W4）
    # ------------------------------------------------------------------
    def _pregrasp_verification_msg(self, header):
        """
        工具帧相对融合袋模型的预抓取残差（观测用，不授权 SetIO）.

        W2/R3：本方法双路并发可达（心跳持 _state_lock vs _publish_all 锁外），
        状态快照与 `_pregrasp_prev` 交换必须入锁（RLock：心跳路径重入安全）；
        三次 latest TF 查询留在锁外（1s 级超时不得占锁）。组装本体在
        publish.build_pregrasp_verification。
        """
        with self._state_lock:
            target_id = str(self.collector.target_id or '')
            fused = self._bag_model
            result = self._refined
            previous = self._pregrasp_prev
        tool_frames = None
        if fused is not None and fused.ok:
            # 袋模型不可用时跳过 TF 查询（build 早退 bag_model_unavailable）
            base = self.params.frames.base_frame
            mouth, _ = lookup_tool_frame(self.tf_buffer, base, 'sleeve_mouth')
            _, tool_z = lookup_tool_frame(self.tf_buffer, base, 'tool_axis')
            blade, _ = lookup_tool_frame(
                self.tf_buffer, base, 'cutting_plane')
            tool_frames = (mouth, tool_z, blade)
        msg, eval_row = build_pregrasp_verification(
            header, fused, result, tool_frames, previous, self.params)
        msg.target_id = target_id
        if eval_row is not None:
            with self._state_lock:
                self._pregrasp_prev = eval_row
        return msg

    def _fill_target_model(self, model: TargetModel) -> None:
        """把融合袋模型写入 TargetModel 扩展字段（本体 publish.fill_target_model）."""
        fill_target_model(
            model, self._bag_model, self._refined, self.params,
            self.get_clock().now(),
            run_id=self._harvest_run_id,
            scene_epoch=self._scene_epoch)

    def destroy_node(self):
        """停止重建单写者 worker 与事件落盘线程后销毁 ROS 节点."""
        self._frame_worker.close(drain=False)
        self._harvest_data.close(drain=True)
        return LifecycleNode.destroy_node(self)


def main(args=None):
    """
    节点入口：rclpy 初始化 → TargetReconstructionNode spin → 干净收尾.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = TargetReconstructionNode()
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
