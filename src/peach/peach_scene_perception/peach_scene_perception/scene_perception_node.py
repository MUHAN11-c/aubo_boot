"""
场景感知 ROS 2 节点（图名 `peach_scene_perception_node`）.

Lifecycle + 接线。热路径：decode_rgbd → pipeline.process → publish。
W3 起计划推进/观测组装材料在 plan_updater（纯核），消息组装在
msg_builders，像素绘制在 debug_draw；节点只持锁、发消息、落盘。
"""
from __future__ import annotations

import json
import time
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
import message_filters
import numpy as np
from peach_common.lifecycle import break_bond
from peach_common.lifecycle import create_bond
from peach_common.qos import latched
from peach_common.qos import stream
from peach_common.ros_log_bridge import bridge_module_loggers
from peach_interfaces.msg import BagFittingArray
from peach_interfaces.msg import BagGraspCandidateArray
from peach_interfaces.msg import HarvestState
from peach_interfaces.msg import PeachTargetObservation
from peach_interfaces.msg import PeachTargetObservationArray
from peach_interfaces.srv import BeginScene
from peach_scene_perception.image_codec import bgr8_view
from peach_scene_perception.image_codec import depth16_view
from peach_scene_perception.motion_gate import arm_is_settled
from peach_scene_perception.msg_builders import bbox_cloud_xyzrgb
from peach_scene_perception.msg_builders import best_axis_direction
from peach_scene_perception.msg_builders import quat_to_msg
from peach_scene_perception.msg_builders import to_detection2d
from peach_scene_perception.msg_builders import TRACKING_STATUS_TO_MSG
from peach_scene_perception.msg_builders import xyzrgb_to_cloud_msg
from peach_scene_perception.params import ScenePerceptionParams
from peach_scene_perception.pipeline import PerceptionPipeline
from peach_scene_perception.pipeline import SyncedRgbd
from peach_scene_perception.plan_updater import harvest_state_dict
from peach_scene_perception.plan_updater import PlanUpdater
from peach_vision_core.clock_adapter import RclpyClockAdapter
from peach_vision_core.geometry import camera_intrinsics
from peach_vision_core.geometry import gravity_camera_from_R
from peach_vision_core.geometry import normalize_depth_to_uint16_mm
from peach_vision_core.geometry import transform_msg_to_matrix
from peach_vision_core.runtime import BoundedWorker
from peach_vision_core.runtime import HarvestDataStore
from peach_vision_core.runtime import view_stage_for_phase
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ScenePerceptionNode(LifecycleNode):
    """RGB-D 感知 Lifecycle 节点：Active 后才处理帧并受理 BeginScene."""

    params: ScenePerceptionParams
    """config/scene_perception.yaml 快照；逐帧键可热更新."""
    pipeline: PerceptionPipeline
    """检测→分割→袋位姿；计划/身份/锁都在管线上，节点不另抄别名."""
    plan_updater: PlanUpdater
    """帧级计划推进（W3 纯核）；本节点持 plan_lock 后调用."""
    harvest_data: HarvestDataStore
    """runs/.../perception_data 事件与掩膜；不写调度 ledger.json."""
    harvest_run_id: str = ''
    """锁定瞬间生成的轮 ID（harvest_时间_s快照号）."""
    _executor_run_id: str = ''
    """调度 HarvestState.run_id，用于轮目录根."""
    _executor_target_id: str = ''
    """调度当前目标；覆盖感知 selected."""
    _executor_phase: int = 0
    """调度 HarvestState.target_phase；决定这一帧记不记视图."""
    _executor_state_seen: bool = False
    """True 后不再回退收齐窗口 cursor."""
    _scene_epoch: int = 0
    """BeginScene 次数；写入观测数组."""
    _scene_key: str = ''
    """物理场景键；与上次不同才清身份表."""
    _lifecycle_active: bool = False
    """仅 Active 为 True：才处理 RGB-D / 受理 BeginScene."""
    _ros_entities_wired: bool = False
    """on_configure 已创建订阅/发布."""
    tf_timeout: Duration
    """精确 stamp TF 查询超时."""
    tf_fallback_timeout: Duration
    """回退最新 TF 的二次查询超时（PF-2 旋钮；默认与 tf_timeout 同值）."""

    def __init__(self):
        """建节点：参数层装载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
        super().__init__('peach_scene_perception_node')
        # 纯核 stdlib 日志桥接 /rosout（P0 观测性）：BoundedWorker/推理
        # 引擎/写线程异常自此进 /rosout 与会话 bag，不再只落 stderr
        bridge_module_loggers(self, [
            'peach_vision_core.runtime',
            'peach_scene_perception.inference',
            'peach_scene_perception.identity',
            'peach_scene_perception.pose_pipelines',
        ])
        self.bridge = CvBridge()
        # W14：nav2_lm 进程死检心跳句柄（configure 建 / deactivate-cleanup 断）
        self._bond = None
        # /diagnostics 进程内只创建一次；cleanup 暂停，configure 恢复。
        self._diag = None
        # 帧流活性（_on_rgbd 刷新；诊断投影用）
        self._last_frame_mono = 0.0
        # 参数层一行接入：yaml 声明 + on-set 动态刷新（见 params.py）。
        self.params = ScenePerceptionParams.attach(self)
        self.tf_timeout = Duration(seconds=self.params.tf_timeout_sec)
        self.tf_fallback_timeout = Duration(
            seconds=self.params.tf_fallback_timeout_sec)
        self.get_logger().info(f'YOLO={self.params.yolo_model_path}')
        self.get_logger().info(f'SAM={self.params.sam_model_path}')
        # 纯核时钟适配器：命名避开 rclpy 私有 _clock 槽位（覆盖会让
        # 运行期 declare_parameter 的 parameter_event 打包取到 float 适配器
        # 而崩——2026-09-29 E2E 冒烟实锤，诊断 Updater 声明期触发）
        self._algo_clock = RclpyClockAdapter(self.get_clock())
        self.pipeline = PerceptionPipeline.from_params(
            self.params, self._algo_clock, logger=self.get_logger(),
            enable_fruit=False)
        self.plan_updater = PlanUpdater(
            self.pipeline, self.params, self._algo_clock)
        self.harvest_data = HarvestDataStore()
        self.get_logger().info(
            f'Subscribed color={self.params.color_topic} depth={self.params.depth_topic} '
            f'info={self.params.camera_info_topic} slop={self.params.sync_slop_s}s '
            f'q={self.params.sync_queue_size} max_age={self.params.max_frame_age_s}s '
            f'optical={self.params.camera_optical_frame or "(msg)"} '
            f'output={self.params.output_frame or "(camera)"} '
            f'depth_scale_unit={self.params.depth_scale_unit} '
            f'gravity_mode={self.params.gravity_mode} '
            f'calib={self.params.calibration_version}')
        if self.pipeline.target_registry is not None:
            registry = self.pipeline.target_registry
            self.get_logger().info(
                f'目标身份记忆已启用：match_radius='
                f'{registry.match_radius} m, max_targets='
                f'{registry.max_targets}, ema='
                f'{registry.alpha}')
        else:
            self.get_logger().info('目标身份记忆已禁用：target_id 为帧内序号')

    def _wire_ros(self) -> None:
        """在 on_configure 创建全部 ROS 实体（发布器/订阅/服务/TF）."""
        if self._ros_entities_wired:
            return
        # ---- 输出话题（规范组 /peach/perception/*，单套发布面）----
        # A5 起旧 ~/ 组已删除；2D 候选随 target_observations 下发。QoS 单源
        # peach_common.qos（W3）：latched()=RELIABLE+TRANSIENT_LOCAL+depth1，
        # stream()=RELIABLE+VOLATILE+KEEP_LAST(10)。业务结果保留有界队列；
        # 高频图像/点云/检测/可视化只保最新一帧，订阅端慢时不积压旧帧
        # （对齐 camera_ros/image_proc 的 depth1 发布习惯）。
        pose_qos = latched()
        default_qos = stream()
        latest_qos = stream(depth=1)
        publisher_spec = (
            ('pub_norm_pose', BagGraspCandidateArray,
             '/peach/perception/initial_pose', pose_qos),
            ('pub_norm_axis', Vector3Stamped,
             '/peach/perception/axis', default_qos),
            ('pub_norm_cloud', PointCloud2,
             '/peach/perception/single_cloud', latest_qos),
            ('pub_norm_dets', Detection2DArray,
             '/peach/perception/detections', latest_qos),
            ('pub_norm_masks', Image,
             '/peach/perception/masks', latest_qos),
            ('pub_norm_diag', BagFittingArray,
             '/peach/perception/diagnostics', default_qos),
            ('pub_norm_markers', MarkerArray,
             '/peach/perception/markers', latest_qos),
            ('pub_norm_debug', Image,
             '/peach/perception/debug_image', latest_qos),
            # RGB-D入口直出的原图预览，不等待推理或臂停稳。
            ('pub_norm_debug_raw', Image,
             '/peach/perception/debug_image_raw', latest_qos),
            ('pub_target_observations', PeachTargetObservationArray,
             '/peach/perception/target_observations', default_qos),
            ('pub_harvest_state', String,
             '/peach/perception/harvest_state', latched()),
        )
        for attr, msg_type, topic, qos in publisher_spec:
            setattr(self, attr, self.create_lifecycle_publisher(
                msg_type, topic, qos))
        self._sub_exec_state = self.create_subscription(
            HarvestState, '/peach_supervisor/state',
            self._on_executor_state, latched())
        # BeginScene 保留独立回调组；独立入口串行执行短 ROS 回调，
        # 与推理 worker 的互斥仍由 plan_lock 保证。
        self._begin_scene_group = MutuallyExclusiveCallbackGroup()
        self._svc_begin = self.create_service(
            BeginScene, '~/begin_scene', self._on_begin_scene,
            callback_group=self._begin_scene_group)

        # 本地 raw 大图优先 RELIABLE + depth=1：best-effort 在 640x480 可能
        # 丢帧（realsense-ros #3427）；depth=1 仍只保最新，队列不回到 10。
        qos = stream(depth=1)
        self._sub_rgb = message_filters.Subscriber(
            self, Image, self.params.color_topic, qos_profile=qos)
        self._sub_depth = message_filters.Subscriber(
            self, Image, self.params.depth_topic, qos_profile=qos)
        self._sub_info = message_filters.Subscriber(
            self, CameraInfo, self.params.camera_info_topic, qos_profile=qos)

        self._frame_worker = BoundedWorker(
            self._process_rgbd, capacity=1, drop_oldest=True)
        self._worker_dropped_reported = 0
        self._worker_drop_log_mono = 0.0
        self._stale_frame_dropped = 0
        self._stale_drop_log_mono = 0.0
        # 没有速度前先允许检测。臂一开始动就停送，避免把途中画面算完后才显示。
        self._arm_settled = True
        self._sub_joints = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth, self._sub_info],
            queue_size=self.params.sync_queue_size,
            slop=self.params.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)

        # 手眼：wrist3_Link→camera_link 由 extrinsics_publisher 发静态 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warned = False
        self._ros_entities_wired = True

    def _unwire_ros(self) -> None:
        """on_cleanup 释放全部 ROS 实体（与 _wire_ros 一一对应）."""
        if not self._ros_entities_wired:
            return
        # 先停 worker 再拆实体：反复 configure/cleanup 不得累积常驻线程
        # （close 丢弃未处理帧并 join；此后 _on_rgbd 的 submit 只会安全失败）。
        try:
            self._frame_worker.close(drain=False)
        except Exception:  # noqa: BLE001 已停止则忽略
            pass
        try:
            self.destroy_service(self._svc_begin)
        except Exception:  # noqa: BLE001 已释放则忽略
            pass
        try:
            for sub in (
                    self._sub_exec_state, self._sub_joints, self._sub_rgb.sub,
                    self._sub_depth.sub, self._sub_info.sub):
                self.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass
        try:
            for pub in (
                    self.pub_norm_pose, self.pub_norm_axis,
                    self.pub_norm_cloud, self.pub_norm_dets,
                    self.pub_norm_masks, self.pub_norm_diag,
                    self.pub_norm_markers, self.pub_norm_debug,
                    self.pub_norm_debug_raw,
                    self.pub_target_observations, self.pub_harvest_state):
                self.destroy_lifecycle_publisher(pub)
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
            # Load and warm both models before subscriptions/actions exist;
            # a slow first frame must not be paid inside a harvest cycle.
            self.pipeline.warmup()
            self._wire_ros()
        except Exception as exc:  # noqa: BLE001 接线失败（话题/参数非法）整包停走
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.ERROR
        # W14：nav2_lm 进程死检心跳（缺 ros-jazzy-bondpy 时守卫降级为 WARN）
        self._bond = create_bond(self, self.get_name())
        # /diagnostics 周期健康（P1：对齐 peach_arm W5 / observability W15）
        if self._diag is None:
            self._diag = diagnostic_updater.Updater(self, period=5.0)
            self._diag.setHardwareID(self.get_name())
            self._diag.add('perception_health', self._diag_health)
        else:
            self._diag.timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def _diag_health(self, stat):
        """感知健康投影：帧流/worker 丢弃/开批前事件丢弃（只读）."""
        now = time.monotonic()
        age = (now - self._last_frame_mono) if self._last_frame_mono else None
        dropped = getattr(self._frame_worker, 'dropped', 0)
        stale = getattr(self, '_stale_frame_dropped', 0)
        pre_run = getattr(self.harvest_data, 'dropped_before_run', 0)
        if self._lifecycle_active and age is None:
            stat.summary(DiagnosticStatus.WARN, 'active 但从未收到 RGB-D 帧')
        elif self._lifecycle_active and age is not None and age > 10.0:
            stat.summary(
                DiagnosticStatus.WARN, f'帧流陈旧 {age:.1f}s')
        elif dropped > 0 or stale > 0 or pre_run > 0:
            stat.summary(DiagnosticStatus.WARN, '有丢弃（见字段）')
        else:
            stat.summary(DiagnosticStatus.OK, 'perception healthy')
        stat.add('active', str(self._lifecycle_active))
        stat.add('last_frame_age_s', '-' if age is None else f'{age:.1f}')
        stat.add('worker_dropped', str(dropped))
        stat.add('stale_dropped', str(stale))
        stat.add('pre_run_dropped_events', str(pre_run))
        return stat

    def on_activate(self, state):
        result = super().on_activate(state)
        self._lifecycle_active = True
        if self._bond is None:
            self._bond = create_bond(self, self.get_name())  # deactivate 后重臂
        self.get_logger().info('perception Active：开始处理 RGB-D')
        return result

    def on_deactivate(self, state):
        with self.pipeline.plan_lock:
            self.pipeline.frame_generation += 1
            self._lifecycle_active = False
        break_bond(self._bond)
        self._bond = None
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        with self.pipeline.plan_lock:
            self.pipeline.frame_generation += 1
            self._lifecycle_active = False
        break_bond(self._bond)
        self._bond = None
        if self._diag is not None:
            self._diag.timer.cancel()
        self._unwire_ros()
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        """直接进入 Finalized 也须先停止采帧并释放配置期资源."""
        self.on_cleanup(state)
        return super().on_shutdown(state)

    def _on_executor_state(self, msg: HarvestState) -> None:
        """执行器当前目标覆盖感知 selected；作业结束写入 completed_ids."""
        new_id = str(msg.target_id or '')
        with self.pipeline.plan_lock:
            self._executor_state_seen = True
            # 单根会话目录（R7）：批次 request_id 驱动 datastore 基目录
            self._executor_run_id = str(msg.run_id or '')
            self._executor_phase = int(msg.target_phase)
            old = self._executor_target_id
            if old and old != new_id:
                self.pipeline.harvest_plan.mark_completed(old)
            self._executor_target_id = new_id

    def _effective_selected_id(self) -> str:
        """批次当前目标；已见执行器状态时不回退收齐窗口 cursor."""
        if self._executor_state_seen:
            return self._executor_target_id
        return self.pipeline.harvest_plan.selected_target_id

    def _publish_harvest_state(self) -> None:
        """发布闩锁 JSON 状态，便于运行中随时查询（快照组装在 plan_updater）."""
        with self.pipeline.plan_lock:
            payload = harvest_state_dict(
                self.pipeline, self.harvest_run_id, self._scene_epoch,
                self._effective_selected_id(), self.harvest_data.query())
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.pub_harvest_state.publish(message)

    def _on_begin_scene(self, request, response):
        """BeginScene：重启收齐窗；仅物理场景切换时清空身份表."""
        if not self._lifecycle_active:
            response.accepted = False
            response.scene_epoch = self._scene_epoch
            response.message = 'perception not Active'
            return response
        with self.pipeline.plan_lock:
            self._scene_epoch += 1
            old_run = self.harvest_run_id
            if old_run:
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'scene_begin',
                    'scene_key': request.scene_key,
                    'scene_epoch': self._scene_epoch,
                })
            previous_store = self.harvest_data
            self.harvest_data = HarvestDataStore(root=previous_store.root)
            self.harvest_run_id = ''
            scene_changed = bool(
                self._scene_key and request.scene_key != self._scene_key)
            cleared = self.pipeline.begin_scene(scene_changed)
            self._scene_key = request.scene_key
            if self._ros_entities_wired:
                self._clear_detection_visuals()
            self._publish_harvest_state()
        previous_store.close(drain=True)
        response.accepted = True
        response.scene_epoch = self._scene_epoch
        response.message = (
            f'scene_epoch={self._scene_epoch} key={request.scene_key} '
            f'identity={"cleared" if scene_changed else "preserved"} '
            f'cleared={cleared} prev_run={old_run or "none"}')
        return response

    def _start_harvest_run(self) -> None:
        """为刚锁定的全局目标集合建轮目录（领域方法在 HarvestDataStore；须持锁）."""
        self.harvest_run_id = self.harvest_data.start_harvest_run(
            self.pipeline.harvest_plan, self.params,
            executor_run_id=self._executor_run_id)

    def _publish_target_observations(
            self, header, mask_header, records, payloads,
            color_bgr=None, debug_bgr=None) -> None:
        """
        发布锁定 ID 的逐目标结果，并记录选中目标掩膜与状态事件.

        全程持 _plan_lock：plan.update 及后续逐字段读取必须与服务回调
        （reset/complete）互斥，保证 plan 单写者与快照一致性。计划推进
        在 plan_updater（W3 纯核），本方法只做 token→msg 映射 + publish
        + save_mask 回调。
        """
        with self.pipeline.plan_lock:
            stamp_ns = Time.from_msg(mask_header.stamp).nanoseconds
            outcome = self.plan_updater.update(
                records, payloads, stamp_ns,
                selected_id=self._effective_selected_id(),
                executor_id=(self._executor_target_id
                             if self._executor_state_seen else None))
            for event in outcome.dropped_events:
                self.harvest_data.append_event(event)
            if outcome.locked_just_now:
                try:
                    self._start_harvest_run()
                except OSError as exc:
                    self.get_logger().error(f'采摘运行目录创建失败: {exc}')
            if outcome.lighting_warning is not None:
                self.get_logger().warning(
                    outcome.lighting_warning, throttle_duration_sec=10.0)

            array = PeachTargetObservationArray()
            array.header = header
            array.snapshot_id = outcome.snapshot_id
            array.scene_epoch = self._scene_epoch
            array.harvest_run_id = self.harvest_run_id
            array.target_set_locked = outcome.target_set_locked
            array.target_count = outcome.target_count
            array.selected_target_id = outcome.selected_target_id
            # R-D8 发现进度摘要：锁定前 observations 恒空，下游经本两字段
            # 跟踪收齐进度；锁定后=锁定集大小/0（语义见 msg 注释）
            array.collecting_count = outcome.collecting_count
            array.pending_count = outcome.pending_count
            for spec in outcome.observations:
                item = PeachTargetObservation()
                item.header = header
                item.target_id = spec.target_id
                item.priority = spec.priority
                item.confirmed = True
                item.selected = spec.selected
                item.harvest_status = spec.harvest_status
                item.tracking_status = TRACKING_STATUS_TO_MSG[
                    spec.tracking_token]
                item.diagnostic_flags = list(spec.diagnostic_flags)
                if spec.payload is not None:
                    item.camera_distance_m = spec.camera_distance_m
                    item.confidence = spec.confidence
                    item.candidate = spec.payload['candidate']
                    item.candidate_2d = spec.payload['candidate_2d']
                    item.fitting = spec.payload['fitting']
                if spec.mask is not None:
                    item.mask = self.bridge.cv2_to_imgmsg(
                        (spec.mask > 0).astype(np.uint8) * 255,
                        encoding='mono8')
                    item.mask.header = mask_header
                if item.selected:
                    self._save_stage_view(
                        spec.target_id, stamp_ns, spec.mask,
                        color_bgr, debug_bgr)
                if spec.anchor_fields is not None:
                    self._apply_memory_anchor(item, spec)
                array.observations.append(item)
            self.pub_target_observations.publish(array)
            if outcome.frame_event is not None:
                self.harvest_data.append_event(outcome.frame_event)
            self._publish_harvest_state()

    def _save_stage_view(self, target_id, stamp_ns, mask, color, debug) -> None:
        """观察和再确认各记一帧；接近、剪切、撤离不记."""
        stage = view_stage_for_phase(
            self._executor_phase, mask is not None,
            self.harvest_data.saved_stages(target_id))
        if stage is None:
            return
        try:
            self.harvest_data.save_stage_view(
                target_id, stage, stamp_ns, mask, color=color, debug=debug)
        except OSError as exc:
            self.get_logger().error(str(exc))

    @staticmethod
    def _apply_memory_anchor(item, spec) -> None:
        """把身份记忆锚点拷入观测消息并打 anchor_from_memory（字段产自 identity）."""
        fields = spec.anchor_fields
        cand = item.candidate
        cand.target_id = spec.target_id
        for dest, src in (
                (cand.bag_bottom, fields['bottom']),
                (cand.bag_neck, fields['neck']),
                (cand.translation_direction, fields['axis']),
                (cand.entry_pose.position, fields['entry_start'])):
            dest.x, dest.y, dest.z = (float(src[0]), float(src[1]),
                                      float(src[2]))
        cand.entry_pose.orientation = quat_to_msg(fields['orientation'])
        cand.status = cand.REOBSERVE
        item.diagnostic_flags.append('anchor_from_memory')

    def _lookup_T_out_cam(self, cam_frame: str,
                          stamp) -> Tuple[Optional[np.ndarray], str]:
        """
        查 output←camera 的 4×4 齐次矩阵与查询状态.

        Args:
            cam_frame: 相机光学系 frame_id.
            stamp: 查询时刻（消息时间戳）；按时刻失败时回退最新 TF 并告警一次
                （二次查询超时独立旋钮 tf_fallback_timeout_sec，PF-2；
                默认 0.5 与拆分前行为一致）.

        Returns
        -------
            (T, status)：T 为 (4, 4) ndarray（output_frame 为空或与 cam_frame
            相同给单位阵；TF 彻底失败给 None，调用方退回相机系）；
            status ∈ {'ok', 'stale', 'unavailable'}——'stale' 表示按 stamp
            查询失败已回退最新 TF，'unavailable' 表示彻底失败，供调用方给
            本帧结果打 tf_stale / tf_unavailable 诊断标记.

        """
        if not self.params.output_frame or self.params.output_frame == cam_frame:
            return np.eye(4), 'ok'
        stamp_time = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.params.output_frame, cam_frame, stamp_time, timeout=self.tf_timeout)
            return transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.params.output_frame, cam_frame, Time(),
                    timeout=self.tf_fallback_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.params.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return transform_msg_to_matrix(tf.transform), 'stale'
            except TransformException as ex:
                self.get_logger().warning(
                    f'TF 失败，输出退回相机系 {cam_frame}: {ex}')
                return None, 'unavailable'

    def _clear_detection_visuals(self) -> None:
        """运动或换窗时覆盖检测云和marker；原图预览仍持续."""
        header = Header(stamp=self.get_clock().now().to_msg(),
                        frame_id=self.params.output_frame or self.params.camera_optical_frame)
        self.pub_norm_cloud.publish(xyzrgb_to_cloud_msg(
            header, np.zeros((0, 3)), np.zeros((0,), dtype=np.float32)))
        self.pub_norm_markers.publish(MarkerArray(markers=[
            Marker(header=header, action=Marker.DELETEALL)]))

    def _on_joint_states(self, msg: JointState) -> None:
        """停稳才检测；离开静止时使已排队和推理中的旧帧失效."""
        velocities = list(msg.velocity) if msg.velocity else None
        settled = arm_is_settled(velocities)
        with self.pipeline.plan_lock:
            if self._arm_settled and not settled:
                self.pipeline.frame_generation += 1
                self._frame_worker.discard_pending()
                self._clear_detection_visuals()
            self._arm_settled = settled

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """原图直接预览；停稳时才把带世代的最新帧交给推理worker."""
        self._last_frame_mono = time.monotonic()
        if not self._lifecycle_active:
            return
        max_age = float(self.params.max_frame_age_s)
        stamp = Time.from_msg(rgb_msg.header.stamp)
        if max_age > 0.0 and stamp.nanoseconds > 0:
            age_s = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age_s > max_age:
                self._stale_frame_dropped += 1
                now_mono = time.monotonic()
                if now_mono - self._stale_drop_log_mono >= 5.0:
                    self.get_logger().warning(
                        f'丢弃陈旧 RGB-D：age={age_s * 1e3:.0f} ms > '
                        f'{max_age * 1e3:.0f} ms，累计 '
                        f'{self._stale_frame_dropped} 帧')
                    self._stale_drop_log_mono = now_mono
                return
        if (self.params.publish_debug_image
                and self.pub_norm_debug_raw.get_subscription_count()):
            try:
                if rgb_msg.encoding == 'bgr8' and self.params.debug_downscale == 1.0:
                    # 同编码同尺寸直接转发，避免在同步器锁内往返复制整帧。
                    self.pub_norm_debug_raw.publish(rgb_msg)
                else:
                    rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                    self._publish_debug_image(
                        self.pub_norm_debug_raw, rgb, rgb_msg.header)
            except Exception as exc:  # noqa: BLE001 相机预览失败不阻断业务帧
                self.get_logger().warning(f'原图预览失败: {exc}')
        with self.pipeline.plan_lock:
            if not self._arm_settled or not self._lifecycle_active:
                return
            generation = self.pipeline.frame_generation
            if not self._frame_worker.submit((rgb_msg, depth_msg, info, generation)):
                self.get_logger().warning('感知 worker 已停止，丢弃 RGB-D 帧')

    def _decode_rgbd(self, rgb_msg, depth_msg, info) -> Optional[SyncedRgbd]:
        """解码、对齐尺寸、查 TF；失败返回 None（已打日志）."""
        depth_ns = Time.from_msg(depth_msg.header.stamp).nanoseconds
        if depth_ns == 0:
            self.get_logger().warning('深度图缺少曝光时间，丢帧', throttle_duration_sec=1.0)
            return None
        dt_ms = (Time.from_msg(rgb_msg.header.stamp).nanoseconds - depth_ns) / 1e6
        self.get_logger().debug(f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms')
        if abs(dt_ms) > self.params.sync_slop_s * 0.8 * 1000.0:
            self.get_logger().warning(
                f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms 已超同步允差 '
                f'{self.params.sync_slop_s * 1000.0:.0f} ms 的 80%，请检查相机时间戳源',
                throttle_duration_sec=1.0)
        try:
            # Hot path: bgr8/16UC1 share the message payload (no cv_bridge
            # round-trip). Other encodings still use the bridge fallback.
            try:
                rgb = bgr8_view(rgb_msg)
            except (ValueError, TypeError):
                rgb = self.bridge.imgmsg_to_cv2(
                    rgb_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            stamp = rgb_msg.header.stamp
            self.get_logger().warning(
                f'RGB 转码失败 frame={rgb_msg.header.frame_id} '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}: {exc}')
            return None
        try:
            try:
                depth_raw = depth16_view(depth_msg)
            except (ValueError, TypeError):
                depth_raw = self.bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding='passthrough')
            depth = normalize_depth_to_uint16_mm(
                depth_raw, self.params.depth_scale_unit)
        except Exception as exc:  # noqa: BLE001
            stamp = depth_msg.header.stamp
            self.get_logger().warning(
                f'深度转码失败 frame={depth_msg.header.frame_id} '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}: {exc}')
            return None
        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warning(
                f'RGB/深度分辨率不一致 {rgb.shape[:2]} vs {depth.shape[:2]} '
                f'rgb_frame={rgb_msg.header.frame_id} '
                f'depth_frame={depth_msg.header.frame_id}',
                throttle_duration_sec=1.0)
            return None
        if info.width and info.height and (
                int(info.width) != depth.shape[1]
                or int(info.height) != depth.shape[0]):
            self.get_logger().warning(
                f'CameraInfo size {info.width}x{info.height} != depth '
                f'{depth.shape[1]}x{depth.shape[0]}',
                throttle_duration_sec=1.0)
            return None
        try:
            K = camera_intrinsics(info.k, depth.shape[1], depth.shape[0])
        except ValueError as error:
            self.get_logger().warning(f'{error}，丢帧', throttle_duration_sec=1.0)
            return None
        if not getattr(self, '_logged_K', False):
            self.get_logger().info(
                f'CameraInfo K fx={K["fx"]:.3f} fy={K["fy"]:.3f} '
                f'cx={K["cx"]:.3f} cy={K["cy"]:.3f} '
                f'{K["width"]}x{K["height"]}')
            self._logged_K = True
        cam_frame = (
            self.params.camera_optical_frame
            or depth_msg.header.frame_id
            or rgb_msg.header.frame_id
            or info.header.frame_id)
        out_frame = self.params.output_frame or cam_frame
        geometry_stamp = depth_msg.header.stamp
        T_out_cam, tf_status = self._lookup_T_out_cam(
            cam_frame, geometry_stamp)
        if T_out_cam is None and self.params.output_frame:
            out_frame = cam_frame
            T_out_cam = np.eye(4)
        gravity_hint = self.params.gravity_hint
        if self.params.gravity_mode == 'tf':
            if self.params.output_frame and tf_status != 'unavailable':
                gravity_hint = gravity_camera_from_R(T_out_cam[:3, :3])
            else:
                self.get_logger().warning(
                    'gravity_mode=tf 但 TF 不可用或未设 output_frame，'
                    '本帧回退 gravity_hint_xyz',
                    throttle_duration_sec=1.0)
        header = Header(stamp=geometry_stamp, frame_id=out_frame)
        img_header = Header(
            stamp=rgb_msg.header.stamp, frame_id=rgb_msg.header.frame_id)
        return SyncedRgbd(
            rgb=rgb, depth=depth, K=K, cam_frame=cam_frame,
            out_frame=out_frame, geometry_stamp=geometry_stamp,
            img_header=img_header, header=header,
            T_out_cam=T_out_cam, tf_status=tf_status,
            gravity_hint=gravity_hint)

    def _process_rgbd(self, frame):
        """Decode RGB-D, run pipeline.process, then publish."""
        rgb_msg, depth_msg, info, generation = frame
        with self.pipeline.plan_lock:
            if (not self._lifecycle_active or not self._arm_settled
                    or generation != self.pipeline.frame_generation):
                return
        t_total_start = self._algo_clock.now()
        self._report_worker_drops()
        self.get_logger().debug(
            f'RGB-D sync frame {rgb_msg.width}x{rgb_msg.height}')
        t_decode = self._algo_clock.now()
        synced = self._decode_rgbd(rgb_msg, depth_msg, info)
        if synced is None:
            return
        self.pipeline.timing.record(
            'decode_ms', (self._algo_clock.now() - t_decode) * 1e3)
        out = self.pipeline.process(
            synced, executor_target_id=self._executor_target_id,
            generation=generation)
        if out is None:
            return
        t_publish = self._algo_clock.now()
        with self.pipeline.plan_lock:
            if (not self._lifecycle_active or not self._arm_settled
                    or generation != self.pipeline.frame_generation):
                return
            self._publish_frame(synced, out)
        self.pipeline.timing.record(
            'publish_ms', (self._algo_clock.now() - t_publish) * 1e3)
        self.pipeline.timing.record(
            'total_ms', (self._algo_clock.now() - t_total_start) * 1e3)

    def _report_worker_drops(self) -> None:
        # drop_oldest 下 submit 恒真，丢帧只有这里能看见。计数每帧都涨，
        # 按增长逐条 WARN 会灌满 /rosout 和 8090；累计数每 5 s 报一次。
        dropped = self._frame_worker.dropped
        now = time.monotonic()
        if (dropped != self._worker_dropped_reported
                and now - self._worker_drop_log_mono >= 5.0):
            self.get_logger().warning(
                f'感知 worker 满队丢帧累计 {dropped}（处理跟不上到达率，'
                '排查热路径耗时或降低到达率）')
            self._worker_dropped_reported = dropped
            self._worker_drop_log_mono = now

    def _publish_debug_image(self, pub, image, header) -> None:
        """有订阅者才缩放和编码 debug 图；不影响推理与阶段视图落盘."""
        if not pub.get_subscription_count():
            return
        scale = float(self.params.debug_downscale)
        if scale < 1.0:
            image = cv2.resize(
                image,
                (max(1, int(image.shape[1] * scale)),
                 max(1, int(image.shape[0] * scale))),
                interpolation=cv2.INTER_AREA)
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = header
        pub.publish(msg)

    def _publish_frame(self, synced, out):
        """Publish one PerceptionResult (ROS I/O only)."""
        det_msg = Detection2DArray()
        det_msg.header = out.img_header
        for det in out.kept:
            det_msg.detections.append(to_detection2d(det, out.img_header))
        self.pub_norm_dets.publish(det_msg)
        self._publish_target_observations(
            out.header, out.mask_header, out.harvest_records,
            out.harvest_payloads, synced.rgb, out.debug)
        self.pub_norm_pose.publish(out.candidates)
        self.pub_norm_diag.publish(out.fittings)
        self.pub_norm_markers.publish(out.markers)
        best_dir = best_axis_direction(out.frame_axes)
        if best_dir is not None:
            axis_msg = Vector3Stamped()
            axis_msg.header = out.header
            axis_msg.vector = Vector3(
                x=float(best_dir[0]), y=float(best_dir[1]),
                z=float(best_dir[2]))
            self.pub_norm_axis.publish(axis_msg)
        self.get_logger().debug(
            f'Published {len(out.candidates.candidates)} candidates '
            f'(detections={len(out.kept)})')
        if self.pipeline.target_registry is not None:
            st = self.pipeline.target_registry.stats()
            self.get_logger().info(
                f'目标注册表在册 {st["n_targets"]} 个目标'
                f'（累计注册 {st["n_registered"]}、累计命中 {st["n_matched"]}）',
                throttle_duration_sec=10.0)
        # 可视化大消息只在确有订阅者时序列化；订阅计数为 0 时仍保留业务
        # 话题（观测/候选/marker/诊断），不构造无人消费的掩膜和点云。
        if (self.params.publish_masks
                and self.pub_norm_masks.get_subscription_count()):
            mask_msg = self.bridge.cv2_to_imgmsg(
                out.mask_canvas, encoding='mono16')
            mask_msg.header = out.img_header
            self.pub_norm_masks.publish(mask_msg)
        if (self.params.publish_detection_cloud
                and self.pub_norm_cloud.get_subscription_count()):
            xyz_cam, rgb_f = np.zeros((0, 3)), np.zeros((0,), dtype=np.float32)
            if out.confirmed_bboxes:
                xyz_cam, rgb_f = bbox_cloud_xyzrgb(
                    synced.rgb, synced.depth, synced.K, out.confirmed_bboxes,
                    stride=self.params.detection_cloud_stride)
            if xyz_cam.shape[0] and synced.T_out_cam is not None:
                rot, trans = synced.T_out_cam[:3, :3], synced.T_out_cam[:3, 3]
                xyz_out = (rot @ xyz_cam.T).T + trans
            else:
                xyz_out = xyz_cam
            self.pub_norm_cloud.publish(
                xyzrgb_to_cloud_msg(out.header, xyz_out, rgb_f))
        if out.debug is not None:
            self._publish_debug_image(
                self.pub_norm_debug, out.debug, out.img_header)

    def request_shutdown(self):
        """使在途推理失效；销毁句柄前停止接收新帧."""
        with self.pipeline.plan_lock:
            if self._lifecycle_active:
                self._lifecycle_active = False
                self.pipeline.frame_generation += 1

    def destroy_node(self):
        """停止推理 worker 后销毁 ROS 节点（未 configure 过则无 worker）."""
        self.request_shutdown()
        worker = getattr(self, '_frame_worker', None)
        if worker is not None:
            worker.close(drain=False)
        harvest = getattr(self, 'harvest_data', None)
        if harvest is not None:
            harvest.close(drain=True)
        return super().destroy_node()


def main(args=None):
    """
    节点入口：rclpy 初始化 → 单线程 executor spin → 干净收尾.

    图像回调只提交有界 worker；单线程执行 ROS 回调，避免多线程
    executor 与推理 worker 争用 GIL。BeginScene 与 worker 仍由
    plan_lock 保证互斥。

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = ScenePerceptionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A SIGINT during teardown can invalidate the rcl context while the
        # executor still holds a take_message handle. Only suppress it when
        # the context is already shutting down; real runtime faults propagate.
        if rclpy.ok():
            raise
    finally:
        node.request_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
