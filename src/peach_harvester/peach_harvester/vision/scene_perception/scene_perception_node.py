"""
场景感知 ROS 2 节点（图名 `peach_scene_perception_node`）.

Lifecycle + 接线。热路径：decode_rgbd → pipeline.process → publish。
W3 起计划推进/观测组装材料在 plan_updater（纯核），消息组装在
msg_builders，像素绘制在 debug_draw；节点只持锁、发消息、落盘。
"""
from __future__ import annotations

import json
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Vector3Stamped
import message_filters
import numpy as np
from peach_common.qos import latched, stream
from peach_harvester.vision.common.geometry import (
    gravity_camera_from_R,
    normalize_depth_to_uint16_mm,
    transform_msg_to_matrix,
)
from peach_harvester.vision.common.ros.clock_adapter import RclpyClockAdapter
from peach_harvester.vision.common.runtime import BoundedWorker, HarvestDataStore
from peach_harvester.vision.scene_perception.msg_builders import (
    bbox_cloud_xyzrgb,
    best_axis_direction,
    quat_to_msg,
    to_detection2d,
    TRACKING_STATUS_TO_MSG,
    xyzrgb_to_cloud_msg,
)
from peach_harvester.vision.scene_perception.params import ScenePerceptionParams
from peach_harvester.vision.scene_perception.pipeline import (
    PerceptionPipeline,
    SyncedRgbd,
)
from peach_harvester.vision.scene_perception.plan_updater import (
    harvest_state_dict,
    PlanUpdater,
)
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    HarvestState,
    PeachTargetObservation,
    PeachTargetObservationArray,
)
from peach_interfaces.srv import BeginScene
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
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
        self.bridge = CvBridge()
        # 参数层一行接入：yaml 声明 + on-set 动态刷新（见 params.py）。
        self.params = ScenePerceptionParams.attach(self)
        self.tf_timeout = Duration(seconds=self.params.tf_timeout_sec)
        self.tf_fallback_timeout = Duration(
            seconds=self.params.tf_fallback_timeout_sec)
        self.get_logger().info(f'YOLO={self.params.yolo_model_path}')
        self.get_logger().info(f'SAM={self.params.sam_model_path}')
        self._clock = RclpyClockAdapter(self.get_clock())
        self.pipeline = PerceptionPipeline.from_params(
            self.params, self._clock, logger=self.get_logger(),
            enable_fruit=False)
        self.plan_updater = PlanUpdater(
            self.pipeline, self.params, self._clock)
        self.harvest_data = HarvestDataStore()
        self.get_logger().info(
            f'Subscribed color={self.params.color_topic} depth={self.params.depth_topic} '
            f'info={self.params.camera_info_topic} slop={self.params.sync_slop_s}s '
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
        # stream()=RELIABLE+VOLATILE+KEEP_LAST(10)——与 manifest/原内联
        # profile 逐字段等值。表驱动建 publisher（话题面单表可查，对齐 io.md）。
        pose_qos = latched()
        default_qos = stream()
        publisher_spec = (
            ('pub_norm_pose', BagGraspCandidateArray,
             '/peach/perception/initial_pose', pose_qos),
            ('pub_norm_axis', Vector3Stamped,
             '/peach/perception/axis', default_qos),
            ('pub_norm_cloud', PointCloud2,
             '/peach/perception/single_cloud', default_qos),
            ('pub_norm_dets', Detection2DArray,
             '/peach/perception/detections', default_qos),
            ('pub_norm_masks', Image,
             '/peach/perception/masks', default_qos),
            ('pub_norm_diag', BagFittingArray,
             '/peach/perception/diagnostics', default_qos),
            ('pub_norm_markers', MarkerArray,
             '/peach/perception/markers', default_qos),
            ('pub_norm_debug', Image,
             '/peach/perception/debug_image', default_qos),
            # 真相流画布（全量检测叠加，含未确认灰框）：只进记录层对比，
            # 不进 RViz（RViz 只订稳定流 debug_image）
            ('pub_norm_debug_raw', Image,
             '/peach/perception/debug_image_raw', default_qos),
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
        # 服务独立 MutuallyExclusive 组 + 双线程 executor（W3，对齐 ROS 2
        # executor/callback group 官方 How-To）：图像回调仍在默认组串行；
        # BeginScene 与 worker 互斥语义由 plan_lock 保证（锁不变，仅不再
        # 让服务回调在 executor 层面饿死帧回调）。
        self._begin_scene_group = MutuallyExclusiveCallbackGroup()
        self._svc_begin = self.create_service(
            BeginScene, '~/begin_scene', self._on_begin_scene,
            callback_group=self._begin_scene_group)

        # 与数据集回放 / 相机驱动对齐：RELIABLE，避免 Best Effort 对不上
        qos = stream()
        self._sub_rgb = message_filters.Subscriber(
            self, Image, self.params.color_topic, qos_profile=qos)
        self._sub_depth = message_filters.Subscriber(
            self, Image, self.params.depth_topic, qos_profile=qos)
        self._sub_info = message_filters.Subscriber(
            self, CameraInfo, self.params.camera_info_topic, qos_profile=qos)

        self._frame_worker = BoundedWorker(
            self._process_rgbd, capacity=1, drop_oldest=True)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth, self._sub_info],
            queue_size=10, slop=self.params.sync_slop_s)
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
                    self._sub_exec_state, self._sub_rgb.sub,
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
            self._wire_ros()
        except Exception as exc:  # noqa: BLE001 接线失败（话题/参数非法）整包停走
            self.get_logger().error(f'configure 失败: {exc}')
            return TransitionCallbackReturn.ERROR
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        result = super().on_activate(state)
        self._lifecycle_active = True
        self.get_logger().info('perception Active：开始处理 RGB-D')
        return result

    def on_deactivate(self, state):
        self._lifecycle_active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self._lifecycle_active = False
        self._unwire_ros()
        return super().on_cleanup(state)

    def _on_executor_state(self, msg: HarvestState) -> None:
        """执行器当前目标覆盖感知 selected；作业结束写入 completed_ids."""
        new_id = str(msg.target_id or '')
        with self.pipeline.plan_lock:
            self._executor_state_seen = True
            # 单根会话目录（R7）：批次 request_id 驱动 datastore 基目录
            self._executor_run_id = str(msg.run_id or '')
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
            self.harvest_data = HarvestDataStore(root=self.harvest_data.root)
            self.harvest_run_id = ''
            scene_changed = bool(
                self._scene_key and request.scene_key != self._scene_key)
            cleared = self.pipeline.begin_scene(scene_changed)
            self._scene_key = request.scene_key
            self._publish_harvest_state()
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
            self, header, mask_header, records, payloads) -> None:
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
                        try:
                            self.harvest_data.save_mask(
                                spec.target_id, stamp_ns, spec.mask)
                        except OSError as exc:
                            self.get_logger().error(str(exc))
                if spec.anchor_fields is not None:
                    self._apply_memory_anchor(item, spec)
                array.observations.append(item)
            self.pub_target_observations.publish(array)
            if outcome.frame_event is not None:
                self.harvest_data.append_event(outcome.frame_event)
            self._publish_harvest_state()

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

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """将最新同步帧交给容量一推理 worker."""
        if not self._lifecycle_active:
            return
        if not self._frame_worker.submit((rgb_msg, depth_msg, info)):
            self.get_logger().warning('感知 worker 已停止，丢弃 RGB-D 帧')

    def _decode_rgbd(self, rgb_msg, depth_msg, info) -> Optional[SyncedRgbd]:
        """解码、对齐尺寸、查 TF；失败返回 None（已打日志）."""
        dt_ms = (Time.from_msg(rgb_msg.header.stamp).nanoseconds
                 - Time.from_msg(depth_msg.header.stamp).nanoseconds) / 1e6
        self.get_logger().debug(f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms')
        if abs(dt_ms) > self.params.sync_slop_s * 0.8 * 1000.0:
            self.get_logger().warning(
                f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms 已超同步允差 '
                f'{self.params.sync_slop_s * 1000.0:.0f} ms 的 80%，请检查相机时间戳源',
                throttle_duration_sec=1.0)
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            stamp = rgb_msg.header.stamp
            self.get_logger().warning(
                f'RGB 转码失败 frame={rgb_msg.header.frame_id} '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}: {exc}')
            return None
        try:
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
            # rclpy 的 throttle_duration_sec 实为毫秒：1000.0 = 1 s
            self.get_logger().warning(
                f'RGB/深度分辨率不一致 {rgb.shape[:2]} vs {depth.shape[:2]} '
                f'rgb_frame={rgb_msg.header.frame_id} '
                f'depth_frame={depth_msg.header.frame_id}',
                throttle_duration_sec=1000.0)
            return None
        if info.width and info.height and (
                int(info.width) != depth.shape[1]
                or int(info.height) != depth.shape[0]):
            self.get_logger().warning(
                f'CameraInfo size {info.width}x{info.height} != depth '
                f'{depth.shape[1]}x{depth.shape[0]}',
                throttle_duration_sec=1000.0)
            return None
        K = {
            'fx': float(info.k[0]), 'fy': float(info.k[4]),
            'cx': float(info.k[2]), 'cy': float(info.k[5]),
            'width': int(depth.shape[1]), 'height': int(depth.shape[0]),
        }
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
        rgb_msg, depth_msg, info = frame
        t_total_start = self._clock.now()
        self.get_logger().debug(
            f'RGB-D sync frame {rgb_msg.width}x{rgb_msg.height}')
        synced = self._decode_rgbd(rgb_msg, depth_msg, info)
        if synced is None:
            return
        out = self.pipeline.process(
            synced, executor_target_id=self._executor_target_id)
        if out is None:
            return
        self._publish_frame(synced, out)
        self.pipeline.timing.record(
            'total_ms', (self._clock.now() - t_total_start) * 1e3)

    def _publish_debug_image(self, pub, image, header) -> None:
        """发布 debug 图（PF-3：按 debug_downscale 缩放，1.0 直通零开销）."""
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
            out.harvest_payloads)
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
        if self.params.publish_masks:
            mask_msg = self.bridge.cv2_to_imgmsg(
                out.mask_canvas, encoding='mono16')
            mask_msg.header = out.img_header
            self.pub_norm_masks.publish(mask_msg)
        if self.params.publish_detection_cloud and out.confirmed_bboxes:
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
        if out.debug_raw is not None:
            self._publish_debug_image(
                self.pub_norm_debug_raw, out.debug_raw, out.img_header)

    def destroy_node(self):
        """停止推理 worker 后销毁 ROS 节点."""
        self._frame_worker.close(drain=False)
        return super().destroy_node()


def main(args=None):
    """
    节点入口：rclpy 初始化 → 双线程 executor spin → KeyboardInterrupt 收尾.

    MultiThreadedExecutor(num_threads=2)（W3）：图像回调默认组不变，
    BeginScene 服务独立组得以并行受理；与 worker 的互斥仍由 plan_lock
    保证（锁协议零变化）。

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = ScenePerceptionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def _spin():
        executor.spin()

    try:
        _spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        # 生命周期重复转换（如已 active 再收 activate）会从 rcl_lifecycle 抛出冲垮
        # spin；状态机已处于目标态，记录后继续 spin（09-17 A/B 台架实测崩溃场景）.
        node.get_logger().warn(f'忽略生命周期重复转换请求: {exc}')
        try:
            _spin()
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        executor.remove_node(node)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
