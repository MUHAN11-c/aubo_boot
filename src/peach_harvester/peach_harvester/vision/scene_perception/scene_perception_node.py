"""
场景感知 ROS 2 节点（图名 `peach_scene_perception_node`）.

Lifecycle + 接线。热路径：decode_rgbd → pipeline.process → publish。
"""
from __future__ import annotations

from datetime import datetime
import json
from typing import Optional, Tuple

from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Vector3Stamped
import message_filters
import numpy as np
from peach_common.paths import safe_component
from peach_harvester.vision.common.geometry import (
    gravity_camera_from_R,
    normalize_depth_to_uint16_mm,
    transform_msg_to_matrix,
)
from peach_harvester.vision.common.ros.clock_adapter import RclpyClockAdapter
from peach_harvester.vision.common.runtime import (
    BoundedWorker,
    default_runs_root,
    HarvestDataStore,
)
from peach_harvester.vision.scene_perception.identity import (
    classify_tracking_status,
    memory_grasp,
    STATUS_DEPTH_VOID,
    STATUS_OUT_OF_VIEW,
)
from peach_harvester.vision.scene_perception.params import ScenePerceptionParams
from peach_harvester.vision.scene_perception.pipeline import (
    PerceptionPipeline,
    SyncedRgbd,
)
from peach_harvester.vision.scene_perception.pose_pipelines import _rotation_to_quat
from peach_harvester.vision.scene_perception.visualization import (
    _bbox_cloud_xyzrgb,
    _to_detection2d,
    _xyzrgb_to_cloud_msg,
    best_axis_direction,
    TRACKING_STATUS_TO_MSG,
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
from rclpy.duration import Duration
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

    def __init__(self):
        """建节点：参数层装载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
        super().__init__('peach_scene_perception_node')
        self.bridge = CvBridge()
        # 参数层一行接入：yaml 声明 + on-set 动态刷新（见 params.py）。
        self.params = ScenePerceptionParams.attach(self)
        self.tf_timeout = Duration(seconds=self.params.tf_timeout_sec)
        self.get_logger().info(f'YOLO={self.params.yolo_model_path}')
        self.get_logger().info(f'SAM={self.params.sam_model_path}')
        self._clock = RclpyClockAdapter(self.get_clock())
        self.pipeline = PerceptionPipeline.from_params(
            self.params, self._clock, logger=self.get_logger(),
            enable_fruit=False)
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
        # A5 起旧 ~/ 组（grasp_candidates/fitting/markers 等）已删除，下游一律
        # 订阅本组固定命名；2D 候选不再单独成话题（随 target_observations 的
        # candidate_2d 字段下发）
        pose_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        # 输出话题默认 QoS = depth 10 / RELIABLE / volatile（等价旧裸 10）
        default_qos = rclpy.qos.QoSProfile(depth=10)
        self.pub_norm_pose = self.create_lifecycle_publisher(
            BagGraspCandidateArray, '/peach/perception/initial_pose', pose_qos)
        self.pub_norm_axis = self.create_lifecycle_publisher(
            Vector3Stamped, '/peach/perception/axis', default_qos)
        self.pub_norm_cloud = self.create_lifecycle_publisher(
            PointCloud2, '/peach/perception/single_cloud', default_qos)
        self.pub_norm_dets = self.create_lifecycle_publisher(
            Detection2DArray, '/peach/perception/detections', default_qos)
        self.pub_norm_masks = self.create_lifecycle_publisher(
            Image, '/peach/perception/masks', default_qos)
        self.pub_norm_diag = self.create_lifecycle_publisher(
            BagFittingArray, '/peach/perception/diagnostics', default_qos)
        self.pub_norm_markers = self.create_lifecycle_publisher(
            MarkerArray, '/peach/perception/markers', default_qos)
        self.pub_norm_debug = self.create_lifecycle_publisher(
            Image, '/peach/perception/debug_image', default_qos)
        # 真相流画布（全量检测叠加，含未确认灰框）：只进记录层对比，
        # 不进 RViz（RViz 只订稳定流 debug_image）
        self.pub_norm_debug_raw = self.create_lifecycle_publisher(
            Image, '/peach/perception/debug_image_raw', default_qos)
        self.pub_target_observations = self.create_lifecycle_publisher(
            PeachTargetObservationArray,
            '/peach/perception/target_observations', default_qos)
        state_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_harvest_state = self.create_lifecycle_publisher(
            String, '/peach/perception/harvest_state', state_qos)
        self._sub_exec_state = self.create_subscription(
            HarvestState, '/peach_supervisor/state',
            self._on_executor_state, state_qos)
        self._svc_begin = self.create_service(
            BeginScene, '~/begin_scene', self._on_begin_scene)

        # 与数据集回放 / 相机驱动对齐：RELIABLE，避免 Best Effort 对不上
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
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

    def _discovery_counts(self) -> Tuple[int, int]:
        """
        发现进度摘要 (collecting_count, pending_count)（缺陷 R-D8；须持锁调用）.

        collecting_count：锁定前=收齐窗口累积的已确认目标数（plan 透传策略
        累积集大小），锁定后=锁定集大小（与 target_count 一致）；
        pending_count：锁定前=注册表确认中（未转正）记录数，锁定后恒 0；
        身份记忆禁用时恒 0（每帧记录即确认，无攒帧过程）。
        """
        collecting = self.pipeline.harvest_plan.collecting_count
        if self.pipeline.harvest_plan.locked or self.pipeline.target_registry is None:
            return collecting, 0
        return collecting, self.pipeline.target_registry.pending_count

    def _harvest_state_dict(self) -> dict:
        """返回可序列化的全局采摘计划与数据路径（持锁读取一致快照）."""
        with self.pipeline.plan_lock:
            collecting_count, pending_count = self._discovery_counts()
            return {
                'harvest_run_id': self.harvest_run_id,
                'snapshot_id': self.pipeline.harvest_plan.snapshot_id,
                'target_set_locked': self.pipeline.harvest_plan.locked,
                'target_count': self.pipeline.harvest_plan.target_count,
                # R-D8 发现进度摘要：与 target_observations 同名字段对齐
                'collecting_count': collecting_count,
                'pending_count': pending_count,
                'target_ids': list(self.pipeline.harvest_plan.locked_ids),
                'completed_target_ids': sorted(self.pipeline.harvest_plan.completed_ids),
                'priorities': dict(self.pipeline.harvest_plan.priorities),
                'selected_target_id': self._effective_selected_id(),
                # 阶段 D1（协议 2.4）：锚点陈旧/出视野/已移除目标集（均为
                # 纯增量键，下游只读消费）与光照质量观测指标
                'anchor_stale_target_ids': sorted(
                    self.pipeline.harvest_plan.anchor_stale_ids),
                'out_of_view_target_ids': sorted(
                    self.pipeline.harvest_plan.out_of_view_ids),
                'dropped_target_ids': sorted(self.pipeline.harvest_plan.dropped_ids),
                'lighting': self.pipeline.lighting.snapshot(),
                'low_light_quality': self.pipeline.lighting.low_quality,
                'scene_epoch': self._scene_epoch,
                # 推理耗时分项 EMA（毫秒）+ 实测 fps；详见 TimingMetrics
                'timing': self.pipeline.timing.snapshot(fps=self.pipeline.frame_rate.rate_hz),
                'data': self.harvest_data.query(),
            }

    def _publish_harvest_state(self) -> None:
        """发布闩锁 JSON 状态，便于运行中随时查询."""
        message = String()
        message.data = json.dumps(
            self._harvest_state_dict(), ensure_ascii=False)
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
        """为刚锁定的全局目标集合创建不可变 manifest（须持 _plan_lock 调用）."""
        with self.pipeline.plan_lock:
            # 批次在跑：轮目录落 runs/<request_id>/perception_data/<轮ID>；
            # 无批次回退旧布局（root/<轮ID>）。request_id 为消息来源，入路径
            # 前经 safe_component 净化（W1 路径穿越修复）。
            if self._executor_run_id:
                self.harvest_data.base_dir = (
                    default_runs_root() /
                    safe_component(self._executor_run_id, 'harvest')
                    / 'perception_data')
            else:
                self.harvest_data.base_dir = None
            now = datetime.now()
            self.harvest_run_id = (
                f'harvest_{now.strftime("%Y%m%dT%H%M%S_%f")}_'
                f's{self.pipeline.harvest_plan.snapshot_id}')
            targets = [
                {'target_id': target_id,
                 'priority': self.pipeline.harvest_plan.priority(target_id)}
                for target_id in self.pipeline.harvest_plan.locked_ids
            ]
            self.harvest_data.start(self.harvest_run_id, {
                'snapshot_id': self.pipeline.harvest_plan.snapshot_id,
                'target_count': self.pipeline.harvest_plan.target_count,
                'selected_target_id': self.pipeline.harvest_plan.selected_target_id,
                'targets': targets,
                'model_version': self.params.model_version,
                'calibration_version': self.params.calibration_version,
                'output_frame': self.params.output_frame,
            })
            self.harvest_data.append_event({
                'source': 'perception', 'event': 'global_targets_locked',
                'target_count': self.pipeline.harvest_plan.target_count,
                'selected_target_id': self.pipeline.harvest_plan.selected_target_id,
            })

    def _publish_target_observations(
            self, header, mask_header, records, payloads) -> None:
        """
        发布锁定 ID 的逐目标结果，并记录选中目标掩膜与状态事件.

        全程持 _plan_lock：plan.update 及后续逐字段读取必须与服务回调
        （reset/complete）互斥，保证 plan 单写者与快照一致性。
        """
        with self.pipeline.plan_lock:
            was_locked = self.pipeline.harvest_plan.locked
            # 帧率自适应收齐兜底：按实测帧间隔 EMA 伸缩 max_collect_s（帧率
            # 以运行状态为准）——低帧率放大防误锁空集，高帧率收紧提速；
            # 配置值 ×0.4 作下限。异常间隔（暂停后首帧）不进 EMA。
            # 协议 I3：now 取节点时钟（同一时钟源同时驱动窗口超时判定）
            now_s = self._clock.now()
            self.pipeline.frame_rate.update(now_s)
            frame_interval = self.pipeline.frame_rate.interval
            if frame_interval is not None:
                self.pipeline.harvest_plan.max_collect_s = (
                    self.pipeline.collect_window_timeout.value(frame_interval))
                # 锚点新鲜度两档时限（阶段 D1，协议 2.4/I4）：秒级上限 ÷
                # 实测帧间隔 EMA = 帧数阈值，逐帧改写；帧率跌落时帧数变少，
                # 墙钟上限保持不变
                self.pipeline.harvest_plan.anchor_max_age_frames = max(
                    1, round(self.params.target_memory.anchor_max_age_s
                             / frame_interval))
                self.pipeline.harvest_plan.anchor_drop_frames = max(
                    1, round(self.params.target_memory.anchor_drop_s
                             / frame_interval))
            # OUT_OF_VIEW 预分类（须在 plan.update 前完成：计划按本集合做
            # 不可选/去选判定）：锁定目标本帧无观测且消失前最后检测框触
            # 图像边缘 → 走出视野；单位姿模型下复扫无益，视为不可选（2.4）
            out_of_view_ids = {
                target_id for target_id in self.pipeline.harvest_plan.locked_ids
                if target_id not in payloads
                and self.pipeline.bbox_at_edge.get(target_id, False)
            }
            current = self.pipeline.harvest_plan.update(
                records, now=now_s, out_of_view_ids=out_of_view_ids)
            # 锚点超龄移除（LOST 超 anchor_drop）：记账 target_dropped 事件，
            # 编排侧据此按 SKIPPED_UNREACHABLE「目标丢失超时」入账（协议 2.4）
            for dropped_id in self.pipeline.harvest_plan.pop_dropped():
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'target_dropped',
                    'target_id': dropped_id,
                    'reason': 'anchor_drop_timeout（目标丢失超时）',
                })
            try:
                if self.pipeline.harvest_plan.locked and not was_locked:
                    self._start_harvest_run()
            except OSError as exc:
                self.get_logger().error(f'采摘运行目录创建失败: {exc}')

            # 光照质量统计（阶段 D1；观测指标，不打阻断旗标）：锁定集中
            # 本帧带掩膜观测的目标，逐帧注入掩膜内有效深度占比与置信度
            if self.pipeline.harvest_plan.locked:
                depth_ratios = []
                confidences = []
                for target_id in self.pipeline.harvest_plan.locked_ids:
                    payload = payloads.get(target_id)
                    if payload is None or payload.get('mask') is None:
                        continue
                    depth_ratios.append(payload.get('mask_depth_ratio', 0.0))
                    confidences.append(float(
                        current.get(target_id, {}).get('confidence', 0.0)))
                self.pipeline.lighting.update(depth_ratios, confidences)
            if self.pipeline.lighting.low_quality:
                self.get_logger().warning(
                    f'光照质量持续偏低（{self.params.lighting.bad_frames} 帧连击：'
                    f'掩膜内有效深度占比 EMA='
                    f'{self.pipeline.lighting.snapshot()["depth_ratio"]} < '
                    f'{self.params.lighting.min_depth_ratio} 或置信度 EMA < '
                    f'{self.params.lighting.min_conf_mean}），建议现场补光/调曝光',
                    throttle_duration_sec=10.0)

            array = PeachTargetObservationArray()
            array.header = header
            array.snapshot_id = self.pipeline.harvest_plan.snapshot_id
            array.scene_epoch = self._scene_epoch
            array.harvest_run_id = self.harvest_run_id
            array.target_set_locked = self.pipeline.harvest_plan.locked
            array.target_count = self.pipeline.harvest_plan.target_count
            array.selected_target_id = self._effective_selected_id()
            # R-D8 发现进度摘要：锁定前 observations 恒空，下游经本两字段
            # 跟踪收齐进度；锁定后=锁定集大小/0（语义见 msg 注释）
            array.collecting_count, array.pending_count = (
                self._discovery_counts())
            observed_ids = []
            stamp_ns = Time.from_msg(mask_header.stamp).nanoseconds
            for target_id in self.pipeline.harvest_plan.locked_ids:
                item = PeachTargetObservation()
                item.header = header
                item.target_id = target_id
                item.priority = self.pipeline.harvest_plan.priority(target_id)
                item.confirmed = True
                item.selected = target_id == array.selected_target_id
                item.harvest_status = self.pipeline.harvest_plan.harvest_status(
                    target_id,
                    executor_id=(
                        self._executor_target_id if self._executor_state_seen
                        else None))
                payload = payloads.get(target_id)
                record = current.get(target_id, {})
                # 跟踪状态四分类（阶段 D1，协议 2.4 第 4 条）：分类纯函数
                # 在 assignment.classify_tracking_status，本处只做 token → msg 映射
                token = classify_tracking_status(
                    has_observation=payload is not None,
                    has_mask=(payload is not None
                              and payload.get('mask') is not None),
                    mask_depth_ratio=(
                        None if payload is None
                        else payload.get('mask_depth_ratio')),
                    min_depth_ratio=self.params.lighting.min_depth_ratio,
                    last_bbox_touched_edge=self.pipeline.bbox_at_edge.get(
                        target_id, False))
                item.tracking_status = TRACKING_STATUS_TO_MSG[token]
                if payload is None:
                    if token == STATUS_OUT_OF_VIEW:
                        item.diagnostic_flags = ['target_out_of_view']
                    else:
                        item.diagnostic_flags = ['target_temporarily_lost']
                else:
                    item.camera_distance_m = float(
                        record.get('camera_distance_m', 0.0))
                    item.confidence = float(record.get('confidence', 0.0))
                    item.candidate = payload['candidate']
                    item.candidate_2d = payload['candidate_2d']
                    item.fitting = payload['fitting']
                    item.diagnostic_flags = list(
                        record.get('diagnostic_flags', ()))
                    mask = payload.get('mask')
                    if mask is None:
                        item.diagnostic_flags.append('mask_unavailable')
                    else:
                        if token == STATUS_DEPTH_VOID:
                            item.diagnostic_flags.append('depth_void')
                        item.mask = self.bridge.cv2_to_imgmsg(
                            (mask > 0).astype(np.uint8) * 255,
                            encoding='mono8')
                        item.mask.header = mask_header
                        observed_ids.append(target_id)
                        if item.selected:
                            try:
                                self.harvest_data.save_mask(
                                    target_id, stamp_ns, mask)
                            except OSError as exc:
                                self.get_logger().error(str(exc))
                # 锚点陈旧旗标（阶段 D1，协议 2.4）：LOST 超 anchor_max_age
                # 的锁定目标已被计划排除出可选集，此处把旗标同步进该目标的
                # diagnostic_flags 供下游/Web 展示
                if (target_id in self.pipeline.harvest_plan.anchor_stale_ids
                        and 'anchor_stale' not in item.diagnostic_flags):
                    item.diagnostic_flags.append('anchor_stale')
                # 几何退化且本帧无活体观测：用身份表记忆锚点回填。
                # 已有检测/掩膜时不要回填，否则能力端把 OBSERVED 当成非新鲜
                # （anchor_from_memory 不刷新 received_s），FULL 再确认空等。
                if payload is None and self._degenerate_candidate(item.candidate):
                    self._fill_memory_anchor(item, target_id)
                array.observations.append(item)
            self.pub_target_observations.publish(array)
            if self.pipeline.harvest_plan.locked:
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'frame_observations',
                    'stamp_ns': stamp_ns, 'observed_target_ids': observed_ids,
                    'selected_target_id': self._effective_selected_id(),
                })
            self._publish_harvest_state()

    @staticmethod
    def _degenerate_candidate(candidate) -> bool:
        """袋底或袋颈近原点，视为本帧几何失败."""
        bottom, neck = candidate.bag_bottom, candidate.bag_neck
        origin = (abs(bottom.x) < 1e-6 and abs(bottom.y) < 1e-6
                  and abs(bottom.z) < 1e-6)
        neck_origin = (abs(neck.x) < 1e-6 and abs(neck.y) < 1e-6
                       and abs(neck.z) < 1e-6)
        return origin or neck_origin

    def _fill_memory_anchor(self, item, target_id: str) -> None:
        """用身份表记忆回填 candidate，并打 anchor_from_memory."""
        entry = (None if self.pipeline.target_registry is None
                 else self.pipeline.target_registry.get(target_id))
        standoff = self.params.tool.entry_standoff
        grasp = memory_grasp(entry, standoff)
        if grasp is None:
            return
        cand = item.candidate
        cand.target_id = target_id
        for dest, src in (
                (cand.bag_bottom, grasp.bottom),
                (cand.bag_neck, grasp.neck),
                (cand.translation_direction, grasp.axis),
                (cand.entry_pose.position, grasp.entry_start)):
            dest.x, dest.y, dest.z = (float(src[0]), float(src[1]),
                                      float(src[2]))
        cand.entry_pose.orientation = _rotation_to_quat(grasp.rotation)
        cand.status = cand.REOBSERVE
        item.diagnostic_flags.append('anchor_from_memory')

    def _lookup_T_out_cam(self, cam_frame: str,
                          stamp) -> Tuple[Optional[np.ndarray], str]:
        """
        查 output←camera 的 4×4 齐次矩阵与查询状态.

        Args:
            cam_frame: 相机光学系 frame_id.
            stamp: 查询时刻（消息时间戳）；按时刻失败时回退最新 TF 并告警一次.

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
                    self.params.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
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
            self.get_logger().warning(
                f'RGB/深度分辨率不一致 {rgb.shape[:2]} vs {depth.shape[:2]} '
                f'rgb_frame={rgb_msg.header.frame_id} '
                f'depth_frame={depth_msg.header.frame_id}')
            return None
        if info.width and info.height and (
                int(info.width) != depth.shape[1]
                or int(info.height) != depth.shape[0]):
            self.get_logger().warning(
                f'CameraInfo size {info.width}x{info.height} != depth '
                f'{depth.shape[1]}x{depth.shape[0]}')
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

    def _publish_frame(self, synced, out):
        """Publish one PerceptionResult (ROS I/O only)."""
        det_msg = Detection2DArray()
        det_msg.header = out.img_header
        for det in out.kept:
            det_msg.detections.append(_to_detection2d(det, out.img_header))
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
            xyz_cam, rgb_f = _bbox_cloud_xyzrgb(
                synced.rgb, synced.depth, synced.K, out.confirmed_bboxes,
                stride=self.params.detection_cloud_stride)
            if xyz_cam.shape[0] and synced.T_out_cam is not None:
                rot, trans = synced.T_out_cam[:3, :3], synced.T_out_cam[:3, 3]
                xyz_out = (rot @ xyz_cam.T).T + trans
            else:
                xyz_out = xyz_cam
            self.pub_norm_cloud.publish(
                _xyzrgb_to_cloud_msg(out.header, xyz_out, rgb_f))
        if out.debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(out.debug, encoding='bgr8')
            dbg_msg.header = out.img_header
            self.pub_norm_debug.publish(dbg_msg)
        if out.debug_raw is not None:
            raw_msg = self.bridge.cv2_to_imgmsg(out.debug_raw, encoding='bgr8')
            raw_msg.header = out.img_header
            self.pub_norm_debug_raw.publish(raw_msg)

    def destroy_node(self):
        """停止推理 worker 后销毁 ROS 节点."""
        self._frame_worker.close(drain=False)
        return super().destroy_node()


def main(args=None):
    """
    节点入口：rclpy 初始化 → ScenePerceptionNode spin → KeyboardInterrupt 干净收尾.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = ScenePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        # 生命周期重复转换（如已 active 再收 activate）会从 rcl_lifecycle 抛出冲垮
        # spin；状态机已处于目标态，记录后继续 spin（09-17 A/B 台架实测崩溃场景）.
        node.get_logger().warn(f'忽略生命周期重复转换请求: {exc}')
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
