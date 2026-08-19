"""
PeachPose ROS 2 感知节点.

订阅时间对齐的 RGB-D + CameraInfo，经 YOLO → MobileSAM → 实测深度几何管线，
发布面为规范组单套话题 ``/peach/perception/*``（initial_pose / axis /
single_cloud / detections / masks / diagnostics / markers / debug_image /
target_observations / harvest_state；2D 候选随 target_observations 的
candidate_2d 字段下发，不再单独成话题）。

只发参考位姿，不发送运动指令。几何默认可经 TF 变到 ``output_frame``
（默认 ``base_link``，依赖 ``hand_eye_extrinsics_publisher``）。
图像编解码统一走 cv_bridge（bgr8 / passthrough uint16 / mono8）。

本模块为编排层（参数、订阅发布、回调编排、main）；纯函数按职责拆分：
  params.py        — 参数层（PeachPoseParams 集中 declare/装载）
  grasp_tf.py      — 抓取几何坐标变换（感知专属残留层，基于 peach_common_py）
  conversions.py   — 算法 dataclass/检测 dict → ROS 消息组装
  visualization.py — RViz Marker 与 debug 叠加图
  cloud_utils.py   — 检测框点云反投影与 PointCloud2 组装
  peach_pose/harvest_plan.py — 全局目标收齐式窗口锁定与多维确定性优先级
  peach_common_py       — 通用纯核（tf_utils/depth_geometry/bounded_worker/
                     harvest_data/registry）
"""
from __future__ import annotations

import dataclasses
from datetime import datetime
import json
from pathlib import Path
import threading
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Vector3Stamped
import message_filters
import numpy as np
from peach_common_py.bounded_worker import BoundedWorker
from peach_common_py.depth_geometry import normalize_depth_to_uint16_mm
from peach_common_py.harvest_data import HarvestDataStore
from peach_common_py.ros.clock_adapter import RclpyClockAdapter
from peach_common_py.tf_utils import gravity_camera_from_R, transform_msg_to_matrix
from peach_interfaces.msg import (
    BagFittingArray,
    BagGraspCandidateArray,
    PeachTargetObservation,
    PeachTargetObservationArray,
)
from peach_interfaces.srv import BeginScene
from peach_scene_perception.cloud_utils import _bbox_cloud_xyzrgb, _xyzrgb_to_cloud
from peach_scene_perception.conversions import (
    _to_candidate,
    _to_candidate_2d,
    _to_detection2d,
    _to_fitting,
)
from peach_scene_perception.grasp_tf import _apply_T_to_grasp3d, _rotation_to_quat
from peach_scene_perception.params import PeachPoseParams
from peach_scene_perception.peach_pose import impls as _impls  # noqa: F401  注册清单
from peach_scene_perception.peach_pose.candidates import (
    CandidateEstimator,
    dedup_overlapping_detections,
)
from peach_scene_perception.peach_pose.contracts import BagObservation, compute_entry_start
from peach_scene_perception.peach_pose.harvest_plan import GlobalHarvestPlan
from peach_scene_perception.peach_pose.inference import InferenceEngine
from peach_scene_perception.peach_pose.interfaces import (
    DETECTORS,
    LOCK_POLICIES,
    MATCHERS,
    POSE_PIPELINES,
    SEGMENTERS,
)
from peach_scene_perception.peach_pose.observation_quality import (
    bbox_touches_image_edge,
    classify_tracking_status,
    LightingMeter,
    STATUS_DEPTH_VOID,
    STATUS_LOST,
    STATUS_OBSERVED,
    STATUS_OCCLUDED,
    STATUS_OUT_OF_VIEW,
)
from peach_scene_perception.peach_pose.pipeline import valid_depth_mask
from peach_scene_perception.peach_pose.segmentation_gate import (
    plan_segmentation_bboxes,
    project_positions_to_pixels,
)
from peach_scene_perception.peach_pose.target_registry import TargetRegistry
from peach_scene_perception.peach_pose.timing_metrics import TimingMetrics
from peach_scene_perception.timing import AdaptiveTimeout, RateEstimator
from peach_scene_perception.visualization import _draw_debug, _to_markers
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

# 跟踪状态四分类 token → msg 常量（阶段 D1；分类纯函数在纯核
# observation_quality，零 ROS import，msg 常量只能在本层映射）
_TRACKING_STATUS_TO_MSG = {
    STATUS_OBSERVED: PeachTargetObservation.OBSERVED,
    STATUS_OCCLUDED: PeachTargetObservation.OCCLUDED,
    STATUS_LOST: PeachTargetObservation.LOST,
    STATUS_OUT_OF_VIEW: PeachTargetObservation.OUT_OF_VIEW,
    STATUS_DEPTH_VOID: PeachTargetObservation.DEPTH_VOID,
}


class PeachPoseNode(Node):
    """RGB-D 同步回调驱动的感知节点：检测 → 分割 → 几何 → TF 变换 → 多话题发布."""

    def __init__(self):
        """建节点：参数层装载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
        super().__init__('peach_scene_perception_node')
        self.bridge = CvBridge()
        # 参数层（params.py）：declare + 集中装载为 frozen dataclass；
        # 字段镜像回同名实例属性，保持本类下游引用零改动（启动期静态参数）
        PeachPoseParams.declare(self)
        self.params = PeachPoseParams.from_node(self)
        for f in dataclasses.fields(self.params):
            setattr(self, f.name, getattr(self.params, f.name))
        self.tf_timeout = Duration(seconds=self.params.tf_timeout_sec)

        share = Path(get_package_share_directory('peach_scene_perception'))
        yolo = self.yolo_model_path or str(share / 'model' / 'best.pt')
        sam = self.sam_model_path or str(share / 'model' / 'mobile_sam.pt')
        self.get_logger().info(f'YOLO={yolo}')
        self.get_logger().info(f'SAM={sam}')

        # 2.14 装配：检测/分割/位姿管线/匹配器/锁定策略全部按 yaml *.impl
        # 注册名 create 注入，调用端只持有接口层 ABC 引用
        detector = DETECTORS.create(
            self.detector_impl, yolo_model=yolo, yolo_conf=self.yolo_conf,
            yolo_iou=self.yolo_nms_iou)
        segmenter = SEGMENTERS.create(
            self.segmenter_impl, sam_model=sam,
            sam_max_bboxes=self.sam_max_bboxes, sam_min_area=self.sam_min_area)
        self.engine = InferenceEngine(detector=detector, segmenter=segmenter)
        # 有效深度窗与前景点数下限随参数下发（阶段 D1 参数化；两条管线共用
        # 同一相机/深度约定，取同一组 pipeline.* 值）
        pipeline_kwargs = {
            'min_depth_m': self.pipeline_min_depth_m,
            'max_depth_m': self.pipeline_max_depth_m,
            'min_points': self.pipeline_min_points,
        }
        bag_pipeline = POSE_PIPELINES.create(
            self.pipeline_bag_impl, tool=self.tool, **pipeline_kwargs)
        fruit_pipeline = POSE_PIPELINES.create(
            self.pipeline_fruit_impl, tool=self.tool, **pipeline_kwargs)
        self.estimator = CandidateEstimator(
            pipeline=bag_pipeline, fruit_pipeline=fruit_pipeline,
            min_mask_points=self.min_mask_points)
        # 目标身份记忆：匹配器按 matcher.impl 创建，表由 TargetRegistry 持有
        if self.target_memory_enable:
            matcher = MATCHERS.create(
                self.matcher_impl,
                match_radius=self.target_memory_match_radius_m,
                recovery_scale=self.target_memory_recovery_scale,
                cross_class_recovery=self.target_memory_cross_class_recovery)
            self.target_registry = TargetRegistry(
                matcher=matcher,
                max_targets=self.target_memory_max_targets,
                position_ema=self.target_memory_position_ema,
                confirm_frames=self.target_memory_confirm_frames,
                tentative_ttl_frames=self.target_memory_tentative_ttl_frames,
                max_age_s=self.target_memory_max_age_s,
                swing_threshold_m=self.wind_swing_threshold_m,
                swing_frames=self.wind_swing_frames)
        else:
            self.target_registry = None
        # plan 竞态防护选型：BoundedWorker（capacity=1, drop_oldest=True）的
        # 丢帧策略会静默吞掉 reset/complete 命令任务，不满足「命令不丢」语义；
        # 改造 worker 支持双优先级队列代价大于收益，故用节点级 RLock 细粒度锁：
        # harvest_plan 与 harvest_run_id/harvest_data 是同一份一致性状态，
        # 全部由本锁保护（worker 线程帧处理 + executor 线程服务回调）；
        # RLock 允许持锁内嵌套 _publish_harvest_state → _harvest_state_dict。
        self._plan_lock = threading.RLock()
        lock_policy = LOCK_POLICIES.create(
            self.lock_impl,
            min_collect_frames=self.harvest_min_collect_frames,
            lock_settle_frames=self.harvest_lock_settle_frames,
            max_collect_s=self.harvest_max_collect_s)
        self.harvest_plan = GlobalHarvestPlan(
            max_targets=self.target_memory_max_targets,
            prefer_lower_first=self.harvest_priority_prefer_lower_first,
            # 锚点帧龄阈值初值按 5 fps 名义帧率折算（30 s/120 s → 150/600
            # 帧）兜底；首帧实测帧间隔 EMA 就位后逐帧改写（协议 I4）
            anchor_max_age_frames=max(
                1, round(self.target_memory_anchor_max_age_s / 0.2)),
            anchor_drop_frames=max(
                1, round(self.target_memory_anchor_drop_s / 0.2)),
            lock_policy=lock_policy)
        self.harvest_data = HarvestDataStore()
        self.harvest_run_id = ''
        self._scene_epoch = 0
        # 阶段 D1：锁定集目标光照质量统计（观测指标，不打阻断旗标）；
        # OUT_OF_VIEW 分类用的「消失前最后检测框是否触图像边缘」记忆
        # （稳定 target_id → bool；clear_target_memory 时一并清空）
        self._lighting = LightingMeter(
            alpha=0.3,
            min_depth_ratio=self.lighting_min_depth_ratio,
            min_conf_mean=self.lighting_min_conf_mean,
            bad_frames=self.lighting_bad_frames)
        self._bbox_at_edge = {}
        # 协议 I3（时钟唯一）：节点时钟为唯一时钟源，经适配器注入纯核；
        # 帧率自适应收齐兜底 = RateEstimator(实测帧间隔 EMA, α=0.3) +
        # AdaptiveTimeout(clamp(0.4×cfg, (min_collect+settle+3)×EMA, ∞))，
        # 与原内联实现逐项等价（异常间隔 ≤1ms/>30s 不进 EMA）
        self._clock = RclpyClockAdapter(self.get_clock())
        self._frame_rate = RateEstimator(alpha=0.3)
        self._collect_window_timeout = AdaptiveTimeout(
            lower=0.4 * self.harvest_max_collect_s, upper=float('inf'),
            factor=(self.harvest_min_collect_frames
                    + self.harvest_lock_settle_frames + 3))
        # 推理耗时分项埋点：_process_rgbd 各段（detect/segment/geometry/total）
        # 用上面同一个注入时钟测量，EMA（α=0.3 与帧率同纪律）后随
        # harvest_state JSON 的 timing 子对象下发，不新增话题
        self._timing = TimingMetrics(alpha=0.3)

        # ---- 输出话题（规范组 /peach/perception/*，单套发布面）----
        # A5 起旧 ~/ 组（grasp_candidates/fitting/markers 等）已删除，下游一律
        # 订阅本组固定命名；2D 候选不再单独成话题（随 target_observations 的
        # candidate_2d 字段下发）
        self.pub_norm_pose = self.create_publisher(
            BagGraspCandidateArray, '/peach/perception/initial_pose', 10)
        self.pub_norm_axis = self.create_publisher(
            Vector3Stamped, '/peach/perception/axis', 10)
        self.pub_norm_cloud = self.create_publisher(
            PointCloud2, '/peach/perception/single_cloud', 10)
        self.pub_norm_dets = self.create_publisher(
            Detection2DArray, '/peach/perception/detections', 10)
        self.pub_norm_masks = self.create_publisher(
            Image, '/peach/perception/masks', 10)
        self.pub_norm_diag = self.create_publisher(
            BagFittingArray, '/peach/perception/diagnostics', 10)
        self.pub_norm_markers = self.create_publisher(
            MarkerArray, '/peach/perception/markers', 10)
        self.pub_norm_debug = self.create_publisher(
            Image, '/peach/perception/debug_image', 10)
        self.pub_target_observations = self.create_publisher(
            PeachTargetObservationArray,
            '/peach/perception/target_observations', 10)
        state_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_harvest_state = self.create_publisher(
            String, '/peach/perception/harvest_state', state_qos)
        self.create_service(
            Trigger, '~/query_harvest_state', self._on_query_harvest_state)
        self.create_service(BeginScene, '~/begin_scene', self._on_begin_scene)

        # 与数据集回放 / 相机驱动对齐：RELIABLE，避免 Best Effort 对不上
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

        self._frame_worker = BoundedWorker(
            self._process_rgbd, capacity=1, drop_oldest=True)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=self.sync_slop_s)
        self.sync.registerCallback(self._on_rgbd)

        # 手眼：wrist3_Link→camera_link 由 extrinsics_publisher 发静态 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_warned = False

        self.get_logger().info(
            f'Subscribed color={self.color_topic} depth={self.depth_topic} '
            f'info={self.camera_info_topic} slop={self.sync_slop_s}s '
            f'optical={self.camera_optical_frame or "(msg)"} '
            f'output={self.output_frame or "(camera)"} '
            f'depth_scale_unit={self.depth_scale_unit} '
            f'gravity_mode={self.gravity_mode} '
            f'calib={self.calibration_version}')
        if self.target_registry is not None:
            self.get_logger().info(
                f'目标身份记忆已启用：match_radius='
                f'{self.target_registry.match_radius} m, max_targets='
                f'{self.target_registry.max_targets}, ema='
                f'{self.target_registry.alpha}')
        else:
            self.get_logger().info('目标身份记忆已禁用：target_id 为帧内序号')

    def _discovery_counts(self) -> Tuple[int, int]:
        """
        发现进度摘要 (collecting_count, pending_count)（缺陷 R-D8；须持锁调用）.

        collecting_count：锁定前=收齐窗口累积的已确认目标数（plan 透传策略
        累积集大小），锁定后=锁定集大小（与 target_count 一致）；
        pending_count：锁定前=注册表确认中（未转正）记录数，锁定后恒 0；
        身份记忆禁用时恒 0（每帧记录即确认，无攒帧过程）。
        """
        collecting = self.harvest_plan.collecting_count
        if self.harvest_plan.locked or self.target_registry is None:
            return collecting, 0
        return collecting, self.target_registry.pending_count

    def _harvest_state_dict(self) -> dict:
        """返回可序列化的全局采摘计划与数据路径（持锁读取一致快照）."""
        with self._plan_lock:
            collecting_count, pending_count = self._discovery_counts()
            return {
                'harvest_run_id': self.harvest_run_id,
                'snapshot_id': self.harvest_plan.snapshot_id,
                'target_set_locked': self.harvest_plan.locked,
                'target_count': self.harvest_plan.target_count,
                # R-D8 发现进度摘要：与 target_observations 同名字段对齐
                'collecting_count': collecting_count,
                'pending_count': pending_count,
                'target_ids': list(self.harvest_plan.locked_ids),
                'completed_target_ids': sorted(self.harvest_plan.completed_ids),
                'priorities': dict(self.harvest_plan.priorities),
                'selected_target_id': self.harvest_plan.selected_target_id,
                # 阶段 D1（协议 2.4）：锚点陈旧/出视野/已移除目标集（均为
                # 纯增量键，下游只读消费）与光照质量观测指标
                'anchor_stale_target_ids': sorted(
                    self.harvest_plan.anchor_stale_ids),
                'out_of_view_target_ids': sorted(
                    self.harvest_plan.out_of_view_ids),
                'dropped_target_ids': sorted(self.harvest_plan.dropped_ids),
                'lighting': self._lighting.snapshot(),
                'low_light_quality': self._lighting.low_quality,
                # 推理耗时分项 EMA（毫秒）+ 实测 fps；详见 TimingMetrics
                'timing': self._timing.snapshot(fps=self._frame_rate.rate_hz),
                'data': self.harvest_data.query(),
            }

    def _publish_harvest_state(self) -> None:
        """发布闩锁 JSON 状态，便于运行中随时查询."""
        message = String()
        message.data = json.dumps(
            self._harvest_state_dict(), ensure_ascii=False)
        self.pub_harvest_state.publish(message)

    def _on_query_harvest_state(self, request, response):
        """~/query_harvest_state：返回当前目标集合、优先级和数据目录."""
        del request
        response.success = True
        response.message = json.dumps(
            self._harvest_state_dict(), ensure_ascii=False)
        return response

    def _on_begin_scene(self, request, response):
        """BeginScene：新 scene_epoch，清空身份表并重置收齐窗口."""
        with self._plan_lock:
            self._scene_epoch += 1
            old_run = self.harvest_run_id
            if old_run:
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'scene_begin',
                    'scene_key': request.scene_key,
                    'scene_epoch': self._scene_epoch,
                })
            self.harvest_plan.reset()
            self.harvest_data = HarvestDataStore(root=self.harvest_data.root)
            self.harvest_run_id = ''
            cleared = 0
            if self.target_registry is not None:
                cleared = self.target_registry.clear()
            self._bbox_at_edge.clear()
            self._publish_harvest_state()
        response.accepted = True
        response.scene_epoch = self._scene_epoch
        response.message = (
            f'scene_epoch={self._scene_epoch} key={request.scene_key} '
            f'cleared={cleared} prev_run={old_run or "none"}')
        return response

    def _start_harvest_run(self) -> None:
        """为刚锁定的全局目标集合创建不可变 manifest（须持 _plan_lock 调用）."""
        with self._plan_lock:
            now = datetime.now()
            self.harvest_run_id = (
                f'harvest_{now.strftime("%Y%m%dT%H%M%S_%f")}_'
                f's{self.harvest_plan.snapshot_id}')
            targets = [
                {'target_id': target_id,
                 'priority': self.harvest_plan.priority(target_id)}
                for target_id in self.harvest_plan.locked_ids
            ]
            self.harvest_data.start(self.harvest_run_id, {
                'snapshot_id': self.harvest_plan.snapshot_id,
                'target_count': self.harvest_plan.target_count,
                'selected_target_id': self.harvest_plan.selected_target_id,
                'targets': targets,
                'model_version': self.model_version,
                'calibration_version': self.calibration_version,
                'output_frame': self.output_frame,
            })
            self.harvest_data.append_event({
                'source': 'perception', 'event': 'global_targets_locked',
                'target_count': self.harvest_plan.target_count,
                'selected_target_id': self.harvest_plan.selected_target_id,
            })

    def _publish_target_observations(
            self, header, mask_header, records, payloads) -> None:
        """
        发布锁定 ID 的逐目标结果，并记录选中目标掩膜与状态事件.

        全程持 _plan_lock：plan.update 及后续逐字段读取必须与服务回调
        （reset/complete）互斥，保证 plan 单写者与快照一致性。
        """
        with self._plan_lock:
            was_locked = self.harvest_plan.locked
            # 帧率自适应收齐兜底：按实测帧间隔 EMA 伸缩 max_collect_s（帧率
            # 以运行状态为准）——低帧率放大防误锁空集，高帧率收紧提速；
            # 配置值 ×0.4 作下限。异常间隔（暂停后首帧）不进 EMA。
            # 协议 I3：now 取节点时钟（同一时钟源同时驱动窗口超时判定）
            now_s = self._clock.now()
            self._frame_rate.update(now_s)
            frame_interval = self._frame_rate.interval
            if frame_interval is not None:
                self.harvest_plan.max_collect_s = (
                    self._collect_window_timeout.value(frame_interval))
                # 锚点新鲜度两档时限（阶段 D1，协议 2.4/I4）：秒级上限 ÷
                # 实测帧间隔 EMA = 帧数阈值，逐帧改写；帧率跌落时帧数变少，
                # 墙钟上限保持不变
                self.harvest_plan.anchor_max_age_frames = max(
                    1, round(self.target_memory_anchor_max_age_s
                             / frame_interval))
                self.harvest_plan.anchor_drop_frames = max(
                    1, round(self.target_memory_anchor_drop_s
                             / frame_interval))
            # OUT_OF_VIEW 预分类（须在 plan.update 前完成：计划按本集合做
            # 不可选/去选判定）：锁定目标本帧无观测且消失前最后检测框触
            # 图像边缘 → 走出视野；单位姿模型下复扫无益，视为不可选（2.4）
            out_of_view_ids = {
                target_id for target_id in self.harvest_plan.locked_ids
                if target_id not in payloads
                and self._bbox_at_edge.get(target_id, False)
            }
            current = self.harvest_plan.update(
                records, now=now_s, out_of_view_ids=out_of_view_ids)
            # 锚点超龄移除（LOST 超 anchor_drop）：记账 target_dropped 事件，
            # 编排侧据此按 SKIPPED_UNREACHABLE「目标丢失超时」入账（协议 2.4）
            for dropped_id in self.harvest_plan.pop_dropped():
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'target_dropped',
                    'target_id': dropped_id,
                    'reason': 'anchor_drop_timeout（目标丢失超时）',
                })
            try:
                if self.harvest_plan.locked and not was_locked:
                    self._start_harvest_run()
            except OSError as exc:
                self.get_logger().error(f'采摘运行目录创建失败: {exc}')

            # 光照质量统计（阶段 D1；观测指标，不打阻断旗标）：锁定集中
            # 本帧带掩膜观测的目标，逐帧注入掩膜内有效深度占比与置信度
            if self.harvest_plan.locked:
                depth_ratios = []
                confidences = []
                for target_id in self.harvest_plan.locked_ids:
                    payload = payloads.get(target_id)
                    if payload is None or payload.get('mask') is None:
                        continue
                    depth_ratios.append(payload.get('mask_depth_ratio', 0.0))
                    confidences.append(float(
                        current.get(target_id, {}).get('confidence', 0.0)))
                self._lighting.update(depth_ratios, confidences)
            if self._lighting.low_quality:
                self.get_logger().warning(
                    f'光照质量持续偏低（{self.lighting_bad_frames} 帧连击：'
                    f'掩膜内有效深度占比 EMA='
                    f'{self._lighting.snapshot()["depth_ratio"]} < '
                    f'{self.lighting_min_depth_ratio} 或置信度 EMA < '
                    f'{self.lighting.min_conf_mean}），建议现场补光/调曝光',
                    throttle_duration_sec=10.0)

            array = PeachTargetObservationArray()
            array.header = header
            array.snapshot_id = self.harvest_plan.snapshot_id
            array.harvest_run_id = self.harvest_run_id
            array.target_set_locked = self.harvest_plan.locked
            array.target_count = self.harvest_plan.target_count
            array.selected_target_id = self.harvest_plan.selected_target_id
            # R-D8 发现进度摘要：锁定前 observations 恒空，下游经本两字段
            # 跟踪收齐进度；锁定后=锁定集大小/0（语义见 msg 注释）
            array.collecting_count, array.pending_count = (
                self._discovery_counts())
            observed_ids = []
            stamp_ns = Time.from_msg(mask_header.stamp).nanoseconds
            for target_id in self.harvest_plan.locked_ids:
                item = PeachTargetObservation()
                item.header = header
                item.target_id = target_id
                item.priority = self.harvest_plan.priority(target_id)
                item.confirmed = True
                item.selected = target_id == self.harvest_plan.selected_target_id
                item.harvest_status = self.harvest_plan.harvest_status(target_id)
                payload = payloads.get(target_id)
                record = current.get(target_id, {})
                # 跟踪状态四分类（阶段 D1，协议 2.4 第 4 条）：分类纯函数
                # 在 observation_quality，本处只做 token → msg 常量映射
                token = classify_tracking_status(
                    has_observation=payload is not None,
                    has_mask=(payload is not None
                              and payload.get('mask') is not None),
                    mask_depth_ratio=(
                        None if payload is None
                        else payload.get('mask_depth_ratio')),
                    min_depth_ratio=self.lighting_min_depth_ratio,
                    last_bbox_touched_edge=self._bbox_at_edge.get(
                        target_id, False))
                item.tracking_status = _TRACKING_STATUS_TO_MSG[token]
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
                if (target_id in self.harvest_plan.anchor_stale_ids
                        and 'anchor_stale' not in item.diagnostic_flags):
                    item.diagnostic_flags.append('anchor_stale')
                # 几何退化统一兜底（LOST 无 payload，或观测帧深度失败导致
                # bag_bottom/neck 近零）：回填注册表记忆锚点——世界系身份记忆
                # 的本职用途；带 anchor_from_memory 标记，能力端不把它当新鲜
                # 观测（不刷新新鲜度），但锚点可派发/可规划
                _b = item.candidate.bag_bottom
                _n = item.candidate.bag_neck
                _degenerate = (
                    abs(_b.x) < 1e-6 and abs(_b.y) < 1e-6 and abs(_b.z) < 1e-6) or (
                    abs(_n.x) < 1e-6 and abs(_n.y) < 1e-6 and abs(_n.z) < 1e-6)
                if _degenerate:
                    entry = (self.target_registry.get(target_id)
                             if self.target_registry is not None else None)
                    if entry is not None and entry.get('position') is not None:
                        center = np.asarray(entry['position'], dtype=np.float64)
                        axis = entry.get('axis')
                        axis = (np.array([0.0, 0.0, 1.0]) if axis is None
                                else np.asarray(axis, dtype=np.float64))
                        half = 0.5 * float(entry.get('diameter') or 0.06)
                        bottom = center - axis * half
                        neck = center + axis * half
                        item.candidate.target_id = target_id
                        item.candidate.bag_bottom.x = float(bottom[0])
                        item.candidate.bag_bottom.y = float(bottom[1])
                        item.candidate.bag_bottom.z = float(bottom[2])
                        item.candidate.bag_neck.x = float(neck[0])
                        item.candidate.bag_neck.y = float(neck[1])
                        item.candidate.bag_neck.z = float(neck[2])
                        item.candidate.translation_direction.x = float(axis[0])
                        item.candidate.translation_direction.y = float(axis[1])
                        item.candidate.translation_direction.z = float(axis[2])
                        # entry_pose 一并回填（下游契约要求完整入口位姿）：位置用
                        # contracts 纯函数 compute_entry_start（standoff 与管线
                        # 一致 = entry_d_tool + entry_d_s）；姿态由袋轴构造右手
                        # 抓取系 R=[Xg,Yg,Zg]（无点云可用，取 pipeline._frame
                        # 退化分支同法的参考轴叉积）
                        axis_norm = float(np.linalg.norm(axis))
                        zg = (axis / axis_norm if axis_norm > 1e-9
                              else np.array([0.0, 0.0, 1.0]))
                        standoff = self.tool.entry_d_tool + self.tool.entry_d_s
                        entry_start = compute_entry_start(bottom, zg, standoff)
                        item.candidate.entry_pose.position.x = float(entry_start[0])
                        item.candidate.entry_pose.position.y = float(entry_start[1])
                        item.candidate.entry_pose.position.z = float(entry_start[2])
                        ref = (np.array([1.0, 0.0, 0.0]) if abs(zg[0]) < 0.9
                               else np.array([0.0, 0.0, 1.0]))
                        xg = np.cross(zg, ref)
                        xg /= np.linalg.norm(xg)
                        if xg[0] < 0:
                            xg = -xg
                        yg = np.cross(zg, xg)
                        frame = np.column_stack((xg, yg, zg))
                        item.candidate.entry_pose.orientation = (
                            _rotation_to_quat(frame))
                        item.candidate.status = item.candidate.REOBSERVE
                        item.diagnostic_flags.append('anchor_from_memory')
                array.observations.append(item)
            self.pub_target_observations.publish(array)
            if self.harvest_plan.locked:
                self.harvest_data.append_event({
                    'source': 'perception', 'event': 'frame_observations',
                    'stamp_ns': stamp_ns, 'observed_target_ids': observed_ids,
                    'selected_target_id': self.harvest_plan.selected_target_id,
                })
            self._publish_harvest_state()

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
        if not self.output_frame or self.output_frame == cam_frame:
            return np.eye(4), 'ok'
        stamp_time = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, cam_frame, stamp_time, timeout=self.tf_timeout)
            return transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return transform_msg_to_matrix(tf.transform), 'stale'
            except TransformException as ex:
                self.get_logger().warning(
                    f'TF 失败，输出退回相机系 {cam_frame}: {ex}')
                return None, 'unavailable'

    def _segmentation_bboxes(self, kept, T_out_cam, K):
        """
        决定本帧送 SAM 的检测框集（阶段 H，协议 2.13-E1 锁定后 selected-only）.

        锁定前/开关关/TF 不可用帧全量（旧行为）；锁定后把锁定且未终局目标
        的世界系记忆锚点（TargetRegistry 表项 position）经 T_out_cam 逆变换
        反投影到本帧像素，只分割「包含锚点投影」的检测框（纯核策略见
        peach_pose/segmentation_gate.py，重叠框宁多勿漏）。已终局
        （completed）目标不再分割：账目已定，掩膜不再进任何判定。
        分割失败的锁定目标在下游被显式判 OCCLUDED + mask_unavailable，
        几何走深度带降级——不漏报、不静默。

        Args:
            kept: 本帧入管线检测 dict 列表（置信度过滤+去重后）.
            T_out_cam: (4, 4) output←camera 齐次矩阵或 None（TF 彻底失败）.
            K: 本帧相机内参 dict（fx/fy/cx/cy/width/height）.

        Returns
        -------
            送 SAM 的 (x1, y1, x2, y2) 框列表（顺序与 kept 一致）.

        """
        # 持 _plan_lock：harvest_plan 与 target_registry 是同一份一致性状态
        # （clear_target_memory 服务回调在同锁下清表），门控读取须与之互斥；
        # 锁内只有 dict 读取与一次 4×4 求逆，耗时微秒级
        with self._plan_lock:
            locked = self.harvest_plan.locked
            anchor_px = None
            if (self.pipeline_locked_only_segmentation and locked
                    and self.target_registry is not None
                    and T_out_cam is not None):
                positions = {}
                for target_id in self.harvest_plan.locked_ids:
                    if target_id in self.harvest_plan.completed_ids:
                        continue
                    entry = self.target_registry.get(target_id)
                    if entry is not None and entry.get('position') is not None:
                        positions[target_id] = np.asarray(
                            entry['position'], dtype=float)
                anchor_px = project_positions_to_pixels(
                    positions, np.linalg.inv(T_out_cam), K)
            return plan_segmentation_bboxes(
                kept, self.pipeline_locked_only_segmentation, locked, anchor_px)

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """将最新同步帧交给容量一推理 worker."""
        if not self._frame_worker.submit((rgb_msg, depth_msg, info)):
            self.get_logger().warning('感知 worker 已停止，丢弃 RGB-D 帧')

    def _process_rgbd(self, frame):
        """
        同步回调 (ApproximateTimeSynchronizer)：一帧 RGB-D → 全套感知输出.

        Args:
            rgb_msg: 彩色图（bgr8）.
            depth_msg: 深度图（uint16 原始值或 32FC1 米制；回调内经
                normalize_depth_to_uint16_mm 统一为 uint16 毫米）.
            info: 彩色相机内参（须与深度图同分辨率）.

        Returns
        -------
            无返回值（None）；感知结果经各发布者发出.

        """
        rgb_msg, depth_msg, info = frame
        # 推理耗时分项埋点（协议 I3：唯一时钟源为注入的节点时钟，禁止
        # time.time/perf_counter 第二时钟）；早退帧（转换失败/检测异常等）
        # 不记录，只统计完整走完管线的帧
        t_total_start = self._clock.now()
        self.get_logger().debug(
            f'RGB-D sync frame {rgb_msg.width}x{rgb_msg.height}')
        # RGB/深度时间戳偏差：DEBUG 每帧记录；接近同步允差时 WARN 节流提示
        dt_ms = (Time.from_msg(rgb_msg.header.stamp).nanoseconds
                 - Time.from_msg(depth_msg.header.stamp).nanoseconds) / 1e6
        self.get_logger().debug(f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms')
        if abs(dt_ms) > self.sync_slop_s * 0.8 * 1000.0:
            self.get_logger().warning(
                f'RGB-D 时间戳偏差 {dt_ms:+.1f} ms 已超同步允差 '
                f'{self.sync_slop_s * 1000.0:.0f} ms 的 80%，请检查相机时间戳源',
                throttle_duration_sec=1.0)
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'RGB convert failed: {exc}')
            return
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Depth convert failed: {exc}')
            return
        # uint16：raw × depth_scale_unit = 毫米；32FC1：米 ×1000 = 毫米
        try:
            depth = normalize_depth_to_uint16_mm(depth_raw, self.depth_scale_unit)
        except ValueError as exc:
            self.get_logger().warning(f'Depth convert failed: {exc}')
            return
        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warning(
                f'RGB/depth size mismatch {rgb.shape[:2]} vs {depth.shape[:2]}')
            return
        if info.width and info.height and (
                int(info.width) != depth.shape[1] or int(info.height) != depth.shape[0]):
            self.get_logger().warning(
                f'CameraInfo size {info.width}x{info.height} != depth '
                f'{depth.shape[1]}x{depth.shape[0]}')
            return

        # 内参：始终用本机 CameraInfo（勿回退 FOV 推导，避免与标定不一致）
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
        # 几何在相机光心系求解，再按需变到 output_frame
        cam_frame = (
            self.camera_optical_frame
            or depth_msg.header.frame_id
            or rgb_msg.header.frame_id
            or info.header.frame_id)
        out_frame = self.output_frame or cam_frame
        # 三维几何来自深度图，TF 与 3D 输出头必须使用 depth.header.stamp。
        # ApproximateTimeSynchronizer 只保证三路时间差落在 slop 内；连续运动时若
        # 错用 RGB 时间戳，几十毫秒偏差也会把目标中心变换到错误的 base 位姿。
        geometry_stamp = depth_msg.header.stamp
        T_out_cam, tf_status = self._lookup_T_out_cam(
            cam_frame, geometry_stamp)
        if T_out_cam is None and self.output_frame:
            # TF 失败则退回相机系，避免静默用错坐标系
            out_frame = cam_frame
            T_out_cam = np.eye(4)

        # 重力方向：fixed 用参数提示；tf 模式由本帧 TF 旋转反推相机系重力
        # （TF 不可用的帧回退 gravity_hint_xyz，结果带 tf_unavailable 标记）
        gravity_hint = self.gravity_hint
        if self.gravity_mode == 'tf':
            if self.output_frame and tf_status != 'unavailable':
                gravity_hint = gravity_camera_from_R(T_out_cam[:3, :3])
            else:
                self.get_logger().warning(
                    'gravity_mode=tf 但 TF 不可用或未设 output_frame，'
                    '本帧回退 gravity_hint_xyz',
                    throttle_duration_sec=1.0)

        header = Header()
        header.stamp = geometry_stamp
        header.frame_id = out_frame
        # 图像平面数据（检测框/掩膜/debug 图）的 frame_id 用 RGB 图自身坐标系；
        # 3D 结果（候选/拟合/Marker/检测点云）仍用输出系
        img_header = Header()
        img_header.stamp = rgb_msg.header.stamp
        img_header.frame_id = rgb_msg.header.frame_id

        # ---- 检测 ----
        # YOLO 异常（权重缺失/CUDA 错误等）不得炸穿 worker：记日志跳过本帧，
        # 与上方 convert 失败的早退形状一致（SAM 侧已在 engine 内捕获返回 []）
        t_detect_start = self._clock.now()
        try:
            dets = self.engine.detect(rgb)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'YOLO 检测异常，跳过本帧: {exc}', throttle_duration_sec=1.0)
            return
        # 置信度过滤（第二级，严出）→ IoS 去重（消一果两框/局部误检小框，
        # 跨类生效，防同一物理目标在身份表重复占号）；发布的 detections
        # 即实际入管线的目标
        kept = [d for d in dets
                if float(d.get('conf', 0.0)) >= self.min_detection_conf]
        kept = dedup_overlapping_detections(kept, self.detection_dedup_ios)
        # detect 段 = YOLO 推理 + 置信度过滤 + IoS 去重（入管线目标的完整出品）
        self._timing.record(
            'detect_ms', (self._clock.now() - t_detect_start) * 1e3)
        det_msg = Detection2DArray()
        det_msg.header = img_header
        for d in kept:
            det_msg.detections.append(_to_detection2d(d, img_header))
        self.pub_norm_dets.publish(det_msg)

        # 检测框内彩色点云（深度反投影），便于 RViz 对照相机全图点云
        if self.publish_detection_cloud:
            bboxes = [d['bbox'] for d in kept]
            xyz_cam, rgb_f = _bbox_cloud_xyzrgb(
                rgb, depth, K, bboxes, stride=self.detection_cloud_stride)
            if xyz_cam.shape[0] and T_out_cam is not None:
                R, t = T_out_cam[:3, :3], T_out_cam[:3, 3]
                xyz_out = (R @ xyz_cam.T).T + t
            else:
                xyz_out = xyz_cam
            # 点已随几何一起变到 out_frame，frame_id 保持输出系（非相机系）
            cloud_msg = _xyzrgb_to_cloud(header, xyz_out, rgb_f)
            self.pub_norm_cloud.publish(cloud_msg)

        mask_canvas = np.zeros(depth.shape[:2], dtype=np.uint16)
        debug = rgb.copy() if self.publish_debug_image else None
        cand_arr = BagGraspCandidateArray()
        cand_arr.header = header
        fit_arr = BagFittingArray()
        fit_arr.header = header
        markers = MarkerArray()
        # DELETEALL 不要设 ns/id：否则会与首个 ADD (peach_pose, 0) 冲突，
        # RViz 报 "same ns and id: (peach_pose, 0)"
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        # 本帧各候选的 (状态, 平移方向)，供 /peach/perception/axis 选最优
        frame_axes: List[Tuple[str, Optional[np.ndarray]]] = []
        harvest_records = []
        harvest_payloads = {}
        mask_header = Header()
        mask_header.stamp = geometry_stamp
        mask_header.frame_id = cam_frame

        # ---- 逐目标：SAM → 前景∩深度 → 袋/果管线 → TF → 消息 ----
        # 目标身份记忆：仅本帧输出在世界系（TF ok/stale）才匹配/注册；
        # tf_unavailable 帧几何退回相机系，注册会污染世界系表，跳过
        track_this_frame = (self.target_registry is not None
                            and tf_status != 'unavailable')
        if track_this_frame:
            # I3：与 match_or_register 注入同一节点时钟——max_age_s 墙钟
            # 淘汰（阶段 D1）要求两入口同一时钟基准；不注入则注册表跳过
            # 墙钟淘汰（防 time.monotonic 兜底与注入时钟混比误清表项）
            self.target_registry.begin_frame(now=self._clock.now())
        # ---- SAM 批量分割：整帧收集全部 bbox 一次 forward（N 目标 N 次 → 1 次），
        # 再按 bbox 精确取回各目标掩膜（segment 丢弃面积过小掩膜，返回项与目标
        # 非一一对齐，故按 bbox 建映射而非按下标）；批量路径自身异常时回退逐目标
        # 调用兜底（engine 内部已捕获的 SAM 推理失败返回 []，不触发本回退）
        t_segment_start = self._clock.now()
        # 阶段 H（协议 2.13-E1）：锁定后 selected-only 门控——只对锁定集目标的
        # 检测框跑 SAM（锁定前/开关关/TF 不可用帧全量，见 _segmentation_bboxes）；
        # segment_ms 分项随之真实回落，是 E1「帧率随目标数回升」的量测口径
        frame_bboxes = self._segmentation_bboxes(kept, T_out_cam, K)
        mask_by_bbox = {}
        if frame_bboxes:
            try:
                segs = self.engine.segment(rgb, frame_bboxes)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(
                    f'SAM 批量分割异常，回退逐目标调用: {exc}')
                segs = []
                for fallback_bbox in frame_bboxes:
                    try:
                        segs.extend(self.engine.segment(rgb, [fallback_bbox]))
                    except Exception as exc_single:  # noqa: BLE001
                        self.get_logger().warning(
                            f'SAM 单目标分割异常: {exc_single}')
            mask_by_bbox = {bbox: mask for mask, bbox in segs}
        # segment 段 = SAM 批量分割（含异常时的逐目标回退兜底耗时）
        self._timing.record(
            'segment_ms', (self._clock.now() - t_segment_start) * 1e3)
        # geometry 段 = 逐目标袋/果双管线位姿估计 + TF 变换 + 候选消息构造
        t_geometry_start = self._clock.now()
        # 有效深度掩膜（全图，阶段 D1）：逐目标「掩膜内有效深度占比」的
        # 分母/分子基础，光照质量指标与 DEPTH_VOID 分类共用；参数与几何
        # 管线同源（pipeline.*），口径一致
        valid_depth_full = valid_depth_mask(
            depth, self.pipeline_min_depth_m, self.pipeline_max_depth_m)
        pending = []
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = mask_by_bbox.get(bbox)
            if sam_mask is not None:
                mask_canvas[sam_mask > 0] = np.uint16(i + 1)

            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=gravity_hint,
                detections=[det],
                metadata={
                    'model_version': self.model_version,
                    'calibration_version': self.calibration_version,
                },
            )
            results = self.estimator.estimate_modes(
                obs, f'target_{i}', bbox, sam_mask)
            result = results['hybrid_dilated']
            # TF 回退打标：本帧几何可信度经 diagnostic_flags 暴露给下游
            if tf_status != 'ok':
                flag = 'tf_stale' if tf_status == 'stale' else 'tf_unavailable'
                if flag not in result.grasp_3d.diagnostic_flags:
                    result.grasp_3d.diagnostic_flags.append(flag)
            camera_anchor = result.grasp_3d.points_centroid
            if camera_anchor is None:
                camera_anchor = result.grasp_3d.bag_bottom
            if camera_anchor is None:
                camera_anchor = result.grasp_3d.position
            if camera_anchor is None:
                camera_anchor = result.grasp_3d.entry_start
            camera_distance_m = (
                0.0 if camera_anchor is None
                else float(np.linalg.norm(camera_anchor)))
            if T_out_cam is not None and out_frame != cam_frame:
                _apply_T_to_grasp3d(result.grasp_3d, T_out_cam)
            frame_axes.append((result.grasp_3d.status,
                               result.grasp_3d.translation_direction))
            pending.append({
                'i': i, 'det': det, 'bbox': bbox, 'sam_mask': sam_mask,
                'result': result, 'camera_distance_m': camera_distance_m,
            })

        # 整帧一次匈牙利分配，避免逐检测贪心交叉误绑
        assigned_ids = [(f'target_{p["i"]}', False) for p in pending]
        if self.target_registry is not None and track_this_frame:
            assign_items = []
            for p in pending:
                g3d = p['result'].grasp_3d
                anchor = g3d.points_centroid
                if anchor is None:
                    anchor = g3d.bag_bottom
                if anchor is None:
                    anchor = g3d.position
                if anchor is None:
                    anchor = g3d.entry_start
                assign_items.append({
                    'position': anchor,
                    'class_id': int(p['det'].get('class_id', 0)),
                    'axis': g3d.translation_direction,
                    'diameter': float(g3d.bag_diameter_upper_m or 0.0),
                    'status': g3d.status,
                })
            assigned_ids = self.target_registry.match_or_register_frame(
                assign_items, now=self._clock.now())

        for p, (tid, is_new) in zip(pending, assigned_ids):
            i, det, bbox, sam_mask = (
                p['i'], p['det'], p['bbox'], p['sam_mask'])
            result = p['result']
            camera_distance_m = p['camera_distance_m']
            if self.target_registry is not None:
                if track_this_frame:
                    result.grasp_3d.diagnostic_flags.append(
                        'target_new' if is_new else 'target_matched')
                    if str(tid).startswith('ambiguous_'):
                        result.grasp_3d.diagnostic_flags.append(
                            'target_ambiguous')
                    tracked_entry = self.target_registry.get(tid)
                    if (tracked_entry is not None
                            and tracked_entry.get('swinging')):
                        result.grasp_3d.diagnostic_flags.append(
                            'target_swinging')
                    self._bbox_at_edge[tid] = bbox_touches_image_edge(
                        bbox, depth.shape[1], depth.shape[0])
                else:
                    result.grasp_3d.diagnostic_flags.append('target_untracked')
            g3d, g2d = result.grasp_3d, result.grasp_2d
            candidate_msg = _to_candidate(
                header, tid, g3d, model_version=self.model_version,
                calibration_version=self.calibration_version,
                tool_version=self.tool.version)
            candidate_2d_msg = _to_candidate_2d(header, tid, g2d)
            fitting_msg = _to_fitting(header, tid, result)
            cand_arr.candidates.append(candidate_msg)
            fit_arr.fittings.append(fitting_msg)
            registry_item = (
                None if self.target_registry is None
                else self.target_registry.get(tid))
            confirmed = (
                True if self.target_registry is None
                else bool(registry_item and registry_item['confirmed']))
            # 果实高度（output_frame 系 Z，供全局计划「先低后高」排序）：
            # 取袋底 bag_bottom——果实最低点最贴近「高度」语义；几何缺失时
            # 按锚点链回退，全 None 则不写该键（排序键按 inf 兜底）。
            # 此处 grasp_3d 已经 _apply_T_to_grasp3d 变到 out_frame；
            # output_frame 为世界系（默认 base_link）时 Z 才是真实高度
            base_anchor = result.grasp_3d.bag_bottom
            if base_anchor is None:
                base_anchor = result.grasp_3d.points_centroid
            if base_anchor is None:
                base_anchor = result.grasp_3d.position
            if base_anchor is None:
                base_anchor = result.grasp_3d.entry_start
            record = {
                'target_id': tid, 'status': int(candidate_msg.status),
                'confidence': float(candidate_msg.confidence),
                'camera_distance_m': camera_distance_m,
                'confirmed': confirmed,
                'diagnostic_flags': list(candidate_msg.diagnostic_flags),
            }
            if base_anchor is not None:
                record['base_height_m'] = float(base_anchor[2])
            harvest_records.append(record)
            # 掩膜内有效深度占比（阶段 D1：光照质量指标 + DEPTH_VOID
            # 分类共用）；无掩膜记 0.0（不会被采信——无掩膜帧不入统计）
            mask_depth_ratio = 0.0
            if sam_mask is not None:
                sam_foreground = np.asarray(sam_mask) > 0
                n_foreground = int(np.count_nonzero(sam_foreground))
                if n_foreground > 0:
                    mask_depth_ratio = float(
                        np.count_nonzero(sam_foreground & valid_depth_full)
                        / n_foreground)
            harvest_payloads[tid] = {
                'candidate': candidate_msg,
                'candidate_2d': candidate_2d_msg,
                'fitting': fitting_msg,
                'mask': sam_mask,
                'mask_depth_ratio': mask_depth_ratio,
            }
            markers.markers.extend(_to_markers(
                header, tid, i, result,
                tool_d_inner=float(self.tool.D_inner)))
            if debug is not None:
                _draw_debug(debug, det, g2d, sam_mask, tid)

        self._timing.record(
            'geometry_ms', (self._clock.now() - t_geometry_start) * 1e3)
        self._publish_target_observations(
            header, mask_header, harvest_records, harvest_payloads)
        self.pub_norm_pose.publish(cand_arr)
        self.pub_norm_diag.publish(fit_arr)
        self.pub_norm_markers.publish(markers)
        # /peach/perception/axis：最优候选（第一个 ACCEPT，否则第一个有效
        # 方向）的平移方向；无候选或无有效方向不发布
        best_dir = None
        for status, direction in frame_axes:
            if direction is None:
                continue
            if status == 'ACCEPT':
                best_dir = direction
                break
            if best_dir is None:
                best_dir = direction
        if best_dir is not None:
            axis_msg = Vector3Stamped()
            axis_msg.header = header
            axis_msg.vector = Vector3(
                x=float(best_dir[0]), y=float(best_dir[1]), z=float(best_dir[2]))
            self.pub_norm_axis.publish(axis_msg)
        self.get_logger().debug(
            f'Published {len(cand_arr.candidates)} candidates '
            f'(dets={len(kept)})')
        if self.target_registry is not None:
            st = self.target_registry.stats()
            self.get_logger().info(
                f'目标注册表在册 {st["n_targets"]} 个目标'
                f'（累计注册 {st["n_registered"]}、累计命中 {st["n_matched"]}）',
                throttle_duration_sec=10.0)
        if self.publish_masks:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_canvas, encoding='mono16')
            mask_msg.header = img_header
            self.pub_norm_masks.publish(mask_msg)
        if debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = img_header
            self.pub_norm_debug.publish(dbg_msg)
        # total 段 = 整帧 _process_rgbd（含转换/检测/分割/几何/发布全链路）
        self._timing.record(
            'total_ms', (self._clock.now() - t_total_start) * 1e3)

    def destroy_node(self):
        """停止推理 worker 后销毁 ROS 节点."""
        self._frame_worker.close(drain=False)
        return super().destroy_node()


def main(args=None):
    """
    节点入口：rclpy 初始化 → PeachPoseNode spin → KeyboardInterrupt 干净收尾.

    Args:
        args: 透传给 rclpy.init 的命令行参数；None 用 sys.argv.

    Returns
    -------
        无返回值（None）；节点随 spin 结束销毁.

    """
    rclpy.init(args=args)
    node = PeachPoseNode()
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
