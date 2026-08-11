"""
PeachPose ROS 2 感知节点.

订阅时间对齐的 RGB-D + CameraInfo，经 YOLO → MobileSAM → 实测深度几何管线，
发布抓取参考候选 / 2D / 拟合诊断 / 检测 / 掩膜 / Marker / debug 图 / 检测框点云。

只发参考位姿，不发送运动指令。几何默认可经 TF 变到 ``output_frame``
（默认 ``base_link``，依赖 ``hand_eye_extrinsics_publisher``）。
图像编解码统一走 cv_bridge（bgr8 / passthrough uint16 / mono8）。

本模块为编排层（参数、订阅发布、回调编排、main）；纯函数按职责拆分：
  params.py        — 参数层（PeachPoseParams 集中 declare/装载）
  tf_utils.py      — TF/旋转工具（官方 tf_transformations）
  conversions.py   — 算法 dataclass/检测 dict → ROS 消息组装
  visualization.py — RViz Marker 与 debug 叠加图
  cloud_utils.py   — 检测框点云反投影与 PointCloud2 组装
  harvest_plan.py  — 全局目标数量锁定与确定性优先级
  harvest_data.py  — manifest/事件/掩膜数据管理
"""
from __future__ import annotations

import dataclasses
from datetime import datetime
import json
from pathlib import Path
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3, Vector3Stamped
import message_filters
import numpy as np
from peach_pose_msgs.msg import (
    BagFittingArray,
    BagGrasp2DArray,
    BagGraspCandidateArray,
    PeachTargetObservation,
    PeachTargetObservationArray,
)
from peach_pose_ros2.cloud_utils import _bbox_cloud_xyzrgb, _xyzrgb_to_cloud
from peach_pose_ros2.conversions import (
    _to_candidate,
    _to_candidate_2d,
    _to_detection2d,
    _to_fitting,
)
from peach_pose_ros2.harvest_data import HarvestDataStore
from peach_pose_ros2.harvest_plan import GlobalHarvestPlan
from peach_pose_ros2.params import PeachPoseParams
from peach_pose_ros2.peach_pose.candidates import (
    CandidateEstimator,
    dedup_overlapping_detections,
)
from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.depth_geometry import normalize_depth_to_uint16_mm
from peach_pose_ros2.peach_pose.inference import InferenceEngine
from peach_pose_ros2.tf_utils import (
    _apply_T_to_grasp3d,
    _gravity_camera_from_R,
    _transform_msg_to_matrix,
)
from peach_pose_ros2.visualization import _draw_debug, _to_markers
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


class PeachPoseNode(Node):
    """RGB-D 同步回调驱动的感知节点：检测 → 分割 → 几何 → TF 变换 → 多话题发布."""

    def __init__(self):
        """建节点：参数层装载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
        super().__init__('peach_pose_node')
        self.bridge = CvBridge()
        # 参数层（params.py）：declare + 集中装载为 frozen dataclass；
        # 字段镜像回同名实例属性，保持本类下游引用零改动（启动期静态参数）
        PeachPoseParams.declare(self)
        self.params = PeachPoseParams.from_node(self)
        for f in dataclasses.fields(self.params):
            setattr(self, f.name, getattr(self.params, f.name))
        self.tf_timeout = Duration(seconds=self.params.tf_timeout_sec)

        share = Path(get_package_share_directory('peach_pose_ros2'))
        yolo = self.yolo_model_path or str(share / 'model' / 'best.pt')
        sam = self.sam_model_path or str(share / 'model' / 'mobile_sam.pt')
        self.get_logger().info(f'YOLO={yolo}')
        self.get_logger().info(f'SAM={sam}')

        self.engine = InferenceEngine(
            yolo_model=yolo, sam_model=sam, yolo_conf=self.yolo_conf)
        from peach_pose_ros2.peach_pose.pipeline import RobustBagPosePipeline
        self.estimator = CandidateEstimator(
            pipeline=RobustBagPosePipeline(tool=self.tool))
        self.harvest_plan = GlobalHarvestPlan(
            max_targets=self.target_memory_max_targets)
        self.harvest_data = HarvestDataStore()
        self.harvest_run_id = ''

        # ---- 输出话题（相对命名空间 ~/）----
        self.pub_cands = self.create_publisher(
            BagGraspCandidateArray, '~/grasp_candidates', 10)
        self.pub_cands_2d = self.create_publisher(
            BagGrasp2DArray, '~/grasp_candidates_2d', 10)
        self.pub_fitting = self.create_publisher(
            BagFittingArray, '~/fitting', 10)
        self.pub_dets = self.create_publisher(
            Detection2DArray, '~/detections', 10)
        self.pub_masks = self.create_publisher(Image, '~/masks', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', 10)
        self.pub_debug = self.create_publisher(Image, '~/debug_image', 10)
        self.pub_det_cloud = self.create_publisher(
            PointCloud2, '~/detection_cloud', 10)

        # ---- 规范化输出话题（/peach/perception/*）----
        # 与上面 ~/ 话题并行发布**同一消息对象**，供下游按固定命名订阅；
        # 旧 ~/ 话题全部保留，行为不变
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
        self.pub_target_observations = self.create_publisher(
            PeachTargetObservationArray,
            '/peach/perception/target_observations', 10)
        state_qos = rclpy.qos.QoSProfile(
            depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_harvest_state = self.create_publisher(
            String, '/peach/perception/harvest_state', state_qos)
        self.create_service(
            Trigger, '~/reset_global_targets', self._on_reset_global_targets)
        self.create_service(
            Trigger, '~/query_harvest_state', self._on_query_harvest_state)
        self.create_service(
            Trigger, '~/complete_selected_target',
            self._on_complete_selected_target)

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

    def _harvest_state_dict(self) -> dict:
        """返回可序列化的全局采摘计划与数据路径."""
        return {
            'harvest_run_id': self.harvest_run_id,
            'snapshot_id': self.harvest_plan.snapshot_id,
            'target_set_locked': self.harvest_plan.locked,
            'target_count': self.harvest_plan.target_count,
            'target_ids': list(self.harvest_plan.locked_ids),
            'completed_target_ids': sorted(self.harvest_plan.completed_ids),
            'priorities': dict(self.harvest_plan.priorities),
            'selected_target_id': self.harvest_plan.selected_target_id,
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

    def _on_reset_global_targets(self, request, response):
        """~/reset_global_targets：结束本轮，允许下一次全局拍照重锁目标."""
        del request
        old_run = self.harvest_run_id
        if old_run:
            self.harvest_data.append_event({
                'source': 'perception', 'event': 'run_reset'})
        self.harvest_plan.reset()
        self.harvest_data = HarvestDataStore(root=self.harvest_data.root)
        self.harvest_run_id = ''
        self._publish_harvest_state()
        response.success = True
        response.message = f'已重置全局目标集合，上一轮={old_run or "无"}'
        return response

    def _on_complete_selected_target(self, request, response):
        """~/complete_selected_target：确认抓取成功并推进固定优先级计划."""
        del request
        completed = self.harvest_plan.selected_target_id
        if not completed:
            response.success = False
            response.message = '当前没有可完成的 selected_target_id'
            return response
        next_target = self.harvest_plan.complete_selected()
        self.harvest_data.append_event({
            'source': 'perception', 'event': 'target_harvested',
            'target_id': completed, 'next_target_id': next_target,
        })
        self._publish_harvest_state()
        response.success = True
        response.message = (
            f'已完成 {completed}，下一目标={next_target or "本轮全部完成"}')
        return response

    def _start_harvest_run(self) -> None:
        """为刚锁定的全局目标集合创建不可变 manifest."""
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
        """发布锁定 ID 的逐目标结果，并记录选中目标掩膜与状态事件."""
        was_locked = self.harvest_plan.locked
        current = self.harvest_plan.update(records)
        try:
            if self.harvest_plan.locked and not was_locked:
                self._start_harvest_run()
        except OSError as exc:
            self.get_logger().error(f'采摘运行目录创建失败: {exc}')

        array = PeachTargetObservationArray()
        array.header = header
        array.snapshot_id = self.harvest_plan.snapshot_id
        array.harvest_run_id = self.harvest_run_id
        array.target_set_locked = self.harvest_plan.locked
        array.target_count = self.harvest_plan.target_count
        array.selected_target_id = self.harvest_plan.selected_target_id
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
            if payload is None:
                item.tracking_status = PeachTargetObservation.LOST
                item.diagnostic_flags = ['target_temporarily_lost']
            else:
                item.tracking_status = PeachTargetObservation.OBSERVED
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
                    item.tracking_status = PeachTargetObservation.OCCLUDED
                    item.diagnostic_flags.append('mask_unavailable')
                else:
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
            return _transform_msg_to_matrix(tf.transform), 'ok'
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return _transform_msg_to_matrix(tf.transform), 'stale'
            except TransformException as ex:
                self.get_logger().warning(
                    f'TF 失败，输出退回相机系 {cam_frame}: {ex}')
                return None, 'unavailable'

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
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
        self.get_logger().info(
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
            self.get_logger().warn(f'RGB convert failed: {exc}')
            return
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Depth convert failed: {exc}')
            return
        # uint16：raw × depth_scale_unit = 毫米；32FC1：米 ×1000 = 毫米
        try:
            depth = normalize_depth_to_uint16_mm(depth_raw, self.depth_scale_unit)
        except ValueError as exc:
            self.get_logger().warn(f'Depth convert failed: {exc}')
            return
        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warn(
                f'RGB/depth size mismatch {rgb.shape[:2]} vs {depth.shape[:2]}')
            return
        if info.width and info.height and (
                int(info.width) != depth.shape[1] or int(info.height) != depth.shape[0]):
            self.get_logger().warn(
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
                gravity_hint = _gravity_camera_from_R(T_out_cam[:3, :3])
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
        dets = self.engine.detect(rgb)
        # 置信度过滤（第二级，严出）→ IoS 去重（消一果两框/局部误检小框，
        # 跨类生效，防同一物理目标在身份表重复占号）；发布的 detections
        # 即实际入管线的目标
        kept = [d for d in dets
                if float(d.get('conf', 0.0)) >= self.min_detection_conf]
        kept = dedup_overlapping_detections(kept, self.detection_dedup_ios)
        det_msg = Detection2DArray()
        det_msg.header = img_header
        for d in kept:
            det_msg.detections.append(_to_detection2d(d, img_header))
        self.pub_dets.publish(det_msg)
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
            self.pub_det_cloud.publish(cloud_msg)
            self.pub_norm_cloud.publish(cloud_msg)

        mask_canvas = np.zeros(depth.shape[:2], dtype=np.uint8)
        debug = rgb.copy() if self.publish_debug_image else None
        cand_arr = BagGraspCandidateArray()
        cand_arr.header = header
        cand2d_arr = BagGrasp2DArray()
        cand2d_arr.header = header
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
            self.target_registry.begin_frame()
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = None
            segs = self.engine.segment(rgb, [bbox])
            if segs:
                sam_mask = segs[0][0]
                mask_canvas[sam_mask > 0] = np.uint8((i % 250) + 1)

            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=gravity_hint,
                detections=[det],
                metadata={
                    'model_version': self.model_version,
                    'calibration_version': self.calibration_version,
                },
            )
            tid = f'target_{i}'
            results = self.estimator.estimate_modes(obs, tid, bbox, sam_mask)
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
            # ---- 目标身份记忆：世界系匹配/注册，帧内序号 → 稳定 ID ----
            # tid 在组消息前重赋值，candidate/2d/fitting/marker 文字随之用稳定 ID；
            # 三态仍逐帧独立计算，记忆只影响身份与位置平滑
            if self.target_registry is not None:
                if track_this_frame:
                    # 空间锚点：前景点云中位质心（base 系，最抗抖）优先，
                    # 回退袋底/position/entry_start；全 None（几何失败帧）
                    # 无锚点可匹配，保留帧内序号
                    anchor = result.grasp_3d.points_centroid
                    if anchor is None:
                        anchor = result.grasp_3d.bag_bottom
                    if anchor is None:
                        anchor = result.grasp_3d.position
                    if anchor is None:
                        anchor = result.grasp_3d.entry_start
                    if anchor is not None:
                        tid, is_new = self.target_registry.match_or_register(
                            anchor, class_id=int(det.get('class_id', 0)),
                            axis=result.grasp_3d.translation_direction,
                            diameter=float(
                                result.grasp_3d.bag_diameter_upper_m or 0.0),
                            status=result.grasp_3d.status)
                        result.grasp_3d.diagnostic_flags.append(
                            'target_new' if is_new else 'target_matched')
                else:
                    # tf_unavailable：保留帧内序号，打标提示下游 ID 不可跨帧追踪
                    result.grasp_3d.diagnostic_flags.append('target_untracked')
            g3d, g2d = result.grasp_3d, result.grasp_2d
            candidate_msg = _to_candidate(
                header, tid, g3d, model_version=self.model_version,
                calibration_version=self.calibration_version,
                tool_version=self.tool.version)
            candidate_2d_msg = _to_candidate_2d(header, tid, g2d)
            fitting_msg = _to_fitting(header, tid, result)
            cand_arr.candidates.append(candidate_msg)
            cand2d_arr.candidates.append(candidate_2d_msg)
            fit_arr.fittings.append(fitting_msg)
            registry_item = (
                None if self.target_registry is None
                else self.target_registry.get(tid))
            confirmed = (
                True if self.target_registry is None
                else bool(registry_item and registry_item['confirmed']))
            harvest_records.append({
                'target_id': tid, 'status': int(candidate_msg.status),
                'confidence': float(candidate_msg.confidence),
                'camera_distance_m': camera_distance_m,
                'confirmed': confirmed,
                'diagnostic_flags': list(candidate_msg.diagnostic_flags),
            })
            harvest_payloads[tid] = {
                'candidate': candidate_msg,
                'candidate_2d': candidate_2d_msg,
                'fitting': fitting_msg,
                'mask': sam_mask,
            }
            markers.markers.extend(_to_markers(
                header, tid, i, result,
                tool_d_inner=float(self.tool.D_inner)))
            if debug is not None:
                _draw_debug(debug, det, g2d, sam_mask, tid)

        self._publish_target_observations(
            header, mask_header, harvest_records, harvest_payloads)
        self.pub_cands.publish(cand_arr)
        self.pub_cands_2d.publish(cand2d_arr)
        self.pub_fitting.publish(fit_arr)
        self.pub_markers.publish(markers)
        # 规范化话题并行发布同一批消息对象（旧 ~/ 话题全保留）
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
        self.get_logger().info(
            f'Published {len(cand_arr.candidates)} candidates '
            f'(dets={len(kept)})')
        if self.target_registry is not None:
            st = self.target_registry.stats()
            self.get_logger().info(
                f'目标注册表在册 {st["n_targets"]} 个目标'
                f'（累计注册 {st["n_registered"]}、累计命中 {st["n_matched"]}）',
                throttle_duration_sec=10.0)
        if self.publish_masks:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_canvas, encoding='mono8')
            mask_msg.header = img_header
            self.pub_masks.publish(mask_msg)
            self.pub_norm_masks.publish(mask_msg)
        if debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = img_header
            self.pub_debug.publish(dbg_msg)


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
