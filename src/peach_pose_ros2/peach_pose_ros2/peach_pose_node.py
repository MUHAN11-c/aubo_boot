"""
PeachPose ROS 2 感知节点（编排层）.

订阅时间对齐的 RGB-D + CameraInfo，经 YOLO → MobileSAM → 实测深度几何管线，
发布抓取参考候选 / 2D / 拟合诊断 / 检测 / 掩膜 / Marker / debug 图 / 检测框点云。

五层结构：params.py（参数声明/加载/热调）、conversions.py（TF 与消息转换）、
visualization.py（Marker/调试图/点云）、peach_pose/（算法包，base.py 为后端
Protocol 抽象接口）；本文件只保留编排：构造、TF 查询与逐帧回调。

只发参考位姿，不发送运动指令。几何默认可经 TF 变到 ``output_frame``
（默认 ``base_link``，依赖 ``hand_eye_extrinsics_publisher``）。
图像编解码统一走 cv_bridge（bgr8 / passthrough uint16 / mono8）。
"""
from pathlib import Path
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import message_filters
import numpy as np
from peach_pose_msgs.msg import (
    BagFittingArray,
    BagGrasp2DArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
    FusionStats,
)
from peach_pose_ros2.conversions import (
    _apply_T_to_grasp3d,
    _resolve_model_path,
    _to_candidate,
    _to_candidate_2d,
    _to_detection2d,
    _to_fitting,
    _transform_msg_to_matrix,
)
from peach_pose_ros2.params import (
    declare_params,
    load_params,
    make_on_set_callback,
)
from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.inference import InferenceEngine
from peach_pose_ros2.peach_pose.pipeline import RobustBagPosePipeline
from peach_pose_ros2.peach_pose.temporal_fuser import TemporalFuser
from peach_pose_ros2.visualization import (
    _accumulate_heatmap,
    _bbox_cloud_xyzrgb,
    _draw_debug,
    _heatmap_to_mono8,
    _to_markers,
    _xyzrgb_to_cloud,
)
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


class PeachPoseNode(Node):
    """RGB-D 同步回调驱动的感知节点：检测 → 分割 → 几何 → TF 变换 → 多话题发布."""

    def __init__(self):
        """建节点：参数声明/加载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
        super().__init__('peach_pose_node')
        self.bridge = CvBridge()
        declare_params(self)
        load_params(self)
        # 运行时参数调整入口（rqt_reconfigure / ros2 param set 经此回调生效）
        self.add_on_set_parameters_callback(make_on_set_callback(self))

        share = Path(get_package_share_directory('peach_pose_ros2'))
        # 模型路径：参数可给文件或目录（目录下找同名权重），空/无效回退包内 model/
        yolo = _resolve_model_path(
            self.yolo_model_path, 'best.pt', share, self.get_logger())
        sam = _resolve_model_path(
            self.sam_model_path, 'mobile_sam.pt', share, self.get_logger())
        self.get_logger().info(f'YOLO={yolo}')
        self.get_logger().info(f'SAM={sam}')

        self.engine = InferenceEngine(
            yolo_model=yolo, sam_model=sam, yolo_conf=self.yolo_conf,
            yolo_iou=self.yolo_iou, sam_max_bboxes=self.sam_max_bboxes,
            sam_min_area=self.sam_min_area)
        self.estimator = CandidateEstimator(
            pipeline=RobustBagPosePipeline(
                tool=self.tool, min_depth_m=self.min_depth_m,
                max_depth_m=self.max_depth_m, min_points=self.min_points))

        # ── 融合决策段（最终方向/抓取定位从这出；算法在算法包内）──
        # TemporalFuser 门控/EMA/DBSCAN 参数用类默认（与原 recon 侧一致）；
        # 重拟合 refiner 懒 import（open3d 在 venv，首次用时才加载）
        self.fuser = TemporalFuser()
        self._roi_cloud_points = None           # 最新 ROI 点云 (N,3)（recon 输出）
        self._last_recon_pub = 0.0
        if self.refined_outputs_enabled:
            self.pub_refined = self.create_publisher(
                BagGraspCandidateArray, '~/refined_candidates', 10)
            self.pub_fusion_stats = self.create_publisher(
                FusionStats, '~/fusion_stats', 10)
            self.pub_evolution = self.create_publisher(
                PoseStamped, '~/refinement_evolution', 10)
            self.pub_recon_refined = self.create_publisher(
                BagGraspCandidateArray, '~/recon_refined', 10)
            self.create_subscription(
                PointCloud2, self.recon_cloud_topic, self._on_roi_cloud, 10)
            self.get_logger().info(
                f'融合决策段已启用：订阅 {self.recon_cloud_topic}，'
                f'recon_min_points={self.recon_min_points} '
                f'kind={self.target_kind}')

        # ---- 输出话题（相对命名空间 ~/，实际为 /peach_pose_node/<名>；队列深 10）----
        # 主输出：3D 抓取参考候选（entry_pose 位姿 + 套入方向 + 建议行程 + 三态
        # status），下游抓取决策唯一正式输入
        self.pub_cands = self.create_publisher(
            BagGraspCandidateArray, '~/grasp_candidates', 10)
        # 2D 关键点：检测框/袋底/袋颈/抓取点/行程终点的像素坐标（配 has_* 标志），
        # 用于 UI 叠加层，与 3D 按 target_id 对齐
        self.pub_cands_2d = self.create_publisher(
            BagGrasp2DArray, '~/grasp_candidates_2d', 10)
        # 拟合诊断：轴来源/置信度/误差预算/净空/点数/拟合统计量等 21 项标量
        # （仅供调参验收，不参与运动；无效标量填 -1）
        self.pub_fitting = self.create_publisher(
            BagFittingArray, '~/fitting', 10)
        # YOLO 原始检测框（过 0.5 置信度后）：2D 框 + 类别 + 分数，未进几何管线
        self.pub_dets = self.create_publisher(
            Detection2DArray, '~/detections', 10)
        # SAM 概率热图（mono8）：直接显示 SAM 原始概率输出，亮=把握大
        # （逐像素取各目标最大值；二值掩膜 >0.5 仅用于算法）
        self.pub_masks = self.create_publisher(Image, '~/masks', 10)
        # RViz 可视化：袋轴线/行程箭头/刀具圆柱/果球/三轴架/状态文字
        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', 10)
        # debug 叠加图（bgr8）：检测框 + 掩膜淡填充与高对比轮廓描边 + 底→颈箭头 + 状态文字
        self.pub_debug = self.create_publisher(Image, '~/debug_image', 10)
        # 检测框内深度反投影彩色点云（xyz+rgb，已变到输出系）——调试用，
        # 可在 RViz 直观看"检测框圈住了哪块空间"
        self.pub_det_cloud = self.create_publisher(
            PointCloud2, '~/detection_cloud', 10)

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

        # 三路近似时间同步：时间戳差 ≤ slop (s) 的消息配对成一组触发回调
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
            f'calib={self.calibration_version}')

    def _lookup_T_out_cam(self, cam_frame: str, stamp) -> Optional[np.ndarray]:
        """
        查 output←camera 的 4×4 齐次矩阵；失败返回 None.

        Args:
            cam_frame: 相机光学系 frame_id.
            stamp: 查询时刻（消息时间戳）；按时刻失败时回退最新 TF 并告警一次.

        Returns
        -------
            (4, 4) ndarray；output_frame 为空或与 cam_frame 相同给单位阵；
            TF 彻底失败给 None（调用方退回相机系）.

        """
        if not self.output_frame or self.output_frame == cam_frame:
            return np.eye(4)
        stamp_time = Time.from_msg(stamp)
        try:
            # 查 目标系←源系 的 TF：按帧时间戳取值，timeout 内等变换可用
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, cam_frame, stamp_time, timeout=self.tf_timeout)
            return _transform_msg_to_matrix(tf.transform, self.get_logger())
        except TransformException:
            try:
                # 空时间戳 Time() = 取最新可用变换（按时间戳失败时的回退）
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return _transform_msg_to_matrix(tf.transform, self.get_logger())
            except TransformException as ex:
                self.get_logger().warning(
                    f'TF 失败，输出退回相机系 {cam_frame}: {ex}',
                    throttle_duration_sec=2.0)
                return None

    def _on_roi_cloud(self, msg: PointCloud2):
        """
        ROI 点云回调：缓存 xyz（降频重拟合用）.

        Args:
            msg: sensor_msgs/PointCloud2（输出系；peach_recon 纯重建输出）.

        Returns
        -------
            无返回值（None）.

        """
        raw = np.asarray(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if raw.size and raw.dtype.fields:
            # 结构化数组（字段 x/y/z）：按字段取列再转 float64
            pts = np.column_stack(
                [raw['x'], raw['y'], raw['z']]).astype(np.float64)
        elif raw.size:
            pts = raw.astype(np.float64).reshape(-1, 3)   # 普通 (N,3) 数组
        else:
            pts = np.zeros((0, 3))
        self._roi_cloud_points = pts

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """
        同步回调 (ApproximateTimeSynchronizer)：一帧 RGB-D → 全套感知输出.

        Args:
            rgb_msg: 彩色图（bgr8）.
            depth_msg: 深度图（uint16 原始值；回调内 × depth_scale_unit 化为毫米）.
            info: 彩色相机内参（须与深度图同分辨率）.

        Returns
        -------
            无返回值（None）；感知结果经各发布者发出.

        """
        # 首帧到达打一次（确认数据链路活了），此后不逐帧刷屏
        self.get_logger().info(
            f'首帧到达 {rgb_msg.width}x{rgb_msg.height}', once=True)
        # ---- ① 转换与校验：任一失败丢整帧，不产出任何结果 ----
        try:
            # cv_bridge：ROS Image → ndarray；'bgr8' 强制转 OpenCV BGR 三通道
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'RGB convert failed: {exc}',
                                   throttle_duration_sec=2.0)
            return
        try:
            # 'passthrough' 不做编码转换，保留 uint16 原始深度
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Depth convert failed: {exc}',
                                   throttle_duration_sec=2.0)
            return
        if depth.dtype != np.uint16:
            self.get_logger().warn(
                f'Depth dtype {depth.dtype} expected uint16',
                throttle_duration_sec=2.0)
            return
        # Percipio: z_m = raw * depth_scale_unit / 1000；管线内部按「毫米」再 /1000
        # 钳到 uint16 值域 [0, 65535]，防缩放后溢出回绕
        if abs(self.depth_scale_unit - 1.0) > 1e-9:
            depth = np.clip(
                np.round(depth.astype(np.float32) * self.depth_scale_unit),
                0, 65535).astype(np.uint16)
        if rgb.shape[:2] != depth.shape[:2]:
            self.get_logger().warn(
                f'RGB/depth size mismatch {rgb.shape[:2]} vs {depth.shape[:2]}',
                throttle_duration_sec=2.0)
            return
        if info.width and info.height and (
                int(info.width) != depth.shape[1] or int(info.height) != depth.shape[0]):
            self.get_logger().warn(
                f'CameraInfo size {info.width}x{info.height} != depth '
                f'{depth.shape[1]}x{depth.shape[0]}',
                throttle_duration_sec=2.0)
            return

        # ---- ② 内参 + 坐标系 + TF ----
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
        # 相机系回退链：参数 camera_optical_frame → 深度图头 → 彩图头 → 内参头
        # 几何先在相机光心系求解，发布前统一变到 out_frame（默认 base_link）
        cam_frame = (
            self.camera_optical_frame
            or depth_msg.header.frame_id
            or rgb_msg.header.frame_id
            or info.header.frame_id)
        out_frame = self.output_frame or cam_frame
        T_out_cam = self._lookup_T_out_cam(cam_frame, rgb_msg.header.stamp)
        if T_out_cam is None and self.output_frame:
            # TF 失败则退回相机系，避免静默用错坐标系
            out_frame = cam_frame
            T_out_cam = np.eye(4)

        header = Header()
        header.stamp = rgb_msg.header.stamp
        header.frame_id = out_frame

        # ---- YOLO 检测：conf≥yolo_conf(0.3) 推理，≥min_detection_conf(0.5) 才入管线 ----
        dets = self.engine.detect(rgb)
        det_msg = Detection2DArray()
        det_msg.header = header
        kept = []                                # 过第二级置信度的检测框（入管线）
        for d in dets:
            if float(d.get('conf', 0.0)) < self.min_detection_conf:
                continue
            kept.append(d)
            det_msg.detections.append(_to_detection2d(d, header))
        self.pub_dets.publish(det_msg)           # ~/detections：含空数组

        # ~/detection_cloud：kept 框内深度像素反投影成彩色点云（纯可视化，不入管线）
        if self.publish_detection_cloud:
            bboxes = [d['bbox'] for d in kept]
            xyz_cam, rgb_f = _bbox_cloud_xyzrgb(
                rgb, depth, K, bboxes, stride=self.detection_cloud_stride)
            if xyz_cam.shape[0] and T_out_cam is not None:
                R, t = T_out_cam[:3, :3], T_out_cam[:3, 3]
                xyz_out = (R @ xyz_cam.T).T + t  # 点云相机系 → 输出系（R@p+t）
            else:
                xyz_out = xyz_cam
            self.pub_det_cloud.publish(
                _xyzrgb_to_cloud(header, xyz_out, rgb_f))

        mask_canvas = np.zeros(depth.shape[:2], dtype=np.float32)  # SAM 概率热图画布
        debug = rgb.copy() if self.publish_debug_image else None   # debug 叠加图底稿
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

        # ---- 逐目标：SAM 精割 → 前景∩深度 → 袋/果几何管线 → 变到输出系 → 分装消息 ----
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = None
            segs = self.engine.segment(rgb, [bbox])
            if segs:
                sam_mask = segs[0][0]            # (H,W) 布尔掩膜（概率 >0.5）
                # 热图画布：逐像素取各目标概率最大值（直接显示 SAM 原始概率输出，
                # 亮=把握大；二值化 >0.5 仍只用于算法，不影响此处显示）
                probs = getattr(self.engine, 'last_probs', None)
                mask_canvas = _accumulate_heatmap(mask_canvas, probs)

            tid = f'target_{i}'
            # 打包本目标的全部输入（图像/深度/内参/重力提示/检测/版本追溯）
            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=self.gravity_hint,
                detections=[det],
                metadata={
                    'model_version': self.model_version,
                    'calibration_version': self.calibration_version,
                },
            )
            # 单模式直通：hybrid_dilated 前景 → 袋/果管线 → 三态结果
            result = self.estimator.estimate(obs, tid, bbox, sam_mask)
            if T_out_cam is not None and out_frame != cam_frame:
                _apply_T_to_grasp3d(result.grasp_3d, T_out_cam)  # 相机系→输出系
            g3d, g2d = result.grasp_3d, result.grasp_2d
            cand_arr.candidates.append(          # 3D 抓取参考（主输出）
                _to_candidate(header, tid, g3d, self.model_version,
                              self.calibration_version, self.tool.version,
                              self.get_logger()))
            cand2d_arr.candidates.append(        # 2D 像素关键点
                _to_candidate_2d(header, tid, g2d))
            fit_arr.fittings.append(             # 拟合诊断指标
                _to_fitting(header, tid, result))
            markers.markers.extend(              # RViz 图形（轴/行程/刀具/状态）
                _to_markers(header, tid, i, result, self.tool,
                            self.get_logger()))
            if debug is not None:
                _draw_debug(debug, det, g2d, sam_mask)

        # ---- 批量发布：四类主消息每帧无条件发（空数组也发，区分"没检出"与"节点死"）----
        self.pub_cands.publish(cand_arr)
        self.pub_cands_2d.publish(cand2d_arr)
        self.pub_fitting.publish(fit_arr)
        self.pub_markers.publish(markers)
        # 候选计数心跳：每 2s 一条，确认节点在产出（不逐帧刷屏）
        self.get_logger().info(
            f'Published {len(cand_arr.candidates)} candidates '
            f'(dets={len(kept)})', throttle_duration_sec=2.0)
        if self.publish_masks:
            # 概率热图 [0,1] → uint8 [0,255]（亮=SAM 把握大；暗≈背景/不确定）
            heat = _heatmap_to_mono8(mask_canvas)
            mask_msg = self.bridge.cv2_to_imgmsg(heat, encoding='mono8')
            mask_msg.header = header
            self.pub_masks.publish(mask_msg)
        if debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = header
            self.pub_debug.publish(dbg_msg)

        # ── 融合决策段：最优候选灌时序融合 + 降频 ROI 云重拟合 ──
        if self.refined_outputs_enabled:
            self._fusion_decision(header, cand_arr)

    def _fusion_decision(self, header, cand_arr):
        """
        每帧融合决策：最优候选灌 TemporalFuser 并四路发布.

        发布 ~/refined_candidates（融合值替换最优候选 + '|fused' 标记 +
        support_frames=N flag）、~/fusion_stats、~/refinement_evolution，
        并按 recon_refined_period_sec 降频尝试 ROI 云重拟合。
        """
        if not cand_arr.candidates:
            return
        # 最优候选：ACCEPT 优先，其余按 confidence 降序
        best = max(cand_arr.candidates,
                   key=lambda c: (int(c.status) == 0, float(c.confidence)))
        p = best.entry_pose.position
        d = best.translation_direction
        self.fuser.update((p.x, p.y, p.z), (d.x, d.y, d.z),
                          diameter=float(best.bag_diameter_upper_m),
                          confidence=float(best.confidence))
        est = self.fuser.current()
        if est is None:
            return                      # 首帧轴零向量等极端情形：本帧无融合值
        self._publish_refined(header, cand_arr, best, est)
        self._publish_fusion_stats(header, best.target_id, est)
        self._publish_evolution(header, best, est)
        # 完整重建云在感知侧按融合位置自裁 ROI 后精化（与 recon 完全解耦）
        self._maybe_publish_recon_refined(header, est['position'])

    def _publish_refined(self, header, cand_arr, best, est):
        """发布 ~/refined_candidates：最优候选替换为融合值并打 fused 标记."""
        out = BagGraspCandidateArray()
        out.header = header
        for c in cand_arr.candidates:
            if c is not best:
                out.candidates.append(c)
                continue
            fused = BagGraspCandidate()
            fused.header = c.header
            fused.target_id = c.target_id
            fused.entry_pose = c.entry_pose
            fused.entry_pose.position.x = float(est['position'][0])
            fused.entry_pose.position.y = float(est['position'][1])
            fused.entry_pose.position.z = float(est['position'][2])
            fused.entry_pose.orientation = c.entry_pose.orientation
            fused.bag_bottom = c.bag_bottom
            fused.bag_neck = c.bag_neck
            fused.translation_direction.x = float(est['axis'][0])
            fused.translation_direction.y = float(est['axis'][1])
            fused.translation_direction.z = float(est['axis'][2])
            fused.bag_diameter_upper_m = float(est['diameter'])
            fused.suggested_travel_m = c.suggested_travel_m
            fused.confidence = float(est['confidence'])
            fused.status = c.status
            fused.diagnostic_flags = list(c.diagnostic_flags) + [
                f"support_frames={est['support_frames']}"]
            fused.strategy_id = (c.strategy_id or '') + '|fused'
            fused.model_version = c.model_version
            fused.calibration_version = c.calibration_version
            fused.tool_version = c.tool_version
            out.candidates.append(fused)
        self.pub_refined.publish(out)

    def _publish_fusion_stats(self, header, target_id, est):
        """发布 ~/fusion_stats：逐帧融合统计（收敛性指标）."""
        msg = FusionStats()
        msg.header = header
        msg.target_id = target_id
        msg.frame_seq = int(est['frame_seq'])
        msg.accepted = int(est['accepted'])
        msg.rejected = int(est['rejected'])
        msg.cluster_size = int(est['support_frames'])
        msg.pos_std_m = float(est['pos_std_m'])
        msg.axis_std_deg = float(est['axis_std_deg'])
        self.pub_fusion_stats.publish(msg)

    def _publish_evolution(self, header, best, est):
        """发布 ~/refinement_evolution：本次融合后的 refined 位姿快照."""
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(est['position'][0])
        pose.pose.position.y = float(est['position'][1])
        pose.pose.position.z = float(est['position'][2])
        pose.pose.orientation = best.entry_pose.orientation
        self.pub_evolution.publish(pose)

    def _maybe_publish_recon_refined(self, header, center):
        """
        完整重建云按融合位置自裁 ROI → 重拟合 → ~/recon_refined（降频发布）.

        recon 发完整 TSDF 云，本处按 roi_size_m 盒裁剪（中心=融合位置）；
        裁剪后点数达 recon_min_points 才拟合（refine_target，open3d 懒 import）；
        输出单候选：中心=拟合中心、方向=拟合轴（bag）、袋径=2×拟合半径、
        置信度=内点率；diagnostic_flags 带 n_points/rms（评估用）。

        Args:
            header: 输出头.
            center: ROI 中心（融合位置，输出系，米）.

        Returns
        -------
            无返回值（None）；满足条件时发布 ~/recon_refined.

        """
        now = time.monotonic()
        if now - self._last_recon_pub < self.recon_refined_period_sec:
            return
        pts = self._roi_cloud_points
        if pts is None or pts.shape[0] == 0:
            return
        half = self.roi_size_m / 2.0
        inside = np.all(np.abs(pts - np.asarray(center, dtype=float)) <= half,
                        axis=1)
        pts = pts[inside]
        if pts.shape[0] < self.recon_min_points:
            return
        from peach_pose_ros2.peach_pose.refiner import refine_target
        fit = refine_target(pts, self.target_kind)
        if fit is None:
            return
        self._last_recon_pub = now
        cand = BagGraspCandidate()
        cand.header = header
        center = np.asarray(fit['center'], dtype=float)
        cand.entry_pose.position.x = float(center[0])
        cand.entry_pose.position.y = float(center[1])
        cand.entry_pose.position.z = float(center[2])
        axis = fit.get('axis')
        if axis is not None:
            n = float(np.linalg.norm(axis))
            if n > 1e-9:
                cand.translation_direction.x = float(axis[0] / n)
                cand.translation_direction.y = float(axis[1] / n)
                cand.translation_direction.z = float(axis[2] / n)
        cand.bag_diameter_upper_m = float(2.0 * fit['radius'])
        cand.confidence = float(fit['inlier_ratio'])
        cand.strategy_id = f'recon_refined:{self.target_kind}'
        cand.diagnostic_flags = [
            f'n_points={pts.shape[0]}',
            f'rms={fit["rms"] * 1000.0:.1f}mm',
        ]
        out = BagGraspCandidateArray()
        out.header = header
        out.candidates.append(cand)
        self.pub_recon_refined.publish(out)


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
