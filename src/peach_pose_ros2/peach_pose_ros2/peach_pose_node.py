"""
PeachPose ROS 2 感知节点.

订阅时间对齐的 RGB-D + CameraInfo，经 YOLO → MobileSAM → 实测深度几何管线，
发布抓取参考候选 / 2D / 拟合诊断 / 检测 / 掩膜 / Marker / debug 图 / 检测框点云。

只发参考位姿，不发送运动指令。几何默认可经 TF 变到 ``output_frame``
（默认 ``base_link``，依赖 ``hand_eye_extrinsics_publisher``）。
图像编解码统一走 cv_bridge（bgr8 / passthrough uint16 / mono8）。
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import message_filters
import numpy as np
from peach_pose_msgs.msg import (
    BagFitting,
    BagFittingArray,
    BagGrasp2D as BagGrasp2DMsg,
    BagGrasp2DArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
)
from peach_pose_ros2.peach_pose.candidates import CandidateEstimator
from peach_pose_ros2.peach_pose.contracts import BagObservation, ToolGeometry
from peach_pose_ros2.peach_pose.inference import InferenceEngine
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


def _transform_msg_to_matrix(t) -> np.ndarray:
    """geometry_msgs/Transform → 4×4 齐次矩阵（p_out = R @ p_in + t）."""
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[0, 3] = t.translation.x
    T[1, 3] = t.translation.y
    T[2, 3] = t.translation.z
    return T


def _apply_T_to_grasp3d(g3d, T: np.ndarray) -> None:
    """
    把相机系抓取几何变到输出系（原地修改 g3d）.

    点做刚体变换；方向向量只乘 R 后归一化；姿态矩阵左乘 R。
    """
    R, t = T[:3, :3], T[:3, 3]

    def _pt(p):
        if p is None:
            return None
        return R @ np.asarray(p, dtype=float) + t

    g3d.entry_start = _pt(g3d.entry_start)
    g3d.bag_bottom = _pt(g3d.bag_bottom)
    g3d.bag_neck = _pt(g3d.bag_neck)
    if g3d.translation_direction is not None:
        d = R @ np.asarray(g3d.translation_direction, dtype=float)
        n = float(np.linalg.norm(d))
        g3d.translation_direction = d / n if n > 1e-9 else d
    if g3d.orientation is not None:
        g3d.orientation = R @ np.asarray(g3d.orientation, dtype=float)


# 与 peach_pose_msgs/BagGraspCandidate.status 枚举一致
STATUS_MAP = {'ACCEPT': 0, 'REOBSERVE': 1, 'REJECT': 2}


def _rotation_to_quat(R: np.ndarray) -> Quaternion:
    """3x3 旋转矩阵 → geometry_msgs/Quaternion（右手系）."""
    # Shepperd 稳健法
    m = np.asarray(R, dtype=float)
    t = float(np.trace(m))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw, qx = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        qy, qz = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw, qx = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        qy, qz = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw, qx = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        qy, qz = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw, qx = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        qy, qz = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = Quaternion()
    q.w, q.x, q.y, q.z = float(qw), float(qx), float(qy), float(qz)
    return q


def _point(xyz) -> Point:
    """三维点 → geometry_msgs/Point；None 则零向量。强制 float 避免 rosidl 断言."""
    p = Point()
    if xyz is None:
        return p
    p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return p


def _px(uv, z=0.0) -> Point:
    """像素 (u,v) 塞进 Point.x/y（2D 消息复用 Point）."""
    p = Point()
    if uv is None:
        return p
    p.x, p.y, p.z = float(uv[0]), float(uv[1]), float(z)
    return p


def _metric(m: dict, key: str, default: float = -1.0) -> float:
    """从 metrics 字典取标量；缺失或不可转 float 时用 default（消息侧常用 -1 表示无）."""
    v = m.get(key, None)
    if v is None:
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _status_color(status: str) -> Tuple[float, float, float, float]:
    """三态 → RViz Marker RGBA（绿/黄/红）."""
    return {
        'ACCEPT': (0.1, 0.85, 0.2, 0.9),
        'REOBSERVE': (0.95, 0.8, 0.1, 0.9),
        'REJECT': (0.9, 0.15, 0.15, 0.9),
    }.get(status, (0.6, 0.6, 0.6, 0.8))


def _pack_rgb_bgr(bgr: np.ndarray) -> np.ndarray:
    """(N,3) uint8 BGR → float32 位打包 rgb（PointCloud2 常用）."""
    b = bgr[:, 0].astype(np.uint32)
    g = bgr[:, 1].astype(np.uint32)
    r = bgr[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def _bbox_cloud_xyzrgb(
    rgb_bgr: np.ndarray,
    depth_mm: np.ndarray,
    K: dict,
    bboxes,
    stride: int = 1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    检测框内深度反投影 → (N,3) xyz 米、(N,) float32 打包 rgb.

    depth_mm 已是管线「毫米」uint16（Percipio 已 × depth_scale_unit）。
    """
    h, w = depth_mm.shape[:2]
    mask = np.zeros((h, w), dtype=bool)
    for bbox in bboxes:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        mask[y1:y2:stride, x1:x2:stride] = True
    # 有效深度：>0 且非饱和
    valid = mask & (depth_mm > 0) & (depth_mm < 65535)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=np.float32)

    vs, us = np.where(valid)
    z = depth_mm[vs, us].astype(np.float64) / 1000.0
    fx, fy = float(K['fx']), float(K['fy'])
    cx, cy = float(K['cx']), float(K['cy'])
    x = (us.astype(np.float64) - cx) * z / fx
    y = (vs.astype(np.float64) - cy) * z / fy
    xyz = np.column_stack((x, y, z))
    rgb_packed = _pack_rgb_bgr(rgb_bgr[vs, us])
    return xyz, rgb_packed


def _xyzrgb_to_cloud(header: Header, xyz: np.ndarray, rgb_f: np.ndarray) -> PointCloud2:
    """组装 xyz+rgb 点云消息（rgb 为 float32 位打包）."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    if xyz.size == 0:
        return pc2.create_cloud(header, fields, [])
    pts = [
        (float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2]), float(rgb_f[i]))
        for i in range(len(xyz))
    ]
    return pc2.create_cloud(header, fields, pts)


class PeachPoseNode(Node):
    """RGB-D 同步回调驱动的感知节点：检测 → 分割 → 几何 → TF 变换 → 多话题发布."""

    def __init__(self):
        super().__init__('peach_pose_node')
        self.bridge = CvBridge()
        self._declare_params()
        self._load_params()

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
            f'calib={self.calibration_version}')

    def _declare_params(self):
        """声明 ROS 参数默认值（与 config/peach_pose.yaml 对齐，可被 yaml 覆盖）."""
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            # 手眼链挂在 color optical；深度 registration 后几何也在此系
            'camera_optical_frame': 'camera_color_optical_frame',
            'output_frame': 'base_link',
            'tf_timeout_sec': 0.5,
            # Percipio 原始深度常需 ×0.25 才是毫米量级（再 /1000→米）
            'depth_scale_unit': 0.25,
            'sync_slop_s': 0.05,
            'min_detection_conf': 0.5,
            'yolo_conf': 0.3,
            'publish_debug_image': True,
            'publish_masks': True,
            'publish_detection_cloud': True,
            'detection_cloud_stride': 2,
            'yolo_model_path': '',
            'sam_model_path': '',
            'model_version': 'yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f',
            'calibration_version':
                'percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006',
            # 空串 → 算法默认相机系 +Y；否则 "x,y,z" 重力提示
            'gravity_hint_xyz': '',
            'tool.D_inner': 0.104,
            'tool.L_insert': 0.200,
            'tool.L_blade': 0.025,
            'tool.entry_d_tool': 0.030,
            'tool.entry_d_s': 0.040,
            'tool.clearance_min': 0.005,
            'tool.margin_neck': 0.015,
            'tool.version': '1.1',
        }
        descriptions = {
            'color_topic': '彩色图话题（bgr8）',
            'depth_topic': '深度图话题（uint16，须与彩色图 registration 对齐）',
            'camera_info_topic': '彩色相机内参话题',
            'camera_optical_frame': '相机光学系 frame_id（手眼链所挂）；'
                                    '空串则用深度图 header.frame_id',
            'output_frame': '输出坐标系：几何经 TF 变到此帧；空串=保持相机系',
            'tf_timeout_sec': 'TF 查询超时 (s)',
            'depth_scale_unit': '深度比例因子：raw × 本值 = 毫米量级（Percipio 常见 '
                                '0.25）；数据集回放（真毫米）设 1.0',
            'sync_slop_s': 'RGB-D 近似同步允差 (s)',
            'min_detection_conf': '检测置信度下限，低于该值的目标不入管线',
            'yolo_conf': 'YOLO 推理置信度阈值',
            'publish_debug_image': '是否发布 debug 叠加图 (~/debug_image)',
            'publish_masks': '是否发布 SAM 掩膜图 (~/masks)',
            'publish_detection_cloud': '是否发布检测框内深度反投影彩色点云 '
                                       '(~/detection_cloud)',
            'detection_cloud_stride': '点云降采样步长（>1 减轻 RViz 负载）',
            'yolo_model_path': 'YOLO 权重路径；空串=包内 model/best.pt',
            'sam_model_path': 'MobileSAM 权重路径；空串=包内 model/mobile_sam.pt',
            'model_version': '模型版本标识（随结果发布，便于追溯）',
            'calibration_version': '内外参版本标识（内参 color_camera_info.yaml；'
                                   '外参 hand_eye/active.yaml）',
            'gravity_hint_xyz': '重力方向提示 "x,y,z"（相机系）；'
                                '空串=算法默认相机系 +Y',
            'tool.D_inner': '工具圆柱内径 (m)，袋子必须能通过',
            'tool.L_insert': '最大插入深度 (m)',
            'tool.L_blade': '刀刃平面到圆柱入口平面的距离 (m)',
            'tool.entry_d_tool': '入口 standoff 的工具分量 (m)',
            'tool.entry_d_s': '入口 standoff 的安全裕量分量 (m)',
            'tool.clearance_min': '袋体与工具内壁的最小径向余量 (m)',
            'tool.margin_neck': '袋颈前方的安全停止距离 (m)',
            'tool.version': '工具几何配置的语义版本号',
        }
        for k, v in defaults.items():
            self.declare_parameter(
                k, v, ParameterDescriptor(description=descriptions[k]))

    def _load_params(self):
        """从参数服务器读出并缓存为实例属性；重力串解析失败则抛错."""
        g = self.get_parameter
        self.color_topic = g('color_topic').get_parameter_value().string_value
        self.depth_topic = g('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = g('camera_info_topic').get_parameter_value().string_value
        self.camera_optical_frame = g(
            'camera_optical_frame').get_parameter_value().string_value.strip()
        self.output_frame = g('output_frame').get_parameter_value().string_value.strip()
        self.tf_timeout = Duration(seconds=float(g('tf_timeout_sec').value))
        self.depth_scale_unit = float(g('depth_scale_unit').value)
        self.sync_slop_s = float(g('sync_slop_s').value)
        self.min_detection_conf = float(g('min_detection_conf').value)
        self.yolo_conf = float(g('yolo_conf').value)
        self.publish_debug_image = bool(g('publish_debug_image').value)
        self.publish_masks = bool(g('publish_masks').value)
        self.publish_detection_cloud = bool(g('publish_detection_cloud').value)
        self.detection_cloud_stride = max(1, int(g('detection_cloud_stride').value))
        self.yolo_model_path = g('yolo_model_path').get_parameter_value().string_value
        self.sam_model_path = g('sam_model_path').get_parameter_value().string_value
        self.model_version = g('model_version').get_parameter_value().string_value
        self.calibration_version = g(
            'calibration_version').get_parameter_value().string_value
        gh_s = g('gravity_hint_xyz').get_parameter_value().string_value.strip()
        if gh_s:
            parts = [float(x) for x in gh_s.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    f'gravity_hint_xyz needs 3 comma-separated floats, got {gh_s!r}')
            self.gravity_hint = np.asarray(parts, dtype=float)
        else:
            self.gravity_hint = None
        # entry_standoff = 刀具伸出 + 安全间隙，与 contracts.ToolGeometry 一致
        self.tool = ToolGeometry(
            D_inner=float(g('tool.D_inner').value),
            L_insert=float(g('tool.L_insert').value),
            L_blade=float(g('tool.L_blade').value),
            entry_d_tool=float(g('tool.entry_d_tool').value),
            entry_d_s=float(g('tool.entry_d_s').value),
            entry_standoff=float(g('tool.entry_d_tool').value)
            + float(g('tool.entry_d_s').value),
            clearance_min=float(g('tool.clearance_min').value),
            margin_neck=float(g('tool.margin_neck').value),
            version=str(g('tool.version').value),
        )

    def _lookup_T_out_cam(self, cam_frame: str, stamp) -> Optional[np.ndarray]:
        """查 output←camera；失败返回 None."""
        if not self.output_frame or self.output_frame == cam_frame:
            return np.eye(4)
        stamp_time = Time.from_msg(stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, cam_frame, stamp_time, timeout=self.tf_timeout)
            return _transform_msg_to_matrix(tf.transform)
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return _transform_msg_to_matrix(tf.transform)
            except TransformException as ex:
                self.get_logger().warning(
                    f'TF 失败，输出退回相机系 {cam_frame}: {ex}')
                return None

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        """同步回调 (ApproximateTimeSynchronizer)：一帧 RGB-D → 全套感知输出."""
        self.get_logger().info(
            f'RGB-D sync frame {rgb_msg.width}x{rgb_msg.height}')
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'RGB convert failed: {exc}')
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Depth convert failed: {exc}')
            return
        if depth.dtype != np.uint16:
            self.get_logger().warn(
                f'Depth dtype {depth.dtype} expected uint16')
            return
        # Percipio: z_m = raw * depth_scale_unit / 1000；管线内部按「毫米」再 /1000
        if abs(self.depth_scale_unit - 1.0) > 1e-9:
            depth = np.clip(
                np.round(depth.astype(np.float32) * self.depth_scale_unit),
                0, 65535).astype(np.uint16)
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
        T_out_cam = self._lookup_T_out_cam(cam_frame, rgb_msg.header.stamp)
        if T_out_cam is None and self.output_frame:
            # TF 失败则退回相机系，避免静默用错坐标系
            out_frame = cam_frame
            T_out_cam = np.eye(4)

        header = Header()
        header.stamp = rgb_msg.header.stamp
        header.frame_id = out_frame

        # ---- 检测 ----
        dets = self.engine.detect(rgb)
        det_msg = Detection2DArray()
        det_msg.header = header
        kept = []
        for d in dets:
            if float(d.get('conf', 0.0)) < self.min_detection_conf:
                continue
            kept.append(d)
            det_msg.detections.append(self._to_detection2d(d, header))
        self.pub_dets.publish(det_msg)

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
            self.pub_det_cloud.publish(
                _xyzrgb_to_cloud(header, xyz_out, rgb_f))

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

        # ---- 逐目标：SAM → 前景∩深度 → 袋/果管线 → TF → 消息 ----
        for i, det in enumerate(kept):
            bbox = tuple(det['bbox'])
            sam_mask = None
            segs = self.engine.segment(rgb, [bbox])
            if segs:
                sam_mask = segs[0][0]
                mask_canvas[sam_mask > 0] = np.uint8((i % 250) + 1)

            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K, frame_id=cam_frame,
                gravity_hint=self.gravity_hint,
                detections=[det],
                metadata={
                    'model_version': self.model_version,
                    'calibration_version': self.calibration_version,
                },
            )
            tid = f'target_{i}'
            results = self.estimator.estimate_modes(obs, tid, bbox, sam_mask)
            result = results['hybrid_dilated']
            if T_out_cam is not None and out_frame != cam_frame:
                _apply_T_to_grasp3d(result.grasp_3d, T_out_cam)
            g3d, g2d = result.grasp_3d, result.grasp_2d
            cand_arr.candidates.append(
                self._to_candidate(header, tid, g3d))
            cand2d_arr.candidates.append(
                self._to_candidate_2d(header, tid, g2d))
            fit_arr.fittings.append(
                self._to_fitting(header, tid, result))
            markers.markers.extend(
                self._to_markers(header, tid, i, result))
            if debug is not None:
                self._draw_debug(debug, det, g2d, sam_mask)

        self.pub_cands.publish(cand_arr)
        self.pub_cands_2d.publish(cand2d_arr)
        self.pub_fitting.publish(fit_arr)
        self.pub_markers.publish(markers)
        self.get_logger().info(
            f'Published {len(cand_arr.candidates)} candidates '
            f'(dets={len(kept)})')
        if self.publish_masks:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_canvas, encoding='mono8')
            mask_msg.header = header
            self.pub_masks.publish(mask_msg)
        if debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = header
            self.pub_debug.publish(dbg_msg)

    def _to_detection2d(self, det: dict, header: Header) -> Detection2D:
        """内部检测 dict → vision_msgs/Detection2D."""
        x1, y1, x2, y2 = det['bbox']
        d = Detection2D()
        d.header = header
        d.bbox.center.position.x = 0.5 * (x1 + x2)
        d.bbox.center.position.y = 0.5 * (y1 + y2)
        d.bbox.center.theta = 0.0
        d.bbox.size_x = float(max(0.0, x2 - x1))
        d.bbox.size_y = float(max(0.0, y2 - y1))
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(det.get('class_name', det.get('class_id', '')))
        hyp.hypothesis.score = float(det.get('conf', 0.0))
        d.results.append(hyp)
        return d

    def _to_candidate(self, header, tid, g3d) -> BagGraspCandidate:
        """3D 抓取参考 → peach_pose_msgs/BagGraspCandidate（坐标系=header.frame_id）."""
        m = BagGraspCandidate()
        m.header = header
        m.target_id = tid
        pose = Pose()
        if g3d.entry_start is not None:
            pose.position = _point(g3d.entry_start)
        if g3d.orientation is not None:
            pose.orientation = _rotation_to_quat(g3d.orientation)
        m.entry_pose = pose
        m.bag_bottom = _point(g3d.bag_bottom)
        m.bag_neck = _point(g3d.bag_neck)
        if g3d.translation_direction is not None:
            m.translation_direction = Vector3(
                x=float(g3d.translation_direction[0]),
                y=float(g3d.translation_direction[1]),
                z=float(g3d.translation_direction[2]))
        m.bag_diameter_upper_m = float(g3d.bag_diameter_upper_m or 0.0)
        m.suggested_travel_m = float(g3d.suggested_travel_m or 0.0)
        m.confidence = float(g3d.confidence or 0.0)
        m.status = STATUS_MAP.get(g3d.status, 2)
        m.diagnostic_flags = list(g3d.diagnostic_flags or [])
        m.strategy_id = g3d.strategy_id or ''
        m.model_version = g3d.model_version or self.model_version
        m.calibration_version = g3d.calibration_version or self.calibration_version
        m.tool_version = g3d.tool_version or self.tool.version
        return m

    def _to_candidate_2d(self, header, tid, g2d) -> BagGrasp2DMsg:
        """图像平面关键点 / 行程线 → BagGrasp2D（像素坐标）."""
        m = BagGrasp2DMsg()
        m.header = header
        m.target_id = tid
        x, y, w, h = g2d.detection_bbox
        m.bbox_x, m.bbox_y, m.bbox_w, m.bbox_h = int(x), int(y), int(w), int(h)
        m.bottom_px = _px(g2d.bottom_px)
        m.neck_px = _px(g2d.neck_px)
        m.grasp_px = _px(g2d.grasp_px)
        travel_end = None
        if g2d.travel_line and len(g2d.travel_line) >= 2:
            travel_end = g2d.travel_line[1]
        m.travel_end_px = _px(travel_end)
        m.has_bottom_px = g2d.bottom_px is not None
        m.has_neck_px = g2d.neck_px is not None
        m.has_grasp_px = g2d.grasp_px is not None
        m.has_travel_end_px = travel_end is not None
        m.confidence = float(g2d.confidence or 0.0)
        m.status = STATUS_MAP.get(g2d.status, 2)
        m.diagnostic_flags = list(g2d.diagnostic_flags or [])
        return m

    def _to_fitting(self, header, tid, result) -> BagFitting:
        """管线 metrics / diagnostic → BagFitting（诊断用，不参与运动）."""
        m = BagFitting()
        m.header = header
        m.target_id = tid
        m.target_kind = result.target_kind or 'bag'
        m.mask_source = result.mask_source or ''
        metrics = result.metrics or {}
        info = result.grasp_3d.diagnostic_info or {}
        m.axis_source = str(info.get('axis_source', ''))
        m.axis_confidence = _metric(metrics, 'axis_confidence')
        m.axis_disagreement_deg = _metric(metrics, 'axis_disagreement_deg')
        m.theta_err_deg = _metric(metrics, 'theta_err_deg')
        m.error_budget_mm = _metric(metrics, 'error_budget_mm')
        m.radial_clearance_mm = _metric(metrics, 'radial_clearance_mm')
        m.valid_depth_ratio = _metric(metrics, 'valid_depth_ratio')
        m.foreground_ratio = _metric(metrics, 'foreground_ratio')
        m.boundary_touch_ratio = _metric(metrics, 'boundary_touch_ratio')
        m.boundary_sides_touched = int(metrics.get('boundary_sides_touched', -1) or -1)
        m.n_points = int(metrics.get('n_points', -1) or -1)
        m.bag_length_m = _metric(metrics, 'bag_length_m')
        m.bag_diameter_upper_m = _metric(metrics, 'bag_diameter_upper_m')
        m.travel_m = _metric(metrics, 'travel_m')
        m.cylinder_rms_m = _metric(metrics, 'cylinder_rms_m')
        m.cylinder_inlier_ratio = _metric(metrics, 'cylinder_inlier_ratio')
        m.fruit_radius_m = _metric(metrics, 'fruit_radius_m')
        m.sphere_rms_m = _metric(metrics, 'sphere_rms_m')
        m.sphere_inlier_ratio = _metric(metrics, 'sphere_inlier_ratio')
        m.cavity_dip_mm = _metric(metrics, 'cavity_dip_mm')
        m.axis_polarity_corrected = bool(metrics.get('axis_polarity_corrected', False))
        m.status = STATUS_MAP.get(result.grasp_3d.status, 2)
        m.diagnostic_flags = list(result.grasp_3d.diagnostic_flags or [])
        return m

    def _to_markers(self, header, tid, idx, result) -> List[Marker]:
        """
        可视化 (RViz)：轴 / 行程箭头 / 刀具圆柱 / 果球 / 三轴架 / 状态文字.

        每个目标占用 id 段 ``idx*20 .. idx*20+19``，避免多目标冲突。
        """
        g3d = result.grasp_3d
        out: List[Marker] = []
        r, g, b, a = _status_color(g3d.status)
        base_id = idx * 20

        def _mk(mid, mtype) -> Marker:
            m = Marker()
            m.header = header
            m.ns = 'peach_pose'
            m.id = base_id + mid
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            # ColorRGBA 字段必须是 Python float（int/np.float64 会触发断言崩溃）
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)
            m.lifetime.sec = 0
            return m

        if g3d.bag_bottom is not None and g3d.bag_neck is not None:
            axis = _mk(0, Marker.LINE_LIST)
            axis.scale.x = 0.004
            axis.points = [_point(g3d.bag_bottom), _point(g3d.bag_neck)]
            out.append(axis)

        if (g3d.entry_start is not None and g3d.suggested_travel_end is not None):
            arrow = _mk(1, Marker.ARROW)
            arrow.scale.x = 0.008
            arrow.scale.y = 0.015
            arrow.scale.z = 0.015
            arrow.points = [_point(g3d.entry_start), _point(g3d.suggested_travel_end)]
            out.append(arrow)

        if (g3d.entry_start is not None and g3d.translation_direction is not None
                and g3d.suggested_travel_m > 0):
            cyl = _mk(2, Marker.CYLINDER)
            mid = g3d.entry_start + 0.5 * g3d.suggested_travel_m * g3d.translation_direction
            cyl.pose.position = _point(mid)
            if g3d.orientation is not None:
                # Marker CYLINDER 默认轴为 Z；抓取架 Zg = translation_direction
                cyl.pose.orientation = _rotation_to_quat(g3d.orientation)
            diam = float(self.tool.D_inner)
            cyl.scale.x = diam
            cyl.scale.y = diam
            cyl.scale.z = float(g3d.suggested_travel_m)
            cyl.color.a = 0.25
            out.append(cyl)

        if (result.target_kind == 'fruit' and g3d.bag_bottom is not None
                and g3d.bag_neck is not None):
            # 球心近似为底/颈中点
            sphere = _mk(3, Marker.SPHERE)
            center = 0.5 * (np.asarray(g3d.bag_bottom) + np.asarray(g3d.bag_neck))
            radius = _metric(result.metrics or {}, 'fruit_radius_m', 0.0)
            if radius > 0:
                sphere.pose.position = _point(center)
                sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * radius)
                sphere.color.a = 0.3
                out.append(sphere)

        if g3d.entry_start is not None and g3d.orientation is not None:
            # 三轴架
            R = np.asarray(g3d.orientation)
            origin = np.asarray(g3d.entry_start, dtype=float)
            colors = [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
            for ax_i, col in enumerate(colors):
                axis_m = _mk(4 + ax_i, Marker.ARROW)
                axis_m.scale.x = 0.005
                axis_m.scale.y = 0.01
                axis_m.scale.z = 0.01
                axis_m.color.r = float(col[0])
                axis_m.color.g = float(col[1])
                axis_m.color.b = float(col[2])
                axis_m.color.a = float(col[3])
                end = origin + 0.05 * R[:, ax_i]
                axis_m.points = [_point(origin), _point(end)]
                out.append(axis_m)

        text = _mk(10, Marker.TEXT_VIEW_FACING)
        text.scale.z = 0.03
        text.text = f'{tid} {g3d.status}'
        if g3d.entry_start is not None:
            text.pose.position = _point(g3d.entry_start)
        elif g3d.bag_bottom is not None:
            text.pose.position = _point(g3d.bag_bottom)
        out.append(text)
        return out

    def _draw_debug(self, img, det, g2d, sam_mask):
        """在 BGR 图上叠检测框、SAM 掩膜、底→颈箭头与状态文字（原地改 img）."""
        x1, y1, x2, y2 = map(int, det['bbox'])
        color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        if sam_mask is not None:
            overlay = img.copy()
            overlay[sam_mask > 0] = (
                0.5 * overlay[sam_mask > 0] + np.array([40, 40, 200])).astype(np.uint8)
            cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
        status = g2d.status
        st_color = {
            'ACCEPT': (0, 220, 0), 'REOBSERVE': (0, 200, 255), 'REJECT': (0, 0, 220)
        }.get(status, (180, 180, 180))
        if g2d.bottom_px and g2d.neck_px:
            cv2.arrowedLine(
                img,
                (int(g2d.bottom_px[0]), int(g2d.bottom_px[1])),
                (int(g2d.neck_px[0]), int(g2d.neck_px[1])),
                (255, 255, 0), 2, tipLength=0.15)
        if g2d.grasp_px:
            cv2.circle(img, (int(g2d.grasp_px[0]), int(g2d.grasp_px[1])), 5, st_color, -1)
        cv2.putText(img, status, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, st_color, 2)


def main(args=None):
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
