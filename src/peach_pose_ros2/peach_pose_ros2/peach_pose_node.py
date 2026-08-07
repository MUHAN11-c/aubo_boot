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
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Vector3Stamped
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
from peach_pose_ros2.peach_pose.depth_geometry import normalize_depth_to_uint16_mm
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
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

# 三态安全门控结果 → ROS 消息枚举的映射（与 peach_pose_msgs/BagGraspCandidate.status 一致）。
# 算法管线内部用字符串状态，发布消息时经本表转成 uint8：
#   ACCEPT=0    可信：无任何诊断标记，位姿可直接用于套袋动作
#   REOBSERVE=1 存疑：信息不足（如掩膜缺失、轴来自重力先验、触边截断等），
#               建议换个视角重采一帧再判，不建议直接动作
#   REJECT=2    不可用：存在硬性失败（如 tool_clearance_failed 净空不足、
#               有效点太少等），禁止据此位姿动作
STATUS_MAP = {'ACCEPT': 0, 'REOBSERVE': 1, 'REJECT': 2}


def _quat_to_matrix_handwritten(q_xyzw) -> np.ndarray:
    """
    手写原理版：单位四元数 (x,y,z,w) → 3×3 旋转矩阵（教学对照用）.

    假定 q 已归一化；与官方 quaternion_matrix 的差别仅在于官方内部会归一化。
    单位四元数输入下两者数值一致（max|Δ|≈1e-15）。

    Args:
        q_xyzw: 单位四元数 (x, y, z, w) 可迭代序列；未归一化时结果按比例偏差.

    Returns
    -------
        (3, 3) float 旋转矩阵.

    """
    x, y, z, w = q_xyzw
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=float)


def _transform_msg_to_matrix(t, logger=None) -> np.ndarray:
    """
    geometry_msgs/Transform → 4×4 齐次矩阵 T（p_out = R@p_in + t）.

    返回官方 tf_transformations 结果（内部归一化，最稳健）；手写原理版
    同步计算对照，diff>1e-9 经 logger 告警（缺省 print）。
    一致性已固化于 test_tf_matrix.py。

    Args:
        t: geometry_msgs/Transform（translation + rotation 四元数）.
        logger: 双实现不一致时的告警出口；None 时退化为 print.

    Returns
    -------
        (4, 4) 齐次变换矩阵（平移单位随消息，通常为米）.

    """
    tr = t.translation
    q = t.rotation
    q_xyzw = (q.x, q.y, q.z, q.w)
    T_api = translation_matrix((tr.x, tr.y, tr.z)) @ quaternion_matrix(q_xyzw)
    # 手写原理版同步计算并对比（教学 + 正确性双保险；两个 4×4 运算开销可忽略）
    T_hw = np.eye(4, dtype=float)
    T_hw[:3, :3] = _quat_to_matrix_handwritten(q_xyzw)
    T_hw[0, 3], T_hw[1, 3], T_hw[2, 3] = tr.x, tr.y, tr.z
    diff = float(np.abs(T_api - T_hw).max())
    if diff > 1e-9:
        msg = f'TF 双实现不一致（diff={diff:.3e}），已采用官方 API 结果'
        (logger.warning if logger is not None else print)(msg)
    return T_api


def _apply_T_to_grasp3d(g3d, T: np.ndarray) -> None:
    """
    抓取几何由相机系变到输出系（默认 base_link），原地修改 g3d.

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t（含 entry_start /
    bag_bottom / bag_neck / suggested_travel_end / legacy position）；
    方向只乘 R 并归一化（平移不影响方向）；姿态矩阵左乘 R。
    None 字段原样保留。

    Args:
        g3d: BagGraspReference3D（相机光学系，米）；被原地改写.
        T: (4, 4) 齐次矩阵，输出系←相机系.

    Returns
    -------
        None（结果写回 g3d）.

    """
    R, t = T[:3, :3], T[:3, 3]

    def _pt(p):
        if p is None:
            return None
        return R @ np.asarray(p, dtype=float) + t

    g3d.entry_start = _pt(g3d.entry_start)
    g3d.bag_bottom = _pt(g3d.bag_bottom)
    g3d.bag_neck = _pt(g3d.bag_neck)
    # 行程终点与 legacy position 也是点，必须同步变换（漏改会让 ~/markers
    # 的行程箭头终点留在相机系，与输出系几何错位）
    g3d.suggested_travel_end = _pt(g3d.suggested_travel_end)
    g3d.position = _pt(g3d.position)
    if g3d.translation_direction is not None:
        d = R @ np.asarray(g3d.translation_direction, dtype=float)
        n = float(np.linalg.norm(d))
        g3d.translation_direction = d / n if n > 1e-9 else d
    if g3d.orientation is not None:
        g3d.orientation = R @ np.asarray(g3d.orientation, dtype=float)


def _gravity_camera_from_R(R_out_cam: np.ndarray) -> np.ndarray:
    """
    由 output←camera 旋转反推相机系重力方向（gravity_mode='tf' 用）.

    约定 output_frame（如 base_link）内重力向量为 [0, 0, -1]（竖直向下）；
    方向向量只乘旋转、不加平移：g_cam = normalize(R_out_cam.T @ g_out)。

    Args:
        R_out_cam: (3, 3) 旋转矩阵，output_frame←相机系.

    Returns
    -------
        (3,) 相机系单位重力向量；退化（近零）时原样返回.

    """
    g = np.asarray(R_out_cam, dtype=float).T @ np.array([0.0, 0.0, -1.0])
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else g


def _rotation_to_quat_handwritten(R: np.ndarray) -> Quaternion:
    """
    手写原理版：3×3 旋转矩阵 → 四元数（Shepperd 法，教学对照用）.

    按迹/最大对角元分四支取值，避免除小量的数值退化。
    四元数有符号二义性（±q 同旋转），对比时取 min(|q1-q2|, |q1+q2|)。

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        geometry_msgs/Quaternion（输入正交时即单位四元数）.

    """
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


def _rotation_to_quat(R: np.ndarray, logger=None) -> Quaternion:
    """
    3×3 旋转矩阵 → geometry_msgs/Quaternion.

    返回官方 quaternion_from_matrix 结果；手写 Shepperd 版同步对照，
    diff>1e-9 经 logger 告警（缺省 print；按 ±q 等价取 min 差）。

    Args:
        R: (3, 3) 旋转矩阵.
        logger: 双实现不一致时的告警出口；None 时退化为 print.

    Returns
    -------
        单位四元数 Quaternion 消息.

    """
    m4 = np.eye(4, dtype=float)
    m4[:3, :3] = np.asarray(R, dtype=float)
    q_api = quaternion_from_matrix(m4)          # numpy [x, y, z, w]
    q_hw_msg = _rotation_to_quat_handwritten(R)
    q_hw = np.array([q_hw_msg.x, q_hw_msg.y, q_hw_msg.z, q_hw_msg.w])
    diff = min(float(np.linalg.norm(q_hw - q_api)),
               float(np.linalg.norm(q_hw + q_api)))
    if diff > 1e-9:
        msg = f'四元数双实现不一致（diff={diff:.3e}），已采用官方 API 结果'
        (logger.warning if logger is not None else print)(msg)
    return Quaternion(x=float(q_api[0]), y=float(q_api[1]),
                      z=float(q_api[2]), w=float(q_api[3]))


def _point(xyz) -> Point:
    """
    3D 点（ndarray/list）→ Point 消息；None 给零点；强制 float 防 rosidl 类型断言.

    Args:
        xyz: (3,) 坐标（单位随上游，通常米）；None 时返回全零 Point.

    Returns
    -------
        geometry_msgs/Point.

    """
    p = Point()
    if xyz is None:
        return p
    p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return p


def _px(uv, z=0.0) -> Point:
    """
    像素 (u,v) → Point（x=u, y=v, z=z）；2D 消息复用 Point 类型；None 给零点.

    Args:
        uv: (2,) 像素坐标；None 时返回全零 Point（有效性由 has_* 标志区分）.
        z: 填入 Point.z 的值（像素语义下恒 0）.

    Returns
    -------
        geometry_msgs/Point.

    """
    p = Point()
    if uv is None:
        return p
    p.x, p.y, p.z = float(uv[0]), float(uv[1]), float(z)
    return p


def _metric(m: dict, key: str, default: float = -1.0) -> float:
    """
    从 metrics 字典取标量转 float；缺失/None/不可转一律给 default（消息以 -1 表无效）.

    Args:
        m: 管线 metrics 字典（值可为 None）.
        key: 指标名.
        default: 缺失/无效时的填充值（BagFitting 约定 -1）.

    Returns
    -------
        float 标量.

    """
    v = m.get(key, None)
    if v is None:
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _status_color(status: str) -> Tuple[float, float, float, float]:
    """
    三态 → Marker RGBA：ACCEPT 绿 / REOBSERVE 黄 / REJECT 红 / 其他灰.

    Args:
        status: 'ACCEPT' | 'REOBSERVE' | 'REJECT'（未知值给灰色）.

    Returns
    -------
        (r, g, b, a) 四元组，各分量 [0, 1].

    """
    return {
        'ACCEPT': (0.1, 0.85, 0.2, 0.9),
        'REOBSERVE': (0.95, 0.8, 0.1, 0.9),
        'REJECT': (0.9, 0.15, 0.15, 0.9),
    }.get(status, (0.6, 0.6, 0.6, 0.8))


def _pack_rgb_bgr(bgr: np.ndarray) -> np.ndarray:
    """
    (N,3) uint8 BGR → (N,) float32：按位打包成 PointCloud2 的 rgb 字段.

    Args:
        bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB，符合 PointCloud2 rgb 打包约定）.

    """
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
    检测框内像素反投影成彩色点云：返回 (N,3) xyz（米）与 (N,) 打包 rgb.

    depth_mm 为毫米单位 uint16（Percipio 原始值已 × depth_scale_unit）；
    剔除无效深度（0/饱和 65535），stride 为降采样步长。

    Args:
        rgb_bgr: (H, W, 3) uint8 BGR 图，与深度对齐.
        depth_mm: (H, W) uint16 深度，单位毫米.
        K: 相机内参 {"fx","fy","cx","cy"}（像素单位）.
        bboxes: 检测框列表 [(x1, y1, x2, y2)]（像素，自动裁剪到图内）.
        stride: 降采样步长（像素）；1 为不降采样.

    Returns
    -------
        (xyz, rgb_packed)：xyz 为 (N, 3) float64 相机系点（米），
        rgb_packed 为 (N,) float32 打包颜色；无有效点时均为空数组.

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
    """
    组装 xyz + 打包 rgb → PointCloud2 消息（x/y/z 各一个 FLOAT32 + rgb 位打包）.

    Args:
        header: 输出消息头（frame_id 决定点云坐标系解释）.
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常米）；空数组给空云.
        rgb_f: (N,) float32 打包 rgb（见 _pack_rgb_bgr）.

    Returns
    -------
        sensor_msgs/PointCloud2.

    """
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
        """建节点：参数声明/加载 → 模型与管线 → 发布者、RGB-D 同步订阅与 TF 监听."""
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
            # fixed=仅用 gravity_hint_xyz；tf=由本帧 TF 旋转反推相机系重力
            'gravity_mode': 'fixed',
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
            'depth_scale_unit': '深度比例因子（仅 uint16 原始深度生效）：raw × 本值 = '
                                '毫米量级（Percipio 常见 0.25）；数据集回放（真毫米）'
                                '设 1.0；32FC1 浮点深度按「米」×1000 转毫米，本参数不生效',
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
            'gravity_mode': '重力来源：fixed=仅用 gravity_hint_xyz（默认，行为与旧版'
                            '一致）；tf=由本帧 output←camera 的 TF 旋转反推相机系重力'
                            '（output_frame 系重力约定 [0,0,-1]，只乘旋转不加平移，'
                            'TF 不可用的帧回退 gravity_hint_xyz）',
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
        self.gravity_mode = g('gravity_mode').get_parameter_value().string_value.strip()
        if self.gravity_mode not in ('fixed', 'tf'):
            self.get_logger().warning(
                f'未知 gravity_mode={self.gravity_mode!r}，回退 fixed')
            self.gravity_mode = 'fixed'
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
            return _transform_msg_to_matrix(tf.transform, self.get_logger()), 'ok'
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, cam_frame, Time(), timeout=self.tf_timeout)
                if not self._tf_warned:
                    self.get_logger().warning(
                        f'TF {self.output_frame}←{cam_frame} 按 stamp 失败，'
                        '已用最新 TF（确认 extrinsics_publisher 已启动）')
                    self._tf_warned = True
                return (_transform_msg_to_matrix(tf.transform, self.get_logger()),
                        'stale')
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
        T_out_cam, tf_status = self._lookup_T_out_cam(
            cam_frame, rgb_msg.header.stamp)
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
        header.stamp = rgb_msg.header.stamp
        header.frame_id = out_frame
        # 图像平面数据（检测框/掩膜/debug 图）的 frame_id 用 RGB 图自身坐标系；
        # 3D 结果（候选/拟合/Marker/检测点云）仍用输出系
        img_header = Header()
        img_header.stamp = rgb_msg.header.stamp
        img_header.frame_id = rgb_msg.header.frame_id

        # ---- 检测 ----
        dets = self.engine.detect(rgb)
        det_msg = Detection2DArray()
        det_msg.header = img_header
        kept = []
        for d in dets:
            if float(d.get('conf', 0.0)) < self.min_detection_conf:
                continue
            kept.append(d)
            det_msg.detections.append(self._to_detection2d(d, img_header))
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
            if T_out_cam is not None and out_frame != cam_frame:
                _apply_T_to_grasp3d(result.grasp_3d, T_out_cam)
            frame_axes.append((result.grasp_3d.status,
                               result.grasp_3d.translation_direction))
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
        if self.publish_masks:
            mask_msg = self.bridge.cv2_to_imgmsg(mask_canvas, encoding='mono8')
            mask_msg.header = img_header
            self.pub_masks.publish(mask_msg)
            self.pub_norm_masks.publish(mask_msg)
        if debug is not None:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            dbg_msg.header = img_header
            self.pub_debug.publish(dbg_msg)

    def _to_detection2d(self, det: dict, header: Header) -> Detection2D:
        """
        内部检测 dict → Detection2D 消息（bbox 中心/尺寸 + 类别名 + 置信度）.

        Args:
            det: engine.detect 的一项（bbox xyxy 像素、class_name/class_id、conf）.
            header: 输出头（stamp/frame_id）.

        Returns
        -------
            vision_msgs/Detection2D.

        """
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
        """
        3D 抓取参考 → BagGraspCandidate 主输出消息（坐标系=header.frame_id）.

        Args:
            header: 输出头；frame_id 即 g3d 当前所在坐标系.
            tid: 目标 ID（target_N）.
            g3d: BagGraspReference3D（米；None 字段在消息中给零/缺省）.

        Returns
        -------
            peach_pose_msgs/BagGraspCandidate.

        """
        m = BagGraspCandidate()
        m.header = header
        m.target_id = tid
        pose = Pose()
        if g3d.entry_start is not None:
            pose.position = _point(g3d.entry_start)
        if g3d.orientation is not None:
            pose.orientation = _rotation_to_quat(g3d.orientation,
                                                 self.get_logger())
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
        """
        图像平面关键点/行程线 → BagGrasp2D 消息（像素坐标；无值点由 has_* 标志区分）.

        Args:
            header: 输出头.
            tid: 目标 ID.
            g2d: BagGrasp2D（像素坐标；None 点给零且对应 has_*=False）.

        Returns
        -------
            peach_pose_msgs/BagGrasp2D.

        """
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
        """
        管线 metrics/诊断 → BagFitting 消息（仅供诊断调参，不参与运动；无效标量填 -1）.

        Args:
            header: 输出头.
            tid: 目标 ID.
            result: pipeline.TargetPoseResult（metrics 缺项按 -1 填充）.

        Returns
        -------
            peach_pose_msgs/BagFitting.

        """
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
        结果 → RViz Marker 列表：袋轴/行程箭头/刀具圆柱/果球/三轴架/状态文字.

        每个目标占用 id 段 ``idx*20 .. idx*20+19``，避免多目标冲突。

        Args:
            header: 输出头（frame_id 与候选消息一致）.
            tid: 目标 ID（用于状态文字）.
            idx: 目标序号（Marker id 段基址 = idx*20）.
            result: pipeline.TargetPoseResult.

        Returns
        -------
            Marker 列表（不含 DELETEALL，由调用方统一添加）.

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
                cyl.pose.orientation = _rotation_to_quat(g3d.orientation,
                                                         self.get_logger())
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
        """
        在 BGR 图上叠检测框、SAM 掩膜轮廓、底→颈箭头与状态文字（原地改 img）.

        Args:
            img: (H, W, 3) uint8 BGR，被原地改写.
            det: 检测 dict（bbox、class_id；class 0 绿框，其他橙框）.
            g2d: BagGrasp2D（提供关键点像素与状态）.
            sam_mask: (H, W) 掩膜或 None（None 时不画轮廓）.

        Returns
        -------
            无返回值（None）；叠加结果写回 img.

        """
        x1, y1, x2, y2 = map(int, det['bbox'])
        color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        if sam_mask is not None:
            # 只画分割轮廓线，不做半透明颜色填充——掩膜上色会盖住果实纹理，
            # 轮廓更便于观察分割边界是否贴边
            contours, _ = cv2.findContours(
                (sam_mask > 0).astype(np.uint8),
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, (80, 80, 230), 2)
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
