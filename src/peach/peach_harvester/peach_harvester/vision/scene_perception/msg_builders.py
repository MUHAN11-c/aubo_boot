"""
观测消息组装（W3 自 visualization 拆出）：内部结果 → ROS 消息.

公开 API：to_detection2d / to_candidate / to_candidate_2d / to_fitting /
to_markers / xyzrgb_to_cloud_msg / bbox_cloud_xyzrgb / best_axis_direction /
TRACKING_STATUS_TO_MSG / quat_to_msg（签名与拆分前一致，仅去下划线）。
像素绘制族在同拆 debug_draw；本模块不画图。
"""
from __future__ import annotations

from typing import List

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
from peach_harvester.vision.common.geometry import (
    bbox_cloud_xyzrgb,  # noqa: F401  W3 单实现落 geometry，此处 re-export 公开面
    QuaternionValue,
    rotation_to_quat,
)
from peach_harvester.vision.scene_perception.identity import (
    STATUS_DEPTH_VOID,
    STATUS_LOST,
    STATUS_OBSERVED,
    STATUS_OCCLUDED,
    STATUS_OUT_OF_VIEW,
)
from peach_interfaces.msg import (
    BagFitting,
    BagGrasp2D as BagGrasp2DMsg,
    BagGraspCandidate,
    PeachTargetObservation,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

# 三态安全门控结果 → ROS 消息枚举的映射（与 peach_interfaces/BagGraspCandidate.status 一致）。
# 算法管线内部用字符串状态，发布消息时经本表转成 uint8：
#   ACCEPT=0    可信：无任何诊断标记，位姿可直接用于套袋动作
#   REOBSERVE=1 存疑：信息不足（如掩膜缺失、轴来自重力先验、触边截断等），
#               建议换个视角重采一帧再判，不建议直接动作
#   REJECT=2    不可用：存在硬性失败（如 tool_clearance_failed 净空不足、
#               有效点太少等），禁止据此位姿动作
STATUS_MAP = {'ACCEPT': 0, 'REOBSERVE': 1, 'REJECT': 2}
TRACKING_STATUS_TO_MSG = {
    STATUS_OBSERVED: PeachTargetObservation.OBSERVED,
    STATUS_OCCLUDED: PeachTargetObservation.OCCLUDED,
    STATUS_LOST: PeachTargetObservation.LOST,
    STATUS_OUT_OF_VIEW: PeachTargetObservation.OUT_OF_VIEW,
    STATUS_DEPTH_VOID: PeachTargetObservation.DEPTH_VOID,
}


def best_axis_direction(frame_axes):
    """First ACCEPT direction, else first valid direction."""
    best = None
    for status, direction in frame_axes:
        if direction is None:
            continue
        if status == 'ACCEPT':
            return direction
        if best is None:
            best = direction
    return best


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


def _rotation_to_quat(R: np.ndarray) -> Quaternion:
    """
    3×3 旋转矩阵 → geometry_msgs/Quaternion（common.geometry 值对象的消息包装）.

    W3 自 pose_pipelines 迁入（V5：位姿纯核顶层零 ROS msg import）；
    数值路径不变——官方 scipy Rotation（geometry.rotation_to_quat），
    此处仅把 QuaternionValue 组装成消息。

    Args:
        R: (3, 3) 旋转矩阵.

    Returns
    -------
        单位四元数 Quaternion 消息（x, y, z, w）.

    """
    q = rotation_to_quat(R)
    return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)


def quat_to_msg(q: QuaternionValue) -> Quaternion:
    """
    四元数值对象（QuaternionValue）→ geometry_msgs/Quaternion 消息，记忆锚点回填用.

    Args:
        q: geometry.rotation_to_quat 产物（identity.memory_anchor_fields
            的 'orientation' 字段）.

    Returns
    -------
        单位四元数 Quaternion 消息（x, y, z, w）.

    """
    return Quaternion(x=float(q.x), y=float(q.y), z=float(q.z), w=float(q.w))


def to_detection2d(det: dict, header) -> Detection2D:
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


def to_candidate(header, tid, grasp_3d, model_version: str,
                 calibration_version: str, tool_version: str) -> BagGraspCandidate:
    """
    3D 抓取参考 → BagGraspCandidate 主输出消息（坐标系=header.frame_id）.

    Args:
        header: 输出头；frame_id 即 grasp_3d 当前所在坐标系.
        tid: 目标 ID（target_N）.
        grasp_3d: BagGraspReference3D（米；None 字段在消息中给零/缺省）.
        model_version: 模型版本回退值（grasp_3d 自带时优先）.
        calibration_version: 内外参版本回退值（grasp_3d 自带时优先）.
        tool_version: 工具版本回退值（grasp_3d 自带时优先）.

    Returns
    -------
        peach_interfaces/BagGraspCandidate.

    """
    m = BagGraspCandidate()
    m.header = header
    m.target_id = tid
    pose = Pose()
    if grasp_3d.entry_start is not None:
        pose.position = _point(grasp_3d.entry_start)
    if grasp_3d.orientation is not None:
        pose.orientation = _rotation_to_quat(grasp_3d.orientation)
    m.entry_pose = pose
    m.bag_bottom = _point(grasp_3d.bag_bottom)
    m.bag_neck = _point(grasp_3d.bag_neck)
    if grasp_3d.translation_direction is not None:
        m.translation_direction = Vector3(
            x=float(grasp_3d.translation_direction[0]),
            y=float(grasp_3d.translation_direction[1]),
            z=float(grasp_3d.translation_direction[2]))
    m.bag_diameter_upper_m = float(grasp_3d.bag_diameter_upper_m or 0.0)
    m.suggested_travel_m = float(grasp_3d.suggested_travel_m or 0.0)
    m.confidence = float(grasp_3d.confidence or 0.0)
    m.status = STATUS_MAP.get(grasp_3d.status, 2)
    m.diagnostic_flags = list(grasp_3d.diagnostic_flags or [])
    m.strategy_id = grasp_3d.strategy_id or ''
    m.model_version = grasp_3d.model_version or model_version
    m.calibration_version = grasp_3d.calibration_version or calibration_version
    m.tool_version = grasp_3d.tool_version or tool_version
    if grasp_3d.position_covariance is not None:
        m.position_covariance = np.asarray(
            grasp_3d.position_covariance, dtype=float).reshape(9).tolist()
    if grasp_3d.direction_covariance is not None:
        m.direction_covariance = np.asarray(
            grasp_3d.direction_covariance, dtype=float).reshape(9).tolist()
    return m


def to_candidate_2d(header, tid, grasp_2d) -> BagGrasp2DMsg:
    """
    图像平面关键点/行程线 → BagGrasp2D 消息（像素坐标；无值点由 has_* 标志区分）.

    Args:
        header: 输出头.
        tid: 目标 ID.
        grasp_2d: BagGrasp2D（像素坐标；None 点给零且对应 has_*=False）.

    Returns
    -------
        peach_interfaces/BagGrasp2D.

    """
    m = BagGrasp2DMsg()
    m.header = header
    m.target_id = tid
    x, y, w, h = grasp_2d.detection_bbox
    m.bbox_x, m.bbox_y, m.bbox_w, m.bbox_h = int(x), int(y), int(w), int(h)
    m.bottom_px = _px(grasp_2d.bottom_px)
    m.neck_px = _px(grasp_2d.neck_px)
    m.grasp_px = _px(grasp_2d.grasp_px)
    travel_end = None
    if grasp_2d.travel_line and len(grasp_2d.travel_line) >= 2:
        travel_end = grasp_2d.travel_line[1]
    m.travel_end_px = _px(travel_end)
    m.has_bottom_px = grasp_2d.bottom_px is not None
    m.has_neck_px = grasp_2d.neck_px is not None
    m.has_grasp_px = grasp_2d.grasp_px is not None
    m.has_travel_end_px = travel_end is not None
    m.confidence = float(grasp_2d.confidence or 0.0)
    m.status = STATUS_MAP.get(grasp_2d.status, 2)
    m.diagnostic_flags = list(grasp_2d.diagnostic_flags or [])
    return m


def to_fitting(header, tid, result) -> BagFitting:
    """
    管线 metrics/诊断 → BagFitting 消息（仅供诊断调参，不参与运动；无效标量填 -1）.

    Args:
        header: 输出头.
        tid: 目标 ID.
        result: pipeline.TargetPoseResult（metrics 缺项按 -1 填充）.

    Returns
    -------
        peach_interfaces/BagFitting.

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


def xyzrgb_to_cloud_msg(header: Header, xyz: np.ndarray,
                        rgb_f: np.ndarray) -> PointCloud2:
    """
    组装 xyz + 打包 rgb → PointCloud2 消息（x/y/z 各一个 FLOAT32 + rgb 位打包）.

    走官方 sensor_msgs_py.point_cloud2.create_cloud 的 numpy 结构化数组
    快路径（与 target_reconstruction.publish.xyzrgb_to_cloud_msg 同款）；
    fields 手工声明是因为官方预置只有 create_cloud_xyz32（纯 xyz 无 rgb），
    带打包 rgb 的自定义布局必须显式给 fields——这是官方 API 对自定义
    布局的标准用法。

    Args:
        header: 输出消息头（frame_id 决定点云坐标系解释）.
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常米）；空数组给空云.
        rgb_f: (N,) float32 打包 rgb（见 geometry.pack_rgb_bgr）.

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
    pts = np.zeros((len(xyz), 4), dtype=np.float32)
    pts[:, :3] = xyz
    pts[:, 3] = rgb_f
    return pc2.create_cloud(header, fields, pts)


def _status_color(status: str):
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


def to_markers(header, tid, idx, result, tool_d_inner: float) -> List[Marker]:
    """
    结果 → RViz Marker 列表：袋轴/行程箭头/刀具圆柱/果球/三轴架/状态文字.

    每个目标占用 id 段 ``idx*20 .. idx*20+19``，避免多目标冲突。

    Args:
        header: 输出头（frame_id 与候选消息一致）.
        tid: 目标 ID（用于状态文字）.
        idx: 目标序号（Marker id 段基址 = idx*20）.
        result: pipeline.TargetPoseResult.
        tool_d_inner: 工具内径 (m)，刀具圆柱直径.

    Returns
    -------
        Marker 列表（不含 DELETEALL，由调用方统一添加）.

    """
    grasp_3d = result.grasp_3d
    out: List[Marker] = []
    r, g, b, a = _status_color(grasp_3d.status)
    base_id = idx * 20

    def _mk(mid, mtype) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'scene_perception'
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

    if grasp_3d.bag_bottom is not None and grasp_3d.bag_neck is not None:
        axis = _mk(0, Marker.LINE_LIST)
        axis.scale.x = 0.004
        axis.points = [_point(grasp_3d.bag_bottom), _point(grasp_3d.bag_neck)]
        out.append(axis)

    if (grasp_3d.entry_start is not None and grasp_3d.suggested_travel_end is not None):
        arrow = _mk(1, Marker.ARROW)
        arrow.scale.x = 0.008
        arrow.scale.y = 0.015
        arrow.scale.z = 0.015
        arrow.points = [_point(grasp_3d.entry_start), _point(grasp_3d.suggested_travel_end)]
        out.append(arrow)

    if (grasp_3d.entry_start is not None and grasp_3d.translation_direction is not None
            and grasp_3d.suggested_travel_m > 0):
        env = _mk(2, Marker.CYLINDER)
        env.ns = 'bag_envelope'
        mid = (grasp_3d.entry_start
               + 0.5 * grasp_3d.suggested_travel_m
               * grasp_3d.translation_direction)
        env.pose.position = _point(mid)
        if grasp_3d.orientation is not None:
            env.pose.orientation = _rotation_to_quat(grasp_3d.orientation)
        bag_d = float(grasp_3d.bag_diameter_upper_m or 0.06)
        env.scale.x = bag_d
        env.scale.y = bag_d
        env.scale.z = float(grasp_3d.suggested_travel_m)
        env.color.r, env.color.g, env.color.b, env.color.a = 0.2, 0.7, 0.9, 0.22
        out.append(env)
        cyl = _mk(12, Marker.CYLINDER)
        cyl.ns = 'tool_swept_volume'
        cyl.pose.position = _point(mid)
        if grasp_3d.orientation is not None:
            cyl.pose.orientation = _rotation_to_quat(grasp_3d.orientation)
        diam = float(tool_d_inner)
        cyl.scale.x = diam
        cyl.scale.y = diam
        cyl.scale.z = float(grasp_3d.suggested_travel_m)
        cyl.color.a = 0.12
        out.append(cyl)

    prior_kind = str(getattr(result, 'target_kind', '') or '')
    info = grasp_3d.diagnostic_info or {}
    prior_r = float(info.get('fruit_prior_radius_m') or 0.0)
    if ((prior_kind in ('fruit', 'sphere') or prior_r > 0)
            and grasp_3d.bag_bottom is not None and grasp_3d.bag_neck is not None):
        sphere = _mk(3, Marker.SPHERE)
        sphere.ns = 'prior'
        center = 0.5 * (np.asarray(grasp_3d.bag_bottom) + np.asarray(grasp_3d.bag_neck))
        radius = prior_r if prior_r > 0 else _metric(
            result.metrics or {}, 'fruit_radius_m', 0.0)
        if radius > 0:
            sphere.pose.position = _point(center)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * radius)
            sphere.color.a = 0.3
            out.append(sphere)

    if grasp_3d.entry_start is not None and grasp_3d.orientation is not None:
        # 三轴架
        R = np.asarray(grasp_3d.orientation)
        origin = np.asarray(grasp_3d.entry_start, dtype=float)
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
    # 三态由 _mk 的状态色表达（ACCEPT 绿/REOBSERVE 黄/REJECT 红），文字只留 ID
    text.text = tid
    if grasp_3d.entry_start is not None:
        text.pose.position = _point(grasp_3d.entry_start)
    elif grasp_3d.bag_bottom is not None:
        text.pose.position = _point(grasp_3d.bag_bottom)
    out.append(text)
    return out


__all__ = [
    'TRACKING_STATUS_TO_MSG',
    'bbox_cloud_xyzrgb',
    'best_axis_direction',
    'quat_to_msg',
    'to_candidate',
    'to_candidate_2d',
    'to_detection2d',
    'to_fitting',
    'to_markers',
    'xyzrgb_to_cloud_msg',
]
