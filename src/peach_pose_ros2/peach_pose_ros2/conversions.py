"""
消息组装 — 算法侧 dataclass / 检测 dict → ROS 消息的纯转换层.

职责:
  把管线输出（contracts.BagGraspReference3D / BagGrasp2D /
  pipeline.TargetPoseResult / engine.detect 的检测 dict）组装成
  peach_pose_msgs / vision_msgs 消息。依赖 ROS 消息类型，不依赖 rclpy 节点；
  节点侧需要的版本号 / logger 由调用方以参数传入。

坐标系/单位约定:
  3D 量坐标系一律跟随 header.frame_id（由节点决定，默认 output_frame
  =base_link，米制）；2D 量为像素坐标。None 点给零值 Point 且对应
  has_*=False；metrics 缺项填 -1（BagFitting 无效约定）。
"""
from __future__ import annotations

from geometry_msgs.msg import Point, Pose, Vector3
from peach_pose_msgs.msg import (
    BagFitting,
    BagGrasp2D as BagGrasp2DMsg,
    BagGraspCandidate,
)
from peach_pose_ros2.grasp_tf import _rotation_to_quat
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

# 三态安全门控结果 → ROS 消息枚举的映射（与 peach_pose_msgs/BagGraspCandidate.status 一致）。
# 算法管线内部用字符串状态，发布消息时经本表转成 uint8：
#   ACCEPT=0    可信：无任何诊断标记，位姿可直接用于套袋动作
#   REOBSERVE=1 存疑：信息不足（如掩膜缺失、轴来自重力先验、触边截断等），
#               建议换个视角重采一帧再判，不建议直接动作
#   REJECT=2    不可用：存在硬性失败（如 tool_clearance_failed 净空不足、
#               有效点太少等），禁止据此位姿动作
STATUS_MAP = {'ACCEPT': 0, 'REOBSERVE': 1, 'REJECT': 2}


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


def _to_detection2d(det: dict, header) -> Detection2D:
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


def _to_candidate(header, tid, g3d, model_version: str,
                  calibration_version: str, tool_version: str) -> BagGraspCandidate:
    """
    3D 抓取参考 → BagGraspCandidate 主输出消息（坐标系=header.frame_id）.

    Args:
        header: 输出头；frame_id 即 g3d 当前所在坐标系.
        tid: 目标 ID（target_N）.
        g3d: BagGraspReference3D（米；None 字段在消息中给零/缺省）.
        model_version: 模型版本回退值（g3d 自带时优先）.
        calibration_version: 内外参版本回退值（g3d 自带时优先）.
        tool_version: 工具版本回退值（g3d 自带时优先）.

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
    m.model_version = g3d.model_version or model_version
    m.calibration_version = g3d.calibration_version or calibration_version
    m.tool_version = g3d.tool_version or tool_version
    return m


def _to_candidate_2d(header, tid, g2d) -> BagGrasp2DMsg:
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


def _to_fitting(header, tid, result) -> BagFitting:
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
