"""
消息转换层 — TF/四元数双实现、几何变换、ROS 消息分装.

含 TF 双实现四函数（手写原理版 + 官方 API 对照，一致性固化于
test_tf_matrix.py）、抓取几何坐标变换、Point/metrics 小工具、模型路径解析，
以及内部结果 → peach_pose_msgs / vision_msgs 消息的四个纯函数。
"""
import math
from pathlib import Path

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
import numpy as np
from peach_pose_msgs.msg import (
    BagFitting,
    BagGrasp2D as BagGrasp2DMsg,
    BagGraspCandidate,
)
from std_msgs.msg import Header
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

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
    # 官方库：平移 (x,y,z) → 4×4 齐次矩阵；四元数 (x,y,z,w) → 4×4 旋转矩阵（内部归一化）
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

    T 为 4×4 齐次矩阵（输出系←相机系）。规则：点 R@p+t；方向只乘 R
    并归一化（平移不影响方向）；姿态矩阵左乘 R。None 字段原样保留。

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
    if g3d.translation_direction is not None:
        d = R @ np.asarray(g3d.translation_direction, dtype=float)
        n = float(np.linalg.norm(d))
        g3d.translation_direction = d / n if n > 1e-9 else d
    if g3d.orientation is not None:
        g3d.orientation = R @ np.asarray(g3d.orientation, dtype=float)


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
    # 官方库：4×4 齐次矩阵 → 四元数 numpy [x,y,z,w]（内部归一化）
    q_api = quaternion_from_matrix(m4)
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


def _resolve_model_path(param_value: str, default_name: str,
                        share: Path, logger) -> str:
    """
    解析模型权重路径：相对路径相对包目录，目录下找同名权重，无效告警回退包内.

    Args:
        param_value: 参数值（空、相对/绝对路径或目录）.
        default_name: 回退文件名（如 'best.pt'）.
        share: 包安装目录（相对路径锚点与回退处）.
        logger: 节点 logger.

    Returns
    -------
        可用权重文件路径（无效时回退 share/model/<default_name>）.

    """
    if not param_value:
        return str(share / 'model' / default_name)
    p = Path(param_value)
    p = p if p.is_absolute() else share / p
    candidate = p / default_name if p.is_dir() else p
    if candidate.is_file():
        return str(candidate)
    logger.warning(f'模型路径 {candidate} 无效，回退包内 model/{default_name}')
    return str(share / 'model' / default_name)


def _to_detection2d(det: dict, header: Header) -> Detection2D:
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
                  calibration_version: str, tool_version: str,
                  logger=None) -> BagGraspCandidate:
    """
    3D 抓取参考 → BagGraspCandidate 主输出消息（坐标系=header.frame_id）.

    Args:
        header: 输出头；frame_id 即 g3d 当前所在坐标系.
        tid: 目标 ID（target_N）.
        g3d: BagGraspReference3D（米；None 字段在消息中给零/缺省）.
        model_version: 模型版本标识（g3d 自带为空时的回退）.
        calibration_version: 内外参版本标识（回退同上）.
        tool_version: 工具几何版本（回退同上）.
        logger: 四元数双实现告警出口；None 时 print.

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
        pose.orientation = _rotation_to_quat(g3d.orientation, logger)
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
