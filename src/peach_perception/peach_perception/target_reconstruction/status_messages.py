"""
诊断/抓取许可 dict → 类型化消息的纯组装（零 rclpy）.

为什么独立成模块：消息组装是无副作用纯函数，放这里让契约测试不必拉起
节点本体（rclpy/cv_bridge/open3d 重依赖）；无效值约定（标量 -1、
target_center_base 全 -1、allowed=false 时几何清零、覆盖无效时方向留空）
在此单点固化，发布侧与全部消费方按同一约定解读。
"""
from geometry_msgs.msg import Point, Vector3
from peach_interfaces.msg import GraspDecision, ReconstructionStatus

# 无效标量约定（沿用 BagFitting 的 -1 惯例，消费方把 <0 视为"无数据"）
INVALID_SCALAR = -1.0
# 未绑定目标的 target_center_base 占位
INVALID_CENTER = (-1.0, -1.0, -1.0)


def _scalar_or_invalid(value) -> float:
    """可选标量 → float；None 折成无效值 -1."""
    return INVALID_SCALAR if value is None else float(value)


def diagnostics_to_status_msg(diag: dict,
                              header) -> ReconstructionStatus:
    """
    _diagnostics() 的完整 dict → ReconstructionStatus（结构化核心子集）.

    视角覆盖有效（view_coverage.valid）时取机位聚类口径的均值/基线指标与
    逐机位"目标→相机"方向；覆盖无效时基线填 -1、方向留空（闩锁覆盖语义，
    与 refined_pose 发空数组一致），valid_depth_ratio 回退最近帧值以区分
    「未采帧（-1）」与「覆盖无效但有帧」。

    Args:
        diag: _diagnostics() 返回的完整诊断 dict.
        header: std_msgs/Header（stamp=发布时刻，frame_id=base_frame）.

    Returns
    -------
        peach_interfaces/ReconstructionStatus.

    """
    msg = ReconstructionStatus()
    msg.header = header
    msg.harvest_run_id = str(diag.get('harvest_run_id') or '')
    msg.selected_target_id = str(diag.get('selected_target_id') or '')
    msg.state = str(diag.get('state') or '')
    msg.target_id = str(diag.get('target_id') or '')
    center = diag.get('target_center_base')
    if center is None:
        msg.target_center_base = list(INVALID_CENTER)
    else:
        msg.target_center_base = [float(v) for v in center]
    msg.captured_views = int(diag.get('captured_views') or 0)
    msg.rejected_views = int(diag.get('rejected_views') or 0)
    msg.tf_failures = int(diag.get('tf_failures') or 0)
    msg.tf_latency_ms = _scalar_or_invalid(diag.get('tf_latency_ms'))
    coverage = diag.get('view_coverage') or {}
    if coverage.get('valid'):
        msg.valid_depth_ratio = _scalar_or_invalid(
            coverage.get('valid_depth_ratio_mean'))
        msg.max_baseline_deg = _scalar_or_invalid(
            coverage.get('max_baseline_deg'))
        msg.mean_nearest_baseline_deg = _scalar_or_invalid(
            coverage.get('mean_nearest_baseline_deg'))
        for view in coverage.get('views') or []:
            direction = view.get('direction_target_to_camera')
            if direction is None or len(direction) != 3:
                continue  # 退化方向（目标≈相机）不入消息
            msg.view_directions.append(Vector3(
                x=float(direction[0]), y=float(direction[1]),
                z=float(direction[2])))
    else:
        msg.valid_depth_ratio = _scalar_or_invalid(
            diag.get('valid_depth_ratio'))
        msg.max_baseline_deg = INVALID_SCALAR
        msg.mean_nearest_baseline_deg = INVALID_SCALAR
    return msg


def grasp_decision_to_msg(decision: dict, header) -> GraspDecision:
    """
    _grasp_decision() 的 dict → GraspDecision（闩锁覆盖语义）.

    allowed=false 时几何/质量字段无意义：entry/axis 保持零、标量填 0/-1
    占位（等价于 refined_pose 发空数组覆盖），reason 是唯一有效信息——
    防止后启动订阅者经闩锁读到上一轮可用几何时误判可抓。

    Args:
        decision: _grasp_decision() 返回的许可 dict.
        header: std_msgs/Header（stamp=发布时刻，frame_id=base_frame）.

    Returns
    -------
        peach_interfaces/GraspDecision.

    """
    msg = GraspDecision()
    msg.header = header
    msg.harvest_run_id = str(decision.get('harvest_run_id') or '')
    msg.target_id = str(decision.get('target_id') or '')
    msg.allowed = bool(decision.get('allowed'))
    msg.reason = str(decision.get('reason') or '')
    if msg.allowed:
        entry = decision.get('entry') or (0.0, 0.0, 0.0)
        axis = decision.get('axis') or (0.0, 0.0, 0.0)
        msg.entry = Point(
            x=float(entry[0]), y=float(entry[1]), z=float(entry[2]))
        msg.axis = Vector3(
            x=float(axis[0]), y=float(axis[1]), z=float(axis[2]))
        msg.diameter_m = float(decision.get('diameter_m') or 0.0)
        msg.rmse_m = _scalar_or_invalid(decision.get('rmse_m'))
        msg.inlier_ratio = _scalar_or_invalid(decision.get('inlier_ratio'))
    else:
        msg.diameter_m = 0.0
        msg.rmse_m = INVALID_SCALAR
        msg.inlier_ratio = INVALID_SCALAR
    return msg
