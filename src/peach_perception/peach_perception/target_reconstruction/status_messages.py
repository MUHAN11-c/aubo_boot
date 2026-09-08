from __future__ import annotations
"""重建状态消息：diagnostics/状态三件套的消息组装。"""


from geometry_msgs.msg import Point, Vector3
from peach_interfaces.msg import GraspDecision, ReconstructionStatus

# 无效标量约定（沿用 BagFitting 的 -1 惯例，消费方把 <0 视为"无数据"）
INVALID_SCALAR = -1.0
# 未绑定目标的 target_center_base 占位
INVALID_CENTER = (-1.0, -1.0, -1.0)


def _scalar_or_invalid(value) -> float:
    """可选标量 → float；None 折成无效值 -1."""
    return INVALID_SCALAR if value is None else float(value)


def _vec_or(value, fallback):
    """向量字段：仅 None 才回退。ndarray 不能写 `a or b`（真值歧义）."""
    return fallback if value is None else value


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


def _fill_decision_geometry(msg, decision: dict) -> None:
    """融合成功时写入入口/轴/剪切参考；与 allowed 无关."""
    entry = _vec_or(decision.get('entry'), (0.0, 0.0, 0.0))
    axis = _vec_or(decision.get('axis'), (0.0, 0.0, 0.0))
    pregrasp = _vec_or(decision.get('pregrasp'), entry)
    cut_pose = _vec_or(decision.get('cut_pose'), entry)
    msg.entry = Point(
        x=float(entry[0]), y=float(entry[1]), z=float(entry[2]))
    msg.pregrasp = Point(
        x=float(pregrasp[0]), y=float(pregrasp[1]), z=float(pregrasp[2]))
    msg.cut_pose = Point(
        x=float(cut_pose[0]), y=float(cut_pose[1]), z=float(cut_pose[2]))
    msg.axis = Vector3(
        x=float(axis[0]), y=float(axis[1]), z=float(axis[2]))
    msg.diameter_m = float(decision.get('diameter_m') or 0.0)
    msg.d95_m = float(decision.get('d95_m') or msg.diameter_m)
    msg.travel_m = float(decision.get('travel_m') or 0.0)
    msg.cut_travel_m = float(decision.get('cut_travel_m') or 0.0)
    msg.radial_margin_m = float(decision.get('radial_margin_m') or 0.0)
    msg.axial_margin_m = float(decision.get('axial_margin_m') or 0.0)
    msg.corridor_clear = bool(decision.get('corridor_clear', False))
    msg.rmse_m = _scalar_or_invalid(decision.get('rmse_m'))
    msg.inlier_ratio = _scalar_or_invalid(decision.get('inlier_ratio'))


def grasp_decision_to_msg(decision: dict, header) -> GraspDecision:
    """
    _grasp_decision() 的 dict → GraspDecision（闩锁覆盖语义）.

    融合成功时写入入口/轴/剪切参考，供预抓取与目视。allowed 只表示
    套入/剪切接触许可；false 时几何仍有效，禁止据此降级接触。
    无几何时入口/轴保持零、标量填 0/-1.

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
    msg.failure_code = int(decision.get('failure_code') or 0)
    msg.model_revision = str(decision.get('model_revision') or '')
    msg.tool_profile_id = str(decision.get('tool_profile_id') or '')
    if decision.get('geometry_valid'):
        _fill_decision_geometry(msg, decision)
    else:
        msg.diameter_m = 0.0
        msg.d95_m = 0.0
        msg.rmse_m = INVALID_SCALAR
        msg.inlier_ratio = INVALID_SCALAR
    return msg
