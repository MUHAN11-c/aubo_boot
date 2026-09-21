"""
重建发布：诊断状态、点云节流、Marker、TargetModel/预抓取组装.

W4：session 文件 IO（save_session/PLY/yaml writer）迁 session_recorder；
PublisherMixin 方法本体迁 reconstruction_core.ReconstructionCore（本文件
留过渡薄壳）；``_fill_target_model``/``_pregrasp_verification_msg``/
``_lookup_tool_frame`` 的组装本体自节点下沉为公开函数.
"""
from __future__ import annotations

import time
from typing import (
    Callable,
    Dict,
    Hashable,
    List,
    Optional,
    Tuple,
)

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, Vector3
import numpy as np
from peach_harvester.vision.common.geometry import (
    pack_rgb_bgr,
    rotation_to_quat,
    transform_msg_to_matrix,
)
from peach_harvester.vision.domain.model_contract import (
    allowed_from_capabilities,
    capabilities_from_decision,
)
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    evaluate_pregrasp,
    RefitResult,
    STATUS_ACCEPT,
    STATUS_REJECT,
    STATUS_REOBSERVE,
)
from peach_interfaces.msg import (
    GraspDecision,
    PregraspVerification,
    ReconstructionStatus,
)
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray


class PublishThrottle:
    """
    on-change + 最小间隔发布节流器（按话题独立记账）.

    生命周期：随节点构造创建、全程复用；min_interval_s<=0 时间隔门失效
    （只留 on-change），on-change 本身的总开关（publish.on_change_only）
    在编排层判定，不经本类。
    """

    def __init__(self, min_interval_s: float = 0.2,
                 now: Optional[Callable] = None):
        """保存最小间隔与注入时钟；now 为单调秒，None 则用 time.perf_counter."""
        self._min_interval = max(0.0, float(min_interval_s))
        self._now = now if now is not None else time.perf_counter
        # 每话题最近一次「实际发布」的版本 key 与时刻；未发布过无记录
        self._published_key: Dict[str, Hashable] = {}
        self._published_at: Dict[str, float] = {}

    def should_publish(self, topic: str, key: Hashable,
                       force: bool = False) -> bool:
        """
        判定本次是否真正发布.

        Args:
        ----
            topic: 话题标识（记账键，用固定字符串如 'local_cloud'）.
            key: 内容版本 key（可哈希；内容未变须相等，变了须不等——由
                调用方用帧数/版本号等廉价标量组元组，不做内容哈希）.
            force: True 绕过 on-change 与间隔门（产物清空同步事件）.

        Returns
        -------
            True=立即发布（并记录 key 与时刻）；False=抑制（不记录，
            变化留待下次调用补发）.

        """
        if not force:
            if topic in self._published_key \
                    and self._published_key[topic] == key:
                return False  # 零变化：抑制（闩锁保留最后一帧）
            last = self._published_at.get(topic)
            if last is not None and self._now() - last < self._min_interval:
                return False  # 间隔内抑制：不记 key，下次调用仍判为已变化
        self._published_key[topic] = key
        self._published_at[topic] = self._now()
        return True

    def reset(self) -> None:
        """清空全部记账（节点复位/测试隔离用；现状无调用方，接口备用）."""
        self._published_key.clear()
        self._published_at.clear()


# 无效标量约定（沿用 BagFitting 的 -1 惯例，消费方把 <0 视为"无数据"）
INVALID_SCALAR = -1.0
# 未绑定目标的 target_center_base 占位
INVALID_CENTER = (-1.0, -1.0, -1.0)
# G1（2026-09-20）：接触许可有效窗 fallback 默认——真实窗口走参数
# decision.validity_s（config/target_reconstruction.yaml，部署默认 120.0），
# 仅当快照缺该键（旧参数对象/直接调用的测试）时兜底。依据：须覆盖
# finalize→套入全链（真机单 LIN 7.5s、FULL 链 30-60s），模型陈旧性主要
# 由 revision 单调性把关，窗口是第二道界。
MODEL_VALIDITY_S = 120.0


def decision_validity_s(params) -> float:
    """
    读 ``decision.validity_s``；缺键（旧快照）回退 ``MODEL_VALIDITY_S``.

    G1：唯一取窗入口——``_lock_decision_validity`` 冻结处、
    ``fill_target_model`` 与 ``grasp_decision_to_msg`` 兜底共用，不另开
    第二套传参。
    """
    try:
        return float(params.decision.validity_s)
    except AttributeError:
        return MODEL_VALIDITY_S


def _time_plus(stamp, extra_s: float) -> Time:
    """Header stamp + extra_s → builtin Time（不回写 stamp，避免心跳续签）."""
    total = float(stamp.sec) + float(stamp.nanosec) * 1e-9 + float(extra_s)
    out = Time()
    out.sec = int(total)
    frac = total - float(out.sec)
    if frac < 0.0:
        out.sec -= 1
        frac += 1.0
    out.nanosec = int(frac * 1e9)
    return out


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


def grasp_decision_to_msg(decision: dict, header,
                          validity_s: float = MODEL_VALIDITY_S) -> GraspDecision:
    """
    _grasp_decision() 的 dict → GraspDecision（闩锁覆盖语义）.

    融合成功时写入入口/轴/剪切参考，供预抓取与目视。allowed 只表示
    套入/剪切接触许可；false 时几何仍有效，禁止据此降级接触。
    无几何时入口/轴保持零、标量填 0/-1.
    validity_s：dict 未携带冻结 valid_until 时的兜底窗（G1 参数化，
    调用方传 decision_validity_s(params)；常量为旧快照 fallback）.

    Args:
        decision: _grasp_decision() 返回的许可 dict.
        header: std_msgs/Header（stamp=发布时刻，frame_id=base_frame）.
        validity_s: 兜底有效窗 [s]（G1；默认 MODEL_VALIDITY_S=120.0）.

    Returns
    -------
        peach_interfaces/GraspDecision.

    """
    msg = GraspDecision()
    msg.header = header
    msg.harvest_run_id = str(decision.get('harvest_run_id') or '')
    msg.target_id = str(decision.get('target_id') or '')
    msg.model_revision = str(decision.get('model_revision') or '')
    msg.tool_profile_id = str(decision.get('tool_profile_id') or '')
    msg.scene_epoch = int(decision.get('scene_epoch') or 0)
    msg.calibration_revision = str(decision.get('calibration_revision') or '')
    msg.config_revision = str(decision.get('config_revision') or '')
    geometry, pregrasp, sleeve, cut = capabilities_from_decision(decision)
    msg.geometry_capability = geometry
    msg.pregrasp_capability = pregrasp
    msg.sleeve_capability = sleeve
    msg.cut_capability = cut
    valid_until = decision.get('valid_until')
    if valid_until is not None:
        msg.valid_until = valid_until
    else:
        msg.valid_until = _time_plus(header.stamp, float(validity_s))
    # M11：allowed 与重建诊断 dict 侧（reconstruction_core._grasp_decision）
    # 同源——能力提取/许可判定单源在 model_contract，两边互指本行。
    msg.allowed = allowed_from_capabilities(geometry, pregrasp, sleeve, cut)
    msg.reason = str(decision.get('reason') or '')
    msg.failure_code = int(decision.get('failure_code') or 0)
    if decision.get('geometry_valid'):
        _fill_decision_geometry(msg, decision)
    else:
        msg.diameter_m = 0.0
        msg.d95_m = 0.0
        msg.rmse_m = INVALID_SCALAR
        msg.inlier_ratio = INVALID_SCALAR
    return msg


_MARKER_NS = 'target_reconstruction'
_REFINED_NS = 'peach_reconstruction/refined'  # refined 轴箭头独立 namespace
_MESH_NS = 'peach_reconstruction/tsdf_mesh'


def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    """
    组 ColorRGBA（强制 float，防 rosidl 类型断言）.

    Args:
        r: 红 [0, 1].
        g: 绿 [0, 1].
        b: 蓝 [0, 1].
        a: 透明度 [0, 1].

    Returns
    -------
        std_msgs/ColorRGBA.

    """
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _point_msg(xyz) -> Point:
    """
    (3,) 坐标 [m] → Point 消息.

    Args:
        xyz: 可转 float 的三元坐标.

    Returns
    -------
        geometry_msgs/Point.

    """
    p = Point()
    p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    return p


def _new_marker(header, mid: int, mtype: int) -> Marker:
    """
    新建带公共字段的 Marker（ns 固定，action=ADD，姿态单位四元数）.

    Args:
        header: std_msgs/Header.
        mid: Marker id.
        mtype: Marker 类型枚举.

    Returns
    -------
        初始化后的 Marker.

    """
    m = Marker()
    m.header = header
    m.ns = _MARKER_NS
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def build_camera_markers(header, frames: List) -> MarkerArray:
    """
    由已采帧构造相机轨迹 MarkerArray.

    Args:
        header: std_msgs/Header（frame_id=base_frame，stamp 由调用方给）.
        frames: CapturedFrame 列表（读 camera_position_base 与 T_base_camera）.

    Returns
    -------
        MarkerArray：DELETEALL + SPHERE_LIST（位置） + LINE_LIST（连线） +
        每帧一个 ARROW（朝向，相机 +Z 在 base 系方向，长 0.05 m）.

    """
    arr = MarkerArray()
    # DELETEALL 不设 ns/id：否则与首个 ADD 冲突（同 peach_scene_perception_node 的 RViz 坑）
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)

    positions = [np.asarray(f.camera_position_base, dtype=np.float64)
                 for f in frames if f.camera_position_base is not None]
    if not positions:
        return arr

    spheres = _new_marker(header, 1, Marker.SPHERE_LIST)
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.012  # 点径 [m]
    spheres.color = _color(0.1, 0.9, 0.2, 0.9)
    spheres.points = [_point_msg(p) for p in positions]
    arr.markers.append(spheres)

    if len(positions) >= 2:
        lines = _new_marker(header, 2, Marker.LINE_LIST)
        lines.scale.x = 0.004  # 线宽 [m]
        lines.color = _color(0.95, 0.85, 0.1, 0.9)
        seg_points = []
        for i in range(len(positions) - 1):
            seg_points.append(_point_msg(positions[i]))
            seg_points.append(_point_msg(positions[i + 1]))
        lines.points = seg_points
        arr.markers.append(lines)

    for i, f in enumerate(frames):
        if f.camera_position_base is None:
            continue
        R = np.asarray(f.T_base_camera, dtype=np.float64)[:3, :3]
        view_dir = R @ np.array([0.0, 0.0, 1.0])  # 相机光轴 +Z 在 base 系方向
        start = np.asarray(f.camera_position_base, dtype=np.float64)
        arrow = _new_marker(header, 10 + i, Marker.ARROW)
        arrow.scale.x = 0.004   # 杆径 [m]
        arrow.scale.y = 0.010   # 箭头径 [m]
        arrow.scale.z = 0.010   # 箭头长 [m]
        arrow.color = _color(0.2, 0.8, 0.95, 0.9)
        arrow.points = [_point_msg(start), _point_msg(start + 0.05 * view_dir)]
        arr.markers.append(arrow)
    return arr


def _status_rgba(status: int):
    """ACCEPT 绿 / REOBSERVE 黄 / REJECT 红 / 其他灰（与感知 Marker 同三态）."""
    return {
        STATUS_ACCEPT: (0.1, 0.85, 0.2, 0.9),
        STATUS_REOBSERVE: (0.95, 0.8, 0.1, 0.9),
        STATUS_REJECT: (0.9, 0.15, 0.15, 0.9),
    }.get(int(status), (0.6, 0.6, 0.6, 0.8))


def _grasp_rotation(axis) -> np.ndarray:
    """抓取架：Z=推进轴（bottom→neck），X 取与轴不平行的参考叉积."""
    z_axis = np.asarray(axis, dtype=np.float64).reshape(3)
    norm = np.linalg.norm(z_axis)
    if norm < 1e-9:
        return np.eye(3, dtype=np.float64)
    z_axis = z_axis / norm
    ref = np.array([0.0, 0.0, 1.0]) if abs(z_axis[2]) < 0.9 else np.array(
        [1.0, 0.0, 0.0])
    x_axis = np.cross(ref, z_axis)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-9:
        x_axis = np.array([1.0, 0.0, 0.0])
    else:
        x_axis = x_axis / x_norm
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / max(np.linalg.norm(y_axis), 1e-12)
    return np.column_stack((x_axis, y_axis, z_axis))


def _quat_from_rotation(rotation: np.ndarray) -> Quaternion:
    """3×3 旋转 → geometry_msgs/Quaternion（xyzw）."""
    value = rotation_to_quat(rotation)
    msg = Quaternion()
    msg.x = float(value.x)
    msg.y = float(value.y)
    msg.z = float(value.z)
    msg.w = float(value.w)
    return msg


def build_refined_grasp_markers(
    header, refined: Optional[RefitResult], target_id: str = '',
    tool_d_inner: float = 0.104,
) -> List[Marker]:
    """
    精化结果 → 与感知同款抓取示意（ns=peach_reconstruction/refined）.

    袋轴用拟合底/颈；半透明圆柱直径用工具内径（与感知 Marker 同，
    不是袋径）；行程从 entry 画到颈。文字放在颈上方，避免叠在入口架上。
    W4：入参改 RefitResult（属性访问；键语义与旧 dict 版一致）。
    """
    if not refined or not refined.ok:
        return []
    bottom = np.asarray(refined.bottom, dtype=np.float64)
    neck = np.asarray(refined.neck, dtype=np.float64)
    axis = np.asarray(refined.axis, dtype=np.float64)
    entry = np.asarray(refined.entry, dtype=np.float64)
    diameter = float(refined.diameter)
    radius = 0.5 * diameter
    rotation = _grasp_rotation(axis)
    cut = refined.cut_pose if refined.cut_pose is not None \
        else refined.cut_plane_point
    if cut is None:
        cut = neck
    cut = np.asarray(cut, dtype=np.float64)
    to_cut = float(np.dot(cut - entry, axis))
    travel = to_cut if to_cut > 1e-6 else float(
        refined.cut_travel_m or refined.span_m)
    travel_end = entry + travel * axis
    red, green, blue, alpha = _status_rgba(refined.status)
    out: List[Marker] = []

    def _mk(mid: int, mtype: int) -> Marker:
        marker = _new_marker(header, mid, mtype)
        marker.ns = _REFINED_NS
        marker.color = _color(red, green, blue, alpha)
        return marker

    axis_line = _mk(0, Marker.LINE_LIST)
    axis_line.scale.x = 0.004
    axis_line.points = [_point_msg(bottom), _point_msg(neck)]
    out.append(axis_line)

    if travel > 1e-6:
        arrow = _mk(1, Marker.ARROW)
        arrow.scale.x = 0.008
        arrow.scale.y = 0.015
        arrow.scale.z = 0.015
        arrow.points = [_point_msg(entry), _point_msg(travel_end)]
        out.append(arrow)
        cyl = _mk(2, Marker.CYLINDER)
        mid = entry + 0.5 * travel * axis
        cyl.pose.position = _point_msg(mid)
        cyl.pose.orientation = _quat_from_rotation(rotation)
        diam = float(tool_d_inner) if tool_d_inner > 1e-6 else 0.104
        cyl.scale.x = diam
        cyl.scale.y = diam
        cyl.scale.z = travel
        cyl.ns = _REFINED_NS + '/tool_swept_volume'
        cyl.color.a = 0.12
        out.append(cyl)
        env = _mk(12, Marker.CYLINDER)
        env.ns = _REFINED_NS + '/bag_envelope'
        env.pose.position = _point_msg(mid)
        env.pose.orientation = _quat_from_rotation(rotation)
        bag_d = diameter if diameter > 1e-6 else 0.06
        env.scale.x = bag_d
        env.scale.y = bag_d
        env.scale.z = travel
        env.color = _color(0.2, 0.7, 0.9, 0.22)
        out.append(env)

    kind = str(refined.kind)
    prior_r = float(refined.fruit_prior_radius_m or 0.0)
    if kind in ('fruit', 'sphere') or prior_r > 1e-6:
        sphere = _mk(3, Marker.SPHERE)
        sphere.ns = 'prior'
        sphere.pose.position = _point_msg(0.5 * (bottom + neck))
        draw_r = prior_r if prior_r > 1e-6 else radius
        sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * draw_r)
        sphere.color.a = 0.3
        out.append(sphere)

    origin = entry
    frame_colors = (
        (1.0, 0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0, 1.0),
        (0.0, 0.0, 1.0, 1.0),
    )
    for axis_i, col in enumerate(frame_colors):
        axis_m = _mk(4 + axis_i, Marker.ARROW)
        axis_m.scale.x = 0.005
        axis_m.scale.y = 0.01
        axis_m.scale.z = 0.01
        axis_m.color = _color(*col)
        end = origin + 0.05 * rotation[:, axis_i]
        axis_m.points = [_point_msg(origin), _point_msg(end)]
        out.append(axis_m)

    text = _mk(10, Marker.TEXT_VIEW_FACING)
    text.scale.z = 0.03
    suffix = 'final' if refined.final else 'live'
    tid = target_id or str(refined.kind or 'refit')
    text.text = f'{tid} {suffix}'
    text.pose.position = _point_msg(neck + np.array([0.0, 0.0, 0.04]))
    out.append(text)
    cut_mark = _mk(11, Marker.SPHERE)
    cut_mark.ns = _REFINED_NS + '/cut'
    cut_mark.pose.position = _point_msg(cut)
    cut_mark.scale.x = cut_mark.scale.y = cut_mark.scale.z = 0.018
    cut_mark.color = _color(0.72, 0.20, 0.90, 0.95)
    out.append(cut_mark)
    return out


def build_mesh_marker(header, mesh_data: Optional[dict],
                      max_triangles: int = 50000) -> Optional[Marker]:
    """把 TSDF 三角网格转换为 RViz TRIANGLE_LIST；过大时均匀抽取."""
    if not mesh_data:
        return None
    vertices = np.asarray(mesh_data.get('vertices', []), dtype=np.float64)
    triangles = np.asarray(mesh_data.get('triangles', []), dtype=np.int64)
    if not len(vertices) or not len(triangles):
        return None
    if len(triangles) > max_triangles:
        indices = np.linspace(
            0, len(triangles) - 1, max_triangles, dtype=np.int64)
        triangles = triangles[indices]
    marker = _new_marker(header, 0, Marker.TRIANGLE_LIST)
    marker.ns = _MESH_NS
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color = _color(0.2, 0.75, 0.95, 0.75)
    # 花式索引一次展开全部三角形顶点（与双层循环同序：三角形序 → 顶点序）
    marker.points = [_point_msg(v) for v in vertices[triangles.reshape(-1)]]
    return marker


def xyzrgb_to_cloud_msg(xyz: np.ndarray, colors_bgr,
                        header: Header) -> PointCloud2:
    """
    (N, 3) 点 [m] + (N, 3) uint8 BGR → PointCloud2（xyz + 位打包 rgb 字段）.

    布局与感知侧 _xyzrgb_to_cloud_msg 一致：x/y/z/rgb 各一个 FLOAT32
    （offset 0/4/8/12，point_step=16），rgb 位内容为 0xRRGGBB（RViz RGB8
    上色约定）；colors_bgr 为 None 或长度不符时 rgb 字段补零（黑色），
    空云/启动首发同样保持该字段布局。消息组装用官方
    sensor_msgs_py.point_cloud2.create_cloud（numpy 结构化数组快速路径，
    逐字节布局与旧手写 tobytes 版一致）；唯一覆写 is_dense=True——
    create_cloud 硬编码 False，而本云已剔无效深度恒 dense。
    pack_rgb_bgr 无官方等价物（RViz float32 位打包），保留。

    Args:
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常 [m]）；空给空云.
        colors_bgr: (N, 3) uint8 BGR 颜色（OpenCV 排列）；None 补零.
        header: 输出消息头（frame_id 决定点云坐标系解释）.

    Returns
    -------
        sensor_msgs/PointCloud2（is_dense=True，反投影已剔除无效深度）.

    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = int(pts.shape[0])
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, :3] = pts
    if colors_bgr is not None and len(colors_bgr) == n:
        arr[:, 3] = pack_rgb_bgr(colors_bgr)
    msg = create_cloud(header, fields, arr)
    msg.is_dense = True  # create_cloud 硬编码 False；本云恒 dense（无效已剔）
    return msg


def lookup_tool_frame(tf_buffer, base_frame: str, child: str):
    """
    Latest TF: base <- child; (None, None) on failure.

    自节点 ``_lookup_tool_frame`` 下沉（W4）。预抓取验证的工具三帧
    （sleeve_mouth/tool_axis/cutting_plane）只有 1 s 级 latest TF 可查
    （tf_buffer 无这些帧的精确 stamp 数据源），显式 latest 豁免以函数名
    与本注释保留——不得用于采帧/积分路径（那两处必须精确 stamp）。

    Args:
        tf_buffer: tf2_ros.Buffer（节点持有）.
        base_frame: 基座系名.
        child: 工具子帧名.

    Returns
    -------
        (position, z_axis)：(3,) base 系平移与旋转矩阵第三列（工具轴
        方向）；查询失败给 (None, None).

    """
    try:
        tf = tf_buffer.lookup_transform(base_frame, child, Time())
    except TransformException:
        return None, None
    T = transform_msg_to_matrix(tf.transform)
    return T[:3, 3].copy(), T[:3, 2].copy()


def build_pregrasp_verification(
        header, fused: Optional[BagModel], refined: Optional[RefitResult],
        tool_frames, previous, params) -> Tuple[PregraspVerification,
                                                Optional[dict]]:
    """
    工具帧相对融合袋模型的预抓取残差（观测用，不授权 SetIO）.

    自节点 ``_pregrasp_verification_msg`` 的组装段下沉（W4；R3 语义保持：
    状态快照与 ``_pregrasp_prev`` 锁内交换留在节点，本函数纯组装）。

    Args:
        header: 输出头（frame_id=base_frame）.
        fused: 融合袋模型缓存 _bag_model（None=未产出）.
        refined: refit 结果缓存 _refined（取 model_revision）.
        tool_frames: latest TF 三帧查询结果 (sleeve_mouth, tool_z_axis,
            cutting_plane)；任一 None 走 tool_tf_missing；传 None 表示
            调用方因袋模型不可用已跳过 TF 查询.
        previous: 上一帧残差行（evaluate_pregrasp 产物）或 None.
        params: TargetReconstructionParams（读 tool.profile_id）.

    Returns
    -------
        (msg, eval_row)：eval_row 为本帧残差 dict（调用方锁内置入
        _pregrasp_prev）；早退路径（袋模型/TF 缺失）给 (msg, None)——
        不得覆盖上一帧比对基点.

    """
    msg = PregraspVerification()
    msg.header = header
    msg.model_revision = (
        '' if refined is None else str(refined.model_revision or ''))
    msg.tool_profile_id = str(params.tool.profile_id)
    if fused is None or not fused.ok:
        msg.reason = 'bag_model_unavailable'
        return msg, None
    if tool_frames is None:
        mouth = tool_z = blade = None
    else:
        mouth, tool_z, blade = tool_frames
    if mouth is None or tool_z is None or blade is None:
        msg.reason = 'tool_tf_missing'
        msg.failure_code = 11
        return msg, None
    cut_pt = fused.cut_plane_point
    if cut_pt is None:
        cut_pt = fused.cut_pose
    if cut_pt is None:
        cut_pt = fused.neck
    eval_row = evaluate_pregrasp(
        tool_z, fused.axis, mouth, fused.bottom,
        blade, cut_pt,
        float(fused.radial_margin_m or 0.0),
        float(fused.axial_margin_m or 0.0),
        previous=previous)
    msg.frames_consistent = bool(eval_row['frames_consistent'])
    msg.axis_angle_deg = float(eval_row['axis_angle_deg'])
    msg.lateral_error_m = float(eval_row['lateral_error_m'])
    msg.axial_error_m = float(eval_row['axial_error_m'])
    msg.radial_margin_m = float(eval_row['radial_margin_m'])
    msg.axial_margin_m = float(eval_row['axial_margin_m'])
    msg.needs_correction = bool(eval_row['needs_correction'])
    msg.passed = bool(eval_row['passed'])
    msg.failure_code = int(eval_row['failure_code'])
    msg.reason = str(eval_row['reason'])
    return msg, eval_row


def fill_target_model(model, fused: Optional[BagModel],
                      refined: Optional[RefitResult], params, now,
                      run_id: str = '', scene_epoch: int = 0) -> None:
    """
    把融合袋模型写入 TargetModel 扩展字段.

    自节点 ``_fill_target_model`` 下沉（W4；字段与协方差手拼原样）。

    Args:
        model: BuildTargetModel.Result.model（调用方已填 target_id/
            scene_epoch/accepted 等）.
        fused: 融合袋模型缓存 _bag_model（None 时能力全置 2=未知）.
        refined: refit 结果缓存 _refined（取 model_revision）.
        params: TargetReconstructionParams（tool 档案与版本）.
        now: rclpy Time（节点时钟；generated_at/valid_until 基准）.
        run_id: 当前 harvest_run_id.
        scene_epoch: 当前批次 scene_epoch（0 保持调用方已填值）.

    Returns
    -------
        无返回值（None）；model 原地填充.

    """
    fused = fused if fused is not None and fused.ok else None
    model.model_revision = (
        '' if refined is None else str(refined.model_revision or ''))
    model.tool_profile_id = str(params.tool.profile_id)
    model.run_id = str(run_id or '')
    if scene_epoch:
        model.scene_epoch = int(scene_epoch)
    cal = str(getattr(params, 'calibration_version', '') or 'unspecified')
    model.calibration_version = cal
    model.calibration_revision = cal
    model.config_revision = str(
        getattr(params.tool, 'version', '') or params.tool.profile_id)
    model.generated_at = now.to_msg()
    model.header.stamp = model.generated_at
    # G1：TargetModel 有效窗与 GraspDecision 同参数同源（原各自硬编码 5s，
    # 与接近链时长错配；见 decision_validity_s 注释）。
    model.valid_until = (
        now + Duration(seconds=decision_validity_s(params))).to_msg()
    model.capture_start = model.generated_at
    model.capture_end = model.generated_at
    fused_ok = fused is not None
    model.geometry_capability = 0 if fused_ok else 2
    model.pregrasp_capability = model.geometry_capability
    budget = fused.budget if fused_ok else {}
    if budget:
        model.sleeve_capability = int(
            budget.get('sleeve_capability', 0 if budget.get('sleeve_ok') else 1))
        model.cut_capability = int(
            budget.get('cut_capability', 0 if budget.get('cut_ok') else 1))
    else:
        model.sleeve_capability = 2
        model.cut_capability = 2
    if not fused_ok:
        return
    if fused.bottom is not None:
        model.bag_bottom = Point(
            x=float(fused.bottom[0]), y=float(fused.bottom[1]),
            z=float(fused.bottom[2]))
    if fused.neck is not None:
        model.bag_neck = Point(
            x=float(fused.neck[0]), y=float(fused.neck[1]),
            z=float(fused.neck[2]))
    cut_pt = fused.cut_plane_point
    if cut_pt is None:
        cut_pt = fused.cut_pose if fused.cut_pose is not None else fused.neck
    if cut_pt is not None:
        model.cut_plane_point = Point(
            x=float(cut_pt[0]), y=float(cut_pt[1]), z=float(cut_pt[2]))
    if fused.axis is not None:
        model.bag_axis = Vector3(
            x=float(fused.axis[0]), y=float(fused.axis[1]),
            z=float(fused.axis[2]))
        model.cut_normal = model.bag_axis
    model.d95_m = float(fused.d95_m or 0.0)
    model.fruit_prior_radius_m = float(fused.fruit_prior_radius_m or 0.0)
    model.fruit_prior_auxiliary = True
    model.radial_margin_m = float(fused.radial_margin_m or 0.0)
    model.axial_margin_m = float(fused.axial_margin_m or 0.0)
    model.corridor_clear = bool(fused.corridor_clear)
    model.occlusion_class = str(fused.occlusion_class or '')
    sig = float(fused.sigma_position_m or 0.02)
    pos_cov = [0.0] * 9
    pos_cov[0] = pos_cov[4] = pos_cov[8] = sig * sig
    model.bottom_covariance = pos_cov
    model.neck_covariance = pos_cov
    sig_axis = float(fused.sigma_axis_deg or 8.0)
    axis_rad = sig_axis * 3.141592653589793 / 180.0
    axis_cov = [0.0] * 9
    axis_cov[0] = axis_cov[4] = axis_cov[8] = axis_rad * axis_rad
    model.axis_covariance = axis_cov


class PublisherMixin:
    """
    过渡薄壳（W4）：方法本体已迁 reconstruction_core.ReconstructionCore.

    本类保留名称与文件位置一个提交期（node 经 MRO 从 Core 取全部发布
    方法）；与 FrameStoreMixin/AutoControllerMixin 同批收口后删除。
    """
