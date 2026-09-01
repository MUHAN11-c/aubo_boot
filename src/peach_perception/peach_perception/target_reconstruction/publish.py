from __future__ import annotations
"""重建发布：诊断、Marker、点云节流、session 落盘。"""

from datetime import datetime
import json
from pathlib import Path
import time
from typing import (
    Callable,
    Dict,
    Hashable,
    List,
    Optional,
)

import cv2
from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Vector3,
    Vector3Stamped,
)
import numpy as np
from peach_interfaces.msg import (
    BagFitting,
    BagFittingArray,
    BagGraspCandidate,
    BagGraspCandidateArray,
    GraspDecision,
    ReconstructionStatus,
    ShapeHypothesis,
)
from peach_perception.common.geometry import rotation_to_quat
from peach_perception.target_reconstruction.integrate import (
    pack_rgb_bgr,
    require_open3d,
    summarize_view_coverage,
)
from peach_perception.target_reconstruction.refine import (
    axis_angle_deg,
    STATUS_ACCEPT,
    STATUS_REJECT,
    STATUS_REOBSERVE,
)
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml


# === publish_throttle.py ===

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


# === status_messages.py ===

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


# === visualization.py ===

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
    header, refined: Optional[dict], target_id: str = '',
    tool_d_inner: float = 0.104,
) -> List[Marker]:
    """
    精化结果 → 与感知同款抓取示意（ns=peach_reconstruction/refined）.

    袋轴用拟合底/颈；半透明圆柱直径用工具内径（与感知 Marker 同，
    不是袋径）；行程从 entry 画到颈。文字放在颈上方，避免叠在入口架上。
    """
    if not refined or not refined.get('ok'):
        return []
    bottom = np.asarray(refined['bottom'], dtype=np.float64)
    neck = np.asarray(refined['neck'], dtype=np.float64)
    axis = np.asarray(refined['axis'], dtype=np.float64)
    entry = np.asarray(refined['entry'], dtype=np.float64)
    diameter = float(refined.get('diameter', 0.0))
    radius = 0.5 * diameter
    rotation = _grasp_rotation(axis)
    cut = refined.get('cut_pose', refined.get('cut_plane_point'))
    if cut is None:
        cut = neck
    cut = np.asarray(cut, dtype=np.float64)
    to_cut = float(np.dot(cut - entry, axis))
    travel = to_cut if to_cut > 1e-6 else float(
        refined.get('cut_travel_m') or refined.get('span_m', 0.0))
    travel_end = entry + travel * axis
    red, green, blue, alpha = _status_rgba(refined.get('status', STATUS_REJECT))
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

    kind = str(refined.get('kind', ''))
    prior_r = float(refined.get('fruit_prior_radius_m') or 0.0)
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
    suffix = 'final' if refined.get('final') else 'live'
    tid = target_id or str(refined.get('kind', 'refit'))
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


def build_refined_marker(header, refined: Optional[dict]) -> Optional[Marker]:
    """兼容旧调用：返回抓取示意中的袋轴线段，无结果给 None."""
    markers = build_refined_grasp_markers(header, refined)
    return markers[0] if markers else None


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
    marker.points = [
        _point_msg(vertices[index])
        for triangle in triangles
        for index in triangle
    ]
    return marker


# === session_io.py ===

def _dump_yaml(data: dict, path) -> None:
    """
    把 dict 写入 yaml 文件（utf-8，不排序保持可读顺序）.

    Args:
        data: 可 yaml 序列化的 dict（numpy 类型须已转原生类型）.
        path: 输出路径.

    Returns
    -------
        无返回值（None）；文件写入 path.

    """
    with open(str(path), 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def _write_ply_xyzrgb(path, xyz: np.ndarray,
                      colors_bgr: np.ndarray) -> None:
    """
    写 ASCII PLY（open3d 官方 write_point_cloud，xyz + uchar red/green/blue）.

    与旧手写版的差异：标量属性为 double（旧为 float）、头部多一行
    ``comment Created by Open3D``——PLY 消费者（CloudCompare/离线脚本）
    均按属性名解析，无语义差异。颜色 BGR→RGB 经 [0,1] float 往返无损。

    Args:
        path: 输出 ply 路径.
        xyz: (N, 3) 点 [m].
        colors_bgr: (N, 3) uint8 BGR（OpenCV 排列）.

    Returns
    -------
        无返回值（None）；文件写入 path；写失败抛 IOError.

    """
    o3d = require_open3d()
    pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64)))
    pcd.colors = o3d.utility.Vector3dVector(
        np.asarray(colors_bgr, dtype=np.uint8)[:, ::-1] / 255.0)  # BGR→RGB
    if not o3d.io.write_point_cloud(str(path), pcd, write_ascii=True):
        raise OSError(f'PLY 写出失败: {path}')


def _write_triangle_mesh(path, mesh_data: dict) -> None:
    """用 Open3D 官方 writer 保存顶点、三角形、法向和颜色."""
    o3d = require_open3d()
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(mesh_data['vertices'])
    mesh.triangles = o3d.utility.Vector3iVector(mesh_data['triangles'])
    if len(mesh_data.get('normals', [])) == len(mesh_data['vertices']):
        mesh.vertex_normals = o3d.utility.Vector3dVector(mesh_data['normals'])
    colors = mesh_data.get('colors_bgr')
    if colors is not None and len(colors) == len(mesh_data['vertices']):
        mesh.vertex_colors = o3d.utility.Vector3dVector(
            np.asarray(colors, dtype=np.uint8)[:, ::-1] / 255.0)
    if not o3d.io.write_triangle_mesh(
            str(path), mesh, write_ascii=True, write_vertex_normals=True,
            write_vertex_colors=True):
        raise OSError(f'网格 PLY 写出失败: {path}')


def save_session(root_dir, frames: List, metadata: dict,
                 tsdf_cloud=None, tsdf_mesh=None) -> Path:
    """
    把一次重建的全部帧写到 root_dir/session_<时间戳>/.

    Args:
        root_dir: session 根目录（不存在自动创建）.
        frames: CapturedFrame 列表（按采集顺序编号 frame_00, frame_01, ...）.
        metadata: 参数快照等元信息（写入 metadata.yaml）.
        tsdf_cloud: 可选 (xyz, colors_bgr) 元组；给出时写
            result/tsdf_cloud.ply（xyz+rgb）.
        tsdf_mesh: 可选 LocalTsdf.extract_mesh() 字典.

    Returns
    -------
        创建成功的 session 目录 Path.

    """
    root = Path(root_dir)
    # 微秒参与目录名并禁止复用：连续 finalize 不得静默覆盖前一次原始数据。
    session_dir = root / f'session_{datetime.now():%Y%m%d_%H%M%S_%f}'
    session_dir.mkdir(parents=True, exist_ok=False)
    for i, frame in enumerate(frames):
        stem = str(session_dir / f'frame_{i:02d}')
        if not cv2.imwrite(stem + '_rgb.png', frame.rgb):
            raise OSError(f'RGB PNG 写出失败: {stem}_rgb.png')
        np.save(stem + '_depth.npy', frame.depth_mm)
        _dump_yaml({
            'stamp_sec': float(frame.stamp),
            'width': int(frame.camera_K.get('width', 0)),
            'height': int(frame.camera_K.get('height', 0)),
            'fx': float(frame.camera_K['fx']),
            'fy': float(frame.camera_K['fy']),
            'cx': float(frame.camera_K['cx']),
            'cy': float(frame.camera_K['cy']),
        }, stem + '_camera_info.yaml')
        _dump_yaml({
            'T_base_camera_used': np.asarray(
                frame.T_base_camera, dtype=np.float64).tolist(),
            'T_base_camera_fk': np.asarray(
                getattr(frame, 'T_base_camera_fk', frame.T_base_camera),
                dtype=np.float64).tolist(),
            'camera_position_base': np.asarray(
                frame.camera_position_base, dtype=np.float64).tolist(),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'registration': dict(getattr(frame, 'registration', {})),
            'diagnostic_flags': list(frame.diagnostic_flags),
        }, stem + '_T_base_camera.yaml')
    if tsdf_cloud is not None:
        xyz, colors = tsdf_cloud
        if xyz is not None and len(xyz):
            result_dir = session_dir / 'result'
            result_dir.mkdir(exist_ok=True)
            if colors is None:
                colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            _write_ply_xyzrgb(result_dir / 'tsdf_cloud.ply', xyz, colors)
    if tsdf_mesh is not None and len(tsdf_mesh.get('vertices', [])):
        result_dir = session_dir / 'result'
        result_dir.mkdir(exist_ok=True)
        _write_triangle_mesh(result_dir / 'tsdf_mesh.ply', tsdf_mesh)
    _dump_yaml(metadata, session_dir / 'metadata.yaml')
    return session_dir


# === publishers.py ===

def xyzrgb_to_cloud_msg(xyz: np.ndarray, colors_bgr,
                        header: Header) -> PointCloud2:
    """
    (N, 3) 点 [m] + (N, 3) uint8 BGR → PointCloud2（xyz + 位打包 rgb 字段）.

    布局与 peach_scene_perception_node._xyzrgb_to_cloud 一致：x/y/z/rgb 各一个 FLOAT32
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


class PublisherMixin:
    """发布面方法集（宿主契约见模块 docstring；不自带 __init__）."""

    def _publish_heartbeat(self):
        """1Hz 活性心跳：状态 + 诊断 + 抓取许可（轻量三件套，不含云/Marker）."""
        if not self._lifecycle_active:
            return
        with self._state_lock:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.params.frames.base_frame
            self._publish_status_trio(header)

    def _publish_status_trio(self, header: Header):
        """
        状态名 + 类型化诊断 + 调试 JSON + 抓取许可统一重发（心跳/状态变化共用）.

        结构化核心走 ReconstructionStatus（/diagnostics），完整明细走
        String JSON（/diagnostics_debug），许可走 GraspDecision——三者同
        transient_local 闩锁，后启动订阅者读到的始终是最新一轮。
        """
        diag = self._diagnostics()
        self.pub_status.publish(String(data=self.collector.state))
        self.pub_diag.publish(diagnostics_to_status_msg(diag, header))
        self.pub_diag_debug.publish(
            String(data=json.dumps(diag, ensure_ascii=False)))
        self.pub_grasp_decision.publish(
            grasp_decision_to_msg(self._grasp_decision(), header))
        if getattr(self, 'pub_pregrasp', None) is not None:
            self.pub_pregrasp.publish(self._pregrasp_verification_msg(header))

    def _publish_all(self):
        """
        状态变化后统一重发：累加云 + 状态三件套 + 相机轨迹 Marker.

        E4 发布节流（publish.on_change_only / publish.min_interval_s）：
        local_cloud/tsdf_cloud/markers 三类大消息仅内容版本变化且距上次
        实际发布超过最小间隔才真正组装发布（on-change key 用帧数/末帧
        时间戳/产物版本号等廉价标量，不做内容哈希）；零变化或间隔内抑制
        ——三话题均为 transient_local 闩锁，订阅者/RViz 保留最后一帧不丢
        显示，被抑制的变化留待下次调用补发最新版本。产物清空事件
        （_reset_products 置 force 标志）绕过间隔门立即透传，保证绑定
        切换/reset 时 RViz 同步清屏。心跳/状态/诊断/refit 三件套不节流
        （就绪门/新鲜度门载体与闩锁覆盖防陈旧语义不动）。
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.params.frames.base_frame
        # on_change_only=false 回退逐次全发旧行为（不查节流器）
        throttle = (self._publish_throttle
                    if self.params.publish.on_change_only else None)
        force = self._products_force_publish
        self._products_force_publish = False
        frames = self.collector.frames
        n_frames = len(frames)
        # 累加云内容随帧栈增删变化；同长度同末帧戳即同内容（reset 后帧栈
        # 为空 n=0 自然判变；remove_last 改变 n_frames）
        last_stamp = float(frames[-1].stamp) if frames else -1.0
        if throttle is None or throttle.should_publish(
                'local_cloud', (n_frames, last_stamp), force=force):
            cloud = self.collector.accumulated_cloud()
            self.pub_cloud.publish(xyzrgb_to_cloud_msg(
                cloud, self.collector.accumulated_rgb(), header))
        # TSDF 云仅 finalize 后非空；无缓存发空云（字段布局保持一致）。
        # 内容版本 = _tsdf_cloud_version（每次全量 extract/清空递增）
        if throttle is None or throttle.should_publish(
                'tsdf_cloud', (self._tsdf_cloud_version,), force=force):
            if self._tsdf_cloud_cache is not None:
                tsdf_xyz, tsdf_rgb = self._tsdf_cloud_cache
            else:
                tsdf_xyz, tsdf_rgb = np.zeros((0, 3)), None
            self.pub_tsdf_cloud.publish(
                xyzrgb_to_cloud_msg(tsdf_xyz, tsdf_rgb, header))
        self._publish_status_trio(header)
        # Marker 内容 = 相机轨迹（帧栈）+ refined 抓取示意 + mesh
        # _products_version 覆盖（refit 写入/finalize 清理均递增）
        if throttle is None or throttle.should_publish(
                'markers', (n_frames, last_stamp, self._products_version),
                force=force):
            markers = build_camera_markers(header, self.collector.frames)
            markers.markers.extend(build_refined_grasp_markers(
                header, self._refined, self.collector.target_id or ''))
            mesh_marker = build_mesh_marker(header, self._mesh_cache)
            if mesh_marker is not None:
                markers.markers.append(mesh_marker)
            self.pub_markers.publish(markers)
        # refit 三件套：闩锁话题每次重发（无结果发空消息，防陈旧数据）
        pose_arr, axis_msg, fit_arr = self._refined_messages(header)
        self.pub_refined_pose.publish(pose_arr)
        self.pub_refined_axis.publish(axis_msg)
        self.pub_refined_diag.publish(fit_arr)
        self.pub_shape.publish(self._shape_hypothesis_msg(header))

    def _shape_hypothesis_msg(self, header: Header) -> ShapeHypothesis:
        """由 refit 缓存组形状假设（几何+协方差占位，不含工具抓取量）."""
        msg = ShapeHypothesis()
        msg.header = header
        msg.target_id = self.collector.target_id or ''
        result = self._refined
        if result is None or not result.get('ok'):
            return msg
        bottom = result['bottom']
        neck = result['neck']
        center = 0.5 * (bottom + neck)
        msg.center = Point(
            x=float(center[0]), y=float(center[1]), z=float(center[2]))
        msg.axis = Vector3(
            x=float(result['axis'][0]),
            y=float(result['axis'][1]),
            z=float(result['axis'][2]))
        msg.diameter_m = float(result['diameter'])
        msg.length_m = float(result.get('span_m', 0.0))
        msg.confidence = float(result.get('inlier_ratio', 0.0))
        msg.model_kind = str(result.get('kind', ''))
        return msg

    def _refined_messages(self, header: Header):
        """
        由 refit 缓存组 refined 三话题消息（闩锁重发/清空共用）.

        无结果（未跑 finalize/refit 关闭）→ 全空消息；拟合成功 →
        pose/axis/diagnostics 按结果填充（status=ACCEPT/REOBSERVE 照常
        发布）；拟合失败（REJECT）→ pose 发空数组、axis 发零向量（无效
        占位），diagnostics 发 status=REJECT 单条记录（标量 -1）——闩锁
        话题必须发消息覆盖，防后启动订阅者读到上一轮陈旧结果。

        Args:
            header: 输出头（frame_id=base_frame）.

        Returns
        -------
            (BagGraspCandidateArray, Vector3Stamped, BagFittingArray).

        """
        pose_arr = BagGraspCandidateArray()
        pose_arr.header = header
        axis_msg = Vector3Stamped()
        axis_msg.header = header
        fit_arr = BagFittingArray()
        fit_arr.header = header
        result = self._refined
        if result is not None and result['ok']:
            cand = BagGraspCandidate()
            cand.header = header
            cand.target_id = self.collector.target_id
            # entry = bottom − axis×standoff（几何在 refiner 内算好）；
            # 姿态未用（接近方向由 translation_direction 给出），置单位四元数
            cand.entry_pose = Pose(
                position=Point(x=float(result['entry'][0]),
                               y=float(result['entry'][1]),
                               z=float(result['entry'][2])),
                orientation=Quaternion(w=1.0))
            cand.bag_bottom = Point(x=float(result['bottom'][0]),
                                    y=float(result['bottom'][1]),
                                    z=float(result['bottom'][2]))
            cand.bag_neck = Point(x=float(result['neck'][0]),
                                  y=float(result['neck'][1]),
                                  z=float(result['neck'][2]))
            # 剪切行进方向 = refined 轴（bottom→neck 单位向量）
            cand.translation_direction = Vector3(x=float(result['axis'][0]),
                                                 y=float(result['axis'][1]),
                                                 z=float(result['axis'][2]))
            # 圆柱为袋径、球为果径（均 = 2r）
            cand.bag_diameter_upper_m = float(result['diameter'])
            # 套入行程 = 入口沿轴到剪切参考；执行端优先用本字段。
            cand.suggested_travel_m = float(
                result.get('cut_travel_m') or 0.0)
            cand.confidence = float(result['inlier_ratio'])
            cand.status = int(result['status'])
            cand.diagnostic_flags = list(result['flags'])
            cand.strategy_id = f"reconstruction_refit_{result['kind']}"
            pose_arr.candidates.append(cand)
            axis_msg.vector = Vector3(x=float(result['axis'][0]),
                                      y=float(result['axis'][1]),
                                      z=float(result['axis'][2]))
        fit = self._refined_fitting_msg(header, result)
        if fit is not None:
            fit_arr.fittings.append(fit)
        return pose_arr, axis_msg, fit_arr

    def _refined_fitting_msg(self, header: Header,
                             result: Optional[dict]) -> Optional[BagFitting]:
        """
        由 refit 结果组 BagFitting（无效标量 -1，语义对齐感知包 _to_fitting）.

        Args:
            header: 输出头.
            result: refit 唯一缓存 _refined（成功结果或 {'ok': False,
                'reason': ...} 失败记录）；None 表示未跑（失败时由
                _refined_info() 取原因，发 REJECT 记录）.

        Returns
        -------
            peach_interfaces/BagFitting；从未跑过 refit 给 None.

        """
        info = self._refined_info()
        if result is None and not info:
            return None
        m = BagFitting()
        m.header = header
        m.target_id = self.collector.target_id
        m.axis_source = 'reconstruction_refit'
        # 全部标量先置 -1（无效约定），再按拟合线逐项覆盖有效字段
        for attr in ('axis_confidence', 'axis_disagreement_deg', 'theta_err_deg',
                     'error_budget_mm', 'radial_clearance_mm', 'valid_depth_ratio',
                     'foreground_ratio', 'boundary_touch_ratio', 'bag_length_m',
                     'bag_diameter_upper_m', 'travel_m', 'cylinder_rms_m',
                     'cylinder_inlier_ratio', 'fruit_radius_m', 'sphere_rms_m',
                     'sphere_inlier_ratio', 'cavity_dip_mm'):
            setattr(m, attr, -1.0)
        m.boundary_sides_touched = -1
        m.n_points = -1
        if result is None or not result['ok']:
            info = info or {}
            m.target_kind = str(info.get('kind', ''))
            m.status = STATUS_REJECT  # 拟合失败不发 pose/axis，仅留诊断记录
            m.diagnostic_flags = ['refit_failed',
                                  str(info.get('reason', 'unknown'))]
            return m
        m.target_kind = 'fruit' if result['kind'] == 'sphere' else 'bag'
        m.n_points = int(result['n_points'])
        m.bag_diameter_upper_m = float(result['diameter'])
        m.travel_m = float(result['span_m'])
        if result['kind'] == 'cylinder':
            m.bag_length_m = float(result['span_m'])
            m.cylinder_rms_m = float(result['rmse'])
            m.cylinder_inlier_ratio = float(result['inlier_ratio'])
        else:
            m.fruit_radius_m = float(result['radius'])
            m.sphere_rms_m = float(result['rmse'])
            m.sphere_inlier_ratio = float(result['inlier_ratio'])
        m.status = int(result['status'])
        m.diagnostic_flags = list(result['flags'])
        return m

    def _grasp_decision(self) -> dict:
        """把最终精化质量归一成只读抓取许可，不发送运动指令."""
        decision = {
            'harvest_run_id': self._harvest_run_id,
            'target_id': self.collector.target_id,
            'allowed': False,
            'geometry_valid': False,
            'reason': 'reconstruction_not_ready',
        }
        if self.collector.state != 'READY':
            return decision
        result = self._refined
        if result is None or not result.get('ok'):
            decision['reason'] = 'refined_geometry_unavailable'
            return decision
        angle = result.get('axis_angle_deg')
        if angle is None:
            angle = axis_angle_deg(result.get('axis'), self._bound_axis_hint)
        if angle is not None:
            decision['axis_angle_deg'] = float(angle)
            max_deg = float(self.params.refit.max_axis_angle_deg)
            decision['diagnostic_axis_mismatch'] = bool(angle > max_deg)
        entry = [float(v) for v in result['entry']]
        axis = [float(v) for v in result['axis']]
        pregrasp = result.get('pregrasp')
        if pregrasp is None:
            pregrasp = [
                entry[0] - 0.10 * axis[0],
                entry[1] - 0.10 * axis[1],
                entry[2] - 0.10 * axis[2],
            ]
        cut_src = result.get('cut_pose', result.get('neck', entry))
        decision.update({
            'geometry_valid': True,
            'entry': entry,
            'axis': axis,
            'pregrasp': [float(v) for v in pregrasp],
            'cut_pose': [float(v) for v in cut_src],
            'diameter_m': float(
                result.get('d95_m') or result.get('diameter') or 0.0),
            'd95_m': float(
                result.get('d95_m') or result.get('diameter') or 0.0),
            'travel_m': float(result.get('cut_travel_m') or result.get(
                'span_m') or 0.0),
            'cut_travel_m': float(result.get('cut_travel_m') or 0.0),
            'radial_margin_m': float(result.get('radial_margin_m') or 0.0),
            'axial_margin_m': float(result.get('axial_margin_m') or 0.0),
            'corridor_clear': bool(result.get('corridor_clear', False)),
            'rmse_m': float(result.get('rmse') or 0.0),
            'inlier_ratio': float(result.get('inlier_ratio') or 0.0),
            'model_revision': str(result.get('model_revision') or ''),
            'tool_profile_id': 'hollow_cylinder_v1',
        })
        budget = result.get('budget') or {}
        if not budget:
            decision['reason'] = 'bag_model_unavailable'
            decision['failure_code'] = 3
            return decision
        if not budget.get('allowed'):
            decision['reason'] = str(
                budget.get('reason') or 'dynamic_budget_negative')
            decision['failure_code'] = int(budget.get('failure_code') or 12)
            return decision
        decision['allowed'] = True
        decision['reason'] = str(
            budget.get('reason') or 'refined_geometry_accept')
        decision['failure_code'] = 0
        return decision

    def _diagnostics(self) -> dict:
        """组装完整诊断 dict（调试明细，随 /diagnostics_debug 以 JSON 发出）."""
        c = self.collector
        cloud = c.accumulated_cloud()
        last_ratio = c.frames[-1].valid_depth_ratio if c.frames else None
        # registration 摘要不存副本，由帧栈各帧 registration 派生
        registrations = [f.registration for f in c.frames]
        coverage = summarize_view_coverage(c.frames, c.target_center)
        return {
            'harvest_run_id': self._harvest_run_id,
            'selected_target_id': self._preferred_target_id,
            'target_mask_cache_size': len(self._target_masks),
            'state': c.state,
            'target_id': c.target_id,
            'target_center_base': (None if c.target_center is None
                                   else [float(v) for v in c.target_center]),
            'bound_axis_hint': (None if self._bound_axis_hint is None
                                else [float(v) for v in self._bound_axis_hint]),
            # 实际积分帧数；机位数见 view_coverage.view_count。
            'captured_views': len(c.frames),
            'pose_count': int(coverage.get('view_count') or 0),
            'rejected_views': c.rejected_views,
            'tf_failures': c.tf_failures,
            'skipped_views': c.skipped_views,
            'skip_reasons': dict(c.skip_reasons),
            'last_skip_code': c.last_skip_code,
            'last_skip_reason': c.last_skip_reason,
            'frame_ring_size': len(getattr(self, '_frame_ring', {})),
            'tf_latency_ms': self._last_tf_latency_ms,
            'valid_depth_ratio': last_ratio,
            'cloud_points': int(cloud.shape[0]),
            'last_rel_translation_m': c.last_rel_translation_m,
            'last_rel_rotation_deg': c.last_rel_rotation_deg,
            # finalize 时的重叠度指标（pairs/质心）；未 finalize 或帧栈已变为 None
            'overlap': self._overlap_cache,
            # finalize 时的 TSDF 摘要（points/integrate_time_s/roi_center 等）
            'tsdf': self._tsdf_info,
            'registration': {
                'accepted': len(registrations),
                'latest': (None if not registrations
                           else registrations[-1]),
                # E4 ICP target 增量复用观测：当前自适应刷新周期 k、缓存
                # target 点数、全量刷新/增量拼接累计次数
                'target_refresh_period': self._icp_target_cache.period,
                'target_points': self._icp_target_cache.target_size,
                'target_full_refreshes':
                    self._icp_target_cache.full_refreshes,
                'target_incremental_appends':
                    self._icp_target_cache.incremental_appends,
            },
            # 主动视觉控制器消费精确采帧位姿，而不是回调时刻的 latest TF。
            # 覆盖指标按机位聚类（同机位连帧不稀释角基线）；积分帧数另由
            # captured_views 报告。
            'view_coverage': coverage,
            # refit 摘要（kind/center/axis/diameter/rmse/inlier_ratio/ok）；
            # 未跑为 None，失败为 {'ok': False, 'reason': ...}
            'refined': self._refined_info(),
            'grasp_decision': self._grasp_decision(),
            # 耗时基线（阶段 C 埋点）：ICP/TSDF/帧总 EMA + refit/finalize
            # last 值 + 计数；键集恒定，随 diagnostics_debug JSON 发出
            'timing': self._timing.snapshot(),
        }

    def _refined_info(self) -> Optional[dict]:
        """
        由唯一 refit 缓存 _refined 投影出 diagnostics JSON 的 refined 键.

        Returns
        -------
            None（未跑/已失效）；失败记录原样拷贝（{'ok': False,
            'reason': ...}）；成功结果经 _refined_diag_dict 转 JSON 形态.

        """
        result = self._refined
        if result is None:
            return None
        if not result.get('ok'):
            return dict(result)
        return self._refined_diag_dict(result)

    @staticmethod
    def _refined_diag_dict(result: dict) -> dict:
        """
        组装 diagnostics JSON 的 refined 键（refit 成功结果，numpy→原生类型）.

        Args:
            result: refine_geometry 的 ok=True 结果.

        Returns
        -------
            JSON 可序列化 dict（kind/center/axis/diameter/rmse/
            inlier_ratio/ok 等）.

        """
        return {
            'ok': True,
            'kind': result['kind'],
            'status': int(result['status']),
            'center': [float(v) for v in result['center']],
            'axis': [float(v) for v in result['axis']],
            'bottom': [float(v) for v in result['bottom']],
            'neck': [float(v) for v in result['neck']],
            'diameter': float(result['diameter']),
            'span_m': float(result['span_m']),
            'rmse': float(result['rmse']),
            'inlier_ratio': float(result['inlier_ratio']),
            'n_points': int(result['n_points']),
            'flags': list(result.get('flags') or []),
            'axis_angle_deg': result.get('axis_angle_deg'),
            'axis_conflict_deg': result.get('axis_conflict_deg'),
            'envelope_conditioned': bool(result.get('envelope_conditioned')),
            'envelope_reason': str(result.get('envelope_reason') or ''),
            'perception_axis': result.get('perception_axis'),
        }
