"""
相机轨迹 + TSDF 网格 + refined 抓取示意 Marker 构造（RViz 可视化）.

每个已采帧画：相机位置球点（SPHERE_LIST）+ 顺序连线（LINE_LIST）+
沿光轴 +Z 的朝向小箭头（ARROW，长 0.05 m）。refit 成功时画与感知
同款的抓取示意（袋轴/行程/圆柱/果球/三轴架/文字，独立 namespace）。
frame_id 由调用方给（通常 base_frame）。
"""
from typing import List, Optional

from geometry_msgs.msg import Point, Quaternion
import numpy as np
from peach_common_py.tf_utils import rotation_to_quat
from peach_target_reconstruction.geometry_refiner import (
    STATUS_ACCEPT,
    STATUS_REJECT,
    STATUS_REOBSERVE,
)
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

_MARKER_NS = 'peach_reconstruction'
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
    to_neck = float(np.dot(neck - entry, axis))
    travel = to_neck if to_neck > 1e-6 else float(refined.get('span_m', 0.0))
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
        cyl.color.a = 0.25
        out.append(cyl)

    if str(refined.get('kind', '')) == 'fruit' and radius > 1e-6:
        sphere = _mk(3, Marker.SPHERE)
        sphere.pose.position = _point_msg(0.5 * (bottom + neck))
        sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * radius)
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
