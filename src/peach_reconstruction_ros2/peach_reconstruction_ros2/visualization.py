"""
相机轨迹 Marker 构造（RViz 可视化，ROS 侧模块）.

每个已采帧画：相机位置球点（SPHERE_LIST）+ 顺序连线（LINE_LIST）+
沿光轴 +Z 的朝向小箭头（ARROW，长 0.05 m）。frame_id 由调用方给
（通常 base_frame）。
"""
from typing import List

from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

_MARKER_NS = 'peach_reconstruction'


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
    # DELETEALL 不设 ns/id：否则与首个 ADD 冲突（同 peach_pose_node 的 RViz 坑）
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
