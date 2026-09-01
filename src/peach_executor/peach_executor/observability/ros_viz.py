"""nav_msgs/Path 与 visualization_msgs/MarkerArray：网页 Marker 字典同源."""

from __future__ import annotations

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


def _header(stamp, frame_id: str) -> Header:
    """Stamp is rclpy Time or a builtin stamp message."""
    header = Header()
    header.frame_id = frame_id or 'base_link'
    if hasattr(stamp, 'to_msg'):
        header.stamp = stamp.to_msg()
    elif stamp is not None:
        header.stamp = stamp
    return header


def path_from_xyz(xyz_flat, stamp, frame_id: str) -> Path:
    """平坦 xyz 列表 → latched Path（RViz Path 显示）."""
    path = Path()
    path.header = _header(stamp, frame_id)
    if not isinstance(xyz_flat, list):
        return path
    for index in range(0, len(xyz_flat) - 2, 3):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(xyz_flat[index])
        pose.pose.position.y = float(xyz_flat[index + 1])
        pose.pose.position.z = float(xyz_flat[index + 2])
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _color(value) -> ColorRGBA:
    """字典 {r,g,b,a} → ColorRGBA；缺键为 0."""
    color = ColorRGBA()
    if isinstance(value, dict):
        color.r = float(value.get('r', 0.0))
        color.g = float(value.get('g', 0.0))
        color.b = float(value.get('b', 0.0))
        color.a = float(value.get('a', 1.0))
    return color


def _point(xyz) -> Point:
    """长度为 3 的序列 → geometry_msgs/Point."""
    point = Point()
    point.x = float(xyz[0])
    point.y = float(xyz[1])
    point.z = float(xyz[2])
    return point


def marker_from_dict(item: dict, stamp, frame_id: str) -> Marker:
    """build_tcp_marker_dicts 的一条 → visualization_msgs/Marker."""
    marker = Marker()
    marker.header = _header(stamp, item.get('frame_id') or frame_id)
    marker.ns = str(item.get('ns') or '')
    marker.id = int(item.get('id') or 0)
    marker.type = int(item.get('type') or 0)
    marker.action = int(item.get('action') or 0)
    pose = item.get('pose') or {}
    position = pose.get('position') or [0.0, 0.0, 0.0]
    orientation = pose.get('orientation') or [0.0, 0.0, 0.0, 1.0]
    if len(position) >= 3:
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
    if len(orientation) >= 4:
        marker.pose.orientation.x = float(orientation[0])
        marker.pose.orientation.y = float(orientation[1])
        marker.pose.orientation.z = float(orientation[2])
        marker.pose.orientation.w = float(orientation[3])
    scale = item.get('scale') or {}
    marker.scale.x = float(scale.get('x', 0.01))
    marker.scale.y = float(scale.get('y', 0.01))
    marker.scale.z = float(scale.get('z', 0.01))
    marker.color = _color(item.get('color'))
    marker.text = str(item.get('text') or '')
    for xyz in item.get('points') or []:
        if isinstance(xyz, (list, tuple)) and len(xyz) >= 3:
            marker.points.append(_point(xyz))
    return marker


def marker_array_from_dicts(items, stamp, frame_id: str) -> MarkerArray:
    """网页/RViz 共用的 Marker 字典列表 → MarkerArray."""
    array = MarkerArray()
    for item in items or []:
        if isinstance(item, dict):
            array.markers.append(marker_from_dict(item, stamp, frame_id))
    return array
