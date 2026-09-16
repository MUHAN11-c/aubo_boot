"""零 ROS：TCP 路径长 / 弦长 / 绕行比（几何纯核）."""

from __future__ import annotations

import math


def finite3(xyz) -> bool:
    """三维坐标是否为有限浮点."""
    return (
        isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and
        all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz[:3]))


def dist(a, b) -> float:
    """两点欧氏距离（米）."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def point_to_segment(point, start, end) -> float:
    """点到闭线段的距离（米）."""
    ab = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    length2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if length2 < 1.0e-16:
        return dist(point, start)
    t = (
        (point[0] - start[0]) * ab[0] +
        (point[1] - start[1]) * ab[1] +
        (point[2] - start[2]) * ab[2]) / length2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    closest = (
        start[0] + t * ab[0],
        start[1] + t * ab[1],
        start[2] + t * ab[2])
    return dist(point, closest)


def path_metrics(xyz_list: list) -> dict:
    """
    由 TCP 位置序列算路径长、起止弦长、相对弦最大偏离、Z 范围.

    绕行比 = 路径长 / 弦长。笛卡尔直线接近时接近 1；关节 PTP 绕行时明显大于 1.
    """
    points = [tuple(item[:3]) for item in xyz_list if finite3(item)]
    empty = {
        'count': len(points),
        'path_length_m': 0.0,
        'chord_m': 0.0,
        'detour_ratio': None,
        'max_dev_m': 0.0,
        'z_min_m': None,
        'z_max_m': None,
        'dz_m': None,
    }
    if not points:
        return empty
    zs = [p[2] for p in points]
    empty['z_min_m'] = round(min(zs), 4)
    empty['z_max_m'] = round(max(zs), 4)
    empty['dz_m'] = round(zs[-1] - zs[0], 4)
    if len(points) == 1:
        return empty
    path = 0.0
    max_dev = 0.0
    start, end = points[0], points[-1]
    for previous, current in zip(points, points[1:]):
        path += dist(previous, current)
        max_dev = max(max_dev, point_to_segment(current, start, end))
    chord = dist(start, end)
    ratio = (path / chord) if chord >= 0.02 else None
    return {
        'count': len(points),
        'path_length_m': round(path, 4),
        'chord_m': round(chord, 4),
        'detour_ratio': None if ratio is None else round(ratio, 3),
        'max_dev_m': round(max_dev, 4),
        'z_min_m': empty['z_min_m'],
        'z_max_m': empty['z_max_m'],
        'dz_m': empty['dz_m'],
    }
