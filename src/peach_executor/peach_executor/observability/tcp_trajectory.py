"""末端 TCP 轨迹缓冲: latest TF 采样、弦长/路径长/绕行比."""

from __future__ import annotations

import math
import threading
from typing import Any


def _finite3(xyz) -> bool:
    """三维坐标是否为有限浮点."""
    return (
        isinstance(xyz, (list, tuple)) and len(xyz) >= 3 and
        all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz[:3]))


def _dist(a, b) -> float:
    """两点欧氏距离（米）."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _point_to_segment(point, start, end) -> float:
    """点到闭线段的距离（米）."""
    ab = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    length2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if length2 < 1.0e-16:
        return _dist(point, start)
    t = (
        (point[0] - start[0]) * ab[0] +
        (point[1] - start[1]) * ab[1] +
        (point[2] - start[2]) * ab[2]) / length2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    closest = (
        start[0] + t * ab[0],
        start[1] + t * ab[1],
        start[2] + t * ab[2])
    return _dist(point, closest)


def path_metrics(xyz_list: list) -> dict:
    """
    由 TCP 位置序列算路径长、起止弦长、相对弦最大偏离、Z 范围.

    绕行比 = 路径长 / 弦长。笛卡尔直线接近时接近 1；关节 PTP 绕行时明显大于 1.
    """
    points = [tuple(item[:3]) for item in xyz_list if _finite3(item)]
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
        path += _dist(previous, current)
        max_dev = max(max_dev, _point_to_segment(current, start, end))
    chord = _dist(start, end)
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


class TcpPathBuffer:
    """环形缓冲；位移过小且未运动则丢弃，避免静止刷盘."""

    def __init__(self, max_points: int = 8000, min_step_m: float = 0.003):
        self._lock = threading.Lock()
        self._max_points = max(100, int(max_points))
        self._min_step_m = float(min_step_m)
        self._points: list[dict] = []
        self._tf_ok = False
        self._tf_failures = 0
        self._last_keep_t = 0.0

    def note_tf_fail(self) -> None:
        """Latest TF 查询失败: 计数，不清空已采轨迹."""
        with self._lock:
            self._tf_ok = False
            self._tf_failures += 1

    def maybe_append(self, point: dict) -> dict | None:
        """
        位移 ≥ min_step、in_motion、或静止超过 1 s 则收下.

        Returns
        -------
            写入缓冲的点（已 round）；丢弃时 None.

        """
        xyz = [point['x'], point['y'], point['z']]
        if not _finite3(xyz):
            return None
        stored = {
            't': round(float(point['t']), 3),
            'x': round(float(xyz[0]), 4),
            'y': round(float(xyz[1]), 4),
            'z': round(float(xyz[2]), 4),
            'qx': round(float(point.get('qx', 0.0)), 5),
            'qy': round(float(point.get('qy', 0.0)), 5),
            'qz': round(float(point.get('qz', 0.0)), 5),
            'qw': round(float(point.get('qw', 1.0)), 5),
            'moving': bool(point.get('moving')),
            'target_id': str(point.get('target_id') or ''),
            'phase': int(point.get('phase') or 0),
            'skill': str(point.get('skill') or ''),
        }
        with self._lock:
            previous = self._points[-1] if self._points else None
            keep = previous is None
            if previous is not None:
                step = _dist(
                    (stored['x'], stored['y'], stored['z']),
                    (previous['x'], previous['y'], previous['z']))
                dt = stored['t'] - previous['t']
                keep = (
                    step >= self._min_step_m or
                    stored['phase'] != previous['phase'] or
                    stored['skill'] != previous['skill'] or
                    dt >= 1.0 or
                    (stored['moving'] and step >= 0.001))
            if not keep:
                self._tf_ok = True
                return None
            self._points.append(stored)
            overflow = len(self._points) - self._max_points
            if overflow > 0:
                del self._points[:overflow]
            self._tf_ok = True
            self._last_keep_t = stored['t']
            return dict(stored)

    def clear(self) -> None:
        """Drop points from the previous run; keep TF failure counts."""
        with self._lock:
            self._points.clear()
            self._last_keep_t = 0.0

    def summary(self) -> dict:
        """供 /api/state 机械臂卡片：末点 + 路径/弦长（不含全点列）."""
        with self._lock:
            points = list(self._points)
            tf_ok = self._tf_ok
            tf_failures = self._tf_failures
        metrics = path_metrics(
            [(item['x'], item['y'], item['z']) for item in points])
        last = points[-1] if points else None
        quat = None
        if last is not None:
            quat = [last['qx'], last['qy'], last['qz'], last['qw']]
        return {
            'frame_id': None,
            'tip_frame': None,
            'tf_ok': tf_ok,
            'tf_failures': tf_failures,
            'xyz': [last['x'], last['y'], last['z']] if last else None,
            'quat': quat,
            'stamp': last['t'] if last else None,
            **metrics,
        }

    def export(self, max_out: int = 2500) -> dict[str, Any]:
        """三维 API：时间/xyz 打包数组；过密则等距抽稀，始终保留首末点."""
        with self._lock:
            points = list(self._points)
            tf_ok = self._tf_ok
            tf_failures = self._tf_failures
        if len(points) > max_out:
            stride = int(math.ceil(len(points) / float(max_out)))
            sampled = points[::stride]
            if sampled[-1] is not points[-1]:
                sampled.append(points[-1])
            points = sampled
        metrics = path_metrics(
            [(item['x'], item['y'], item['z']) for item in points])
        xyz: list[float] = []
        t_list: list[float] = []
        moving: list[int] = []
        phase: list[int] = []
        for item in points:
            t_list.append(item['t'])
            xyz.extend((item['x'], item['y'], item['z']))
            moving.append(1 if item['moving'] else 0)
            phase.append(item['phase'])
        last = points[-1] if points else None
        return {
            'tf_ok': tf_ok,
            'tf_failures': tf_failures,
            't': t_list,
            'xyz': xyz,
            'moving': moving,
            'phase': phase,
            'metrics': {
                **metrics,
                'xyz': [last['x'], last['y'], last['z']] if last else None,
                'quat': (
                    [last['qx'], last['qy'], last['qz'], last['qw']]
                    if last else None),
                'stamp': last['t'] if last else None,
            },
        }


# visualization_msgs/Marker 类型/动作（与 RViz / 重建 Marker 同一套枚举）
MARKER_ARROW = 0
MARKER_SPHERE = 2
MARKER_LINE_STRIP = 4
MARKER_LINE_LIST = 5
MARKER_TEXT = 9
ACTION_ADD = 0
ACTION_DELETEALL = 3

_PHASE_RGBA = {
    2: (0.31, 0.64, 0.88, 0.95),
    5: (0.88, 0.66, 0.24, 0.95),
    6: (0.77, 0.49, 1.0, 0.95),
    7: (0.25, 0.75, 0.45, 0.95),
}


def downsample_path(
        xyz_flat: list, phases: list | None = None, max_poses: int = 800):
    """Keep first/last poses; downsample Path/Marker for RViz, jsonl stays dense."""
    if not isinstance(xyz_flat, list):
        return [], []
    count = len(xyz_flat) // 3
    phase_list = phases if isinstance(phases, list) else []
    if count <= 0:
        return [], []
    if count <= max_poses:
        return list(xyz_flat[:count * 3]), list(phase_list[:count])
    step = (count - 1) / float(max_poses - 1)
    xyz_out = []
    phase_out = []
    for i in range(max_poses):
        index = int(round(i * step))
        if index >= count:
            index = count - 1
        base = index * 3
        xyz_out.extend(xyz_flat[base:base + 3])
        if index < len(phase_list):
            phase_out.append(phase_list[index])
        else:
            phase_out.append(0)
    return xyz_out, phase_out


def _color(r, g, b, a=0.9) -> dict:
    """RGBA 0–1 字典，对齐 visualization_msgs/ColorRGBA."""
    return {'r': float(r), 'g': float(g), 'b': float(b), 'a': float(a)}


def _marker(
        ns: str, mid: int, mtype: int, frame_id: str,
        *, points=None, xyz=None, scale=None, color=None, text='') -> dict:
    """JSON Marker，字段对齐 visualization_msgs/Marker（供 RViz 与网页同源）."""
    position = xyz if _finite3(xyz or ()) else [0.0, 0.0, 0.0]
    packed_points = []
    for item in points or []:
        if _finite3(item):
            packed_points.append([
                round(float(item[0]), 4),
                round(float(item[1]), 4),
                round(float(item[2]), 4)])
    return {
        'ns': ns,
        'id': int(mid),
        'type': int(mtype),
        'action': ACTION_ADD,
        'frame_id': frame_id,
        'pose': {
            'position': [
                round(float(position[0]), 4),
                round(float(position[1]), 4),
                round(float(position[2]), 4)],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        },
        'scale': scale or {'x': 0.01, 'y': 0.01, 'z': 0.01},
        'color': color or _color(1.0, 1.0, 1.0),
        'points': packed_points,
        'text': text,
    }


def build_selection_marker_dicts(
        observations: list, selected_id: str, filtered: dict,
        frame_id: str) -> list:
    """
    选择叠加（L4 呈现层）：selected 高亮环 + 淘汰 X 与原因文字.

    数据全部来自 observability 已订阅源（perception.targets 的
    candidate.entry_position、作业票 target_id、events 里最近的
    targets_filtered），零新增订阅；不含 DELETEALL（随 tcp markers 同帧发）。
    filtered: tid→原因串（如 'ik_no_solution:no_ik'）。
    """
    frame_id = frame_id or 'base_link'

    def _pos(item):
        entry = ((item.get('candidate') or {}).get('entry_position'))
        return entry if entry and len(entry) == 3 and all(
            isinstance(v, (int, float)) for v in entry) else None

    markers = []
    for item in observations or []:
        tid = str(item.get('target_id') or '')
        xyz = _pos(item)
        if not tid or not xyz:
            continue
        if tid == selected_id:
            markers.append(_marker(
                'selection', 1, MARKER_SPHERE, frame_id, xyz=xyz,
                scale={'x': 0.045, 'y': 0.045, 'z': 0.045},
                color=_color(0.16, 0.95, 0.25, 0.9)))
            markers.append(_marker(
                'selection', 11, MARKER_TEXT, frame_id, xyz=xyz,
                scale={'x': 0.0, 'y': 0.0, 'z': 0.03},
                color=_color(0.16, 0.95, 0.25, 1.0),
                text=f'selected {tid}'))
        elif tid in filtered:
            size = 0.035
            cross = [
                [xyz[0] - size, xyz[1] - size, xyz[2]],
                [xyz[0] + size, xyz[1] + size, xyz[2]],
                [xyz[0] - size, xyz[1] + size, xyz[2]],
                [xyz[0] + size, xyz[1] - size, xyz[2]]]
            markers.append(_marker(
                'selection', 2, MARKER_LINE_LIST, frame_id,
                points=cross, scale={'x': 0.006, 'y': 0.0, 'z': 0.0},
                color=_color(0.95, 0.22, 0.22, 0.95)))
            reason = str(filtered[tid])[:24]
            markers.append(_marker(
                'selection', 12, MARKER_TEXT, frame_id, xyz=xyz,
                scale={'x': 0.0, 'y': 0.0, 'z': 0.026},
                color=_color(0.95, 0.45, 0.45, 1.0),
                text=f'{tid} {reason}'))
    return markers


def build_tcp_marker_dicts(
        xyz_flat: list, phases: list, landmarks: dict, frame_id: str) -> list:
    """
    末端轨迹 → Marker 字典列表（重建相机轨迹同一套：DELETEALL、LINE_LIST、SPHERE、ARROW）.

    ns：tcp_path / tcp_chord / tcp_now / grasp. 网页与 RViz MarkerArray 都吃这份.
    """
    frame_id = frame_id or 'base_link'
    markers = [{
        'ns': '',
        'id': 0,
        'type': 0,
        'action': ACTION_DELETEALL,
        'frame_id': frame_id,
        'pose': {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]},
        'scale': {'x': 1.0, 'y': 1.0, 'z': 1.0},
        'color': _color(0.0, 0.0, 0.0, 0.0),
        'points': [],
        'text': '',
    }]
    points = []
    if isinstance(xyz_flat, list):
        for index in range(0, len(xyz_flat) - 2, 3):
            triple = xyz_flat[index:index + 3]
            if _finite3(triple):
                points.append((float(triple[0]), float(triple[1]), float(triple[2])))
    if len(points) >= 2:
        # 分段着色：观察青 / 靠近黄 / 工具紫 / 撤离绿（与 HarvestState.target_phase）
        segs = {}
        for index in range(len(points) - 1):
            phase = 0
            if isinstance(phases, list) and index < len(phases):
                try:
                    phase = int(phases[index + 1] if index + 1 < len(phases) else phases[index])
                except (TypeError, ValueError):
                    phase = 0
            segs.setdefault(phase, []).extend((points[index], points[index + 1]))
        for phase, chain in segs.items():
            rgba = _PHASE_RGBA.get(phase, (0.87, 0.90, 0.93, 0.95))
            markers.append(_marker(
                'tcp_path', 10 + int(phase), MARKER_LINE_LIST, frame_id,
                points=chain, scale={'x': 0.006, 'y': 0.0, 'z': 0.0},
                color=_color(*rgba)))
        markers.append(_marker(
            'tcp_chord', 1, MARKER_LINE_STRIP, frame_id,
            points=[points[0], points[-1]],
            scale={'x': 0.004, 'y': 0.0, 'z': 0.0},
            color=_color(0.85, 0.90, 0.95, 0.9)))
    if points:
        markers.append(_marker(
            'tcp_now', 1, MARKER_SPHERE, frame_id, xyz=points[-1],
            scale={'x': 0.025, 'y': 0.025, 'z': 0.025},
            color=_color(0.95, 0.95, 0.95, 0.95)))
        markers.append(_marker(
            'tcp_now', 2, MARKER_TEXT, frame_id, xyz=points[-1],
            scale={'x': 0.0, 'y': 0.0, 'z': 0.035},
            color=_color(0.87, 0.90, 0.93, 1.0), text='tcp'))

    specs = (
        ('perception_entry', 'grasp', 1, (0.25, 0.75, 0.45), '感知入口', 0.02),
        ('reconstruction_center', 'grasp', 2, (0.77, 0.49, 1.0), '重建中心', 0.018),
        ('grasp_pregrasp', 'grasp', 3, (0.88, 0.66, 0.24), '预抓取', 0.028),
        ('grasp_entry', 'grasp', 4, (0.88, 0.36, 0.36), '抓取入口', 0.028),
    )
    landmarks = landmarks or {}
    for key, ns, mid, rgb, label, radius in specs:
        xyz = landmarks.get(key)
        if not _finite3(xyz):
            continue
        markers.append(_marker(
            ns, mid, MARKER_SPHERE, frame_id, xyz=xyz,
            scale={'x': radius, 'y': radius, 'z': radius},
            color=_color(rgb[0], rgb[1], rgb[2], 0.92)))
        markers.append(_marker(
            ns, mid + 10, MARKER_TEXT, frame_id, xyz=xyz,
            scale={'x': 0.0, 'y': 0.0, 'z': 0.03},
            color=_color(0.87, 0.90, 0.93, 1.0), text=label))

    axis = landmarks.get('axis')
    entry = landmarks.get('grasp_entry')
    if _finite3(axis) and _finite3(entry):
        length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
        if length > 1.0e-6:
            unit = [axis[0] / length, axis[1] / length, axis[2] / length]
            start = [
                entry[0] - 0.12 * unit[0],
                entry[1] - 0.12 * unit[1],
                entry[2] - 0.12 * unit[2]]
            end = [
                entry[0] + 0.22 * unit[0],
                entry[1] + 0.22 * unit[1],
                entry[2] + 0.22 * unit[2]]
            markers.append(_marker(
                'grasp_axis', 1, MARKER_ARROW, frame_id,
                points=[start, end],
                scale={'x': 0.006, 'y': 0.014, 'z': 0.014},
                color=_color(0.2, 0.8, 0.95, 0.95)))
    return markers
