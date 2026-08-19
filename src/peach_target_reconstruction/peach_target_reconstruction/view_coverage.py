"""多视角覆盖诊断：把精确采帧位姿归一成主动视觉可消费的数据."""
import math

import numpy as np


def _safe_unit(vector):
    """返回单位向量；退化向量返回 None."""
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm < 1e-9:
        return None
    return value / norm


def _angle_deg(first, second):
    """两个单位方向的夹角 [deg]，点积先裁剪避免浮点越界."""
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def summarize_view_coverage(frames, target_center, cluster_angle_deg=5.0):
    """
    汇总目标到相机的观察方向覆盖与逐机位质量.

    同一机位停留期间会连续接受多帧：按帧计算会让兄弟帧互为最近邻
    （≈0°），view_count 虚高、mean_nearest_baseline 被稀释成接近 0。
    这里先按「目标→相机」方向贪心聚类（夹角 ≤ cluster_angle_deg 同机位，
    代表方向取成员均值），view_count/基线/分布指标按机位代表计算，
    反映真实机位分布；原始总帧数放在 frame_count 供参考。

    Args:
        frames: 采帧列表（需含 camera_position_base/stamp/valid_depth_ratio）.
        target_center: (3,) 目标中心（base 系 [m]）；None 返回 invalid.
        cluster_angle_deg: 机位聚类角阈值（默认 5°）.

    Returns
    -------
        dict：valid/view_count(机位数)/max_baseline_deg/
        mean_nearest_baseline_deg/range 与深度统计/views(机位代表列表)。

    """
    if target_center is None:
        return {
            'valid': False,
            'reason': 'target_center_unavailable',
            'view_count': 0,
            'frame_count': len(frames),
            'views': [],
        }
    center = np.asarray(target_center, dtype=np.float64).reshape(3)
    frame_views = []
    for index, frame in enumerate(frames):
        position = np.asarray(frame.camera_position_base,
                              dtype=np.float64).reshape(3)
        offset = position - center
        direction = _safe_unit(offset)
        if direction is None:
            continue
        frame_views.append({
            'index': index,
            'stamp_sec': float(frame.stamp),
            'position': position,
            'direction': direction,
            'range_m': float(np.linalg.norm(offset)),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'registration': dict(frame.registration),
            'diagnostic_flags': list(frame.diagnostic_flags),
        })
    if not frame_views:
        return {
            'valid': False,
            'reason': 'no_valid_camera_direction',
            'view_count': 0,
            'frame_count': len(frames),
            'views': [],
        }

    # 按时间序贪心聚类：与既有机位代表方向夹角 ≤ 阈值即并入该机位
    clusters = []  # [{'rep': 单位方向, 'members': [frame_view, ...]}]
    for view in frame_views:
        for cluster in clusters:
            if _angle_deg(view['direction'], cluster['rep']) <= cluster_angle_deg:
                cluster['members'].append(view)
                merged = np.sum(
                    [m['direction'] for m in cluster['members']], axis=0)
                cluster['rep'] = _safe_unit(merged)
                break
        else:
            clusters.append({'rep': view['direction'], 'members': [view]})

    views = []
    directions = []
    for cluster in clusters:
        members = cluster['members']
        mean_position = np.mean([m['position'] for m in members], axis=0)
        first = members[0]
        views.append({
            'index': first['index'],
            'stamp_sec': first['stamp_sec'],
            'camera_position_base': [float(v) for v in mean_position],
            'direction_target_to_camera': [float(v) for v in cluster['rep']],
            'range_m': float(np.mean([m['range_m'] for m in members])),
            'valid_depth_ratio': float(np.mean(
                [m['valid_depth_ratio'] for m in members])),
            'frame_count': len(members),
            'registration': first['registration'],
            'diagnostic_flags': first['diagnostic_flags'],
        })
        directions.append(cluster['rep'])

    pair_angles = []
    nearest_angles = []
    for i, direction in enumerate(directions):
        distances = [
            _angle_deg(direction, other)
            for j, other in enumerate(directions) if i != j
        ]
        if distances:
            nearest_angles.append(min(distances))
            pair_angles.extend(distances)
    ranges = [item['range_m'] for item in views]
    depth_ratios = [item['valid_depth_ratio'] for item in views]
    return {
        'valid': True,
        'reason': 'ok',
        'view_count': len(views),
        'frame_count': len(frame_views),
        'max_baseline_deg': (0.0 if not pair_angles else max(pair_angles)),
        'mean_nearest_baseline_deg': (
            0.0 if not nearest_angles else float(np.mean(nearest_angles))),
        'range_min_m': min(ranges),
        'range_max_m': max(ranges),
        'range_mean_m': float(np.mean(ranges)),
        'valid_depth_ratio_mean': float(np.mean(depth_ratios)),
        'valid_depth_ratio_min': min(depth_ratios),
        'views': views,
    }
