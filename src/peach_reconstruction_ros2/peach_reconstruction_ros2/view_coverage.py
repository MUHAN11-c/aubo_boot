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


def summarize_view_coverage(frames, target_center):
    """汇总目标到相机的观察方向、距离、角基线和逐帧质量."""
    if target_center is None:
        return {
            'valid': False,
            'reason': 'target_center_unavailable',
            'view_count': len(frames),
            'views': [],
        }
    center = np.asarray(target_center, dtype=np.float64).reshape(3)
    views = []
    directions = []
    for index, frame in enumerate(frames):
        position = np.asarray(frame.camera_position_base,
                              dtype=np.float64).reshape(3)
        offset = position - center
        direction = _safe_unit(offset)
        if direction is None:
            continue
        directions.append(direction)
        views.append({
            'index': index,
            'stamp_sec': float(frame.stamp),
            'camera_position_base': [float(v) for v in position],
            'direction_target_to_camera': [float(v) for v in direction],
            'range_m': float(np.linalg.norm(offset)),
            'valid_depth_ratio': float(frame.valid_depth_ratio),
            'registration': dict(frame.registration),
            'diagnostic_flags': list(frame.diagnostic_flags),
        })
    if not directions:
        return {
            'valid': False,
            'reason': 'no_valid_camera_direction',
            'view_count': len(frames),
            'views': [],
        }

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
