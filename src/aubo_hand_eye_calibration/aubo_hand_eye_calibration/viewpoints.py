"""
相机无关的自动标定视点生成 (纯核, 零 ROS).

不写死任何相机型号/FOV/分辨率常量: 全部几何量来自调用方传入的内参
(K, D) 与图像尺寸; 视点距离按「棋盘格画面占比」反推, 自动适配任意
焦距。FOV 保证是**视点位的静态保证**(投影掩码), 视点间 transit 不做
承诺 —— 采集仅发生在 settle 之后。
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .transforms import inverse, make_transform

DEFAULT_POLAR_DEG = (0.0, 15.0, 30.0, 45.0)
DEFAULT_AZIMUTH_STEP_DEG = 60.0
DEFAULT_FILL_FRACTIONS = (0.40, 0.55, 0.70)
DEFAULT_MARGIN_FRACTION = 0.08
DEFAULT_BORDER_SQUARES = 1.0
DEFAULT_DISTANCE_RANGE_M = (0.15, 1.20)


def board_outline(board, border_squares=DEFAULT_BORDER_SQUARES):
    """棋盘物理外沿角点 (target 系): 内角点格向外扩 border_squares 格."""
    square = board.square_size_m
    x_lo = -border_squares * square
    x_hi = (board.columns - 1 + border_squares) * square
    y_lo = -border_squares * square
    y_hi = (board.rows - 1 + border_squares) * square
    return np.array([
        [x_lo, y_lo, 0.0],
        [x_hi, y_lo, 0.0],
        [x_hi, y_hi, 0.0],
        [x_lo, y_hi, 0.0],
    ], dtype=np.float64)


def board_frame(board, base_from_target):
    """返回 (板心, 板法向, 板面内 x/y 轴), 均为 base 系表达."""
    rotation = base_from_target[:3, :3]
    translation = base_from_target[:3, 3]
    square = board.square_size_m
    center = rotation @ np.array([
        (board.columns - 1) * square / 2,
        (board.rows - 1) * square / 2,
        0.0,
    ]) + translation
    return (
        center,
        rotation @ np.array([0.0, 0.0, 1.0]),
        rotation @ np.array([1.0, 0.0, 0.0]),
        rotation @ np.array([0.0, 1.0, 0.0]),
    )


def look_at_transform(eye, target, image_down_hint):
    """
    构造光学系位姿 T_base_cameraOptical (z 前向 target, x 右, y 下).

    image_down_hint 是图像「向下」方向的世界系近似参考; 与视线正交化后
    作 y_cam (x_cam = y x z, 满足光学系手性 x cross y = z)。
    """
    forward = np.asarray(target, dtype=np.float64) - np.asarray(
        eye, dtype=np.float64)
    distance = float(np.linalg.norm(forward))
    if distance < 1e-9:
        raise ValueError('look_at: eye 与 target 重合')
    z_axis = forward / distance
    hint = np.asarray(image_down_hint, dtype=np.float64)
    hint_norm = float(np.linalg.norm(hint))
    if hint_norm < 1e-9:
        raise ValueError('look_at: image_down_hint 为零向量')
    y_axis = hint / hint_norm
    y_axis = y_axis - np.dot(y_axis, z_axis) * z_axis
    if float(np.linalg.norm(y_axis)) < 1e-6:
        raise ValueError('look_at: image_down_hint 与视线平行')
    y_axis = y_axis / np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return make_transform(np.column_stack([x_axis, y_axis, z_axis]), eye)


class FovMargin:
    """投影掩码: target 系点集经 T_camera_target 投影后须落在带余量图内."""

    def __init__(self, camera_matrix, distortion, image_size, points_target,
                 margin_fraction=DEFAULT_MARGIN_FRACTION, min_depth_m=0.05):
        self.camera_matrix = np.asarray(
            camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
        self.width, self.height = (int(image_size[0]), int(image_size[1]))
        self.margin_x = margin_fraction * self.width
        self.margin_y = margin_fraction * self.height
        self.points = np.asarray(points_target, dtype=np.float64).reshape(-1, 3)
        self.min_depth_m = float(min_depth_m)

    def margin_px(self, camera_from_target):
        """最小边界余量 (px, 可负); 点在相机后方时为 -inf."""
        points_camera = (
            camera_from_target[:3, :3] @ self.points.T
            + camera_from_target[:3, 3:4]).T
        if np.any(points_camera[:, 2] < self.min_depth_m):
            return float('-inf')
        rotation, _ = cv2.Rodrigues(camera_from_target[:3, :3])
        projected, _ = cv2.projectPoints(
            self.points.astype(np.float32),
            rotation,
            camera_from_target[:3, 3],
            self.camera_matrix,
            self.distortion,
        )
        projected = projected.reshape(-1, 2)
        u, v = projected[:, 0], projected[:, 1]
        return float(np.min(np.minimum(
            np.minimum(u - self.margin_x, self.width - self.margin_x - u),
            np.minimum(v - self.margin_y, self.height - self.margin_y - v))))

    def __call__(self, camera_from_target):
        return self.margin_px(camera_from_target) >= 0.0


def distances_for_fill(camera_matrix, board, image_size,
                       fill_fractions=DEFAULT_FILL_FRACTIONS,
                       border_squares=DEFAULT_BORDER_SQUARES,
                       distance_range_m=DEFAULT_DISTANCE_RANGE_M):
    """按「板宽占画面宽的比例」反推距离, 而非绝对米数 (适配任意焦距)."""
    fx = float(np.asarray(camera_matrix).reshape(3, 3)[0, 0])
    width = float(image_size[0])
    extent = (board.columns - 1 + 2 * border_squares) * board.square_size_m
    return [
        float(np.clip(fx * extent / (fill * width), *distance_range_m))
        for fill in fill_fractions
    ]


@dataclass(frozen=True)
class Viewpoint:
    base_from_wrist: np.ndarray
    base_from_camera_optical: np.ndarray
    polar_deg: float
    azimuth_deg: float
    distance_m: float
    fill_fraction: float
    margin_px: float

    def metadata(self):
        return {
            'polar_deg': round(float(self.polar_deg), 2),
            'azimuth_deg': round(float(self.azimuth_deg), 2),
            'distance_m': round(float(self.distance_m), 4),
            'fill_fraction': float(self.fill_fraction),
            'margin_px': round(float(self.margin_px), 1),
        }


def candidate_viewpoints(
    board,
    base_from_target,
    wrist_from_camera_optical,
    camera_matrix,
    distortion,
    image_size,
    polar_degrees=DEFAULT_POLAR_DEG,
    azimuth_step_deg=DEFAULT_AZIMUTH_STEP_DEG,
    fill_fractions=DEFAULT_FILL_FRACTIONS,
    margin_fraction=DEFAULT_MARGIN_FRACTION,
    border_squares=DEFAULT_BORDER_SQUARES,
    distance_range_m=DEFAULT_DISTANCE_RANGE_M,
):
    """
    在固定棋盘格周围生成通过 FOV 掩码的候选视点 (腕部位姿列表).

    视点位于板法向半球上: 极角 (相对法向) x 方位角 x 距离(按占比反推);
    相机光轴指向板心, 板面 +y 作为图像「上」参考 (角点接近正立, 利于检测)。
    """
    center, normal, axis_x, axis_y = board_frame(board, base_from_target)
    mask = FovMargin(
        camera_matrix, distortion, image_size,
        board_outline(board, border_squares), margin_fraction)
    distances = distances_for_fill(
        camera_matrix, board, image_size, fill_fractions,
        border_squares, distance_range_m)
    camera_from_wrist = inverse(wrist_from_camera_optical)
    candidates = []
    steps = max(1, int(round(360.0 / float(azimuth_step_deg))))
    for polar_deg in polar_degrees:
        polar = np.radians(float(polar_deg))
        for step in range(steps if polar_deg else 1):
            azimuth_deg = step * float(azimuth_step_deg)
            azimuth = np.radians(azimuth_deg)
            in_plane = np.cos(azimuth) * axis_x + np.sin(azimuth) * axis_y
            direction = np.sin(polar) * in_plane + np.cos(polar) * normal
            for distance, fill in zip(distances, fill_fractions):
                eye = center + distance * direction
                base_from_camera = look_at_transform(
                    eye, center, -axis_y)
                camera_from_target = (
                    inverse(base_from_camera) @ base_from_target)
                margin = mask.margin_px(camera_from_target)
                if margin < 0.0:
                    continue
                candidates.append(Viewpoint(
                    base_from_wrist=base_from_camera @ camera_from_wrist,
                    base_from_camera_optical=base_from_camera,
                    polar_deg=float(polar_deg),
                    azimuth_deg=azimuth_deg,
                    distance_m=distance,
                    fill_fraction=float(fill),
                    margin_px=margin,
                ))
    return candidates


def _rotation_degrees(first, second):
    return float(np.degrees(np.linalg.norm(
        (first.inv() * second).as_rotvec())))


def select_diverse(viewpoints, count, min_span_deg):
    """
    贪心最远点选择, 最大化腕部旋转多样性.

    返回 (选中列表, 旋转跨度 deg); 候选顺序确定 => 结果可复现。
    """
    if not viewpoints:
        return [], 0.0
    rotations = [
        Rotation.from_matrix(v.base_from_wrist[:3, :3])
        for v in viewpoints
    ]
    selected = [0]
    while len(selected) < min(count, len(viewpoints)):
        best_index, best_distance = None, -1.0
        for index in range(len(viewpoints)):
            if index in selected:
                continue
            closest = min(
                _rotation_degrees(rotations[index], rotations[kept])
                for kept in selected)
            if closest > best_distance:
                best_index, best_distance = index, closest
        selected.append(best_index)
    chosen = [viewpoints[index] for index in selected]
    chosen_rotations = [rotations[index] for index in selected]
    span = 0.0
    for i, first in enumerate(chosen_rotations):
        for second in chosen_rotations[i + 1:]:
            span = max(span, _rotation_degrees(first, second))
    # span 语义与 solver.rotation_span 一致: 腕部姿态两两最大夹角
    return chosen, span
