# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Robust eye-in-hand solver with explicit transform conventions."""

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .transforms import (
    from_se3_vector,
    inverse,
    mean_transform,
    se3_vector,
    transform_error,
)


@dataclass(frozen=True)
class CalibrationSample:
    # T_base_gripper: gripper coordinates transformed into the base frame.
    base_from_gripper: np.ndarray
    # T_camera_target: checkerboard coordinates transformed into camera frame.
    camera_from_target: np.ndarray
    reprojection_rms_px: float = 0.0
    sample_id: str = ''


@dataclass
class CalibrationResult:
    gripper_from_camera: np.ndarray
    base_from_target: np.ndarray
    method: str
    accepted_indices: list[int]
    rejected_indices: list[int]
    translation_rms_m: float
    rotation_rms_deg: float
    reprojection_rms_px: float
    rotation_span_deg: float
    passed: bool = False
    failures: list[str] = field(default_factory=list)
    # 最终解下全部样本的逐样本一致性残差:
    # {index, accepted, translation_m, rotation_deg}
    sample_errors: list[dict] = field(default_factory=list)
    # 每个参与求解方法的打分:
    # {method, translation_rms_m, rotation_rms_deg, score, selected, failed}
    method_scores: list[dict] = field(default_factory=list)
    # Huber 精化统计: {converged, initial_cost, final_cost, nfev};
    # 未跑精化时 converged=False、cost 为 None
    refine_stats: dict = field(default_factory=dict)


_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}

# 合法的 method 取值; 'auto' 表示 5 方法全跑按归一化分数取最优
VALID_METHODS = ('auto',) + tuple(_METHODS)


def _opencv_solve(samples, method):
    rotations_gripper_to_base = [
        sample.base_from_gripper[:3, :3] for sample in samples]
    translations_gripper_to_base = [
        sample.base_from_gripper[:3, 3] for sample in samples]
    rotations_target_to_camera = [
        sample.camera_from_target[:3, :3] for sample in samples]
    translations_target_to_camera = [
        sample.camera_from_target[:3, 3] for sample in samples]
    rotation, translation = cv2.calibrateHandEye(
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_target_to_camera,
        translations_target_to_camera,
        method=method,
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def _target_estimates(samples, gripper_from_camera):
    return [
        sample.base_from_gripper
        @ gripper_from_camera
        @ sample.camera_from_target
        for sample in samples
    ]


def _consistency(samples, gripper_from_camera):
    targets = _target_estimates(samples, gripper_from_camera)
    average = mean_transform(targets)
    errors = np.array(
        [transform_error(average, target) for target in targets],
        dtype=np.float64,
    )
    return average, errors


def _mad_inliers(errors):
    if len(errors) < 5:
        return np.ones(len(errors), dtype=bool)
    normalized = errors.copy()
    for column in range(2):
        median = np.median(normalized[:, column])
        mad = np.median(np.abs(normalized[:, column] - median))
        scale = max(1.4826 * mad, 1e-9)
        normalized[:, column] = np.abs(
            normalized[:, column] - median) / scale
    return np.max(normalized, axis=1) <= 3.5


def _refine(samples, initial_camera, initial_target):
    x0 = np.r_[se3_vector(initial_camera), se3_vector(initial_target)]

    def residual(vector):
        gripper_from_camera = from_se3_vector(vector[:6])
        base_from_target = from_se3_vector(vector[6:])
        values = []
        for sample in samples:
            estimate = (
                sample.base_from_gripper
                @ gripper_from_camera
                @ sample.camera_from_target
            )
            delta = inverse(base_from_target) @ estimate
            # Metres and radians are both physically meaningful residuals.
            values.extend(delta[:3, 3])
            values.extend(
                Rotation.from_matrix(delta[:3, :3]).as_rotvec())
        return np.asarray(values)

    # 精化前的初始代价 (0.5 * 残差平方和, 与 least_squares.cost 同定义)
    initial_cost = 0.5 * float(np.sum(residual(x0) ** 2))
    optimized = least_squares(
        residual, x0, loss='huber', f_scale=0.01, max_nfev=500)
    stats = {
        'converged': bool(optimized.success),
        'initial_cost': initial_cost,
        'final_cost': float(optimized.cost),
        'nfev': int(optimized.nfev),
    }
    if not optimized.success:
        # 精化未收敛时回退到初始解, 避免静默采用病态结果
        return initial_camera, initial_target, stats
    return (
        from_se3_vector(optimized.x[:6]),
        from_se3_vector(optimized.x[6:]),
        stats,
    )


def _rotation_span(samples):
    rotations = [Rotation.from_matrix(s.base_from_gripper[:3, :3])
                 for s in samples]
    maximum = 0.0
    for index, first in enumerate(rotations):
        for second in rotations[index + 1:]:
            maximum = max(
                maximum,
                np.degrees(np.linalg.norm((first.inv() * second).as_rotvec())),
            )
    return float(maximum)


def solve_hand_eye(
    samples,
    min_samples=12,
    max_reprojection_rms_px=1.0,
    max_translation_rms_m=0.003,
    max_rotation_rms_deg=0.5,
    min_rotation_span_deg=20.0,
    method='auto',
):
    samples = list(samples)
    if len(samples) < 3:
        raise ValueError('at least three samples are required by OpenCV')
    if method not in VALID_METHODS:
        raise ValueError(
            f'unknown hand-eye method: {method} '
            f'(valid: {"/".join(VALID_METHODS)})')

    # 'auto' 时 5 方法全跑按归一化分数取最优; 指定方法时只跑该方法,
    # 失败直接报错 (不再静默换方法)
    names = list(_METHODS) if method == 'auto' else [method]
    candidates = []
    method_scores = []
    for name in names:
        try:
            estimate = _opencv_solve(samples, _METHODS[name])
            if not np.all(np.isfinite(estimate)):
                raise RuntimeError('non-finite estimate')
            target, errors = _consistency(samples, estimate)
            # 平移/旋转分别归一化到各自门限的比例再相加, 消除量纲任意性
            translation_rms = float(np.sqrt(np.mean(errors[:, 0] ** 2)))
            rotation_rms = float(np.sqrt(np.mean(errors[:, 1] ** 2)))
            score = (
                translation_rms / max(max_translation_rms_m, 1e-9)
                + rotation_rms / max(max_rotation_rms_deg, 1e-9)
            )
            candidates.append((score, name, estimate, target, errors))
            method_scores.append({
                'method': name,
                'translation_rms_m': translation_rms,
                'rotation_rms_deg': rotation_rms,
                'score': score,
                'selected': False,
                'failed': False,
            })
        except (cv2.error, RuntimeError) as error:
            if method != 'auto':
                raise RuntimeError(
                    f"OpenCV hand-eye method '{name}' failed: {error}"
                ) from error
            method_scores.append({
                'method': name,
                'translation_rms_m': None,
                'rotation_rms_deg': None,
                'score': None,
                'selected': False,
                'failed': True,
            })
    if not candidates:
        raise RuntimeError('all OpenCV hand-eye methods failed')

    _, method, initial_camera, initial_target, errors = min(
        candidates, key=lambda item: item[0])
    for entry in method_scores:
        if entry['method'] == method and not entry['failed']:
            entry['selected'] = True
    inliers = _mad_inliers(errors)
    accepted = np.flatnonzero(inliers).tolist()
    inlier_samples = [samples[index] for index in accepted]
    if len(inlier_samples) >= 3:
        camera, target, refine_stats = _refine(
            inlier_samples, initial_camera, initial_target)
    else:
        camera, target = initial_camera, initial_target
        # 内点不足未跑精化时的兜底统计
        refine_stats = {
            'converged': False,
            'initial_cost': None,
            'final_cost': None,
            'nfev': 0,
        }

    # 精化后按精化解的残差复核一次离群剔除
    _, refined_errors = _consistency(samples, camera)
    refined_inliers = _mad_inliers(refined_errors)
    if int(refined_inliers.sum()) >= 3:
        accepted = np.flatnonzero(refined_inliers).tolist()
        inlier_samples = [samples[index] for index in accepted]
    rejected = [
        index for index in range(len(samples)) if index not in accepted]
    sample_errors = [
        {
            'index': index,
            'accepted': index in accepted,
            'translation_m': float(refined_errors[index, 0]),
            'rotation_deg': float(refined_errors[index, 1]),
        }
        for index in range(len(samples))
    ]
    _, final_errors = _consistency(inlier_samples, camera)
    translation_rms = float(np.sqrt(np.mean(final_errors[:, 0] ** 2)))
    rotation_rms = float(np.sqrt(np.mean(final_errors[:, 1] ** 2)))
    reprojection_rms = float(np.sqrt(np.mean([
        sample.reprojection_rms_px ** 2 for sample in inlier_samples
    ])))
    rotation_span = _rotation_span(inlier_samples)

    failures = []
    if len(inlier_samples) < min_samples:
        failures.append(f'accepted samples {len(inlier_samples)} < {min_samples}')
    if reprojection_rms > max_reprojection_rms_px:
        failures.append(
            f'reprojection RMS {reprojection_rms:.3f}px exceeds '
            f'{max_reprojection_rms_px:.3f}px')
    if translation_rms > max_translation_rms_m:
        failures.append(
            f'translation RMS {translation_rms:.6f}m exceeds '
            f'{max_translation_rms_m:.6f}m')
    if rotation_rms > max_rotation_rms_deg:
        failures.append(
            f'rotation RMS {rotation_rms:.3f}deg exceeds '
            f'{max_rotation_rms_deg:.3f}deg')
    if rotation_span < min_rotation_span_deg:
        failures.append(
            f'rotation span {rotation_span:.1f}deg below '
            f'{min_rotation_span_deg:.1f}deg')

    return CalibrationResult(
        gripper_from_camera=camera,
        base_from_target=target,
        method=method,
        accepted_indices=accepted,
        rejected_indices=rejected,
        translation_rms_m=translation_rms,
        rotation_rms_deg=rotation_rms,
        reprojection_rms_px=reprojection_rms,
        rotation_span_deg=rotation_span,
        passed=not failures,
        failures=failures,
        sample_errors=sample_errors,
        method_scores=method_scores,
        refine_stats=refine_stats,
    )
