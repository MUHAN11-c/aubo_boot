"""Zero-ROS synthetic AX=XB tests for the eye-in-hand solver."""

from aubo_hand_eye_calibration.solver import (
    CalibrationSample,
    solve_hand_eye,
    VALID_METHODS,
)
from aubo_hand_eye_calibration.transforms import (
    from_se3_vector,
    inverse,
    make_transform,
    se3_vector,
    transform_error,
)
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def random_transform(rng):
    rotation = Rotation.random(random_state=rng).as_matrix()
    return make_transform(rotation, rng.uniform(-0.3, 0.3, 3))


def perturb(transform, rng, translation_sigma, rotation_sigma):
    delta = np.r_[
        rng.normal(0.0, translation_sigma, 3),
        rng.normal(0.0, rotation_sigma, 3),
    ]
    return from_se3_vector(se3_vector(transform) + delta)


def synthetic_samples(count=20, noise=0.0, seed=7):
    """
    按真值 X (gripper_from_camera) 与 B (base_from_target) 构造样本.

    约定: base_from_gripper @ gripper_from_camera @ camera_from_target
          = base_from_target (solver.CalibrationSample 注释)。
    """
    rng = np.random.default_rng(seed)
    true_camera = make_transform(
        Rotation.from_euler('xyz', [5.0, 100.0, -3.0], degrees=True).as_matrix(),
        [0.045, 0.108, 0.002],
    )
    true_target = random_transform(rng)
    samples = []
    for index in range(count):
        base_from_gripper = random_transform(rng)
        camera_from_target = (
            inverse(true_camera) @ inverse(base_from_gripper) @ true_target)
        if noise:
            camera_from_target = perturb(
                camera_from_target, rng, noise, noise * 0.5)
        samples.append(CalibrationSample(
            base_from_gripper,
            camera_from_target,
            reprojection_rms_px=0.30,
            sample_id=f'pose_{index + 1:02d}',
        ))
    return samples, true_camera


def assert_recovers(true_camera, result, max_translation_m, max_rotation_deg):
    translation_m, rotation_deg = transform_error(
        true_camera, result.gripper_from_camera)
    assert translation_m < max_translation_m, (
        f'translation error {translation_m}m')
    assert rotation_deg < max_rotation_deg, (
        f'rotation error {rotation_deg}deg')


def test_all_methods_are_supported():
    assert VALID_METHODS == (
        'auto', 'tsai', 'park', 'horaud', 'andreff', 'daniilidis')


def test_noise_free_recovery_all_methods():
    samples, true_camera = synthetic_samples()
    for method in VALID_METHODS[1:]:
        result = solve_hand_eye(samples, method=method)
        assert_recovers(true_camera, result, 1e-9, 1e-6)


def test_noisy_recovery_auto():
    samples, true_camera = synthetic_samples(noise=1e-4)
    result = solve_hand_eye(samples, method='auto')
    assert_recovers(true_camera, result, 1e-3, 0.5)
    assert result.passed, result.failures
    assert result.method_scores


def test_planted_outlier_is_rejected():
    samples, true_camera = synthetic_samples(seed=11)
    corrupted = from_se3_vector(
        se3_vector(samples[5].camera_from_target) + np.array([0.02, 0, 0, 0, 0, 0]))
    samples[5] = CalibrationSample(
        samples[5].base_from_gripper,
        corrupted,
        samples[5].reprojection_rms_px,
        samples[5].sample_id,
    )
    result = solve_hand_eye(samples, method='auto')
    assert 5 in result.rejected_indices
    assert len(result.accepted_indices) == len(samples) - 1
    assert_recovers(true_camera, result, 1e-3, 0.5)


def test_min_samples_gate_fails():
    samples, _ = synthetic_samples(count=20)
    result = solve_hand_eye(samples, min_samples=30, method='auto')
    assert not result.passed
    assert any('accepted samples' in failure for failure in result.failures)


def test_rotation_span_gate_fails():
    # 所有腕部姿态共享同一姿态基线, 仅平移变化 => 旋转跨度不足
    rng = np.random.default_rng(3)
    base_rotation = Rotation.from_euler('xyz', [10, 20, 30], degrees=True
                                        ).as_matrix()
    samples = []
    for index in range(12):
        base_from_gripper = make_transform(
            Rotation.from_rotvec(
                Rotation.from_matrix(base_rotation).as_rotvec()
                + rng.normal(0.0, 0.002, 3)).as_matrix(),
            rng.uniform(-0.2, 0.2, 3),
        )
        camera_from_target = make_transform(
            Rotation.from_rotvec(rng.normal(0.0, 0.01, 3)).as_matrix(),
            rng.uniform(0.1, 0.3, 3),
        )
        samples.append(CalibrationSample(
            base_from_gripper, camera_from_target, 0.3, f'pose_{index:02d}'))
    result = solve_hand_eye(samples, method='auto')
    assert not result.passed
    assert any('rotation span' in failure for failure in result.failures)


def test_invalid_method_raises():
    samples, _ = synthetic_samples()
    with pytest.raises(ValueError, match='unknown hand-eye method'):
        solve_hand_eye(samples, method='nope')


def test_too_few_samples_raises():
    samples, _ = synthetic_samples(count=2)
    with pytest.raises(ValueError, match='at least three samples'):
        solve_hand_eye(samples)
