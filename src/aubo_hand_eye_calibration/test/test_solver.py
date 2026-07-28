# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import unittest

from aubo_hand_eye_calibration.solver import (
    CalibrationSample,
    solve_hand_eye,
)
from aubo_hand_eye_calibration.transforms import (
    inverse,
    make_transform,
    transform_error,
)
import numpy as np
from scipy.spatial.transform import Rotation


def random_transform(random, translation_scale=0.5, rotation_scale=1.0):
    rotation = (
        Rotation.random(random_state=random)
        * Rotation.from_rotvec(
            random.normal(0.0, rotation_scale, 3))
    )
    return make_transform(
        rotation.as_matrix(),
        random.uniform(-translation_scale, translation_scale, 3),
    )


def noisy_transform(random, transform, translation_std=0.0, rotation_std_deg=0.0):
    result = transform.copy()
    result[:3, 3] += random.normal(0.0, translation_std, 3)
    perturb = Rotation.from_rotvec(
        random.normal(0.0, np.radians(rotation_std_deg), 3))
    result[:3, :3] = result[:3, :3] @ perturb.as_matrix()
    return result


def make_samples(random, wrist_from_camera, base_from_target, count,
                 translation_std=0.0, rotation_std_deg=0.0):
    samples = []
    for index in range(count):
        base_from_wrist = random_transform(random)
        camera_from_target = (
            inverse(wrist_from_camera)
            @ inverse(base_from_wrist)
            @ base_from_target
        )
        camera_from_target = noisy_transform(
            random, camera_from_target, translation_std, rotation_std_deg)
        samples.append(CalibrationSample(
            base_from_wrist, camera_from_target, 0.2, str(index)))
    return samples


class SolverTest(unittest.TestCase):
    def test_recovers_known_eye_in_hand_transform(self):
        random = np.random.default_rng(7)
        wrist_from_camera = make_transform(
            Rotation.from_euler('xyz', [0.2, -0.1, 0.35]).as_matrix(),
            [0.03, -0.02, 0.08],
        )
        base_from_target = make_transform(
            Rotation.from_euler('xyz', [-0.1, 0.05, 0.4]).as_matrix(),
            [0.55, 0.10, 0.20],
        )
        samples = make_samples(
            random, wrist_from_camera, base_from_target, 20)

        result = solve_hand_eye(samples)
        translation_m, rotation_deg = transform_error(
            wrist_from_camera, result.gripper_from_camera)
        self.assertLess(translation_m, 1e-4)
        self.assertLess(rotation_deg, 0.01)
        self.assertTrue(result.passed)

    def test_tolerates_gaussian_noise(self):
        random = np.random.default_rng(11)
        wrist_from_camera = make_transform(
            Rotation.from_euler('xyz', [0.15, -0.05, 0.3]).as_matrix(),
            [0.03, -0.02, 0.08],
        )
        base_from_target = make_transform(
            np.eye(3), [0.5, 0.1, 0.2])
        samples = make_samples(
            random, wrist_from_camera, base_from_target, 20,
            translation_std=0.001, rotation_std_deg=0.1)

        result = solve_hand_eye(samples)
        translation_m, rotation_deg = transform_error(
            wrist_from_camera, result.gripper_from_camera)
        self.assertLess(translation_m, 0.003)
        self.assertLess(rotation_deg, 0.5)
        self.assertTrue(result.passed)

    def test_rejects_corrupted_sample(self):
        random = np.random.default_rng(9)
        wrist_from_camera = make_transform(np.eye(3), [0.02, 0.01, 0.1])
        base_from_target = make_transform(np.eye(3), [0.5, 0.0, 0.2])
        samples = make_samples(
            random, wrist_from_camera, base_from_target, 18)
        samples[5] = CalibrationSample(
            samples[5].base_from_gripper,
            samples[5].camera_from_target + make_transform(
                np.eye(3), [0.2, -0.1, 0.1]),
            0.2, '5')

        result = solve_hand_eye(samples)
        self.assertIn(5, result.rejected_indices)
        self.assertGreaterEqual(len(result.accepted_indices), 12)

    def test_fails_on_insufficient_rotation_span(self):
        random = np.random.default_rng(13)
        wrist_from_camera = make_transform(np.eye(3), [0.02, 0.01, 0.1])
        base_from_target = make_transform(np.eye(3), [0.5, 0.0, 0.2])
        # 所有腕部姿态几乎相同 -> 旋转跨度接近 0
        base_pose = random_transform(random)
        samples = []
        for index in range(15):
            base_from_wrist = noisy_transform(
                random, base_pose, 0.0, 1.0)
            camera_from_target = (
                inverse(wrist_from_camera)
                @ inverse(base_from_wrist)
                @ base_from_target
            )
            samples.append(CalibrationSample(
                base_from_wrist, camera_from_target, 0.2, str(index)))

        try:
            result = solve_hand_eye(samples)
        except RuntimeError:
            # 近重复姿态可能导致全部 OpenCV 方法数值失败, 同样视为拒绝
            return
        self.assertFalse(result.passed)
        self.assertTrue(any(
            'rotation span' in failure for failure in result.failures))

    def test_fails_on_insufficient_samples(self):
        random = np.random.default_rng(17)
        wrist_from_camera = make_transform(np.eye(3), [0.02, 0.01, 0.1])
        base_from_target = make_transform(np.eye(3), [0.5, 0.0, 0.2])
        samples = make_samples(
            random, wrist_from_camera, base_from_target, 5)

        result = solve_hand_eye(samples)
        self.assertFalse(result.passed)
        self.assertTrue(any(
            'accepted samples' in failure for failure in result.failures))

    def test_requires_at_least_three_samples(self):
        random = np.random.default_rng(19)
        wrist_from_camera = make_transform(np.eye(3), [0.02, 0.01, 0.1])
        base_from_target = make_transform(np.eye(3), [0.5, 0.0, 0.2])
        samples = make_samples(
            random, wrist_from_camera, base_from_target, 2)
        with self.assertRaises(ValueError):
            solve_hand_eye(samples)


if __name__ == '__main__':
    unittest.main()
