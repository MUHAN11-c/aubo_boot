# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import unittest

from aubo_hand_eye_calibration.transforms import (
    from_se3_vector,
    inverse,
    make_transform,
    mean_transform,
    se3_vector,
    transform_error,
    transform_from_xyz_quat,
    transform_to_xyz_quat,
)
import numpy as np
from scipy.spatial.transform import Rotation


class TransformsTest(unittest.TestCase):
    def test_inverse_roundtrip(self):
        transform = make_transform(
            Rotation.from_euler('xyz', [0.3, -0.2, 0.5]).as_matrix(),
            [0.1, -0.2, 0.3],
        )
        product = inverse(transform) @ transform
        self.assertTrue(np.allclose(product, np.eye(4), atol=1e-12))

    def test_inverse_rejects_non_rotation(self):
        bad = make_transform(np.diag([2.0, 1.0, 1.0]), [0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            inverse(bad)

    def test_xyz_quat_roundtrip(self):
        transform = transform_from_xyz_quat(
            [0.1, 0.2, 0.3],
            Rotation.from_euler('z', 0.4).as_quat(),
        )
        xyz, quaternion = transform_to_xyz_quat(transform)
        rebuilt = transform_from_xyz_quat(xyz, quaternion)
        translation_m, rotation_deg = transform_error(transform, rebuilt)
        self.assertLess(translation_m, 1e-12)
        self.assertLess(rotation_deg, 1e-9)

    def test_se3_vector_roundtrip(self):
        transform = make_transform(
            Rotation.from_euler('xyz', [0.1, 0.2, -0.3]).as_matrix(),
            [0.4, -0.1, 0.05],
        )
        rebuilt = from_se3_vector(se3_vector(transform))
        translation_m, rotation_deg = transform_error(transform, rebuilt)
        self.assertLess(translation_m, 1e-12)
        self.assertLess(rotation_deg, 1e-9)

    def test_mean_transform_of_identical_is_identity_ish(self):
        transform = make_transform(
            Rotation.from_euler('y', 0.3).as_matrix(), [0.1, 0.0, 0.2])
        averaged = mean_transform([transform, transform, transform])
        translation_m, rotation_deg = transform_error(transform, averaged)
        self.assertLess(translation_m, 1e-12)
        self.assertLess(rotation_deg, 1e-9)

    def test_mean_transform_requires_input(self):
        with self.assertRaises(ValueError):
            mean_transform([])


if __name__ == '__main__':
    unittest.main()
