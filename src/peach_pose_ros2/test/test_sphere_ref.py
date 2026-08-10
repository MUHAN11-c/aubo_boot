"""offline.sphere_ref：合成点云球拟合参考."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.offline.sphere_ref import fit_sphere_reference


class SphereReferenceTest(unittest.TestCase):
    def _sphere_points(self, center, radius, n=400, noise=0.001, seed=0):
        rng = np.random.default_rng(seed)
        dirs = rng.normal(size=(n, 3))
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        return center + dirs * radius + rng.normal(scale=noise, size=(n, 3))

    def test_recovers_synthetic_sphere(self):
        center = np.array([0.1, 0.2, 0.8])
        pts = self._sphere_points(center, 0.035)
        c, r = fit_sphere_reference(pts)
        self.assertIsNotNone(c)
        self.assertLess(np.linalg.norm(c - center), 0.005)
        self.assertAlmostEqual(r, 0.035, delta=0.003)

    def test_outliers_are_rejected(self):
        center = np.array([0.0, 0.0, 0.7])
        pts = self._sphere_points(center, 0.030)
        outliers = center + np.random.default_rng(1).uniform(-0.3, 0.3, size=(40, 3))
        c, r = fit_sphere_reference(np.vstack([pts, outliers]))
        self.assertIsNotNone(c)
        self.assertAlmostEqual(r, 0.030, delta=0.005)

    def test_too_few_points_gives_up(self):
        c, r = fit_sphere_reference(np.zeros((5, 3)))
        self.assertIsNone(c)
        self.assertEqual(r, 0.0)

    def test_implausible_radius_rejected(self):
        pts = self._sphere_points(np.array([0.0, 0.0, 0.8]), 0.20)
        c, r = fit_sphere_reference(pts)
        self.assertIsNone(c)


if __name__ == '__main__':
    unittest.main()
