import unittest

import numpy as np

from peach_pose_ros2.peach_pose.fitting import (
    estimate_normals, fit_sphere_robust, fit_cylinder_robust,
    _cylinder_radial_dist,
)

K = {"fx": 640.0, "fy": 636.0, "cx": 640.0, "cy": 360.0}


def _sphere_cap(center, radius, half_angle_deg=45, n=600, noise=0.0015, seed=0):
    """相机在 +Z 看向原点，生成朝向相机的球冠点云+解析法线。"""
    rng = np.random.default_rng(seed)
    a = np.radians(half_angle_deg)
    u = rng.uniform(np.cos(a), 1.0, n)
    v = rng.uniform(0, 2 * np.pi, n)
    dirs = np.column_stack((np.sqrt(1 - u ** 2) * np.cos(v),
                            np.sqrt(1 - u ** 2) * np.sin(v), u))
    # 球冠朝向 -Z（朝相机）：相机在球心 -Z 侧
    normals = -dirs
    pts = center + normals * radius + rng.normal(scale=noise, size=(n, 3))
    return pts, normals


def _cylinder_patch(q0, axis, radius, arc_deg=120, n=800, noise=0.0015, seed=0):
    """部分弧圆柱面点云+解析法线。"""
    rng = np.random.default_rng(seed)
    axis = axis / np.linalg.norm(axis)
    ref = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0])
    u = np.cross(axis, ref); u /= np.linalg.norm(u)
    w = np.cross(axis, u)
    ang = np.radians(arc_deg)
    thetas = rng.uniform(-ang / 2, ang / 2, n)
    hs = rng.uniform(-0.04, 0.04, n)
    normals = np.cos(thetas)[:, None] * u + np.sin(thetas)[:, None] * w
    pts = q0 + normals * radius + hs[:, None] * axis
    pts += rng.normal(scale=noise, size=(n, 3))
    return pts, normals


class NormalsTest(unittest.TestCase):
    def test_plane_normals_point_to_camera(self):
        depth = np.full((60, 80), 900, dtype=np.uint16)
        n, nv = estimate_normals(depth, 100, 100, K)
        self.assertGreater(nv.sum(), 60 * 80 * 0.8)
        # 平面前方法线应指向 -Z（朝相机）
        self.assertLess(float(n[nv][:, 2].mean()), -0.99)


class SphereFitTest(unittest.TestCase):
    def test_small_cap_with_fixed_radius(self):
        center = np.array([0.05, -0.03, 0.8])
        pts, normals = _sphere_cap(center, 0.035, half_angle_deg=45)
        est = fit_sphere_robust(pts, normals, radius_prior=0.035)
        self.assertIsNotNone(est)
        self.assertLess(np.linalg.norm(est["center"] - center), 0.008)
        self.assertGreater(est["inlier_ratio"], 0.8)

    def test_outliers_rejected(self):
        center = np.array([0.0, 0.0, 0.7])
        pts, normals = _sphere_cap(center, 0.030, half_angle_deg=60, n=500)
        out = np.random.default_rng(7).uniform(-0.2, 0.2, (80, 3)) + [0, 0, 0.7]
        est = fit_sphere_robust(np.vstack([pts, out]),
                                np.vstack([normals, out * 0 + [0, 0, -1]]),
                                radius_prior=0.030)
        self.assertIsNotNone(est)
        self.assertLess(np.linalg.norm(est["center"] - center), 0.01)


class CylinderFitTest(unittest.TestCase):
    def test_axis_recovers_vertical_cylinder(self):
        axis = np.array([0.0, 1.0, 0.0])
        pts, normals = _cylinder_patch(np.array([0.02, 0.5, 0.9]), axis, 0.038)
        est = fit_cylinder_robust(pts, normals)
        self.assertIsNotNone(est)
        cos = abs(float(est["axis"] @ axis))
        self.assertGreater(cos, np.cos(np.radians(3.0)))
        self.assertAlmostEqual(est["radius"], 0.038, delta=0.004)
        self.assertGreater(est["inlier_ratio"], 0.7)

    def test_tilted_cylinder(self):
        axis = np.array([0.3, 1.0, 0.2])
        pts, normals = _cylinder_patch(np.array([0.0, 0.4, 0.85]), axis, 0.030)
        est = fit_cylinder_robust(pts, normals)
        self.assertIsNotNone(est)
        a = axis / np.linalg.norm(axis)
        cos = abs(float(est["axis"] @ a))
        self.assertGreater(cos, np.cos(np.radians(4.0)))

    def test_planar_points_give_up(self):
        # 平面法线全部平行 → 叉积≈0 → 应返回 None 而不是垃圾轴
        rng = np.random.default_rng(3)
        xs = rng.uniform(-0.1, 0.1, (300, 2))
        pts = np.column_stack([xs[:, 0], xs[:, 1], np.full(300, 0.9)])
        normals = np.tile([0.0, 0.0, -1.0], (300, 1))
        est = fit_cylinder_robust(pts, normals)
        self.assertIsNone(est)


if __name__ == "__main__":
    unittest.main()
