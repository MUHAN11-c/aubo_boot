"""orbit_planner：端点正确、半径单调逼近、TCP 朝向目标、截断行为."""
import unittest

import numpy as np

from peach_motion.orbit_planner import make_orbit_waypoints


class OrbitPlannerTest(unittest.TestCase):
    def setUp(self):
        self.target = np.array([0.30, 0.0, 0.50])
        self.axis = np.array([0.0, 0.0, 1.0])       # 套入轴 +Z（底→颈朝上）
        self.tcp = np.array([0.30, 0.40, 0.50])     # 当前 TCP 在目标侧向 0.4m

    def test_endpoints_exact(self):
        """首点=当前 TCP，末点=预接近点（target − standoff·axis）."""
        wps = make_orbit_waypoints(self.tcp, self.target, self.axis, 0.25, 90, 7)
        self.assertEqual(len(wps), 7)
        np.testing.assert_allclose(wps[0][0], self.tcp, atol=1e-9)
        np.testing.assert_allclose(wps[-1][0],
                                   self.target - 0.25 * self.axis, atol=1e-9)

    def test_radius_monotone_approach(self):
        """路径点到球心距离从起点半径单调收敛到 standoff."""
        wps = make_orbit_waypoints(self.tcp, self.target, self.axis, 0.25, 90, 7)
        radii = [float(np.linalg.norm(p - self.target)) for p, _ in wps]
        self.assertAlmostEqual(radii[0], 0.4, places=9)
        self.assertAlmostEqual(radii[-1], 0.25, places=9)
        for a, b in zip(radii, radii[1:]):
            self.assertGreaterEqual(a + 1e-12, b)   # 单调不增

    def test_orientation_points_at_target(self):
        """每个 waypoint 的 TCP Z 轴（四元数第三列）指向目标."""
        wps = make_orbit_waypoints(self.tcp, self.target, self.axis, 0.25, 90, 7)
        for p, q in wps:
            x, y, z, w = q
            # 四元数 → 旋转矩阵第三列（Z 轴方向）
            zaxis = np.array([
                2 * (x * z + w * y), 2 * (y * z - w * x),
                1 - 2 * (x * x + y * y)])
            expect = (self.target - p) / np.linalg.norm(self.target - p)
            np.testing.assert_allclose(zaxis, expect, atol=1e-6)

    def test_arc_cap_truncates_start_not_end(self):
        """弧角上限 < 自然夹角时：终点仍精确，起点被截断在弧线上."""
        # TCP 与预接近方向夹角 ~180°（TCP 在 +axis 侧），arc=30° 必截断
        tcp = self.target + 0.4 * self.axis
        wps = make_orbit_waypoints(tcp, self.target, self.axis, 0.25, 30, 5)
        np.testing.assert_allclose(wps[-1][0],
                                   self.target - 0.25 * self.axis, atol=1e-9)
        self.assertGreater(float(np.linalg.norm(wps[0][0] - tcp)), 0.05)

    def test_degenerate_inputs(self):
        """零轴/起点即球心/点数不足 → 空列表."""
        self.assertEqual(make_orbit_waypoints(
            self.tcp, self.target, np.zeros(3), 0.25, 90, 5), [])
        self.assertEqual(make_orbit_waypoints(
            self.target, self.target, self.axis, 0.25, 90, 5), [])
        self.assertEqual(make_orbit_waypoints(
            self.tcp, self.target, self.axis, 0.25, 90, 1), [])


if __name__ == '__main__':
    unittest.main()
