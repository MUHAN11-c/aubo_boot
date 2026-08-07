"""TemporalFuser：离群剔除、EMA 收敛、方差下降（纯 numpy，系统 python3 可跑）."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.temporal_fuser import _largest_cluster_size, TemporalFuser


def _make(**kw):
    """默认门控参数的融合器."""
    kw.setdefault('pos_jump_gate_m', 0.05)
    kw.setdefault('axis_gate_deg', 20.0)
    kw.setdefault('dbscan_eps_m', 0.03)
    kw.setdefault('ema_alpha', 0.3)
    return TemporalFuser(**kw)


class TemporalFuserTest(unittest.TestCase):
    def test_first_frame_accepted(self):
        """首帧直接接受并给出融合值."""
        f = _make()
        self.assertTrue(f.update([0.3, 0.0, 0.5], [0, 0, 1], 0.07, 0.9))
        est = f.current()
        np.testing.assert_allclose(est['position'], [0.3, 0.0, 0.5], atol=1e-12)
        np.testing.assert_allclose(est['axis'], [0, 0, 1], atol=1e-12)

    def test_position_outlier_rejected(self):
        """位置跳变超门 → 判离群，融合值不动."""
        f = _make()
        f.update([0.3, 0.0, 0.5], [0, 0, 1], 0.07, 0.9)
        self.assertFalse(f.update([0.5, 0.0, 0.5], [0, 0, 1], 0.07, 0.9))
        est = f.current()
        np.testing.assert_allclose(est['position'], [0.3, 0.0, 0.5], atol=1e-12)
        self.assertEqual((est['accepted'], est['rejected']), (1, 1))

    def test_axis_outlier_rejected(self):
        """轴夹角超门（无向）→ 判离群."""
        f = _make()
        f.update([0.3, 0.0, 0.5], [0, 0, 1], 0.07, 0.9)
        # 与 [0,0,1] 夹角 90° > 20°
        self.assertFalse(f.update([0.31, 0.0, 0.5], [1, 0, 0], 0.07, 0.9))
        self.assertEqual(f.current()['rejected'], 1)

    def test_axis_sign_aligned(self):
        """反向轴（±二义性）按同一方向融合，不被判离群."""
        f = _make()
        f.update([0.3, 0.0, 0.5], [0, 0, 1], 0.07, 0.9)
        self.assertTrue(f.update([0.3, 0.0, 0.5], [0, 0, -1], 0.07, 0.9))
        np.testing.assert_allclose(
            np.abs(f.current()['axis']), [0, 0, 1], atol=1e-6)

    def test_ema_converges_to_mean(self):
        """均值附近的噪声观测：融合位置收敛到真值附近."""
        rng = np.random.default_rng(0)
        f = _make()
        true = np.array([0.30, 0.02, 0.50])
        for _ in range(60):
            noisy = true + rng.normal(scale=0.005, size=3)   # 5mm 噪声
            f.update(noisy, [0, 0, 1], 0.07, 0.9)
        err = float(np.linalg.norm(f.current()['position'] - true))
        # 收敛残差（≈3 mm）显著小于单帧噪声幅度（5mm/轴 → |噪声|≈8.7mm）
        self.assertLess(err, 0.004)

    def test_variance_decreases_with_support(self):
        """一致观测越攒越多：pos_std 下降、support_frames 增长."""
        rng = np.random.default_rng(1)
        f = _make()
        true = np.array([0.3, 0.0, 0.5])
        stds = []
        for i in range(40):
            f.update(true + rng.normal(scale=0.004, size=3), [0, 0, 1],
                     0.07, 0.9)
            if i in (4, 39):
                stds.append(f.pos_std_m)
        self.assertLess(stds[1], stds[0])
        self.assertEqual(f.support_frames, 40)     # 全部同簇（噪声 4mm<eps）

    def test_reset_clears_state(self):
        """调用 reset 后融合值/计数/历史全部清零."""
        f = _make()
        f.update([0.3, 0.0, 0.5], [0, 0, 1], 0.07, 0.9)
        f.reset()
        self.assertIsNone(f.current())
        self.assertEqual((f.accepted, f.rejected, f.frame_seq), (0, 0, 0))
        self.assertEqual(f.support_frames, 0)

    def test_largest_cluster_size(self):
        """迷你 DBSCAN：两团分离点各自成簇，取最大簇."""
        pts = ([[0, 0, 0], [0.01, 0, 0], [0.02, 0, 0]]      # 簇 1（3 点）
               + [[1.0, 0, 0], [1.01, 0, 0]])               # 簇 2（2 点）
        self.assertEqual(_largest_cluster_size(pts, 0.03), 3)
        self.assertEqual(_largest_cluster_size([], 0.03), 0)


if __name__ == '__main__':
    unittest.main()
