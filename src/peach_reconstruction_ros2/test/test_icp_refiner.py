"""有界 ICP：预热、刚性修正和越界拒绝."""
import unittest

import numpy as np

from peach_reconstruction_ros2.icp_refiner import (
    BoundedIcp,
    IcpConfig,
    transform_points,
)


def _surface():
    """构造非退化波纹面，避免平面对切向平移不可观."""
    x, y = np.meshgrid(
        np.linspace(-0.05, 0.05, 31),
        np.linspace(-0.04, 0.04, 25))
    z = 0.4 + 0.02 * x + 0.01 * np.sin(50.0 * y)
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


class BoundedIcpTest(unittest.TestCase):
    def test_model_warmup_uses_fk(self):
        """模型点不足时不从零猜位姿，直接保留 FK."""
        result = BoundedIcp(IcpConfig()).refine(
            _surface(), np.zeros((10, 3)))
        self.assertEqual(result.mode, 'fk')
        self.assertEqual(result.reason, 'model_warmup')

    def test_small_error_is_refined(self):
        """4 mm FK 平移误差应被 ICP 小范围修正."""
        target = _surface()
        source = target + np.array([0.004, 0.0, 0.0])
        cfg = IcpConfig(min_fitness=0.2, max_rmse=0.01)
        result = BoundedIcp(cfg).refine(source, target)
        self.assertEqual(result.mode, 'icp')
        aligned = transform_points(source, result.correction)
        self.assertLess(np.mean(np.linalg.norm(aligned - target, axis=1)), 0.003)
        self.assertLessEqual(result.translation_m, cfg.max_translation)

    def test_large_correction_is_not_accepted(self):
        """需要超过边界的修正不得作为 ICP 位姿写入 TSDF."""
        target = _surface()
        source = target + np.array([0.03, 0.0, 0.0])
        result = BoundedIcp(IcpConfig()).refine(source, target)
        self.assertNotEqual(result.mode, 'icp')


if __name__ == '__main__':
    unittest.main()
