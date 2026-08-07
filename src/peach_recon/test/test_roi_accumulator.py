"""TsdfRecon：合成两帧 RGB-D → integrate → extract 非空（需 venv，open3d）."""
import unittest

import numpy as np

from peach_recon.roi_accumulator import TsdfRecon


def _frame(depth_mm=800):
    """合成一帧 80×60 RGB-D（平面墙 z=depth_mm，灰色）."""
    rgb = np.full((60, 80, 3), 128, dtype=np.uint8)
    depth = np.full((60, 80), depth_mm, dtype=np.uint16)
    K = {'fx': 70.0, 'fy': 70.0, 'cx': 40.0, 'cy': 30.0,
         'width': 80, 'height': 60}
    return rgb, depth, K


class TsdfReconTest(unittest.TestCase):
    def test_integrate_two_frames_extracts_cloud(self):
        """两帧合成 RGB-D 融合后提取到非空彩色点云."""
        recon = TsdfRecon(tsdf_voxel_length=0.01, sdf_trunc=0.04)
        T = np.eye(4)
        recon.integrate_rgbd(*_frame(800), T)
        recon.integrate_rgbd(*_frame(820), T)
        xyz, rgb = recon.extract_cloud()
        self.assertGreater(len(xyz), 0)
        self.assertEqual(xyz.shape[1], 3)
        self.assertEqual(rgb.shape, xyz.shape)
        self.assertGreater(recon.n_points, 0)
        # 平面在相机正前方 0.8m 附近：z 均值应接近 0.8 m
        self.assertAlmostEqual(float(xyz[:, 2].mean()), 0.81, delta=0.05)

    def test_reset_clears_volume(self):
        """调用 reset 后提取为空."""
        recon = TsdfRecon(tsdf_voxel_length=0.01, sdf_trunc=0.04)
        recon.integrate_rgbd(*_frame(800), np.eye(4))
        self.assertGreater(recon.n_points, 0)
        recon.reset()
        self.assertEqual(recon.n_points, 0)
        xyz, rgb = recon.extract_cloud()
        self.assertEqual(xyz.shape, (0, 3))
        self.assertEqual(rgb.shape, (0, 3))


if __name__ == '__main__':
    unittest.main()
