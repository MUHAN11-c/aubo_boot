"""果线 RobustFruitPosePipeline：合成球+梗腔深度上的位姿估计."""
import unittest

import numpy as np

from peach_pose_ros2.peach_pose.contracts import BagObservation
from peach_pose_ros2.peach_pose.pipeline import RobustFruitPosePipeline


K = {'fx': 640.0, 'fy': 636.0, 'cx': 640.0, 'cy': 360.0}


def _render_sphere_depth(center, radius, cavity_dir=None, cavity_dip=0.0,
                         shape=(720, 1280)):
    """光线-球求交渲染合成深度图 (uint16 mm)；cavity_dir 处表面下陷 cavity_dip."""
    h, w = shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    d = np.stack([(us - K['cx']) / K['fx'], (vs - K['cy']) / K['fy'],
                  np.ones_like(us)], axis=-1)
    c = np.asarray(center, dtype=float)
    b = d @ c                              # t 方向投影
    disc = b ** 2 - (d ** 2).sum(-1) * (c @ c - radius ** 2)
    hit = disc > 0
    t1 = np.zeros((h, w))
    t1[hit] = (b[hit] - np.sqrt(disc[hit])) / (d[hit] ** 2).sum(-1)
    pts = t1[..., None] * d
    rel = pts - c
    norm = np.linalg.norm(rel, axis=-1, keepdims=True)
    norm[norm == 0] = 1.0
    u = rel / norm
    depth = np.zeros((h, w), dtype=np.uint16)
    if cavity_dir is not None:
        cd = np.asarray(cavity_dir, dtype=float)
        cd /= np.linalg.norm(cd)
        cap = (u @ cd) > np.cos(np.radians(12.0))
        depressed = hit & cap
        pts = np.where(depressed[..., None],
                       c + u * (radius - cavity_dip), pts)
    z = pts[..., 2]
    valid = hit & (z > 0.3) & (z < 2.5)
    depth[valid] = (z[valid] * 1000).astype(np.uint16)
    return depth, valid


class RobustFruitPosePipelineTest(unittest.TestCase):
    def _obs(self, depth, bbox, class_id=1):
        return BagObservation(
            rgb=np.zeros((*depth.shape, 3), dtype=np.uint8), depth=depth,
            camera_K=K, detections=[{'bbox': bbox, 'class_id': class_id, 'conf': .9}])

    def _bbox_of(self, mask):
        ys, xs = np.where(mask)
        return (int(xs.min()) - 5, int(ys.min()) - 5,
                int(xs.max()) + 5, int(ys.max()) + 5)

    def test_stem_cavity_sets_axis(self):
        center = np.array([0.02, -0.05, 0.75])
        cavity = np.array([0.05, -0.35, -0.93])
        cavity = cavity / np.linalg.norm(cavity)
        depth, valid = _render_sphere_depth(center, 0.030, cavity, cavity_dip=0.004)
        bbox = self._bbox_of(valid)
        mask = np.zeros(depth.shape, dtype=bool)
        mask[valid] = True
        res = RobustFruitPosePipeline().estimate(self._obs(depth, bbox), 'f-1', bbox, mask, 'test')
        self.assertEqual(res.target_kind, 'fruit')
        self.assertEqual(res.grasp_3d.diagnostic_info['axis_source'], 'stem_cavity')
        cos = float(res.grasp_3d.translation_direction @ cavity)
        self.assertGreater(cos, np.cos(np.radians(12.0)))
        self.assertAlmostEqual(res.metrics['fruit_radius_m'], 0.030, delta=0.004)
        self.assertEqual(res.grasp_3d.status, 'ACCEPT')
        # entry 必须在球心下方（远离梗端）且比球底更靠外
        axis = res.grasp_3d.translation_direction
        entry_off = float((res.grasp_3d.entry_start - center) @ axis)
        self.assertLess(entry_off, -0.030)

    def test_smooth_sphere_falls_back_to_gravity(self):
        center = np.array([0.0, 0.0, 0.8])
        depth, valid = _render_sphere_depth(center, 0.032)
        bbox = self._bbox_of(valid)
        mask = np.zeros(depth.shape, dtype=bool)
        mask[valid] = True
        res = RobustFruitPosePipeline().estimate(self._obs(depth, bbox), 'f-2', bbox, mask, 'test')
        info = res.grasp_3d.diagnostic_info
        self.assertEqual(info['axis_source'], 'gravity_prior')
        self.assertIn('axis_from_gravity_prior', res.grasp_3d.diagnostic_flags)
        self.assertNotEqual(res.grasp_3d.status, 'ACCEPT')

    def test_oversize_fruit_is_never_accepted(self):
        center = np.array([0.0, 0.0, 0.9])
        depth, valid = _render_sphere_depth(center, 0.055)
        bbox = self._bbox_of(valid)
        mask = np.zeros(depth.shape, dtype=bool)
        mask[valid] = True
        res = RobustFruitPosePipeline().estimate(self._obs(depth, bbox), 'f-3', bbox, mask, 'test')
        self.assertNotEqual(res.grasp_3d.status, 'ACCEPT')
        self.assertIn('tool_clearance_failed', res.grasp_3d.diagnostic_flags)

    def test_missing_depth_is_rejected(self):
        depth = np.zeros((720, 1280), dtype=np.uint16)
        res = RobustFruitPosePipeline().estimate(
            self._obs(depth, (500, 220, 780, 620)), 'f-4', (500, 220, 780, 620))
        self.assertEqual(res.grasp_3d.status, 'REJECT')
        self.assertIn('insufficient_measured_points', res.grasp_3d.diagnostic_flags)


if __name__ == '__main__':
    unittest.main()
