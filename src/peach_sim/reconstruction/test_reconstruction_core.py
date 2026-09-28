"""M2/M3 纯核单测：光照预设 / 分布采样 / 受控遮挡 / 停走轨迹 / 深度量化.

运行方式同 test_measurement.py（pytest 收集或直接 python 运行）。
补视几何与 peach_harvester view_policy 的对拍用 importlib 按文件加载，
运行时零跨包依赖。
"""

import importlib.util
import math
import random
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import depth_io  # noqa: E402
from distributions import sample_percentile, sample_width_height  # noqa: E402
import lighting  # noqa: E402
import occlusion  # noqa: E402
from viewpoints import (alley_stops, build_trajectory,  # noqa: E402
                        supplemental_viewpoint)


def _load_view_policy():
    path = (ROOT / 'src/peach_harvester/peach_harvester/cycle_core'
            / 'view_policy.py')
    spec = importlib.util.spec_from_file_location('view_policy_ref', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses needs the module registered
    spec.loader.exec_module(module)
    return module


class LightingTests(unittest.TestCase):
    def test_presets_cover_day_and_validate(self):
        self.assertEqual(
            sorted(lighting.PRESETS),
            ['late_afternoon', 'morning', 'noon', 'overcast'])
        for preset in lighting.PRESETS.values():
            lighting.validate(preset)

    def test_unknown_preset_rejected(self):
        with self.assertRaises(KeyError):
            lighting.get('midnight')

    def test_manifest_entry_records_parameters(self):
        entry = lighting.manifest_entry('noon')
        self.assertEqual(entry['preset'], 'noon')
        self.assertAlmostEqual(entry['sun_elevation_deg'], 38.0)


class DistributionTests(unittest.TestCase):
    STATS = {'n': 100, 'p10': .05, 'p50': .10, 'p90': .20, 'mean': .11}

    def test_percentile_boundaries(self):
        self.assertAlmostEqual(sample_percentile(self.STATS, 0.)[0], .05)
        self.assertAlmostEqual(sample_percentile(self.STATS, .5)[0], .10)
        self.assertAlmostEqual(sample_percentile(self.STATS, 1.)[0], .20)
        self.assertAlmostEqual(
            sample_percentile(self.STATS, .25)[0], .075)

    def test_percentile_clamps_out_of_range(self):
        self.assertAlmostEqual(sample_percentile(self.STATS, -.5)[0], .05)
        self.assertAlmostEqual(sample_percentile(self.STATS, 2.)[0], .20)

    def test_width_height_deterministic(self):
        bag = {'width_m': {'p10': .05, 'p50': .11, 'p90': .21},
               'aspect_h_over_w': {'p10': .65, 'p50': 1.03, 'p90': 1.6}}
        a = sample_width_height(bag, random.Random(7))
        b = sample_width_height(bag, random.Random(7))
        self.assertEqual(a, b)
        aspect, _ = sample_percentile(
            bag['aspect_h_over_w'], a['aspect_source_percentile'])
        # recorded percentile is rounded to 4 decimals for the manifest
        self.assertAlmostEqual(
            a['height_m'], a['width_m'] * aspect, places=4)


class OcclusionTests(unittest.TestCase):
    def _frame(self):
        return occlusion.BagFrame(
            neck=(0., 0., 1.0), center=(0., -.02, .9), bottom=(0., 0., .80),
            face_dir=(0., -1., 0.), width=.11, height=.12)

    def test_assign_levels_deterministic_and_weighted(self):
        a = occlusion.assign_levels(random.Random(3), 200)
        b = occlusion.assign_levels(random.Random(3), 200)
        self.assertEqual(a, b)
        levels = {p.level for p in a}
        self.assertEqual(levels, {'none', 'light', 'heavy'})
        heavy = sum(1 for p in a if p.level == 'heavy')
        # weights .40/.35/.25 over 200 draws: sanity band, not exact counts
        self.assertGreater(heavy, 20)

    def test_none_level_has_no_slots(self):
        plan = occlusion.OcclusionPlan('none', 0, 0., False, False)
        self.assertEqual(
            occlusion.foreground_leaf_slots(
                self._frame(), plan, random.Random(1), (0., -1., 0.)), [])

    def test_heavy_slots_sit_on_camera_side(self):
        plan = occlusion.OcclusionPlan('heavy', 4, .45, True, False)
        slots = occlusion.foreground_leaf_slots(
            self._frame(), plan, random.Random(2), (0., -1., 0.))
        self.assertEqual(len(slots), 4)
        for root, tip in slots:
            # camera sits at more-negative Y: leaf tips must not drift
            # behind the bag (past the bag center toward +Y)
            self.assertLessEqual(tip[1], self._frame().center[1] + .005)

    def test_front_branch_between_bag_and_camera(self):
        p0, p1, radius = occlusion.front_branch_segment(
            self._frame(), random.Random(5))
        mid = [(p0[i] + p1[i]) / 2 for i in range(3)]
        self.assertLess(mid[1], self._frame().center[1])
        self.assertGreaterEqual(radius, .008)
        self.assertLessEqual(radius, .012)

    def test_corridor_branch_lies_beyond_bottom(self):
        approach = (0., 0., -1.)
        p0, p1, _ = occlusion.corridor_branch_segment(
            self._frame(), random.Random(6), approach)
        for p in (p0, p1):
            self.assertLess(p[2], self._frame().bottom[2] + .005)


class ViewpointTests(unittest.TestCase):
    def test_supplemental_matches_view_policy_reference(self):
        ref = _load_view_policy()
        rng = random.Random(11)
        for _ in range(60):
            target = [rng.uniform(-1, 1), rng.uniform(0, 2), rng.uniform(.5, 2)]
            camera = [rng.uniform(-1, 1), rng.uniform(-2, 0), rng.uniform(.5, 2)]
            axis = [rng.uniform(-.4, .4), rng.uniform(-.4, .4), 1.]
            mine = supplemental_viewpoint(target, camera, axis)
            theirs = ref.supplemental_viewpoint(target, camera, axis)
            for a, b in zip(mine, theirs):
                self.assertAlmostEqual(a, b, places=9)

    def test_supplemental_step_capped(self):
        camera = [0., -.7, 1.0]
        target = [0., 0., 1.0]
        out = supplemental_viewpoint(target, camera, [0., 0., 1.])
        step = math.dist(out, camera)
        self.assertLessEqual(step, .1501)

    def test_trajectory_structure(self):
        targets = [{'id': 1, 'name': 'a', 'center_world': [0., 1., 1.2],
                    'axis_world': [0., 0., 1.],
                    'occlusion': {'level': 'light'}},
                   {'id': 2, 'name': 'b', 'center_world': [1.5, 1., 1.2]}]
        stops = alley_stops(-1., 1., 2, -.5, 1.45, (0., 1.5, 1.2))
        traj = build_trajectory(targets, stops)
        self.assertEqual(traj['intrinsics']['fx'], 640.)
        ids = [v['view_id'] for v in traj['views']]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(traj['per_target'][1],
                         ['tgt1_v0', 'tgt1_v1', 'tgt1_v2'])
        kinds = {v['kind'] for v in traj['views']}
        self.assertEqual(kinds, {'alley_stop', 'primary', 'supplemental'})
        # primary eye sits on the camera side at the sampled distance window
        primary = next(v for v in traj['views'] if v['view_id'] == 'tgt1_v0')
        self.assertLess(primary['eye'][1], targets[0]['center_world'][1])


class DepthIOTests(unittest.TestCase):
    def test_uint16_mm_quantization(self):
        depth = np.array([[0.5, 1.234], [0., 70.]])
        out = depth_io.to_uint16_mm(depth)
        self.assertEqual(out.dtype, np.uint16)
        self.assertEqual(out[0, 0], 500)
        self.assertEqual(out[0, 1], 1234)
        self.assertEqual(out[1, 0], 0)
        self.assertEqual(out[1, 1], 0)  # beyond 65 m cap -> invalid

    def test_nan_is_invalid(self):
        out = depth_io.to_uint16_mm(np.array([[np.nan, -0.1]]))
        self.assertEqual(out.tolist(), [[0, 0]])


if __name__ == '__main__':
    unittest.main()
