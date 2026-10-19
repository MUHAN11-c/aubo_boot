"""
Real mesh perturbation regressions; Blender CPU, no render or ROS.

blender -b -t 2 --python-exit-code 1 -P check_real_features_blender.py
"""
from pathlib import Path
import sys
import unittest

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_scene  # noqa: E402
from materials import create  # noqa: E402
from validate_real_features import audit  # noqa: E402


class RealFeatureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)
        build_scene.TARGETS.clear()
        build_scene.CONNECTIONS.clear()
        build_scene.tree('Tree00', (0, 0, 0), create(), 80)
        bpy.context.view_layer.update()

    def test_valid_saved_geometry(self):
        report = audit()
        self.assertTrue(report['passed'], report['errors'])
        self.assertGreater(report['trees']['Tree00']['leaves']['count'], 0)
        self.assertGreaterEqual(len(report['bag_form_counts']), 2)

    def test_floating_mesh_root_is_detected_despite_unchanged_metadata(self):
        obj = bpy.data.objects['Tree00/crown0']
        original = [v.co.copy() for v in obj.data.vertices]
        try:
            for vertex in obj.data.vertices:
                vertex.co.x += .03
            obj.data.update()
            bpy.context.view_layer.update()
            report = audit()
            self.assertTrue(any('crown0: root floats' in error for error in report['errors']))
        finally:
            for vertex, coordinate in zip(obj.data.vertices, original):
                vertex.co = coordinate
            obj.data.update()
            bpy.context.view_layer.update()

    def test_disconnected_internal_fruit_stem_is_detected(self):
        obj = next(o for o in bpy.data.objects if o.name.endswith('/fruit peduncle'))
        original = [v.co.copy() for v in obj.data.vertices]
        try:
            for vertex in obj.data.vertices:
                vertex.co.x += .02
            obj.data.update()
            bpy.context.view_layer.update()
            report = audit()
            self.assertTrue(any('peduncle does not connect' in error
                                for error in report['errors']))
        finally:
            for vertex, coordinate in zip(obj.data.vertices, original):
                vertex.co = coordinate
            obj.data.update()
            bpy.context.view_layer.update()


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(RealFeatureTests))
    if not result.wasSuccessful():
        raise RuntimeError('Real-feature mesh regressions failed')
