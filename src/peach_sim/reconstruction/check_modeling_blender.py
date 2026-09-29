"""
Check geometry regressions in Blender, without rendering or ROS.

Run with blender -b --python-exit-code 1 -P check_modeling_blender.py.
The crown-root test measures generated meshes, not connection bookkeeping.
"""

from pathlib import Path
import sys
import unittest

import bpy
from mathutils import Vector
from mathutils.geometry import intersect_point_line

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_scene  # noqa: E402,I100
from geometry import bag_rings, blade_vertices, fruit_center_z, paper_bag  # noqa: E402
import lighting  # noqa: E402
from lighting_apply import apply_lighting  # noqa: E402
from materials import create  # noqa: E402


class ModelingTests(unittest.TestCase):
    def test_paper_height_is_packed_non_color_data(self):
        """Keep height values independent of display colour transforms."""
        mat = create()['paper0']
        images = [node.image for node in mat.node_tree.nodes
                  if node.type == 'TEX_IMAGE' and 'paper_height' in node.image.filepath]
        self.assertEqual(len(images), 1)
        self.assertTrue(images[0].colorspace_settings.is_data)
        self.assertIsNotNone(images[0].packed_file)

    def test_paper_panel_is_flat_and_closed_at_measured_outline(self):
        """Keep broad panels flat without breaking measured width or closure."""
        # Constant measured profile: microfolds must not inflate its boundary
        # into the broad rounded cross-section of a cloth pouch.
        obj = paper_bag('Paper probe', [(0, 0, .10, 0, .04),
                                       (.15, 0, .10, 0, .04)], None, 17)
        front = [v.co.y for v in obj.data.vertices
                 if abs(v.co.z) < 1e-8 and abs(v.co.x) < .07 and v.co.y < .02]
        self.assertLess(max(front) - min(front), .0003)
        self.assertAlmostEqual(max(v.co.x for v in obj.data.vertices), .10, places=5)
        edge_use = {}
        for polygon in obj.data.polygons:
            for edge in polygon.edge_keys:
                edge_use[edge] = edge_use.get(edge, 0) + 1
        self.assertTrue(all(count == 2 for count in edge_use.values()))

    def test_twisted_leaf_still_tapers_to_a_tip(self):
        vertices, _faces, _uv = blade_vertices(.12, .03, -.1, .2, .02)
        tip_ring = [Vector(v) for v in vertices[-5:]]
        span = max((a - b).length for a in tip_ring for b in tip_ring)
        self.assertLess(span, .001, 'Twist must not turn the tip into a wide cut edge')

    def test_mature_fruit_clears_the_paper(self):
        from mathutils.bvhtree import BVHTree
        diameter = .075
        rings = bag_rings(.11, .12, .09, 3, fruit_diameter=diameter)
        obj = paper_bag('Fruit fit probe', rings, None, 3)
        center = Vector((0, 0, fruit_center_z(diameter)))
        tree = BVHTree.FromPolygons(
            [v.co for v in obj.data.vertices],
            [p.vertices[:] for p in obj.data.polygons])
        _point, _normal, _index, distance = tree.find_nearest(center)
        self.assertGreater(distance, diameter / 2 + .0005)
        edge_use = {}
        for polygon in obj.data.polygons:
            for edge in polygon.edge_keys:
                edge_use[edge] = edge_use.get(edge, 0) + 1
        self.assertTrue(all(count == 2 for count in edge_use.values()))

    def test_crown_roots_lie_on_generated_parent_centerlines(self):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)
        build_scene.TARGETS.clear()
        build_scene.CONNECTIONS.clear()
        build_scene.tree('Probe', (0, 0, 0), create(), 80)
        segments = []
        for obj in bpy.data.objects:
            if '/scaffold' not in obj.name and '/secondary' not in obj.name:
                continue
            vertices = obj.data.vertices
            centers = [sum((v.co for v in vertices[i:i + 9]), Vector()) / 9
                       for i in range(0, len(vertices), 9)]
            segments.extend(zip(centers, centers[1:]))
        errors = []
        for obj in bpy.data.objects:
            if '/crown' not in obj.name or '/crownshoot' in obj.name:
                continue
            root = sum((v.co for v in obj.data.vertices[:9]), Vector()) / 9
            distances = []
            for a, b in segments:
                _, factor = intersect_point_line(root, a, b)
                closest = a.lerp(b, min(1, max(0, factor)))
                distances.append((root - closest).length)
            if min(distances) > 0.0001:
                errors.append((obj.name, min(distances)))
        self.assertFalse(errors, f'Crown roots float off bent parent axes: {errors}')

    def test_overcast_has_no_direct_sun(self):
        scene = bpy.context.scene
        scene.world.use_nodes = True
        apply_lighting(scene, lighting.get('overcast'))
        sky = next(n for n in scene.world.node_tree.nodes if n.type == 'TEX_SKY')
        self.assertFalse(sky.sun_disc, 'Overcast must not contain a solar disc')
        lamp = bpy.data.objects['Sun through canopy']
        self.assertEqual(lamp.data.energy, 0., 'Cloud cover must remove direct SUN')


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(ModelingTests))
    if not result.wasSuccessful():
        raise RuntimeError('Blender modeling regressions failed')
