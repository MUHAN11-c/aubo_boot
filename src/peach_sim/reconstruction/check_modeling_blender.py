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
from geometry import bag_rings, blade_vertices, fruit_center_z, Leaves, paper_bag  # noqa: E402
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

    def test_leaf_instances_preserve_requested_direction_and_prototype(self):
        """Catch attribute references invalidated by later attribute creation."""
        leaves = Leaves(create(), seed=31)
        cases = [((3, 0, 0), (0, .12, 0)),
                 ((4, 0, 0), (0, 0, .12)),
                 ((5, 0, 0), (-.12, 0, 0))]
        for root, axis in cases:
            leaves.add(root, Vector(root) + Vector(axis))
        obj = leaves.finish('Leaf instance probe')[0]
        bpy.context.view_layer.update()
        instances = {}
        for instance in bpy.context.evaluated_depsgraph_get().object_instances:
            if (instance.is_instance and instance.parent
                    and instance.parent.original == obj):
                matrix = instance.matrix_world.copy()
                instances[round(matrix.translation.x)] = (
                    matrix, instance.object.original.name)
        self.assertEqual(len(instances), len(cases))
        for (root, axis), record in zip(cases, leaves.records):
            matrix, prototype = instances[root[0]]
            actual_axis = (matrix.to_3x3() @ Vector((1, 0, 0))).normalized()
            self.assertGreater(actual_axis.dot(Vector(axis).normalized()), .99999)
            index = record[0]
            self.assertEqual(prototype, f'leaf proto {index // 4}_{index % 4}')
            self.assertAlmostEqual(
                (matrix.to_3x3() @ Vector((1, 0, 0))).length, 1.2, places=5)

    def test_mature_fruit_clears_the_paper(self):
        from mathutils.bvhtree import BVHTree
        forms = set()
        for diameter in (.06, .075, .085):
            for seed in range(3):
                rings = bag_rings(diameter + .012, diameter + .027, .09, seed,
                                  fruit_diameter=diameter)
                obj = paper_bag('Fruit fit probe', rings, None, seed)
                forms.add(obj['paper_form'])
                center = Vector((0, 0, fruit_center_z(diameter)))
                tree = BVHTree.FromPolygons(
                    [v.co for v in obj.data.vertices],
                    [p.vertices[:] for p in obj.data.polygons])
                point, normal, _index, distance = tree.find_nearest(center)
                self.assertLess((center - point).dot(normal), 0)
                self.assertGreater(distance, diameter / 2 + .0005)
                equator = [v.co.x for v in obj.data.vertices
                           if abs(v.co.z - center.z) < .004]
                self.assertGreater(max(equator), diameter / 2)
                self.assertLess(max(equator), diameter / 2 + .025)
                edge_use = {}
                for polygon in obj.data.polygons:
                    for edge in polygon.edge_keys:
                        edge_use[edge] = edge_use.get(edge, 0) + 1
                self.assertTrue(all(count == 2 for count in edge_use.values()))
        self.assertEqual(forms, {'folded_gusset', 'broad_panel', 'creased_panel'})

    def test_filled_bag_retains_wide_paper_faces(self):
        """A contained peach must not force the external shell into a ball."""
        for seed in range(3):
            obj = paper_bag('Loose paper probe',
                            bag_rings(.115, .12, seed=seed, fruit_diameter=.075),
                            None, seed)
            low = [v.co.x for v in obj.data.vertices if abs(v.co.z) < .003]
            self.assertGreater(max(low) - min(low), .09)
            front = [v.co.y for v in obj.data.vertices
                     if .040 < v.co.z < .045 and abs(v.co.x) < .025 and v.co.y < 0]
            self.assertLess(max(front) - min(front), .003)

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

    def test_fruit_suture_survives_angle_wrap(self):
        mat = create()['fruit']
        for seed in (0, 2, 5, 6, 10):
            obj = build_scene.peach('Suture probe', (0, 0, 0), .04,
                                    1., (0, 0, 0), mat, seed)
            equator = [v.co.xy.length for v in obj.data.vertices
                       if abs(v.co.z) < 1e-6]
            self.assertLess(min(equator), .04 * .98,
                            'Fruit suture disappeared across atan2 wrap')

    def test_scaffolds_have_distinct_roots_on_trunk(self):
        build_scene.TARGETS.clear()
        build_scene.CONNECTIONS.clear()
        build_scene.tree('Structural probe', (10, 0, 0), create(), 80)
        roots = [c['root'] for c in build_scene.CONNECTIONS
                 if c['name'].startswith('Structural probe/scaffold')]
        self.assertGreater(len({round(r[2], 5) for r in roots}), 2)
        self.assertTrue(all(.40 <= r[2] <= .50 for r in roots))

    def test_enclosed_peach_has_a_connected_peduncle(self):
        fruit = build_scene.sample_mature_fruit(
            build_scene.NOBAG_STATS, build_scene.random.Random(31))
        obj = build_scene.add_bag('Peduncle probe', (0, 0, 1),
                                  .10, .12, create(), fruit, seed=31)
        stem = bpy.data.objects.get(obj.name + '/fruit peduncle')
        self.assertIsNotNone(stem)
        end = sum((v.co for v in stem.data.vertices[-9:]), Vector()) / 9
        self.assertLess((end - Vector((0, 0, 1))).length, 1e-6)
        from mathutils.bvhtree import BVHTree
        peach = bpy.data.objects[obj.name + '/enclosed peach']
        bpy.context.view_layer.update()
        root = sum((v.co for v in stem.data.vertices[:9]), Vector()) / 9
        tree = BVHTree.FromPolygons([v.co for v in peach.data.vertices],
                                    [p.vertices[:] for p in peach.data.polygons])
        self.assertLess(tree.find_nearest(peach.matrix_world.inverted() @ root)[3], 1e-6)

    def test_field_scaffolds_do_not_outgrow_supporting_trunk(self):
        """Crown reach must not enlarge child wood beyond its actual parent."""
        from validate_real_features import nearest, tube_axis
        build_scene.tree('Field radius probe', (20, 0, 0), create(), 80, crown=1.5)
        parent = tube_axis(bpy.data.objects['Field radius probe/trunk'])
        children = [obj for obj in bpy.data.objects
                    if obj.name.startswith('Field radius probe/scaffold')]
        self.assertGreater(len(children), 2)
        for child in children:
            axis = tube_axis(child)
            support = nearest(axis['centers'][0], [parent])
            self.assertLessEqual(axis['radii'][0], support[1] + 1e-6)

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
