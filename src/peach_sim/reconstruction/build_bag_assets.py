"""Build standalone bagged peaches with front and side review views."""

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys

import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import build_scene as scene_builder  # noqa: E402,I100
from distributions import sample_bag_for_fruit, sample_mature_fruit  # noqa: E402
from materials import create  # noqa: E402


def digest(path):
    """Hash a source file or saved artifact."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Generate complete assets using the same bag geometry as the orchard."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=HERE / 'output/bag_assets')
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])
    args.out.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene_builder.TARGETS.clear()
    mats = create()
    rng = random.Random(240930)
    for i in range(3):
        fruit = sample_mature_fruit(scene_builder.NOBAG_STATS, rng)
        paper = sample_bag_for_fruit(scene_builder.BAG_STATS, fruit, rng)
        scene_builder.add_bag(
            'Bag' + str(i), ((i - 1) * .17, 0, paper['height_m']),
            paper['width_m'], paper['height_m'], mats, fruit, seed=i, tilt=0)
        scene_builder.TARGETS[-1]['sampled_paper'] = paper
    scene_builder.setup_render(args.out, 64, 'noon')
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    for node in scene.node_tree.nodes:
        if node.type == 'OUTPUT_FILE':
            node.mute = True
    cams = {
        'bagged_peaches': scene_builder.camera(
            'Three bagged peaches', (.22, -.72, .25), (0, 0, .078), 55),
        'front': scene_builder.camera('Bag front', (0, -.46, .083), (0, 0, .083), 55),
        'side': scene_builder.camera('Bag side', (.46, -.025, .095), (0, 0, .083), 55)}
    scene.camera = cams['bagged_peaches']
    blend = args.out / 'bagged_peaches.blend'
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    views = {}
    for name, cam in cams.items():
        scene.camera = cam
        visible = []
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.hide_render = name != 'bagged_peaches' and not (
                    obj.name == 'Bag1' or obj.name.startswith('Bag1/'))
                if not obj.hide_render:
                    visible.append(obj.name)
        path = args.out / (name + '.png')
        scene.render.filepath = str(path)
        bpy.ops.render.render(write_still=True)
        views[name] = {'rgb_sha256': digest(path), 'shown_meshes': sorted(visible),
                       'camera_world': [list(row) for row in cam.matrix_world]}
    sources = ('build_bag_assets.py', 'build_scene.py', 'geometry.py', 'distributions.py',
               'materials.py', 'lighting.py', 'lighting_apply.py', 'evidence/priors.json',
               'textures/paper_height.png')
    report = {
        'revision': '2026-09-30-bag-envelope-v10',
        'asset_blend_sha256': digest(blend),
        'rgb_sha256': digest(args.out / 'bagged_peaches.png'),
        'source_sha256': {name: digest(HERE / name) for name in sources},
        'targets': scene_builder.TARGETS, 'views': views, 'samples': 64,
        'lighting': 'noon', 'units': 'metres',
        'limits': ['Interior and paper construction are inferred, not a measured scan.',
                   'Standalone assets only; existing orchard renders remain v9.',
                   'Front and side isolate the middle bag only for review.']}
    (args.out / 'asset_manifest.json').write_text(json.dumps(report, indent=2))
    print('STANDALONE BAG ASSETS READY', flush=True)


if __name__ == '__main__':
    main()
