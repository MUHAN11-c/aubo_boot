"""Export only three complete bagged-peach assets from a validated orchard."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_scene import _enable_gpu, camera  # noqa: E402,I100


def digest(path):
    """Hash a saved source or output file."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Keep exact bag meshes and interiors, changing only their review poses."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    args.out.mkdir(parents=True, exist_ok=True)
    source = Path(bpy.data.filepath)
    origin = json.loads((source.parent / 'scene_manifest.json').read_text())
    forms = ('folded_gusset', 'broad_panel', 'creased_panel')
    selected = [next(t for t in origin['targets'] if t.get('paper_form') == form)
                for form in forms]
    keep = set()
    for i, target in enumerate(selected):
        prefix = target['name']
        bag = bpy.data.objects[prefix]
        location = bag.location.copy()
        inverse = bag.rotation_euler.to_matrix().inverted()
        offset = Vector(((i - 1) * .16, 0, 0))
        members = [obj for obj in bpy.data.objects
                   if obj.name == prefix or obj.name.startswith(prefix + '/')]
        for obj in members:
            obj.location = offset + inverse @ (obj.location - location)
            obj.rotation_euler = (inverse @ obj.rotation_euler.to_matrix()).to_euler()
            keep.add(obj.name)
        target['review_pose'] = 'upright separated asset; exact source mesh retained'
    for obj in list(bpy.data.objects):
        if obj.type in ('MESH', 'CAMERA') and obj.name not in keep:
            bpy.data.objects.remove(obj, do_unlink=True)
    scene = bpy.context.scene
    scene.cycles.device = 'GPU' if _enable_gpu() != 'CPU' else 'CPU'
    scene.cycles.samples = 64
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 900
    scene.camera = camera('Only bagged peaches', (.22, -.64, .23), (0, 0, .06), 55)
    for node in scene.node_tree.nodes:
        if node.type == 'OUTPUT_FILE':
            node.mute = True
    path = args.out / 'bagged_peaches.blend'
    bpy.ops.wm.save_as_mainfile(filepath=str(path))
    scene.render.filepath = str(args.out / 'bagged_peaches.png')
    bpy.ops.render.render(write_still=True)
    report = {
        'revision': origin['modeling_revision'],
        'source_blend_sha256': digest(source),
        'asset_blend_sha256': digest(path),
        'export_tool_sha256': digest(Path(__file__)),
        'rgb_sha256': digest(args.out / 'bagged_peaches.png'),
        'source_sha256': origin['source_sha256'],
        'source_target_ids': [t['id'] for t in selected],
        'paper_forms': list(forms), 'shown_meshes': sorted(keep),
        'lighting': origin['lighting'], 'samples': 64,
        'limits': ['Review poses differ from orchard; source mesh coordinates unchanged.',
                   'Internal peaches remain enclosed; no naked-fruit diagnostic shown.']}
    (args.out / 'asset_manifest.json').write_text(json.dumps(report, indent=2))
    print('BAGGED ASSETS EXPORTED', flush=True)


if __name__ == '__main__':
    main()
