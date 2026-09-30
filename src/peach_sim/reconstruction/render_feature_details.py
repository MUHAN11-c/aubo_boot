"""Render only bagged-peach witnesses from a saved model, without saving edits."""

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    args.out.mkdir(parents=True, exist_ok=True)
    source = Path(bpy.data.filepath)
    manifest = json.loads((source.parent / 'scene_manifest.json').read_text())
    targets = [t for t in manifest['targets'] if t.get('fruit_radius_m')]
    round_bag = next(t for t in targets if t['paper_form'] == 'folded_gusset')
    panel = next(t for t in targets if t['paper_form'] == 'broad_panel')
    soft = next(t for t in targets if t['paper_form'] == 'creased_panel')
    selected = [round_bag, panel, soft]
    scene = bpy.context.scene
    scene.cycles.device = 'GPU' if _enable_gpu() != 'CPU' else 'CPU'
    scene.cycles.samples = 24
    scene.render.resolution_x = 960
    scene.render.resolution_y = 720
    for node in scene.node_tree.nodes:
        if node.type == 'OUTPUT_FILE':
            node.mute = True
    witnesses = []

    def shot(name, keep, focus, offset, lens, description):
        for obj in bpy.data.objects:
            if obj.type == 'MESH':
                obj.hide_render = obj.name not in keep
        cam = camera('Feature witness ' + name, Vector(focus) + Vector(offset), focus, lens)
        scene.camera = cam
        scene.render.filepath = str(args.out / (name + '.png'))
        bpy.ops.render.render(write_still=True)
        witnesses.append({'view': name, 'shown_meshes': sorted(keep),
                          'camera_world': [list(row) for row in cam.matrix_world],
                          'description': description,
                          'rgb_sha256': hashlib.sha256(
                              (args.out / (name + '.png')).read_bytes()).hexdigest()})

    for i, target in enumerate(selected):
        prefix = target['name']
        keep = {obj.name for obj in bpy.data.objects
                if obj.name == prefix or obj.name.startswith(prefix + '/')}
        shot('bag_form_' + str(i), keep, target['center_world'], (.10, -.26, .055), 55,
             'Isolated paper shell with internal mature peach; other vegetation hidden.')
    report = {'source_blend_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'lighting': manifest['lighting'], 'samples': 24, 'views': witnesses,
              'limits': ['Isolated diagnostics, not photometric orchard comparisons.',
                         'Visibility changed only in memory; source blend is never saved.']}
    (args.out / 'feature_render_manifest.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
