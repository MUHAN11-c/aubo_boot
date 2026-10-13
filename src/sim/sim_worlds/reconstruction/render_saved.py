"""Render an existing .blend without rebuilding geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_scene import _enable_gpu  # noqa: E402,I100
import lighting  # noqa: E402,I100
from lighting_apply import apply_lighting  # noqa: E402
from render_evidence import record_render  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument('--view', default='reference',
               choices=['reference', 'detail', 'orchard', 'all'])
p.add_argument('--samples', type=int, default=32)
p.add_argument('--width', type=int, default=1280)
p.add_argument('--lighting', choices=sorted(lighting.PRESETS))
p.add_argument('--out', type=Path, default=HERE / 'output')
a = p.parse_args(sys.argv[sys.argv.index('--') + 1:])
s = bpy.context.scene
s.cycles.device = 'GPU' if _enable_gpu() != 'CPU' else 'CPU'
names = {
    'reference': 'Reference 1200 / approximate 90deg',
    'detail': 'Filled bag close-up / inferred tree',
    'orchard': 'Orchard / aisle overview'}
out = a.out
out.mkdir(parents=True, exist_ok=True)
s.cycles.samples = a.samples
s.render.resolution_x = a.width
s.render.resolution_y = round(a.width * 9 / 16)
source = Path(bpy.data.filepath)
manifest_path = source.parent / 'scene_manifest.json'
manifest = json.loads(manifest_path.read_text())
# A previous render may have updated this manifest without saving the blend.
# Always apply the declared preset so metadata and actual lighting agree.
preset_name = a.lighting or manifest['lighting']['preset']
apply_lighting(s, lighting.get(preset_name))
existing = out / 'scene_manifest.json'
manifest['rendered_views'] = (json.loads(existing.read_text()).get('rendered_views', {})
                              if existing.exists() else {})
manifest['source_blend_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
manifest['render_settings'] = {
    'samples': a.samples, 'width': a.width,
    'height': s.render.resolution_y, 'exposure_ev': s.view_settings.exposure,
    'view_transform': s.view_settings.view_transform,
    'look': s.view_settings.look,
}
manifest['lighting'] = lighting.manifest_entry(preset_name)
(out / 'scene_manifest.json').write_text(json.dumps(manifest, indent=2))
for view in names if a.view == 'all' else [a.view]:
    camera_name = manifest['cameras'][view].get('object_name', names[view])
    s.camera = bpy.data.objects[camera_name]
    s.render.filepath = str(out / f'{view}.png')
    for node in s.node_tree.nodes:
        if node.type == 'OUTPUT_FILE':
            node.base_path = str(out)
            for slot, key in zip(node.file_slots, ['Depth', 'IndexOB', 'Position']):
                slot.path = f'{view}_{key}_'
    bpy.ops.render.render(write_still=True)
    record_render(out, view, manifest)
