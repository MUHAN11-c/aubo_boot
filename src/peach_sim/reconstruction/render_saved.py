"""Render an existing .blend without rebuilding geometry."""
from pathlib import Path
import sys
import argparse
import bpy
p = argparse.ArgumentParser()
p.add_argument('--view', default='reference')
p.add_argument('--samples', type=int, default=32)
p.add_argument('--width', type=int, default=1280)
a = p.parse_args(sys.argv[sys.argv.index('--') + 1:])
s = bpy.context.scene
names = {
    'reference': 'Reference 1200 / approximate 90deg',
    'detail': 'Fruit branch / oblique',
    'orchard': 'Orchard / aisle overview'}
out = Path(__file__).resolve().parent / 'output'
s.camera = bpy.data.objects[names[a.view]]
s.cycles.samples = a.samples
s.render.resolution_x = a.width
s.render.resolution_y = round(a.width * 9 / 16)
s.render.filepath = str(out / f'{a.view}.png')
for node in s.node_tree.nodes:
    if node.type == 'OUTPUT_FILE':
        node.base_path = str(out)
        for slot, key in zip(
                node.file_slots, ['Depth', 'IndexOB', 'Position']):
            slot.path = f'{a.view}_{key}_'
bpy.ops.render.render(write_still=True)
