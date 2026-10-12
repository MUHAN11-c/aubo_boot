#!/usr/bin/env python3
"""Split the original CAD assembly into palm/finger rigid bodies (offline only).

Uses trimesh connected components; no remeshing or synthetic visual fingers.
The moving subassemblies lie above z=68mm in the four CAD quadrants.
"""

from pathlib import Path

import numpy as np
import trimesh


def main():
    root = Path(__file__).resolve().parents[1]
    source = trimesh.load(root / 'meshes/visual/gripper2_link.stl', force='mesh')
    groups = {'palm': [], 'pp': [], 'pn': [], 'np': [], 'nn': []}
    for component in source.split(only_watertight=False):
        c = component.centroid
        moving = (component.bounds[0, 2] > 0.068 and
                  abs(c[0]) > 0.005 and abs(c[1]) > 0.010)
        key = ('p' if c[0] > 0 else 'n') + ('p' if c[1] > 0 else 'n')
        groups[key if moving else 'palm'].append(component)
    out = root / 'meshes/gripper2'
    out.mkdir(exist_ok=True)
    for key, components in groups.items():
        if not components:
            raise RuntimeError(f'Missing CAD subassembly: {key}')
        mesh = trimesh.util.concatenate(components)
        mesh.export(out / f'{key}_visual.stl')
        mesh.convex_hull.export(out / f'{key}_collision.stl')
        print(key, len(mesh.faces), np.round(mesh.bounds, 6).tolist())


if __name__ == '__main__':
    main()
