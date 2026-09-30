"""Triangulate only observed green RGB-D surfaces, without photo textures."""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
DATA = Path('/home/mu/Downloads/PeachDataSet/Peach_bag')


def surface_mesh(rgb, depth, step=4):
    """Back-project valid samples; reject faces crossing depth discontinuities."""
    a = np.asarray(rgb, dtype=float)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    yellow = (r > b * 1.15) & (g > b * 1.15) & (np.abs(r - g) < 30)
    mask = ((g > r * 1.06) & (g > b * 1.02) & (g > 20) & (g < 235)
            & ~yellow & (depth > 220) & (depth < 1200))
    filtered = cv2.medianBlur(depth.astype('uint16'), 5).astype(float) * .001
    vv, uu = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    good = mask[vv, uu] & (filtered[vv, uu] > .22)
    z = filtered[vv, uu]
    index = np.full(good.shape, -1, dtype=int)
    index[good] = np.arange(good.sum())
    vertices = np.column_stack(((uu[good] - 640) * z[good] / 640,
                                z[good], 1.6 + (360 - vv[good]) * z[good] / 640))
    faces = []
    for y in range(good.shape[0] - 1):
        for x in range(good.shape[1] - 1):
            for corners in (((y, x), (y + 1, x), (y, x + 1)),
                            ((y, x + 1), (y + 1, x), (y + 1, x + 1))):
                ids = [index[p] for p in corners]
                if min(ids) < 0:
                    continue
                values = [z[p] for p in corners]
                if max(values) - min(values) <= .025:
                    # Front normals point towards the source camera (-Y).
                    faces.append(ids)
    return vertices, np.asarray(faces, dtype=np.int32).reshape(-1, 3)


def main():
    """Save observed geometry separately from inferred leaf instances."""
    rp, dp = DATA / 'RGB/1200.png', DATA / 'Depth/1200.png'
    vertices, faces = surface_mesh(np.asarray(Image.open(rp).convert('RGB')),
                                   np.asarray(Image.open(dp)))
    out = HERE / 'evidence/observed_foliage.npz'
    np.savez_compressed(out, vertices=vertices, faces=faces)
    report = {'vertices': len(vertices), 'triangles': len(faces), 'step_pixels': 4,
              'max_face_depth_jump_m': .025,
              'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (rp, dp)},
              'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'mesh_sha256': hashlib.sha256(out.read_bytes()).hexdigest(),
              'limits': ['Partial visible foliage surface, not watertight individual leaves.',
                         'No zero-depth filling or source RGB texture/color baking.',
                         'Green proxy excludes yellow watermark-like pixels.',
                         'Intrinsics approximate; unseen leaf backs remain inferred.']}
    (HERE / 'evidence/observed_foliage.json').write_text(json.dumps(report, indent=2))
    print(len(vertices), 'vertices;', len(faces), 'observed triangles')


if __name__ == '__main__':
    main()
