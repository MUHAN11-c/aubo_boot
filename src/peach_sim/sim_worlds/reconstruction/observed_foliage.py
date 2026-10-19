"""Build visible foliage patches from estimated masks and robust leaf depth."""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
DATA = Path('/home/mu/Downloads/PeachDataSet/Peach_bag')


def leaf_surface(mask, depth, step=2):
    """Complete a mask with a shallow inferred sheet at supported median depth."""
    valid = mask & (depth > 220) & (depth < 1200)
    if valid.sum() < 30:
        return None
    samples = depth[valid].astype(float) * .001
    median = float(np.median(samples))
    inliers = samples[np.abs(samples - median) < .04]
    if len(inliers) < 30:
        return None
    median = float(np.median(inliers))
    yy, xx = np.where(mask)
    coords = np.column_stack((xx, yy)).astype('float64')
    center, axes, _ = cv2.PCACompute2(coords, mean=None)
    y0, y1, x0, x1 = yy.min(), yy.max() + 1, xx.min(), xx.max() + 1
    vv, uu = np.mgrid[y0:y1:step, x0:x1:step]
    good = mask[vv, uu]
    pixels = np.column_stack((uu[good], vv[good]))
    projected = (pixels - center) @ axes.T
    lo, hi = projected.min(axis=0), projected.max(axis=0)
    uv = (projected - lo) / np.maximum(hi - lo, 1.)
    # Curvature is inferred; individual zero-depth pixels are not called measured.
    z = median + .002 * (2 * uv[:, 1] - 1) ** 2
    vertices = np.column_stack(((pixels[:, 0] - 640) * z / 640, z,
                                1.6 + (360 - pixels[:, 1]) * z / 640))
    index = np.full(good.shape, -1, dtype=int)
    index[good] = np.arange(good.sum())
    faces = []
    for y in range(good.shape[0] - 1):
        for x in range(good.shape[1] - 1):
            for corners in (((y, x), (y + 1, x), (y, x + 1)),
                            ((y, x + 1), (y + 1, x), (y + 1, x + 1))):
                ids = [index[p] for p in corners]
                if min(ids) >= 0:
                    faces.append(ids)
    endpoints = vertices[[np.argmin(projected[:, 0]), np.argmax(projected[:, 0])]]
    return {'vertices': vertices, 'faces': np.asarray(faces).reshape(-1, 3), 'uv': uv,
            'endpoints': endpoints.tolist(), 'depth_m': median,
            'valid_depth_fraction': float(valid.sum() / mask.sum()),
            'depth_inlier_fraction': len(inliers) / len(samples)}


def main():
    """Keep measured silhouettes separate from inferred surface completion."""
    masks_path = HERE / 'evidence/reference_leaf_masks.npz'
    records = np.load(masks_path)
    depth_path = DATA / 'Depth/1200.png'
    depth = np.asarray(Image.open(depth_path))
    vertices, faces, uvs, materials, patches = [], [], [], [], []
    for mask, prompt in zip(records['masks'], records['prompt_ids']):
        item = leaf_surface(mask, depth)
        if item is None or not len(item['faces']):
            continue
        offset = len(vertices)
        vertices.extend(item['vertices'])
        faces.extend(item['faces'] + offset)
        uvs.extend(item['uv'])
        materials.extend([int(prompt) % 5] * len(item['faces']))
        patches.append({k: v for k, v in item.items() if k not in ('vertices', 'faces', 'uv')}
                       | {'prompt_id': int(prompt)})
    out = HERE / 'evidence/observed_foliage.npz'
    np.savez_compressed(out, vertices=vertices, faces=faces, uv=uvs, materials=materials)
    sources = [masks_path, depth_path, HERE / 'evidence/reference_leaf_prompts.json',
               HERE / 'evidence/reference_leaf_measurement.json']
    report = {'patches': patches, 'vertices': len(vertices), 'triangles': len(faces),
              'step_pixels': 2, 'source_sha256': {
                  str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'mesh_sha256': hashlib.sha256(out.read_bytes()).hexdigest(),
              'limits': ['SAM silhouettes are estimates, not manual leaf ground truth.',
                         'Median-depth curved sheets complete missing per-pixel depth.',
                         'Surface curvature, normals, veins and hidden support inferred.',
                         'No source RGB texture or per-pixel color baking.',
                         'Single-view partial foliage; no unseen-backside reconstruction.']}
    (HERE / 'evidence/observed_foliage.json').write_text(json.dumps(report, indent=2))
    print(len(patches), 'foliage patches;', len(faces), 'triangles')


if __name__ == '__main__':
    main()
