"""Measure prompted visible foliage masks using the existing MobileSAM model."""

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
import torch
from ultralytics import SAM

HERE = Path(__file__).resolve().parent
DATA = Path('/home/mu/Downloads/PeachDataSet/Peach_bag')


def digest(path):
    """Hash inputs for reproducible source attribution."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Keep leaf-like proposals and review their contours against the source."""
    prompts_path = HERE / 'evidence/reference_leaf_prompts.json'
    prompts = json.loads(prompts_path.read_text())
    boxes = prompts['boxes']
    manual_count = prompts['manual_count']
    rgb_path = DATA / 'RGB/1200.png'
    model_path = HERE.parents[2] / 'src/peach_harvester/model/mobile_sam.pt'
    torch.set_num_threads(6)
    sam = SAM(str(model_path))
    result = sam(str(rgb_path), bboxes=boxes, device='cpu', verbose=False)[0]
    raw = result.masks.data.cpu().numpy() > .5
    rgb = Image.open(rgb_path).convert('RGB')
    arr = np.asarray(rgb, dtype=float)
    green = (arr[..., 1] > arr[..., 0] * 1.03) & (arr[..., 1] > arr[..., 2] * .95)
    masks, accepted = [], []
    draw = ImageDraw.Draw(rgb)
    for index, (mask, box) in enumerate(zip(raw, boxes)):
        if mask.shape != (720, 1280):
            mask = cv2.resize(mask.astype('uint8'), (1280, 720),
                              interpolation=cv2.INTER_NEAREST) > 0
        x0, y0, x1, y1 = box
        crop = np.zeros_like(mask)
        crop[y0:y1, x0:x1] = True
        contours = cv2.findContours((mask & crop).astype('uint8'),
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        filled = np.zeros_like(mask, dtype='uint8')
        cv2.drawContours(filled, [contour], -1, 1, -1)
        mask = filled > 0
        if mask.sum() < 250 or green[mask].mean() < (.3 if index < manual_count else .50):
            continue
        if any((mask & old).sum() / max(1, (mask | old).sum()) > .60 for old in masks):
            continue
        masks.append(mask)
        accepted.append(index)
        points = [tuple(p[0]) for p in contour]
        draw.line(points + [points[0]], fill=(255, 80, 20), width=2)
        draw.text((x0, y0), str(index), fill='yellow')
    out = HERE / 'evidence/reference_leaf_masks.npz'
    np.savez_compressed(out, masks=np.array(masks), prompt_ids=accepted)
    rgb.save(HERE / 'evidence/reference_leaf_masks_review.jpg')
    report = {'accepted': len(masks), 'proposals': len(boxes),
              'union_image_fraction': float(np.any(masks, axis=0).mean()),
              'accepted_prompt_ids': accepted,
              'source_sha256': {str(p): digest(p) for p in (rgb_path, prompts_path, model_path)},
              'tool_sha256': digest(Path(__file__)), 'masks_sha256': digest(out),
              'limits': ['SAM masks are estimates and may contain overlapping leaves.',
                         'Prompt boxes are manual or green-support proposals, not labels.',
                         'Interior contour holes completed as one visible foliage patch.']}
    (HERE / 'evidence/reference_leaf_measurement.json').write_text(json.dumps(report, indent=2))
    print('Visible foliage patches:', len(masks), flush=True)


if __name__ == '__main__':
    main()
