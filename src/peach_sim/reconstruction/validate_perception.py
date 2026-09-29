"""Evaluate actual visible Blender instance masks with project YOLO and MobileSAM."""
from PIL import Image, ImageDraw, ImageOps
import numpy as np
import json
import argparse
import hashlib
from pathlib import Path
import os
os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '1'
import cv2  # noqa: E402
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
from render_evidence import validate_render_set  # noqa: E402


def iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    return inter / max(1, (a[2] - a[0]) * (a[3] - a[1]) +
                       (b[2] - b[0]) * (b[3] - b[1]) - inter)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=HERE / 'output',
                        help='Directory containing same-scene renders and manifest')
    parser.add_argument('--views', default='reference,detail,orchard',
                        help='comma list; field scale renders orchard,aisle')
    args = parser.parse_args()
    out = args.out
    views = tuple(args.views.split(','))
    manifest_path = out / 'scene_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    validate_render_set(out, manifest, views)
    import torch
    torch.set_num_threads(6)
    from ultralytics import SAM, YOLO
    det = YOLO(str(ROOT / 'src/peach_harvester/model/best.pt'))
    sam = SAM(str(ROOT / 'src/peach_harvester/model/mobile_sam.pt'))
    measure = json.loads(
        (HERE / 'evidence/reference_measurements.json').read_text())
    result = {
        'model_classes': det.names,
        'confidence_threshold': .25,
        'box_iou_threshold': .5,
        'scene_manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        'lighting': manifest.get('lighting'),
        'model_sha256': hashlib.sha256(
            (ROOT / 'src/peach_harvester/model/best.pt').read_bytes()).hexdigest(),
        'sam_model_sha256': hashlib.sha256(
            (ROOT / 'src/peach_harvester/model/mobile_sam.pt').read_bytes()).hexdigest(),
        'views': {},
        'notes': [
            'SAM is GT-box prompted here; mask score is not detector recall.',
            'Reference optical depth is independently derived from world Position Y.',
            'Source SAM masks are estimated labels, not manual ground truth.']}
    for view in views:
        path = out / f'{view}.png'
        rgb = Image.open(path).convert('RGB')
        width, height = rgb.size
        if view=="reference" and (width,height)!=(1280,720):
            raise ValueError("Reference depth comparison requires native 1280x720 rendering")
        ids = cv2.imread(
            str(
                out /
                f'{view}_IndexOB_0001.exr'),
            cv2.IMREAD_UNCHANGED)[
            :,
            :,
            0].round().astype(int)
        gt = []
        for t in manifest['targets']:
            mask = ids == t['id']
            ys, xs = np.where(mask)
            if len(xs) < 100:
                continue
            # Tiny distant objects are reported but excluded from near-camera
            # validation.
            box = [int(xs.min()), int(ys.min()), int(
                xs.max() + 1), int(ys.max() + 1)]
            if max(box[2] - box[0], box[3] - box[1]) < 20:
                continue
            gt.append({'id': t['id'], 'box': box,
                      'pixels': len(xs), 'name': t['name']})
        pred = det(str(path), conf=.25, device='cpu', verbose=False)[0]
        pred.save(filename=str(out / f'{view}_detection.jpg'))
        detections = [{'box': b.xyxy[0].tolist(), 'confidence': float(
            b.conf[0]), 'class': int(b.cls[0])} for b in pred.boxes]
        bag_ids = {int(k) for k, v in det.names.items()
                   if 'bag' in v and 'nobag' not in v}
        candidates = sorted([(iou(g['box'], d['box']), j, k) for j, g in enumerate(
            gt) for k, d in enumerate(detections) if d['class'] in bag_ids], reverse=True)
        matched_g = set()
        matched_d = set()
        matches = []
        for overlap, j, k in candidates:
            if overlap < .5:
                break
            if j in matched_g or k in matched_d:
                continue
            matched_g.add(j)
            matched_d.add(k)
            matches.append(
                {'gt_id': gt[j]['id'], 'detection_index': k, 'box_iou': overlap})
        mask_scores = []
        prompts = gt[:16]
        overlay = rgb.copy()
        draw = ImageDraw.Draw(overlay)
        if prompts:
            seg = sam(
                str(path),
                bboxes=[
                    g['box'] for g in prompts],
                device='cpu',
                verbose=False)[0]
            masks = seg.masks.data.cpu().numpy() > .5
            for g, m in zip(prompts, masks):
                if m.shape != ids.shape:
                    m = cv2.resize(
                        m.astype('uint8'), (width, height), interpolation=cv2.INTER_NEAREST) > 0
                target = ids == g['id']
                score = float((m & target).sum() / max(1, (m | target).sum()))
                mask_scores.append({'id': g['id'], 'prompted_sam_iou': score})
                contour = cv2.findContours(
                    m.astype('uint8'),
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[0]
                for c in contour:
                    if len(c) > 2:
                        draw.line([tuple(p[0]) for p in c] +
                                  [tuple(c[0][0])], fill=(40, 255, 170), width=2)
                draw.text(
                    (g['box'][0], g['box'][1]), f"GT {
                        g['id']} SAM {
                        score:.2f}", fill=(
                        255, 255, 0))
        overlay.save(out / f'{view}_segmentation.jpg')
        entry = {'visible_gt': gt, 'detections': detections, 'matches': matches, 'recall_iou50': len(matches) / max(1, len(gt)),
                 'precision_iou50': len(matches) / max(1, sum(d['class'] in bag_ids for d in detections)), 'prompted_sam': mask_scores}
        if view == 'reference':
            depth = cv2.imread(
                str(out / 'reference_Position_0001.exr'), cv2.IMREAD_UNCHANGED)[:, :, 1]
            real = np.load(
                HERE / 'evidence/1200_depth.npy').astype(float) * .001
            masks = np.load(HERE / 'evidence/1200_masks.npz')['masks']
            errors = []
            for target in manifest['targets']:
                if target.get('source_frame') != '1200':
                    continue
                rmask = masks[target['source_instance']]
                render = ids == target['id']
                valid = render & rmask & (real > 0)
                source_iou = float((render & rmask).sum() /
                                   max(1, (render | rmask).sum()))
                errors.append(
                    {
                        'id': target['id'],
                        'source_mask_iou': source_iou,
                        'overlap_valid_pixels': int(
                            valid.sum()),
                        'depth_mae_m': float(
                            np.abs(
                                depth[valid] -
                                real[valid]).mean()) if valid.any() else None})
            entry['reference_comparison'] = errors
            # Export actual axial depth, preserving invalid pixels at zero.
            depth_mm = np.where(
                np.isfinite(depth) & (
                    depth > 0) & (
                    depth < 65),
                depth *
                1000,
                0).astype(
                np.uint16)
            Image.fromarray(depth_mm).save(
                out / 'reference_depth_mm.png')
        result['views'][view] = entry
        print(
            view,
            'GT',
            len(gt),
            'det',
            len(detections),
            'matches',
            len(matches),
            flush=True)
    (out / 'perception_validation.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
