"""Measure aligned reference frames with the project's MobileSAM weights (offline)."""
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def measure(mask, depth, box, focal=640.0):
    """Exclude holes and retain visible, not invented, depth support."""
    mask = mask.copy()
    x0, y0, x1, y1 = box
    crop = np.zeros_like(mask)
    crop[max(0, y0):min(mask.shape[0], y1), max(
        0, x0):min(mask.shape[1], x1)] = True
    mask &= crop
    valid = mask & (depth > 0)
    values = depth[valid].astype(float) * 0.001
    if values.size < 50:
        return None
    z = float(np.median(values))
    ys, xs = np.where(mask)
    profile = []
    for v in np.linspace(ys.min(), ys.max(), 33):
        band = mask[max(0, int(v) - 3):min(mask.shape[0], int(v) + 4)]
        _, bx = np.where(band)
        if len(bx):
            band_depth=depth[max(0,int(v)-3):min(mask.shape[0],int(v)+4)]
            support=band_depth[band & (band_depth>0)]
            row_depth=float(np.median(support))*.001 if len(support)>=8 else None
            profile.append([float(v),float(np.percentile(bx,1)),float(np.percentile(bx,99)),row_depth])
    return dict(box=box, depth_m=z, depth_p10_p90_m=np.percentile(values, [10, 90]).tolist(),
                valid_fraction=float(valid.sum() / mask.sum()), mask_pixels=int(mask.sum()),
                centroid_uv=[float(np.median(xs)), float(np.median(ys))],
                visible_width_m=float((xs.max() - xs.min()) * z / focal),
                visible_height_m=float((ys.max() - ys.min()) * z / focal), profile=profile)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset',
        type=Path,
        default=Path('/home/mu/Downloads/PeachDataSet'))
    parser.add_argument(
        '--frames',
        nargs='+',
        default=[
            '255',
            '800',
            '1000',
            '1200',
            '1600',
            '2000'])
    args = parser.parse_args()
    import torch
    torch.set_num_threads(6)
    from ultralytics import SAM, YOLO
    sam = SAM(str(ROOT / 'src/peach_harvester/model/mobile_sam.pt'))
    detector = YOLO(str(ROOT / 'src/peach_harvester/model/best.pt'))
    out = HERE / 'evidence'
    out.mkdir(exist_ok=True)
    report = {'source': str(args.dataset), 'intrinsics': {'width': 1280, 'height': 720, 'fx': 640., 'fy': 640., 'cx': 640., 'cy': 360.,
                                                          'status': 'FOV approximation, NOT calibrated; square pixels; author RGB HFOV=90deg'},
              'depth_scale_m': 0.001, 'depth_units_status': 'Azure Kinect mm convention; no per-device calibration file provided',
              'sources': ['https://github.com/tsing-luo/Multi-class-peach-RGB-D-dataset',
                          'https://eorganic.org/node/25727', 'https://extension.uga.edu/publications/detail.html?number=C1087'],
              'frames': {}}
    for fid in args.frames:
        p = args.dataset / 'Peach_bag'
        rgb = p / 'RGB' / f'{fid}.png'
        depth = np.asarray(Image.open(p / 'Depth' / f'{fid}.png'))
        ir = np.asarray(Image.open(p / 'Infrared' / f'{fid}.png'))
        tree = ET.parse(p / 'Annotations_VOC/VOC_1label' / f'{fid}.xml')
        boxes = [[int(o.find('bndbox/' + k).text) for k in ['xmin',
                                                            'ymin', 'xmax', 'ymax']] for o in tree.findall('object')]
        result = sam(str(rgb), bboxes=boxes, device='cpu', verbose=False)[0]
        masks = result.masks.data.cpu().numpy() > 0.5
        if masks.shape[1:] != depth.shape:
            masks = np.array([np.asarray(Image.fromarray(m).resize(
                (1280, 720), Image.Resampling.NEAREST)) for m in masks])
        for mask, box in zip(masks, boxes):
            crop = np.zeros_like(mask)
            x0, y0, x1, y1 = box
            crop[y0:y1, x0:x1] = True
            mask &= crop
        items = []
        overlay = Image.open(rgb).convert('RGB')
        draw = ImageDraw.Draw(overlay)
        for i, (mask, box) in enumerate(zip(masks, boxes)):
            item = measure(mask, depth, box)
            if item is None:
                continue
            item['id'] = i
            item['truncated'] = bool(
                box[0] <= 1 or box[1] <= 1 or box[2] >= 1279 or box[3] >= 719)
            items.append(item)
            draw.rectangle(box, outline=(40, 255, 200), width=2)
            draw.text(
                (box[0],
                 box[1] + 4),
                f"{i} z={
                    item['depth_m']:.3f}m valid={
                    item['valid_fraction']:.2f}",
                fill=(
                    255,
                    255,
                    0))
        detection = detector(str(rgb), device='cpu', verbose=False)[0]
        detection.save(filename=str(out / f'{fid}_yolo.jpg'))
        overlay.save(out / f'{fid}_measured.jpg')
        np.savez_compressed(out / f'{fid}_masks.npz', masks=masks)
        report['frames'][fid] = {'rgb': str(rgb), 'valid_depth_fraction': float((depth > 0).mean()), 'ir_nonzero_fraction': float((ir > 0).mean()), 'objects': items,
                                 'detections': [{'xyxy': b.xyxy[0].tolist(), 'class': int(b.cls[0]), 'confidence': float(b.conf[0])} for b in detection.boxes]}
        print(fid, 'objects', len(items), 'det',
              len(detection.boxes), flush=True)
    # Native RGB-D sparse samples support branch tracing; no filling of zero
    # depth.
    depth = np.array(Image.open(args.dataset / 'Peach_bag/Depth/1200.png'))
    np.save(out / '1200_depth.npy', depth)
    (out / 'reference_measurements.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
