"""用本仓 YOLO + MobileSAM 核对 Blender 渲染。不导出。"""

from __future__ import annotations

import json
import os

import cv2
import numpy as np
from ultralytics import SAM, YOLO

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
YOLO_W = (
    '/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_harvester/model/best.pt')
SAM_W = (
    '/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_harvester/model/mobile_sam.pt')
RENDERS = os.path.join(ROOT, 'renders')
OUT = os.path.join(ROOT, 'data', 'render_check.json')


def main() -> None:
    detector = YOLO(YOLO_W)
    segmenter = SAM(SAM_W)
    report = []
    for name in ('cam_alley.png', 'cam_up.png', 'cam_rows.png', 'cam_block.png'):
        path = os.path.join(RENDERS, name)
        image = cv2.imread(path)
        result = detector.predict(
            image, conf=0.35, imgsz=640, verbose=False, device='cpu')[0]
        boxes = []
        for cls, conf, xyxy in zip(
                result.boxes.cls, result.boxes.conf, result.boxes.xyxy):
            boxes.append({
                'class_name': detector.names[int(cls)],
                'conf': round(float(conf), 3),
                'bbox': [int(v) for v in xyxy.tolist()],
            })
        masks = 0
        if boxes:
            prompts = [item['bbox'] for item in boxes[:8]]
            segmented = segmenter.predict(
                image, bboxes=prompts, verbose=False, device='cpu')[0]
            if segmented.masks is not None:
                masks = len(segmented.masks)
                overlay = image.copy()
                for mask in segmented.masks.data.cpu().numpy():
                    mask = cv2.resize(
                        mask.astype('float32'),
                        (image.shape[1], image.shape[0])) > 0.5
                    overlay[mask] = (
                        0.55 * overlay[mask]
                        + 0.45 * np.array((0, 180, 40), dtype=np.float32)
                    ).astype(overlay.dtype)
                cv2.imwrite(
                    os.path.join(RENDERS, name.replace('.png', '_sam.png')),
                    overlay)
        report.append({
            'image': name, 'detections': boxes, 'sam_masks': masks})
        print(name, boxes, 'masks', masks)
    with open(OUT, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    main()
