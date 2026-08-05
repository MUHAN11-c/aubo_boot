"""
离线真值标注 schema 与评估指标.

供 ``e2e_validate`` 在有人工标注时计算：
检测召回、状态一致率、错误 ACCEPT（安全首要）、关键点像素误差、掩膜 IoU。
无标注时 e2e 只报状态计数与代理指标，不走本模块。
"""
from __future__ import annotations

from collections import defaultdict
import json
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


VALID_STATUSES = {'ACCEPT', 'REOBSERVE', 'REJECT'}


def load_annotations(path: Optional[Path]) -> dict[str, list[dict]]:
    """
    加载 JSONL / JSON 标注，按 frame_id 分组，并校验必填字段.

    每条至少含：frame_id, target_id, class_id, bbox(4), expected_status。

    Args:
        path: 标注文件路径（.jsonl 逐行一条 JSON；.json 为列表或
            {"annotations": [...]}）；None 返回空 dict；字段缺失/非法
            抛 ValueError.

    Returns
    -------
        {frame_id(str): [标注 dict, ...]}.

    """
    if path is None:
        return {}
    path = Path(path)
    rows = []
    if path.suffix.lower() == '.jsonl':
        for line_no, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f'{path}:{line_no}: invalid JSON: {exc}') from exc
    else:
        payload = json.loads(path.read_text(encoding='utf-8'))
        rows = payload['annotations'] if isinstance(payload, dict) else payload
    grouped = defaultdict(list)
    for index, row in enumerate(rows):
        for field in ('frame_id', 'target_id', 'class_id', 'bbox', 'expected_status'):
            if field not in row:
                raise ValueError(f'annotation {index} missing {field}')
        if len(row['bbox']) != 4:
            raise ValueError(f'annotation {index} bbox must have four values')
        if row['expected_status'] not in VALID_STATUSES:
            raise ValueError(f'annotation {index} invalid expected_status')
        grouped[str(row['frame_id'])].append(row)
    return dict(grouped)


def bbox_iou(a, b) -> float:
    """
    轴对齐框 IoU（xyxy）。用于检测框与真值匹配.

    Args:
        a: (x1, y1, x2, y2) 框（像素）.
        b: (x1, y1, x2, y2) 框（像素）.

    Returns
    -------
        IoU ∈ [0, 1]；并集为 0 时给 0.

    """
    ax1, ay1, ax2, ay2 = map(float, a)
    bx1, by1, bx2, by2 = map(float, b)
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    union = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    union += max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - inter
    return inter / union if union > 0 else 0.0


def rasterize_segmentation(annotation: dict, shape: tuple[int, int]) -> Optional[np.ndarray]:
    """
    把标注里的多边形 segmentation 栅格化为 bool 掩膜；无分割则返回 None.

    Args:
        annotation: 标注 dict（segmentation 为 [[x1, y1, x2, y2, ...], ...]
            像素多边形；少于 3 点的多边形跳过）.
        shape: (H, W) 全图尺寸.

    Returns
    -------
        (H, W) bool 掩膜；无 segmentation 或为空给 None.

    """
    polygons = annotation.get('segmentation')
    if not polygons:
        return None
    mask = np.zeros(shape, dtype=np.uint8)
    for flat in polygons:
        points = np.asarray(flat, dtype=np.float32).reshape(-1, 2).round().astype(np.int32)
        if len(points) >= 3:
            cv2.fillPoly(mask, [points], 1)
    return mask.astype(bool)


class AnnotationMetrics:
    """
    逐帧累积检测 / 掩膜 / 关键点 / 选择性状态指标，最后 ``report()`` 汇总.

    匹配策略：仅 class_id==0（袋）的真值；与检测按 IoU 贪心一对一。
    安全门：错误 ACCEPT 的 Beta(0.95) 上界 < 1% 才算 offline_safety_gate_pass。
    """

    def __init__(self, annotations: dict[str, list[dict]], modes: tuple[str, ...],
                 min_match_iou: float = 0.5):
        """
        初始化累积器.

        Args:
            annotations: load_annotations 的返回（{frame_id: [标注]}）.
            modes: 参与评估的前景模式 ID 元组.
            min_match_iou: 检测框与真值框匹配的最小 IoU.

        Returns
        -------
            无返回值（None）.

        """
        self.annotations = annotations
        self.modes = modes
        self.min_match_iou = min_match_iou
        self.gt_targets = 0
        self.matched_targets = 0
        self.false_positive_detections = 0
        self.mode = {m: defaultdict(list) for m in modes}

    def add_frame(self, frame_id: str, detections: list[dict],
                  results_by_detection: list[dict], image_shape: tuple[int, int]):
        """
        用本帧检测结果与真值对齐，累加各 mode 的指标样本.

        Args:
            frame_id: 帧 ID（在 annotations 中查真值；无真值帧只累计误检）.
            detections: 本帧入选检测列表（含 bbox/conf/class_id）.
            results_by_detection: 与 detections 对齐的 [{mode: TargetPoseResult}].
            image_shape: (H, W) 全图尺寸（掩膜 IoU 贴回全图用）.

        Returns
        -------
            无返回值（None）；指标累加进 self.mode.

        """
        # 当前真值协议只评估袋类（class 0）
        truth = [a for a in self.annotations.get(frame_id, []) if a['class_id'] == 0]
        self.gt_targets += len(truth)
        unused = set(range(len(detections)))
        matches = []
        for ann in truth:
            scored = sorted(((bbox_iou(ann['bbox'], detections[i]['bbox']), i)
                             for i in unused), reverse=True)
            if scored and scored[0][0] >= self.min_match_iou:
                _, index = scored[0]
                unused.remove(index)
                matches.append((ann, index))
        self.matched_targets += len(matches)
        self.false_positive_detections += len(unused)

        for ann, det_index in matches:
            gt_mask = rasterize_segmentation(ann, image_shape)
            for mode in self.modes:
                result = results_by_detection[det_index][mode]
                pred_status = result.grasp_3d.status
                expected = ann['expected_status']
                self.mode[mode]['status_correct'].append(pred_status == expected)
                # 安全首要：预测 ACCEPT 但真值不是 → 错误接受
                self.mode[mode]['false_accept'].append(
                    pred_status == 'ACCEPT' and expected != 'ACCEPT')
                self.mode[mode]['accepted'].append(pred_status == 'ACCEPT')
                self._add_point_error(mode, 'bottom_px', ann, result.grasp_2d.bottom_px)
                self._add_point_error(mode, 'neck_px', ann, result.grasp_2d.neck_px)
                if gt_mask is not None and result.grasp_2d.foreground_mask is not None:
                    x1, y1, x2, y2 = map(int, detections[det_index]['bbox'])
                    pred = np.zeros(image_shape, dtype=bool)
                    local = result.grasp_2d.foreground_mask
                    # 管线前景是 ROI 局部掩膜，贴回全图再算 IoU
                    if local.shape == (y2-y1, x2-x1):
                        pred[y1:y2, x1:x2] = local
                        union = np.logical_or(pred, gt_mask).sum()
                        iou = np.logical_and(pred, gt_mask).sum() / max(union, 1)
                        self.mode[mode]['mask_iou'].append(float(iou))

    def _add_point_error(self, mode, key, ann, predicted):
        """
        关键点像素欧氏误差（真值与预测都有时才记）.

        Args:
            mode: 模式 ID.
            key: 关键点名（'bottom_px' / 'neck_px'）.
            ann: 真值标注 dict（取 ann[key]，(u, v) 像素）.
            predicted: 预测关键点 (u, v) 像素或 None.

        Returns
        -------
            无返回值（None）；误差累加进 self.mode[mode].

        """
        target = ann.get(key)
        if target is not None and predicted is not None:
            error = float(np.linalg.norm(np.asarray(predicted) - np.asarray(target)))
            self.mode[mode][f'{key}_error_px'].append(error)

    def report(self) -> dict:
        """
        汇总检测召回与各 mode 的 rate / 分位数 / 错误 ACCEPT 置信上界.

        Returns
        -------
            dict：ground_truth/matched/召回率/误检数 + modes{mode: 指标}；
            有 ACCEPT 样本时含 false_accept_95pct_upper 与
            offline_safety_gate_pass（Beta(0.95) 上界 <1% 才算通过）.

        """
        result = {
            'ground_truth_targets': self.gt_targets,
            'matched_targets': self.matched_targets,
            'detection_recall': (
                self.matched_targets / self.gt_targets if self.gt_targets else None),
            'false_positive_detections': self.false_positive_detections,
            'modes': {},
        }
        for mode, metrics in self.mode.items():
            out = {}
            for key, values in metrics.items():
                if not values:
                    continue
                array = np.asarray(values, dtype=float)
                if key in {'false_accept', 'accepted', 'status_correct'}:
                    out[key + '_count'] = int(array.sum())
                    out[key + '_rate'] = float(array.mean())
                else:
                    out[key] = {
                        'n': len(values), 'median': float(np.median(array)),
                        'p95': float(np.percentile(array, 95)),
                    }
            false_accepts = int(np.sum(metrics.get('false_accept', [])))
            accepted = int(np.sum(metrics.get('accepted', [])))
            if accepted:
                # Clopper-Pearson 式 Beta 上界：P(错误ACCEPT | ACCEPT) 的 95% 上界
                from scipy.stats import beta
                upper = (1.0 if false_accepts >= accepted else
                         float(beta.ppf(0.95, false_accepts + 1,
                                        accepted - false_accepts)))
                out['false_accept_given_accept_rate'] = false_accepts / accepted
                out['false_accept_95pct_upper'] = upper
                out['offline_safety_gate_pass'] = upper < 0.01
            else:
                out['false_accept_given_accept_rate'] = None
                out['false_accept_95pct_upper'] = None
                out['offline_safety_gate_pass'] = False
            result['modes'][mode] = out
        return result
