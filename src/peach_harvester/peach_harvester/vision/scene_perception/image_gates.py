"""
2D 图像门控：锚点投影、SAM 框规划、深度/前景掩膜（纯核）.

掩膜降级路线与边界判定是现场调出的行为（详见各函数 docstring），
cv2 承担批量投影与连通域（官方 API）。
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# 检测框外扩比例（每边各扩 10% 宽高）：容忍锚点投影与 YOLO 框边的贴边
# 误差（质心投影理论上在框内，外扩只为深度噪声/框回归抖动兜底）
DEFAULT_MARGIN_FRAC = 0.1


def project_positions_to_pixels(
        positions: Dict[str, np.ndarray],
        T_cam_world: np.ndarray,
        camera_K: dict) -> Dict[str, Tuple[float, float]]:
    """
    世界系锚点集 → 本帧像素坐标（cv2.projectPoints 批量 pinhole 投影）.

    Args:
        positions: target_id → (3,) 世界系（output_frame）锚点（米）.
        T_cam_world: (4, 4) 世界系→相机光学系齐次变换（即 output←camera
            的逆；调用方负责取逆）.
        camera_K: 内参 dict（fx/fy/cx/cy；width/height 存在时用于裁剪
            视野外投影）.

    Returns
    -------
        target_id → (u, v) 像素坐标；非有限、相机后方（z≤0）或明确
        落在图像外的锚点被剔除（剔除即视为本帧不可见，不参与门控）.

    """
    T = np.asarray(T_cam_world, dtype=float).reshape(4, 4)
    intrinsics_mat = np.array(
        [[float(camera_K['fx']), 0.0, float(camera_K['cx'])],
         [0.0, float(camera_K['fy']), float(camera_K['cy'])],
         [0.0, 0.0, 1.0]], dtype=float)
    width = camera_K.get('width')
    height = camera_K.get('height')
    out: Dict[str, Tuple[float, float]] = {}
    ids = list(positions)
    if not ids:
        return out
    P = np.asarray([np.asarray(positions[tid], dtype=float).reshape(3)
                    for tid in ids])
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    uv, _ = cv2.projectPoints(P.reshape(-1, 1, 3), rvec, T[:3, 3], intrinsics_mat, None)
    uv = uv.reshape(-1, 2)
    zc = (P @ T[:3, :3].T + T[:3, 3])[:, 2]
    for target_id, (u, v), z in zip(ids, uv, zc):
        if not (np.isfinite(u) and np.isfinite(v)) or not np.isfinite(z) \
                or z <= 1e-8:
            continue
        if width is not None and height is not None:
            if not (0.0 <= u < float(width) and 0.0 <= v < float(height)):
                continue
        out[target_id] = (float(u), float(v))
    return out


def plan_segmentation_bboxes(
        detections: List[dict],
        locked_only: bool,
        locked: bool,
        anchor_px: Optional[Dict[str, Tuple[float, float]]],
        margin_frac: float = DEFAULT_MARGIN_FRAC) -> List[Tuple[int, int, int, int]]:
    """
    决定本帧送 SAM 的检测框集（2.13-E1 门控策略，纯函数）.

    语义矩阵（详见模块 docstring 降级语义）：
      - ``locked_only=False`` 或 ``locked=False`` → 全量框（旧行为）；
      - 已锁定但 ``anchor_px=None``（锚点不可投影，如 TF 不可用帧）
        → 全量框（无法识别哪些框属于锁定集，宁多勿漏）；
      - 已锁定且 ``anchor_px`` 为空 dict → 空列表（锁定目标本帧均不
        可见，SAM 零推理）；
      - 否则只选「外扩 margin 后包含至少一个锁定锚点像素」的框。

    Args:
        detections: 本帧入管线检测 dict 列表（须带 'bbox' 键，
            (x1, y1, x2, y2) 像素框）.
        locked_only: 锁定后 selected-only 开关（yaml
            pipeline.locked_only_segmentation）.
        locked: 目标集合是否已锁定（harvest_plan.locked）.
        anchor_px: 锁定目标锚点像素集（project_positions_to_pixels
            输出）；None 表示本帧锚点不可投影.
        margin_frac: 框外扩比例（每边各扩 margin_frac×宽/高）.

    Returns
    -------
        送 SAM 的 (x1, y1, x2, y2) 框列表，顺序与 detections 一致.

    """
    all_bboxes = [tuple(d['bbox']) for d in detections]
    if not locked_only or not locked or anchor_px is None:
        return all_bboxes
    if not anchor_px:
        return []
    points = list(anchor_px.values())
    selected = []
    for bbox in all_bboxes:
        x1, y1, x2, y2 = (float(v) for v in bbox)
        mx = (x2 - x1) * margin_frac
        my = (y2 - y1) * margin_frac
        if any(x1 - mx <= u <= x2 + mx and y1 - my <= v <= y2 + my
               for u, v in points):
            selected.append(bbox)
    return selected


def clip_bbox(bbox, shape):
    """
    检测框裁剪到图像范围内.

    Args:
        bbox: (x1, y1, x2, y2) 像素框（可越界，先取整）.
        shape: 图像 shape（取前两维 h, w）.

    Returns
    -------
        (x1, y1, x2, y2) 裁剪后的整数框（可能退化）.

    """
    h, w = shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    return max(0, x1), max(0, y1), min(w, x2), min(h, y2)


def valid_depth_mask(depth, min_depth_m: float, max_depth_m: float):
    """
    有效深度掩膜：非 0、非饱和，且在 [min_depth_m, max_depth_m] 内.

    Args:
        depth: (h, w) uint16 深度（毫米）.
        min_depth_m: 有效深度下限 (m)，过近视为噪声.
        max_depth_m: 有效深度上限 (m)，过远视为背景.

    Returns
    -------
        (h, w) bool 掩膜.

    """
    z = depth.astype(np.float32) / 1000.0
    return (depth > 0) & (depth < 65535) & (z > min_depth_m) & (z < max_depth_m)


def foreground_mask(depth, valid, supplied_mask, bbox, source):
    """
    前景掩膜：优先外部掩膜 ∩ 有效深度；否则深度带连通域显式降级.

    降级路线：ROI 中心 1/3 区域的深度中位数 ± max(25mm, 3·MAD) 带
    → 8 连通域，取中心所在域（中心无域则取最大域）。

    Args:
        depth: (h, w) uint16 ROI 深度（毫米）.
        valid: (h, w) bool 有效深度掩膜.
        supplied_mask: 外部掩膜（全图或本 ROI）；形状相符且与有效深度
            交集 ≥50 像素才采用，否则忽略走降级.
        bbox: 全图坐标下的 (x1, y1, x2, y2)，用于裁全图掩膜.
        source: 外部掩膜来源标签.

    Returns
    -------
        ((h, w) bool 前景掩膜, 来源标签)；降级时标签为 'depth_fallback'.

    """
    h, w = depth.shape
    if supplied_mask is not None:
        m = np.asarray(supplied_mask, dtype=bool)
        if m.shape != (h, w):
            x1, y1, x2, y2 = map(int, bbox)
            if m.ndim == 2 and m.shape[0] >= y2 and m.shape[1] >= x2:
                m = m[y1:y2, x1:x2]
        if m.shape == (h, w) and int((m & valid).sum()) >= 50:
            return m & valid, source
    # Explicit fallback: depth mode in central region + connected component.
    ch, cw = max(1, h // 3), max(1, w // 3)
    cy, cx = h // 2, w // 2
    centre = depth[cy-ch//2:cy+(ch+1)//2, cx-cw//2:cx+(cw+1)//2]
    values = centre[(centre > 0) & (centre < 65535)]
    if len(values) < 20:
        return np.zeros_like(valid), 'depth_fallback'
    z0 = np.median(values)
    mad = np.median(np.abs(values.astype(float) - z0))
    band = max(25.0, 3.0 * mad)
    binary = ((np.abs(depth.astype(float) - z0) <= band) & valid).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    if n <= 1:
        return binary.astype(bool), 'depth_fallback'
    label = labels[cy, cx]
    if label == 0:
        label = int(1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == label, 'depth_fallback'
