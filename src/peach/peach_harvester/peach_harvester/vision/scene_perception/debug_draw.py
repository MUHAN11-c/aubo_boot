"""
Debug 图像素绘制族（W3 自 visualization 拆出）：框/轮廓/箭头/文字叠加.

公开 API：draw_debug（原地改写 BGR 画布）。观测消息组装在同拆
msg_builders；本模块不构造任何 ROS 消息。
"""
from __future__ import annotations

import cv2
import numpy as np

from peach_harvester.vision.scene_perception.image_gates import clip_bbox


def _clamp_px(img, x, y):
    """像素点裁到图内（含边界），供 cv2 画线/点用."""
    h, w = img.shape[:2]
    return int(np.clip(int(round(x)), 0, w - 1)), int(
        np.clip(int(round(y)), 0, h - 1))


def _draw_label(img, text, x, y, color):
    """
    在 (x,y) 框左上角附近画带底的 ID/置信度，整段文字钳在图内.

    OpenCV putText 的 y 是基线，字高会伸到基线上方；贴顶的框若只减几像素
    会把置信度画出画面。先 getTextSize，优先画在框顶上方，不够则落到框内。
    """
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thickness = 0.55, 2
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    pad = 3
    box_w = tw + 2 * pad
    box_h = th + baseline + 2 * pad
    tx = int(max(0, min(x, w - box_w)))
    above = y - box_h
    if above >= 0:
        ty_box = above
    else:
        ty_box = int(max(0, min(y + 2, h - box_h)))
    tx = int(tx)
    ty_box = int(ty_box)
    cv2.rectangle(
        img, (tx, ty_box),
        (min(w - 1, tx + box_w), min(h - 1, ty_box + box_h)),
        (0, 0, 0), -1)
    cv2.putText(
        img, text, (tx + pad, ty_box + pad + th),
        font, scale, color, thickness)


# 官方 plot 风格逐实例调色板（BGR；按 target_id 稳定取色，多目标可区分）
_PALETTE = (
    (80, 220, 80), (235, 140, 60), (60, 60, 235), (200, 60, 220),
    (50, 200, 220), (40, 120, 240), (180, 200, 60), (120, 160, 255),
)
_MASK_ALPHA = 0.40
"""官方 plot 同量级的半透明掩膜填充强度（0-1；纹理仍可透见）."""


def grasp_order_by_id(locked, priorities) -> dict:
    """
    确认锁定之后的抓取顺序，1 为最先抓.

    锁定前还没有这套顺序，返回空表。画面上是否沿用上一套由 orders_for_image 决定。
    """
    if not locked:
        return {}
    return {
        str(target_id): int(order)
        for target_id, order in priorities.items()
        if int(order) > 0
    }


def orders_for_image(locked, priorities, remembered=None) -> dict:
    """画面上的抓取顺序：有新锁定用新的，否则沿用上一套同一坐标身份的顺序."""
    current = grasp_order_by_id(locked, priorities)
    if current:
        return current
    if not remembered:
        return {}
    return {
        str(target_id): int(order)
        for target_id, order in remembered.items()
        if int(order) > 0
    }


def stick_labels(current, remembered, orders):
    """
    抓取顺序附到本帧检测；绝不复用上一帧像素框或二维几何.

    世界身份可以持续存在，像素几何仅属于当前曝光。remembered仅保留签名兼容。
    """
    allowed = {
        str(target_id): int(order)
        for target_id, order in (orders or {}).items()
        if int(order) > 0
    }
    memory = {}
    drawn = []
    for item in current:
        tid = str(item.get('tid') or '')
        # 还没有抓取顺序时，检测和置信度都画。顺序已经在画面上之后，
        # 锁定集以外的新号和裸置信度不再叠上去（过程中会盖住已有的 #n）。
        if not allowed:
            drawn.append(item)
            continue
        if tid not in allowed:
            continue
        updated = dict(item)
        updated['tid'] = tid
        updated['order'] = allowed[tid]
        memory[tid] = {k: v for k, v in updated.items() if k != 'mask'}
        drawn.append(updated)
    return drawn, memory


def _paint_mask(img, sam_mask, color):
    """半透明填充分割并描轮廓。掩膜尺寸不符时最近邻放大到整图."""
    h, w = img.shape[:2]
    if sam_mask.shape[:2] != (h, w):
        sam_mask = cv2.resize(
            (sam_mask > 0).astype(np.uint8), (w, h),
            interpolation=cv2.INTER_NEAREST)
    mask_bool = np.asarray(sam_mask) > 0
    if not np.any(mask_bool):
        return
    fill = np.zeros_like(img)
    fill[mask_bool] = color
    cv2.addWeighted(fill, _MASK_ALPHA, img, 1.0, 0, dst=img)
    contours, _ = cv2.findContours(
        mask_bool.astype(np.uint8),
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, color, 2)


def _overlay_text(conf, order, tid='') -> str:
    """确认后显示场景源 ID + 抓取序号；锁定前只留检测置信度."""
    if order:
        order_text = f'#{int(order)}'
        return f'{tid} {order_text}' if tid else order_text
    return f'{float(conf):.2f}'


def draw_debug(img, det, grasp_2d, sam_mask, tid='', confirmed: bool = True,
               order=None):
    """
    叠检测框、分割、底→颈箭头、剪切线和抓取序号（原地改写 img）.

    Args:
        img: (H, W, 3) uint8 BGR，被原地改写.
        det: 检测 dict（bbox、class_id、conf；class 0 绿框，其他橙框）.
        grasp_2d: BagGrasp2D（提供关键点像素与状态）.
        sam_mask: (H, W) 掩膜或 None（None 时仍画检测框）.
        tid: 目标稳定 ID（target_registry 匹配结果；空串则不显示）.
        confirmed: False 时画灰框和分割，不画袋轴和剪切线.
        order: 确认锁定后的抓取顺序，1 为最先抓。空则标签只写置信度.

    Returns
    -------
        无返回值（None）；叠加结果写回 img.

    """
    x1, y1, x2, y2 = clip_bbox(det['bbox'], img.shape)
    # OpenCV 矩形角点含边界；clip_bbox 的 x2/y2 可等于 w/h（切片右开）
    x1d, y1d = _clamp_px(img, x1, y1)
    x2d, y2d = _clamp_px(img, max(x1, x2 - 1), max(y1, y2 - 1))
    if tid:
        idx = sum(str(tid).encode('utf-8')) % len(_PALETTE)
    else:
        idx = int(det.get('class_id', 0)) % len(_PALETTE)
    mask_color = (160, 160, 160) if not confirmed else _PALETTE[idx]
    if sam_mask is not None:
        _paint_mask(img, sam_mask, mask_color)
    if not confirmed:
        cv2.rectangle(img, (x1d, y1d), (x2d, y2d), (160, 160, 160), 1)
        _draw_label(
            img, _overlay_text(det.get('conf', 0.0), order, tid),
            x1d, y1d, (180, 180, 180))
        return
    color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
    cv2.rectangle(img, (x1d, y1d), (x2d, y2d), color, 2)
    status = grasp_2d.status
    st_color = {
        'ACCEPT': (0, 220, 0), 'REOBSERVE': (0, 200, 255), 'REJECT': (0, 0, 220)
    }.get(status, (180, 180, 180))
    # 黄箭头：袋底（宽）→袋口（窄）；反了就是口底标反
    if grasp_2d.bottom_px and grasp_2d.neck_px:
        cv2.arrowedLine(
            img,
            _clamp_px(img, grasp_2d.bottom_px[0], grasp_2d.bottom_px[1]),
            _clamp_px(img, grasp_2d.neck_px[0], grasp_2d.neck_px[1]),
            (255, 255, 0), 2, tipLength=0.15)
    if grasp_2d.grasp_px:
        cv2.circle(
            img, _clamp_px(img, grasp_2d.grasp_px[0], grasp_2d.grasp_px[1]),
            5, st_color, -1)
    # TCP 是工具圆柱前端面圆心，也就是物理剪切点；travel_line 终点因此
    # 同时代表 TCP 终点与剪切中心（袋口 / 分割贴检测框极限）。投影失败
    # 或行程退化时跳过，紫色空心圆标出剪切中心，垂直于袋轴投影的紫线
    # 表示刃口切割方向。线段半长取检测框宽 1/4（近似工具刃口尺度）。
    if (grasp_2d.travel_line and len(grasp_2d.travel_line) >= 2
            and grasp_2d.travel_line[0] is not None
            and grasp_2d.travel_line[1] is not None):
        gx, gy = grasp_2d.travel_line[0]
        ex, ey = grasp_2d.travel_line[1]
        dx, dy = ex - gx, ey - gy
        norm = float((dx * dx + dy * dy) ** 0.5)
        if norm > 1e-6:
            px, py = -dy / norm, dx / norm      # 袋轴投影的图像平面法向
            half = max(16, (x2d - x1d) // 4)
            cv2.line(
                img,
                _clamp_px(img, ex - px * half, ey - py * half),
                _clamp_px(img, ex + px * half, ey + py * half),
                (255, 0, 255), 2)
            cv2.circle(img, _clamp_px(img, ex, ey), 5, (255, 0, 255), 2)
    # 确认锁定后，图上的 id 就是抓取顺序，1 最先抓。
    _draw_label(
        img, _overlay_text(det.get('conf', 0.0), order, tid),
        x1d, y1d, st_color)
