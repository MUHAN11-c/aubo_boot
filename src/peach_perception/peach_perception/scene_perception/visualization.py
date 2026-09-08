"""
感知可视化：Debug 图叠加（确认/未确认框、ID 标签）与锁定目标 Marker.

消息转换在 conversions.py、检测点云在 cloud_utils.py；本模块只画。
三态配色与重建侧 Marker 同约定（绿 ACCEPT / 黄 REOBSERVE / 红 REJECT）。
"""
from __future__ import annotations

from typing import List

import cv2
import numpy as np
from peach_perception.scene_perception.conversions import _metric, _point
from peach_perception.scene_perception.image_gates import clip_bbox
from peach_perception.scene_perception.pose_pipelines import _rotation_to_quat
from visualization_msgs.msg import Marker

__all__ = [
    '_draw_debug',
    '_to_markers',
]


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


def _status_color(status: str):
    """
    三态 → Marker RGBA：ACCEPT 绿 / REOBSERVE 黄 / REJECT 红 / 其他灰.

    Args:
        status: 'ACCEPT' | 'REOBSERVE' | 'REJECT'（未知值给灰色）.

    Returns
    -------
        (r, g, b, a) 四元组，各分量 [0, 1].

    """
    return {
        'ACCEPT': (0.1, 0.85, 0.2, 0.9),
        'REOBSERVE': (0.95, 0.8, 0.1, 0.9),
        'REJECT': (0.9, 0.15, 0.15, 0.9),
    }.get(status, (0.6, 0.6, 0.6, 0.8))


def _to_markers(header, tid, idx, result, tool_d_inner: float) -> List[Marker]:
    """
    结果 → RViz Marker 列表：袋轴/行程箭头/刀具圆柱/果球/三轴架/状态文字.

    每个目标占用 id 段 ``idx*20 .. idx*20+19``，避免多目标冲突。

    Args:
        header: 输出头（frame_id 与候选消息一致）.
        tid: 目标 ID（用于状态文字）.
        idx: 目标序号（Marker id 段基址 = idx*20）.
        result: pipeline.TargetPoseResult.
        tool_d_inner: 工具内径 (m)，刀具圆柱直径.

    Returns
    -------
        Marker 列表（不含 DELETEALL，由调用方统一添加）.

    """
    grasp_3d = result.grasp_3d
    out: List[Marker] = []
    r, g, b, a = _status_color(grasp_3d.status)
    base_id = idx * 20

    def _mk(mid, mtype) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'scene_perception'
        m.id = base_id + mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        # ColorRGBA 字段必须是 Python float（int/np.float64 会触发断言崩溃）
        m.color.r = float(r)
        m.color.g = float(g)
        m.color.b = float(b)
        m.color.a = float(a)
        m.lifetime.sec = 0
        return m

    if grasp_3d.bag_bottom is not None and grasp_3d.bag_neck is not None:
        axis = _mk(0, Marker.LINE_LIST)
        axis.scale.x = 0.004
        axis.points = [_point(grasp_3d.bag_bottom), _point(grasp_3d.bag_neck)]
        out.append(axis)

    if (grasp_3d.entry_start is not None and grasp_3d.suggested_travel_end is not None):
        arrow = _mk(1, Marker.ARROW)
        arrow.scale.x = 0.008
        arrow.scale.y = 0.015
        arrow.scale.z = 0.015
        arrow.points = [_point(grasp_3d.entry_start), _point(grasp_3d.suggested_travel_end)]
        out.append(arrow)

    if (grasp_3d.entry_start is not None and grasp_3d.translation_direction is not None
            and grasp_3d.suggested_travel_m > 0):
        env = _mk(2, Marker.CYLINDER)
        env.ns = 'bag_envelope'
        mid = (grasp_3d.entry_start
               + 0.5 * grasp_3d.suggested_travel_m
               * grasp_3d.translation_direction)
        env.pose.position = _point(mid)
        if grasp_3d.orientation is not None:
            env.pose.orientation = _rotation_to_quat(grasp_3d.orientation)
        bag_d = float(grasp_3d.bag_diameter_upper_m or 0.06)
        env.scale.x = bag_d
        env.scale.y = bag_d
        env.scale.z = float(grasp_3d.suggested_travel_m)
        env.color.r, env.color.g, env.color.b, env.color.a = 0.2, 0.7, 0.9, 0.22
        out.append(env)
        cyl = _mk(12, Marker.CYLINDER)
        cyl.ns = 'tool_swept_volume'
        cyl.pose.position = _point(mid)
        if grasp_3d.orientation is not None:
            cyl.pose.orientation = _rotation_to_quat(grasp_3d.orientation)
        diam = float(tool_d_inner)
        cyl.scale.x = diam
        cyl.scale.y = diam
        cyl.scale.z = float(grasp_3d.suggested_travel_m)
        cyl.color.a = 0.12
        out.append(cyl)

    prior_kind = str(getattr(result, 'target_kind', '') or '')
    info = grasp_3d.diagnostic_info or {}
    prior_r = float(info.get('fruit_prior_radius_m') or 0.0)
    if ((prior_kind in ('fruit', 'sphere') or prior_r > 0)
            and grasp_3d.bag_bottom is not None and grasp_3d.bag_neck is not None):
        sphere = _mk(3, Marker.SPHERE)
        sphere.ns = 'prior'
        center = 0.5 * (np.asarray(grasp_3d.bag_bottom) + np.asarray(grasp_3d.bag_neck))
        radius = prior_r if prior_r > 0 else _metric(
            result.metrics or {}, 'fruit_radius_m', 0.0)
        if radius > 0:
            sphere.pose.position = _point(center)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * radius)
            sphere.color.a = 0.3
            out.append(sphere)

    if grasp_3d.entry_start is not None and grasp_3d.orientation is not None:
        # 三轴架
        R = np.asarray(grasp_3d.orientation)
        origin = np.asarray(grasp_3d.entry_start, dtype=float)
        colors = [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
        for ax_i, col in enumerate(colors):
            axis_m = _mk(4 + ax_i, Marker.ARROW)
            axis_m.scale.x = 0.005
            axis_m.scale.y = 0.01
            axis_m.scale.z = 0.01
            axis_m.color.r = float(col[0])
            axis_m.color.g = float(col[1])
            axis_m.color.b = float(col[2])
            axis_m.color.a = float(col[3])
            end = origin + 0.05 * R[:, ax_i]
            axis_m.points = [_point(origin), _point(end)]
            out.append(axis_m)

    text = _mk(10, Marker.TEXT_VIEW_FACING)
    text.scale.z = 0.03
    # 三态由 _mk 的状态色表达（ACCEPT 绿/REOBSERVE 黄/REJECT 红），文字只留 ID
    text.text = tid
    if grasp_3d.entry_start is not None:
        text.pose.position = _point(grasp_3d.entry_start)
    elif grasp_3d.bag_bottom is not None:
        text.pose.position = _point(grasp_3d.bag_bottom)
    out.append(text)
    return out


def _draw_debug(img, det, grasp_2d, sam_mask, tid='', confirmed: bool = True):
    """
    叠检测框/掩膜轮廓/底→颈箭头/剪切线/ID 置信度文字（原地改 img，三态用颜色表达）.

    Args:
        img: (H, W, 3) uint8 BGR，被原地改写.
        det: 检测 dict（bbox、class_id、conf；class 0 绿框，其他橙框）.
        grasp_2d: BagGrasp2D（提供关键点像素与状态）.
        sam_mask: (H, W) 掩膜或 None（None 时不画轮廓）.
        tid: 目标稳定 ID（target_registry 匹配结果；空串则不显示）.
        confirmed: False 时只画灰框+文字，不把突现误检画成正式目标.

    Returns
    -------
        无返回值（None）；叠加结果写回 img.

    """
    h, w = img.shape[:2]
    x1, y1, x2, y2 = clip_bbox(det['bbox'], img.shape)
    # OpenCV 矩形角点含边界；clip_bbox 的 x2/y2 可等于 w/h（切片右开）
    x1d, y1d = _clamp_px(img, x1, y1)
    x2d, y2d = _clamp_px(img, max(x1, x2 - 1), max(y1, y2 - 1))
    if not confirmed:
        cv2.rectangle(img, (x1d, y1d), (x2d, y2d), (160, 160, 160), 1)
        label = f'{tid} {det.get("conf", 0.0):.2f}'.strip()
        _draw_label(img, label, x1d, y1d, (180, 180, 180))
        return
    color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
    cv2.rectangle(img, (x1d, y1d), (x2d, y2d), color, 2)
    if sam_mask is not None:
        if sam_mask.shape[:2] != (h, w):
            sam_mask = cv2.resize(
                (sam_mask > 0).astype(np.uint8), (w, h),
                interpolation=cv2.INTER_NEAREST)
        # 只画分割轮廓线，不做半透明颜色填充——掩膜上色会盖住果实纹理，
        # 轮廓更便于观察分割边界是否贴边
        contours, _ = cv2.findContours(
            (sam_mask > 0).astype(np.uint8),
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (80, 80, 230), 2)
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
    # 稳定 ID + YOLO 检测置信度（det['conf']，与位姿管线 confidence 区分）。
    # OpenCV putText 的 y 是基线：写在框顶上方会画出图外。贴在框内左上，
    # 黑底保证绿/黄/红字在果面纹理上仍可读。
    label = f'{tid} ' if tid else ''
    label += f"{det.get('conf', 0.0):.2f}"
    _draw_label(img, label.strip(), x1d, y1d, st_color)
