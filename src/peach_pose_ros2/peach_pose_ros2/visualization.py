"""
可视化 — RViz Marker 组装与 debug 叠加图绘制.

职责:
  把 pipeline.TargetPoseResult 画出来：`_to_markers` 生成 RViz Marker
  （依赖 visualization_msgs），`_draw_debug` 在 BGR 图上叠加检测框 /
  掩膜轮廓 / 关键点 / 剪切线 / 稳定 ID 与置信度文字（颜色表三态，依赖 cv2）。
  依赖 ROS 消息类型与 conversions/grasp_tf 的转换函数，不依赖 rclpy 节点。

坐标系/单位约定:
  Marker 坐标系随 header.frame_id（与候选消息一致，米制；每个目标占用
  id 段 ``idx*20 .. idx*20+19`` 防多目标冲突）；debug 图为图像平面像素
  坐标（BGR，原地改写）。
"""
from __future__ import annotations

from typing import List

import cv2
import numpy as np
from peach_pose_ros2.conversions import _metric, _point
from peach_pose_ros2.grasp_tf import _rotation_to_quat
from visualization_msgs.msg import Marker


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
    g3d = result.grasp_3d
    out: List[Marker] = []
    r, g, b, a = _status_color(g3d.status)
    base_id = idx * 20

    def _mk(mid, mtype) -> Marker:
        m = Marker()
        m.header = header
        m.ns = 'peach_pose'
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

    if g3d.bag_bottom is not None and g3d.bag_neck is not None:
        axis = _mk(0, Marker.LINE_LIST)
        axis.scale.x = 0.004
        axis.points = [_point(g3d.bag_bottom), _point(g3d.bag_neck)]
        out.append(axis)

    if (g3d.entry_start is not None and g3d.suggested_travel_end is not None):
        arrow = _mk(1, Marker.ARROW)
        arrow.scale.x = 0.008
        arrow.scale.y = 0.015
        arrow.scale.z = 0.015
        arrow.points = [_point(g3d.entry_start), _point(g3d.suggested_travel_end)]
        out.append(arrow)

    if (g3d.entry_start is not None and g3d.translation_direction is not None
            and g3d.suggested_travel_m > 0):
        cyl = _mk(2, Marker.CYLINDER)
        mid = g3d.entry_start + 0.5 * g3d.suggested_travel_m * g3d.translation_direction
        cyl.pose.position = _point(mid)
        if g3d.orientation is not None:
            # Marker CYLINDER 默认轴为 Z；抓取架 Zg = translation_direction
            cyl.pose.orientation = _rotation_to_quat(g3d.orientation)
        diam = float(tool_d_inner)
        cyl.scale.x = diam
        cyl.scale.y = diam
        cyl.scale.z = float(g3d.suggested_travel_m)
        cyl.color.a = 0.25
        out.append(cyl)

    if (result.target_kind == 'fruit' and g3d.bag_bottom is not None
            and g3d.bag_neck is not None):
        # 球心近似为底/颈中点
        sphere = _mk(3, Marker.SPHERE)
        center = 0.5 * (np.asarray(g3d.bag_bottom) + np.asarray(g3d.bag_neck))
        radius = _metric(result.metrics or {}, 'fruit_radius_m', 0.0)
        if radius > 0:
            sphere.pose.position = _point(center)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = float(2.0 * radius)
            sphere.color.a = 0.3
            out.append(sphere)

    if g3d.entry_start is not None and g3d.orientation is not None:
        # 三轴架
        R = np.asarray(g3d.orientation)
        origin = np.asarray(g3d.entry_start, dtype=float)
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
    if g3d.entry_start is not None:
        text.pose.position = _point(g3d.entry_start)
    elif g3d.bag_bottom is not None:
        text.pose.position = _point(g3d.bag_bottom)
    out.append(text)
    return out


def _draw_debug(img, det, g2d, sam_mask, tid=''):
    """
    叠检测框/掩膜轮廓/底→颈箭头/剪切线/ID 置信度文字（原地改 img，三态用颜色表达）.

    Args:
        img: (H, W, 3) uint8 BGR，被原地改写.
        det: 检测 dict（bbox、class_id、conf；class 0 绿框，其他橙框）.
        g2d: BagGrasp2D（提供关键点像素与状态）.
        sam_mask: (H, W) 掩膜或 None（None 时不画轮廓）.
        tid: 目标稳定 ID（target_registry 匹配结果；空串则不显示）.

    Returns
    -------
        无返回值（None）；叠加结果写回 img.

    """
    x1, y1, x2, y2 = map(int, det['bbox'])
    color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    if sam_mask is not None:
        # 只画分割轮廓线，不做半透明颜色填充——掩膜上色会盖住果实纹理，
        # 轮廓更便于观察分割边界是否贴边
        contours, _ = cv2.findContours(
            (sam_mask > 0).astype(np.uint8),
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (80, 80, 230), 2)
    status = g2d.status
    st_color = {
        'ACCEPT': (0, 220, 0), 'REOBSERVE': (0, 200, 255), 'REJECT': (0, 0, 220)
    }.get(status, (180, 180, 180))
    if g2d.bottom_px and g2d.neck_px:
        cv2.arrowedLine(
            img,
            (int(g2d.bottom_px[0]), int(g2d.bottom_px[1])),
            (int(g2d.neck_px[0]), int(g2d.neck_px[1])),
            (255, 255, 0), 2, tipLength=0.15)
    if g2d.grasp_px:
        cv2.circle(img, (int(g2d.grasp_px[0]), int(g2d.grasp_px[1])), 5, st_color, -1)
    # 剪切线：行程终点（果柄处）垂直于袋轴方向的线段，表示刃口切割方向——
    # 套入沿轴推进，剪切动作与推进方向垂直。取数与 _to_candidate_2d 同源
    # （g2d.travel_line = [grasp_px, travel_end_px]，方向即袋轴投影）；
    # 两端点任一 None（投影失败）或行程退化（零长度无法定方向）则跳过；
    # 半长取检测框宽 1/4（近似刀具刃口尺度）
    if (g2d.travel_line and len(g2d.travel_line) >= 2
            and g2d.travel_line[0] is not None
            and g2d.travel_line[1] is not None):
        gx, gy = g2d.travel_line[0]
        ex, ey = g2d.travel_line[1]
        dx, dy = ex - gx, ey - gy
        norm = float((dx * dx + dy * dy) ** 0.5)
        if norm > 1e-6:
            px, py = -dy / norm, dx / norm      # 袋轴投影的图像平面法向
            half = max(16, (x2 - x1) // 4)
            cv2.line(img,
                     (int(ex - px * half), int(ey - py * half)),
                     (int(ex + px * half), int(ey + py * half)),
                     (255, 0, 255), 2)
    # 稳定 ID + YOLO 检测置信度（det['conf']，与位姿管线 confidence 区分）；
    # 三态不写文字，直接用 st_color 文字颜色表达（ACCEPT 绿/REOBSERVE 黄/
    # REJECT 红）；放在检测框左上角上方，与 3D Marker 文字（entry_start/
    # 袋底处）互补，不遮挡掩膜轮廓
    label = f'{tid} ' if tid else ''
    label += f"{det.get('conf', 0.0):.2f}"
    cv2.putText(img, label, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, st_color, 2)
