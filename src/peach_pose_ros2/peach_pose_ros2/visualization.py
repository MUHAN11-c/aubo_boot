"""
可视化层 — Marker / debug 叠加图 / 概率热图 / 检测框彩色点云.

全部为纯函数（显式传参，不依赖节点实例）：三态配色、RViz Marker 组装、
debug 图叠加、SAM 概率热图累积与灰度化、检测框内深度反投影点云。
"""
from typing import List, Tuple

import cv2
import numpy as np
from peach_pose_ros2.conversions import _metric, _point, _rotation_to_quat
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


def _status_color(status: str) -> Tuple[float, float, float, float]:
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


def _pack_rgb_bgr(bgr: np.ndarray) -> np.ndarray:
    """
    (N,3) uint8 BGR → (N,) float32：按位打包成 PointCloud2 的 rgb 字段.

    Args:
        bgr: (N, 3) uint8 数组，列序为 B、G、R（OpenCV 惯例）.

    Returns
    -------
        (N,) float32 视图（位内容为 0xRRGGBB，符合 PointCloud2 rgb 打包约定）.

    """
    b = bgr[:, 0].astype(np.uint32)
    g = bgr[:, 1].astype(np.uint32)
    r = bgr[:, 2].astype(np.uint32)
    packed = (r << 16) | (g << 8) | b
    return packed.view(np.float32)


def _bbox_cloud_xyzrgb(
    rgb_bgr: np.ndarray,
    depth_mm: np.ndarray,
    K: dict,
    bboxes,
    stride: int = 1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    检测框内像素反投影成彩色点云：返回 (N,3) xyz（米）与 (N,) 打包 rgb.

    depth_mm 为毫米单位 uint16（Percipio 原始值已 × depth_scale_unit）；
    剔除无效深度（0/饱和 65535），stride 为降采样步长。

    Args:
        rgb_bgr: (H, W, 3) uint8 BGR 图，与深度对齐.
        depth_mm: (H, W) uint16 深度，单位毫米.
        K: 相机内参 {"fx","fy","cx","cy"}（像素单位）.
        bboxes: 检测框列表 [(x1, y1, x2, y2)]（像素，自动裁剪到图内）.
        stride: 降采样步长（像素）；1 为不降采样.

    Returns
    -------
        (xyz, rgb_packed)：xyz 为 (N, 3) float64 相机系点（米），
        rgb_packed 为 (N,) float32 打包颜色；无有效点时均为空数组.

    """
    h, w = depth_mm.shape[:2]
    mask = np.zeros((h, w), dtype=bool)
    for bbox in bboxes:
        x1, y1, x2, y2 = [int(v) for v in bbox]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        mask[y1:y2:stride, x1:x2:stride] = True
    # 有效深度：>0 且非饱和
    valid = mask & (depth_mm > 0) & (depth_mm < 65535)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64), np.zeros((0,), dtype=np.float32)

    # 有效深度的像素行/列索引（vs=行 y，us=列 x）
    vs, us = np.where(valid)
    z = depth_mm[vs, us].astype(np.float64) / 1000.0
    fx, fy = float(K['fx']), float(K['fy'])
    cx, cy = float(K['cx']), float(K['cy'])
    x = (us.astype(np.float64) - cx) * z / fx
    y = (vs.astype(np.float64) - cy) * z / fy
    # 三个 (N,) 拼成 (N,3) 点矩阵
    xyz = np.column_stack((x, y, z))
    rgb_packed = _pack_rgb_bgr(rgb_bgr[vs, us])
    return xyz, rgb_packed


def _xyzrgb_to_cloud(header: Header, xyz: np.ndarray, rgb_f: np.ndarray) -> PointCloud2:
    """
    组装 xyz + 打包 rgb → PointCloud2 消息（x/y/z 各一个 FLOAT32 + rgb 位打包）.

    Args:
        header: 输出消息头（frame_id 决定点云坐标系解释）.
        xyz: (N, 3) 点坐标（单位随 header 坐标系，通常米）；空数组给空云.
        rgb_f: (N,) float32 打包 rgb（见 _pack_rgb_bgr）.

    Returns
    -------
        sensor_msgs/PointCloud2.

    """
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    if xyz.size == 0:
        # create_cloud：按 fields 描述把点列序列化成 PointCloud2；空点列给空云
        return pc2.create_cloud(header, fields, [])
    pts = [
        (float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2]), float(rgb_f[i]))
        for i in range(len(xyz))
    ]
    return pc2.create_cloud(header, fields, pts)


def _accumulate_heatmap(canvas: np.ndarray, probs) -> np.ndarray:
    """
    SAM 概率热图累积：逐像素取各目标概率最大值.

    Args:
        canvas: (H, W) float32 概率画布（[0,1]，逐目标累积）.
        probs: 本次 segment 的概率图列表（engine.last_probs）；None/空原样返回.

    Returns
    -------
        更新后的画布（新数组；无概率时返回原 canvas）.

    """
    if probs is not None and len(probs) > 0:
        return np.maximum(canvas, probs[0])
    return canvas


def _heatmap_to_mono8(canvas: np.ndarray) -> np.ndarray:
    """
    概率热图 [0,1] → uint8 [0,255]（亮=SAM 把握大；暗≈背景/不确定）.

    Args:
        canvas: (H, W) float32 概率画布.

    Returns
    -------
        (H, W) uint8 灰度图（mono8 发布用）.

    """
    return np.round(canvas * 255).astype(np.uint8)


def _to_markers(header, tid, idx, result, tool, logger=None) -> List[Marker]:
    """
    结果 → RViz Marker 列表：袋轴/行程箭头/刀具圆柱/果球/三轴架/状态文字.

    每个目标占用 id 段 ``idx*20 .. idx*20+19``，避免多目标冲突。

    Args:
        header: 输出头（frame_id 与候选消息一致）.
        tid: 目标 ID（用于状态文字）.
        idx: 目标序号（Marker id 段基址 = idx*20）.
        result: pipeline.TargetPoseResult.
        tool: ToolGeometry（刀具圆柱直径取 tool.D_inner）.
        logger: 四元数双实现告警出口；None 时 print.

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
            cyl.pose.orientation = _rotation_to_quat(g3d.orientation, logger)
        diam = float(tool.D_inner)
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
    text.text = f'{tid} {g3d.status}'
    if g3d.entry_start is not None:
        text.pose.position = _point(g3d.entry_start)
    elif g3d.bag_bottom is not None:
        text.pose.position = _point(g3d.bag_bottom)
    out.append(text)
    return out


def _draw_debug(img, det, g2d, sam_mask):
    """
    在 BGR 图上叠检测框、SAM 掩膜、底→颈箭头与状态文字（原地改 img）.

    Args:
        img: (H, W, 3) uint8 BGR，被原地改写.
        det: 检测 dict（bbox、class_id；class 0 绿框，其他橙框）.
        g2d: BagGrasp2D（提供关键点像素与状态）.
        sam_mask: (H, W) 掩膜或 None（None 时不叠掩膜）.

    Returns
    -------
        无返回值（None）；叠加结果写回 img.

    """
    x1, y1, x2, y2 = map(int, det['bbox'])
    color = (0, 220, 0) if det.get('class_id', 0) == 0 else (0, 180, 255)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    if sam_mask is not None:
        # 淡填充（35% 目标色）：保留果实纹理，能看清掩膜覆盖范围
        fill = np.array([60, 180, 60]) if det.get(
            'class_id', 0) == 0 else np.array([60, 150, 240])
        img[sam_mask > 0] = (
            0.65 * img[sam_mask > 0] + 0.35 * fill).astype(np.uint8)
        # 掩膜轮廓高对比描边（袋装=青、裸果=品红）：精确核对分割边缘贴合度
        contours, _ = cv2.findContours(
            (sam_mask > 0).astype(np.uint8),
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        edge_color = (255, 255, 0) if det.get(
            'class_id', 0) == 0 else (255, 0, 255)
        cv2.drawContours(img, contours, -1, edge_color, 2)
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
    cv2.putText(img, status, (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, st_color, 2)
