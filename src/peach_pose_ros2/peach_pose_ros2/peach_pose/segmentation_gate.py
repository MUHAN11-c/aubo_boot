"""
锁定后 selected-only 分割门控（阶段 H，协议 2.13-E1；纯 Python + numpy，零 ROS）.

职责:
  目标集合锁定后，MobileSAM 只对**锁定集内目标**的检测框推理——SAM 批量
  耗时随 prompt 框数近似线性增长，锁定后帧内其余检测框（新出现/未入集
  目标）继续走 YOLO 检测与几何管线（掩膜缺失显式走深度带降级），但不再
  消耗 SAM 推理；segment_ms 分项 EMA（harvest_state.timing）随之真实回落，
  帧率随目标数线性回升（2.13-E1 验收依据）。

识别问题与选型依据:
  SAM 在管线时序上先于「几何 → 世界系身份匹配」，分割前拿不到本帧检测框
  的 target_id。两种预识别方案：
    a) 与上一帧检测框做 IoU 关联——相机随臂运动时框位移大，靠近/转移段
       会系统性失配；
    b) 把锁定目标的世界系记忆锚点（TargetRegistry 表项 position，经 TF
       反投影到本帧像素）落进哪个检测框，就分割哪个——TF 精确吸收相机
       运动，锚点是点云质心、必落在正确框内。
  本模块实现 b)：``project_positions_to_pixels`` 锚点反投影 +
  ``plan_segmentation_bboxes`` 门控策略两个纯函数，节点负责从
  harvest_plan / target_registry 取锁定 ID 集与锚点并注入 TF/内参。

降级语义（不省但不错）:
  - 未锁定 / 开关关闭 / 锚点不可投影（TF 不可用帧）→ 全量框照常（旧行为）；
  - 锁定但锚点全部不可见（相机背对/出视野）→ 空集，本帧不跑 SAM；
  - 锚点落进多个重叠框或一个框含多个锚点：按「框内含任一锚点即选中」，
    宁多勿漏（漏分割的锁定目标会被判 OCCLUDED，代价高于多跑一个框）。

时钟约定（协议 I3）：本模块不取时钟，无状态。
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

# 检测框外扩比例（每边各扩 10% 宽高）：容忍锚点投影与 YOLO 框边的贴边
# 误差（质心投影理论上在框内，外扩只为深度噪声/框回归抖动兜底）
DEFAULT_MARGIN_FRAC = 0.1


def project_positions_to_pixels(
        positions: Dict[str, np.ndarray],
        T_cam_world: np.ndarray,
        camera_K: dict) -> Dict[str, Tuple[float, float]]:
    """
    世界系锚点集 → 本帧像素坐标（pinhole 投影）.

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
    fx = float(camera_K['fx'])
    fy = float(camera_K['fy'])
    cx = float(camera_K['cx'])
    cy = float(camera_K['cy'])
    width = camera_K.get('width')
    height = camera_K.get('height')
    out: Dict[str, Tuple[float, float]] = {}
    for target_id, pos in positions.items():
        p = np.asarray(pos, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            continue
        pc = T[:3, :3] @ p + T[:3, 3]
        if not np.all(np.isfinite(pc)) or pc[2] <= 1e-8:
            continue
        u = fx * pc[0] / pc[2] + cx
        v = fy * pc[1] / pc[2] + cy
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
