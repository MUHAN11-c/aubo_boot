from __future__ import annotations
"""感知契约类型：工具几何、单帧输入/输出、纯函数与匹配/锁定事件值对象."""

from dataclasses import dataclass, field
from typing import (
    List,
    Optional,
    Tuple,
)

import numpy as np


@dataclass
class ToolGeometry:
    """
    空心圆柱工具几何参数（台架测量, 版本化）.

    所有长度单位为米。

    Fields:
        d_inner_m: 圆柱内径 — 袋子必须能通过
        insert_length_m: 最大插入深度 (从入口起点计)
        blade_offset_m: TCP 到剪切平面的轴向距离 (沿Z_tool正方向；当前为 0)
        entry_d_tool: 入口相对袋底；由 ROS 参数装载
        entry_d_s: 附加安全距离；由 ROS 参数装载
        entry_standoff: [legacy] = entry_d_tool + entry_d_s
        clearance_min: 袋体与工具内壁之间的最小径向余量
        margin_neck: 袋颈候选前方的安全停止距离
        version: 此工具配置的语义版本号
    """

    d_inner_m: float = 0.104          # 104mm 内径
    insert_length_m: float = 0.200         # 200mm 最大插入
    blade_offset_m: float = 0.0            # TCP 与剪切平面重合
    entry_d_tool: float = 0.0
    entry_d_s: float = 0.0
    entry_standoff: float = 0.0   # = d_tool + d_s
    clearance_min: float = 0.005    # 5mm 最小径向余量
    margin_neck: float = 0.015      # 袋颈前 15mm 安全距离
    version: str = '1.1'


# ═══════════════════════════════════════════════════════════════
# 全局工具实例 (台架测量, 版本化)
# ═══════════════════════════════════════════════════════════════

TOOL_GEOMETRY = ToolGeometry(
    d_inner_m=0.104,          # 104mm 内径
    insert_length_m=0.200,         # 200mm 最大插入
    blade_offset_m=0.0,            # TCP 与剪切平面重合
    entry_d_tool=0.0,
    entry_d_s=0.0,
    entry_standoff=0.0,   # = d_tool + d_s
    clearance_min=0.005,    # 5mm 最小径向余量
    margin_neck=0.015,      # 袋颈前 15mm
    version='1.1',
)


# ═══════════════════════════════════════════════════════════════
# 输入
# ═══════════════════════════════════════════════════════════════

@dataclass
class BagObservation:
    """单帧感知输入：对齐的 RGB-D + YOLO 检测列表."""

    rgb: np.ndarray                        # (H, W, 3) BGR（OpenCV 惯例）
    depth: np.ndarray                      # (H, W) uint16，单位 mm，与 RGB 对齐
    camera_K: dict                         # {"fx","fy","cx","cy","width","height"}
    frame_id: str = 'camera_depth_optical_frame'  # 相机光学系 frame_id
    gravity_hint: Optional[np.ndarray] = None  # (3,) 相机系重力方向；IMU 不可用时 None
    # [{"bbox","class_id","conf"}]，bbox 常为 xyxy
    detections: List[dict] = field(default_factory=list)
    metadata: dict = field(default_factory=dict)  # 版本追溯（model/calibration_version）


# ═══════════════════════════════════════════════════════════════
# 2D 输出
# ═══════════════════════════════════════════════════════════════

@dataclass
class BagGrasp2D:
    """2D 视觉参考 (像素坐标)."""

    detection_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # x, y, w, h
    foreground_mask: Optional[np.ndarray] = None    # bbox深度前景伪mask
    bottom_px: Optional[Tuple[float, float]] = None  # 袋底像素 (u, v)
    neck_px: Optional[Tuple[float, float]] = None   # 袋颈像素 (u, v)
    grasp_px: Optional[Tuple[float, float]] = None  # 抓取参考点像素 (u, v)
    bag_axis_line: Optional[Tuple] = None           # [bottom_px, neck_px]
    travel_line: Optional[Tuple] = None             # [grasp_px, travel_end_px]
    confidence: float = 0.0                         # [0, 1]，越高越可信
    status: str = 'REJECT'                          # ACCEPT|REOBSERVE|REJECT
    diagnostic_flags: List[str] = field(default_factory=list)  # 门控诊断标记


# ═══════════════════════════════════════════════════════════════
# 3D 输出
# ═══════════════════════════════════════════════════════════════

@dataclass
class BagGraspReference3D:
    """3D 抓取参考位姿 (相机坐标系, 米)."""

    frame_id: str = 'camera_depth_optical_frame'  # 坐标系（默认相机光学系）
    entry_start: Optional[np.ndarray] = None  # P_entry_start (3,) — 圆柱顶面圆心 = 末端TCP, 位于袋底外侧
    position: Optional[np.ndarray] = None  # P_grasp (3,) — [legacy] 保留兼容, 新代码优先用 entry_start
    points_centroid: Optional[np.ndarray] = None  # 检测框前景点云中位质心 (3,) — 身份锚点，比端点抗抖
    orientation: Optional[np.ndarray] = None          # R = [Xg, Yg, Zg] (3×3)
    bag_bottom: Optional[np.ndarray] = None           # P_bottom (3,)
    bag_neck: Optional[np.ndarray] = None             # P_neck (3,)
    translation_direction: Optional[np.ndarray] = None  # +Zg = bag bottom → bag neck = 圆柱轴线
    bag_diameter_upper_m: float = 0.0                # 保守袋体直径上界 (m)
    suggested_travel_m: float = 0.0                   # 视觉建议行程 (圆柱长度)
    suggested_travel_end: Optional[np.ndarray] = None    # P_entry_start + travel × Zg
    position_covariance: Optional[np.ndarray] = None     # (3×3)
    direction_covariance: Optional[np.ndarray] = None    # (3×3)
    confidence: float = 0.0                             # [0, 1]
    status: str = 'REJECT'                            # ACCEPT|REOBSERVE|REJECT
    diagnostic_flags: List[str] = field(default_factory=list)  # 门控诊断标记
    diagnostic_info: dict = field(default_factory=dict)  # 诊断详情
    strategy_id: str = ''                             # 策略标识（管线:前景模式）
    model_version: str = ''                           # 模型版本标识
    calibration_version: str = ''                     # 内外参版本标识
    tool_version: str = ''                            # 工具几何版本


# ═══════════════════════════════════════════════════════════════
# 圆柱套入位姿计算 (纯函数, 工具物理约束)
# ═══════════════════════════════════════════════════════════════

def compute_entry_start(P_bottom: np.ndarray, Z_tool: np.ndarray,
                        entry_standoff: float) -> np.ndarray:
    """
    计算圆柱入口起点 = 圆柱顶面圆心 = 末端TCP.

    P_entry_start = P_bottom - entry_standoff × Z_tool

    后撤量由调用方传入（节点从 ROS 参数读）。0 时入口与袋底重合。

    Args:
        P_bottom: (3,) 袋底3D位置（米，相机光学系）.
        Z_tool: (3,) 归一化的工具轴方向 (袋底→袋颈).
        entry_standoff: 袋底外侧后撤 (m)，= entry_d_tool + entry_d_s.

    Returns
    -------
        P_entry_start: (3,) 圆柱入口起点（米）.

    """
    return P_bottom - entry_standoff * Z_tool


def compute_travel_range(P_entry_start: np.ndarray, P_neck: np.ndarray,
                         Z_tool: np.ndarray, tool: 'ToolGeometry') -> Tuple[float, float]:
    """
    基于工具几何参数计算建议行程区间.

    s_neck = dot(P_neck - P_entry_start, Z_tool) - tool.blade_offset_m

    行程受 insert_length_m 上限约束, 并在袋颈前方保留 margin_neck 安全距离。

    Args:
        P_entry_start: (3,) 入口起点（米）.
        P_neck: (3,) 袋颈候选位置（米）.
        Z_tool: (3,) 归一化的工具轴方向.
        tool: ToolGeometry 实例（读 blade_offset_m / margin_neck / insert_length_m）.

    Returns
    -------
        (travel_min, travel_max): 建议行程区间 (m)；travel_min 为 0.8 倍
        安全行程的保守下限，travel_max 受 insert_length_m 封顶.

    """
    s_neck = float(np.dot(P_neck - P_entry_start, Z_tool) - tool.blade_offset_m)
    s_safe = max(0.0, s_neck - tool.margin_neck)
    s_min = max(0.0, s_safe * 0.8)   # 保守下限
    s_max = min(s_safe, tool.insert_length_m)  # 上限受工具长度约束
    return (s_min, s_max)


@dataclass(frozen=True)
class MatchResult:
    """
    SpatialEmaMatcher 匹配结果（不可变值对象）.

    target_id 为命中的历史表项 ID；None 表示未命中（调用方发新 ID）。
    distance 为命中距离（米），未命中时为查询半径（诊断用）。
    """

    target_id: Optional[str]
    distance: float
    status: str = 'ok'


@dataclass(frozen=True)
class LockEvent:
    """
    CollectLockPolicy 窗口关闭事件（不可变值对象）.

    records 为收齐窗口关闭时累积的确认记录快照（每 target_id 取窗口内
    最新一帧，顺序为首次入集序）；排序/截断/锁定记账由调用方
    （GlobalHarvestPlan）完成，策略本身不持有锁定后状态。
    """

    records: Tuple[dict, ...]
