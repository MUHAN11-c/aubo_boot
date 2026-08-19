"""
数据合约 — 输入 / 输出 / 工具几何 / 圆柱套入计算.

管线位置:
  全管线共享的「接口层」：感知管线与 GUI 只通过本模块的 dataclass 交换数据，
  避免隐式字段约定漂移。

核心理论:
  1. 相机系 (ROS optical): Xc 图像右、Yc 图像下、Zc 相机前。
  2. 抓取系 (右手): Zg=袋底→袋颈（接近/插入轴）；Xg=⊥Zg 平面内宽度主方向；
     Yg = Zg × Xg。与 ROS2 末端常用 RGB=XYZ 着色一致（Z 蓝、X 红、Y 绿）。
  3. 三态门控: ACCEPT（可执行）/ REOBSERVE（再观测）/ REJECT（本帧不可用），
     由 pipeline.py 的安全管线判定。
  4. 工具套入: P_entry_start = P_bottom - entry_standoff * Z_tool，保证从袋外起套。

主要 API:
  ToolGeometry, TOOL_GEOMETRY, BagObservation, BagGrasp2D, BagGraspReference3D,
  compute_entry_start, compute_travel_range

注: 原 docs/architecture.md 已随旧项目移除，契约说明以本文件各 docstring 为准。
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ═══════════════════════════════════════════════════════════════
# 工具几何配置
# ═══════════════════════════════════════════════════════════════

@dataclass
class ToolGeometry:
    """
    空心圆柱工具几何参数（台架测量, 版本化）.

    所有长度单位为米。

    Fields:
        D_inner: 圆柱内径 — 袋子必须能通过
        L_insert: 最大插入深度 (从入口起点计)
        L_blade: 刀刃平面到圆柱入口平面的距离 (沿Z_tool正方向)
        entry_d_tool: 入口 standoff 的工具分量（套入余量，Gürsoy 分解之 d_tool）
        entry_d_s: 入口 standoff 的安全裕量分量（防碰撞，文献基准 30–50mm）
        entry_standoff: [legacy] 旧版单一 standoff = entry_d_tool + entry_d_s
                        P_entry_start = P_bottom - (d_tool + d_s) × Z_tool
        clearance_min: 袋体与工具内壁之间的最小径向余量
        margin_neck: 袋颈候选前方的安全停止距离
        version: 此工具配置的语义版本号
    """

    D_inner: float = 0.104          # 104mm 内径
    L_insert: float = 0.200         # 200mm 最大插入
    L_blade: float = 0.025          # 25mm 刀刃偏移
    entry_d_tool: float = 0.030     # 30mm 工具分量 standoff
    entry_d_s: float = 0.040        # 40mm 安全裕量 standoff
    entry_standoff: float = 0.070   # legacy: = d_tool + d_s
    clearance_min: float = 0.005    # 5mm 最小径向余量
    margin_neck: float = 0.015      # 袋颈前 15mm 安全距离
    version: str = '1.1'


# ═══════════════════════════════════════════════════════════════
# 全局工具实例 (台架测量, 版本化)
# ═══════════════════════════════════════════════════════════════

TOOL_GEOMETRY = ToolGeometry(
    D_inner=0.104,          # 104mm 内径
    L_insert=0.200,         # 200mm 最大插入
    L_blade=0.025,          # 25mm 刀刃偏移
    entry_d_tool=0.030,     # 30mm 工具分量
    entry_d_s=0.040,        # 40mm 安全裕量
    entry_standoff=0.070,   # = d_tool + d_s
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

    入口起点位于袋底外侧, 保证圆柱从袋子外部开始套入。

    Args:
        P_bottom: (3,) 袋底3D位置（米，相机光学系）.
        Z_tool: (3,) 归一化的工具轴方向 (袋底→袋颈).
        entry_standoff: 袋底外侧安全距离 (m)，= entry_d_tool + entry_d_s.

    Returns
    -------
        P_entry_start: (3,) 圆柱入口起点（米）.

    """
    return P_bottom - entry_standoff * Z_tool


def compute_travel_range(P_entry_start: np.ndarray, P_neck: np.ndarray,
                         Z_tool: np.ndarray, tool: 'ToolGeometry') -> Tuple[float, float]:
    """
    基于工具几何参数计算建议行程区间.

    s_neck = dot(P_neck - P_entry_start, Z_tool) - tool.L_blade

    行程受 L_insert 上限约束, 并在袋颈前方保留 margin_neck 安全距离。

    Args:
        P_entry_start: (3,) 入口起点（米）.
        P_neck: (3,) 袋颈候选位置（米）.
        Z_tool: (3,) 归一化的工具轴方向.
        tool: ToolGeometry 实例（读 L_blade / margin_neck / L_insert）.

    Returns
    -------
        (travel_min, travel_max): 建议行程区间 (m)；travel_min 为 0.8 倍
        安全行程的保守下限，travel_max 受 L_insert 封顶.

    """
    s_neck = float(np.dot(P_neck - P_entry_start, Z_tool) - tool.L_blade)
    s_safe = max(0.0, s_neck - tool.margin_neck)
    s_min = max(0.0, s_safe * 0.8)   # 保守下限
    s_max = min(s_safe, tool.L_insert)  # 上限受工具长度约束
    return (s_min, s_max)
