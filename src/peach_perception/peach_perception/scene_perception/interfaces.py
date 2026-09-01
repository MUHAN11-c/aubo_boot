from __future__ import annotations
"""感知缝位 ABC、契约类型与注册表。实现注册在 pipeline / identity 末尾。"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
    TYPE_CHECKING,
)

import numpy as np
from peach_perception.common.runtime import Registry


# === contracts.py ===

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
        L_blade: TCP 到剪切平面的轴向距离 (沿Z_tool正方向；当前为 0)
        entry_d_tool: 入口相对袋底；由 ROS 参数装载
        entry_d_s: 附加安全距离；由 ROS 参数装载
        entry_standoff: [legacy] = entry_d_tool + entry_d_s
        clearance_min: 袋体与工具内壁之间的最小径向余量
        margin_neck: 袋颈候选前方的安全停止距离
        version: 此工具配置的语义版本号
    """

    D_inner: float = 0.104          # 104mm 内径
    L_insert: float = 0.200         # 200mm 最大插入
    L_blade: float = 0.0            # TCP 与剪切平面重合
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
    D_inner=0.104,          # 104mm 内径
    L_insert=0.200,         # 200mm 最大插入
    L_blade=0.0,            # TCP 与剪切平面重合
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


# === interfaces.py ===

if TYPE_CHECKING:
    # 仅类型标注用，运行期不 import（避免 interfaces ↔ pipeline 循环导入）
    from .pipeline import TargetPoseResult

# 实现注册表（2.14）：按名登记/创建，默认实现在 pipeline.py / identity.py 末尾
DETECTORS: Registry['Detector'] = Registry('检测器')
SEGMENTERS: Registry['Segmenter'] = Registry('分割器')
POSE_PIPELINES: Registry['PosePipeline'] = Registry('位姿管线')
MATCHERS: Registry['TargetMatcher'] = Registry('目标匹配器')
LOCK_POLICIES: Registry['LockPolicy'] = Registry('锁定策略')


@dataclass(frozen=True)
class MatchResult:
    """
    TargetMatcher 匹配结果（不可变值对象）.

    target_id 为命中的历史表项 ID；None 表示未命中（调用方发新 ID）。
    distance 为命中距离（米），未命中时为查询半径（诊断用）。
    """

    target_id: Optional[str]
    distance: float
    status: str = 'ok'


@dataclass(frozen=True)
class LockEvent:
    """
    LockPolicy 窗口关闭事件（不可变值对象）.

    records 为收齐窗口关闭时累积的确认记录快照（每 target_id 取窗口内
    最新一帧，顺序为首次入集序）；排序/截断/锁定记账由调用方
    （GlobalHarvestPlan）完成，策略本身不持有锁定后状态。
    """

    records: Tuple[dict, ...]


class Detector(ABC):
    """目标检测器接口：RGB 进 → 检测 dict 列表出（无副作用的纯推理面）."""

    @abstractmethod
    def detect(self, rgb: np.ndarray) -> List[dict]:
        """
        对 (H, W, 3) BGR 图跑检测，返回 [{'bbox','class_id','conf',...}].

        Args:
            rgb: (H, W, 3) BGR 图像.

        Returns
        -------
            检测 dict 列表（字段约定见 UltralyticsYolo.detect docstring）.

        """


class Segmenter(ABC):
    """实例分割器接口：RGB + prompt 框 → 二值掩膜列表（批量签名）."""

    @abstractmethod
    def segment(
        self,
        rgb: np.ndarray,
        bboxes: List[Tuple[int, int, int, int]],
    ) -> List[Tuple[np.ndarray, Tuple[int, int, int, int]]]:
        """
        对 (H, W, 3) BGR 图按 prompt 框分割，返回 [(binary_mask, bbox)].

        Args:
            rgb: (H, W, 3) BGR 图像.
            bboxes: [(x1, y1, x2, y2), ...] prompt 框（像素）.

        Returns
        -------
            [(掩膜, 框)] 列表；掩膜按 bbox 与 prompt 框对应（过小掩膜被
            丢弃，返回项与 prompt 非一一对齐）.

        """


class PosePipeline(ABC):
    """
    单目标位姿管线接口：观测 + 前景掩膜 → 显式安全状态的位姿结果.

    类属性 ``kind`` 为管线种类键（'bag' / 'fruit'），candidates 类别路由
    与 strategy_id 拼写以它为准。实例属性契约（调用端 candidates 构造
    前景掩膜时读取）：``tool``（ToolGeometry，版本追溯与净空门控）、
    ``min_depth_m`` / ``max_depth_m``（有效深度区间，掩膜构造共用）。
    """

    kind: str = ''
    tool: object = None
    min_depth_m: float = 0.3
    max_depth_m: float = 2.5

    @abstractmethod
    def estimate(self, obs: BagObservation, target_id: str, bbox: tuple,
                 mask: Optional[np.ndarray] = None,
                 mask_source: str = 'depth_fallback') -> 'TargetPoseResult':
        """
        估计 bbox 内单个目标的抓取位姿与三态安全门控.

        Args:
            obs: 单帧输入（深度 uint16 毫米）.
            target_id: 目标 ID.
            bbox: (x1, y1, x2, y2) 检测框（像素）.
            mask: 外部前景掩膜或 None.
            mask_source: 掩膜来源标签（写入结果追溯）.

        Returns
        -------
            TargetPoseResult（grasp_2d/grasp_3d + status ∈ 三态）.

        """


class TargetMatcher(ABC):
    """
    世界系目标匹配器接口：候选锚点 + 目标表 → 命中结果（不持表）.

    表（table）由调用方 TargetRegistry 持有，匹配器只读：键为
    target_id，值为表项 dict（至少含 'class_id'、'position' (3,)、
    'confirmed' 三个键，见 target_registry.py 表项模式）。
    """

    @abstractmethod
    def match(self, anchor: np.ndarray, class_id: int,
              table: Dict[str, dict], frame_used: set) -> MatchResult:
        """
        在表中为一个世界系候选找匹配表项（含恢复段策略）.

        Args:
            anchor: (3,) 世界系位置（米），候选目标的空间锚点.
            class_id: 候选类别.
            table: target_id → 表项 dict（只读）.
            frame_used: 本帧已命中的 target_id 集合（同帧去重，跳过）.

        Returns
        -------
            MatchResult：target_id 为 None 表示未命中（调用方发新 ID）.

        """


class LockPolicy(ABC):
    """
    收齐窗口锁定策略接口：逐帧记录进 → 窗口关闭事件出（不持锁定后状态）.

    实例属性契约（调用端节点帧率自适应时读写）：
    ``min_collect_frames`` / ``lock_settle_frames``（静止判定参数，只读
    使用）、``max_collect_s``（超时上限，节点按实测帧率 EMA 自适应改写）、
    ``accumulated_count``（收齐窗口累积的已确认目标数，只读；缺陷 R-D8
    发现进度摘要用，窗口未累积任何确认目标时为 0）。
    """

    min_collect_frames: int = 10
    lock_settle_frames: int = 5
    max_collect_s: float = 25.0
    accumulated_count: int = 0

    @abstractmethod
    def update(self, records: List[dict], now: float) -> Optional[LockEvent]:
        """
        输入当前帧记录；窗口关闭时返回 LockEvent，否则 None.

        Args:
            records: 本帧候选 record dict 列表（须带 target_id；含
                confirmed/status/距离等键）.
            now: 当前时刻 (s)，由调用方注入（协议 I3：实现内禁止自行取
                时钟）.

        Returns
        -------
            LockEvent（窗口关闭，含累积确认记录快照）或 None.

        """

    @abstractmethod
    def reset(self) -> None:
        """清空窗口状态，允许下一轮全局观测重新收齐."""
