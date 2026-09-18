"""
ScenePerceptionParams: yaml 直读 + tool / gravity 派生.

部署事实源 ``config/scene_perception.yaml``。主节点一行
``ScenePerceptionParams.attach(node)``：声明叶子并挂规则校验（越界值启动
期拒绝、空模型路径拒绝启动），``ros2 param set`` 原地刷新。逐帧读取的键
热生效；YOLO/SAM/管线等构造期捕获键改后须重启。派生失败（gravity 非法 /
min_depth ≥ max_depth）整批拒绝，不改当前一致快照。本模块顶层零 ROS 导入。
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from peach_harvester.vision.param_rules import check, check_min_max
from peach_harvester.vision.scene_perception.contracts import ToolGeometry


_RULES = {  # 键 -> 校验规则表（启动期非法即拒启；运行期非法 set 即拒）
    'yolo_model_path': (('nonempty',),),
    'sam_model_path': (('nonempty',),),
    'tf_timeout_sec': (('gt_eq', 0.0),),
    'depth_scale_unit': (('gt', 0.0),),
    'sync_slop_s': (('gt_eq', 0.0),),
    'sam_max_bboxes': (('gt_eq', 1),),
    'sam_min_area': (('gt_eq', 0),),
    'min_mask_points': (('gt_eq', 1),),
    'detection_cloud_stride': (('gt_eq', 1),),
    'pipeline.min_depth_m': (('gt', 0.0),),
    'pipeline.max_depth_m': (('gt', 0.0),),
    'pipeline.min_points': (('gt_eq', 1),),
    'tool.D_inner': (('gt', 0.0),),
    'tool.L_insert': (('gt', 0.0),),
    'tool.entry_d_tool': (('gt_eq', 0.0),),
    'tool.entry_d_s': (('gt_eq', 0.0),),
    'tool.clearance_min': (('gt', 0.0),),
    'tool.margin_neck': (('gt_eq', 0.0),),
    'target_memory.match_radius_m': (('gt', 0.0),),
    'target_memory.max_targets': (('gt_eq', 1),),
    'target_memory.position_ema': (('gt', 0.0), ('lt_eq', 1.0),),
    'target_memory.recovery_scale': (('gt_eq', 1.0),),
    'target_memory.confirm_frames': (('gt_eq', 1),),
    'target_memory.tentative_ttl_frames': (('gt_eq', 1),),
    'target_memory.anchor_max_age_s': (('gt', 0.0),),
    'target_memory.anchor_drop_s': (('gt', 0.0),),
    'target_memory.max_age_s': (('gt', 0.0),),
    'wind.swing_threshold_m': (('gt', 0.0),),
    'wind.swing_frames': (('gt_eq', 1),),
    'lighting.min_depth_ratio': (('bounds', 0.0, 1.0),),
    'lighting.min_conf_mean': (('bounds', 0.0, 1.0),),
    'lighting.bad_frames': (('gt_eq', 1),),
    'harvest.min_collect_frames': (('gt_eq', 1),),
    'harvest.lock_settle_frames': (('gt_eq', 1),),
    'harvest.max_collect_s': (('gt', 0.0),),
}


def _validate(name, value):
    """逐条规则校验；返回拒绝理由或 None（空模型路径一并拒绝）."""
    for rule in _RULES.get(name, ()):
        why = check(rule, value, name)
        if why:
            return why
    return None


class _PipelineNs:
    """scene_perception.yaml ``pipeline.*``（仅注解，运行时仍是 yaml 命名空间）."""

    bag_impl: str
    """PIPELINES_BY_IMPL 映射名；默认 robust_bag."""
    fruit_impl: str
    """裸果线映射名；from_params(enable_fruit=False) 时不构造."""
    min_depth_m: float
    """有效深度下限 [m]."""
    max_depth_m: float
    """有效深度上限 [m]."""
    min_points: int
    """前景点数下限，不足 REJECT."""
    locked_only_segmentation: bool
    """锁定后 SAM 只对锁定集反投影命中的框推理."""


class _TargetMemoryNs:
    """``target_memory.*`` 身份表."""

    enable: bool
    """False 时不用 TargetRegistry，target_id 用帧内序号."""
    match_radius_m: float
    """世界系匹配半径 [m]."""
    max_targets: int
    """表容量."""
    position_ema: float
    """位置/轴/直径 EMA α∈(0,1]."""
    recovery_scale: float
    """马氏 Σ 膨胀倍率 ≥1."""
    confirm_frames: int
    """转正确认帧数."""
    tentative_ttl_frames: int
    """未确认 TTL [帧]."""
    anchor_max_age_s: float
    """锁定集 LOST 打 stale 的墙钟上限 [s]."""
    anchor_drop_s: float
    """锁定集 LOST 移除上限 [s]."""
    max_age_s: float
    """表项墙钟龄上限 [s]."""


class _HarvestNs:
    """``harvest.*`` 收齐窗口."""

    min_collect_frames: int
    """关窗最少帧数."""
    lock_settle_frames: int
    """静止判定帧数."""
    max_collect_s: float
    """窗口最长时长配置基准 [s]."""
    priority_prefer_lower_first: bool
    """True=先低后高."""


class _LightingNs:
    """``lighting.*`` 光照质量（观测指标，不阻断）."""

    min_depth_ratio: float
    """掩膜有效深度占比 EMA 下限."""
    min_conf_mean: float
    """检测置信度均值 EMA 下限."""
    bad_frames: int
    """连续低质帧数才置位."""


class _WindNs:
    """``wind.*`` 摆动判定."""

    swing_threshold_m: float
    """观测残差阈值 [m]."""
    swing_frames: int
    """对称连击帧数."""


class ScenePerceptionParams:
    """config/scene_perception.yaml 快照 + ToolGeometry / gravity 派生."""

    tool: ToolGeometry
    """由 yaml tool.* 派生的空心圆柱几何 [m]."""
    gravity_hint: Optional[np.ndarray]
    """相机系重力 (3,)；空串 yaml 则为 None."""
    pipeline: _PipelineNs
    """位姿管线嵌套组."""
    target_memory: _TargetMemoryNs
    """身份记忆嵌套组."""
    harvest: _HarvestNs
    """收齐窗口嵌套组."""
    lighting: _LightingNs
    """光照质量嵌套组."""
    wind: _WindNs
    """摆动判定嵌套组."""
    yolo_model_path: str
    """Ultralytics 权重路径；空串拒启；构造期捕获."""
    sam_model_path: str
    """MobileSAM 权重路径；空串拒启；构造期捕获."""
    yolo_conf: float
    """YOLO 第一级置信度阈值."""
    yolo_nms_iou: float
    """YOLO NMS IoU."""
    min_detection_conf: float
    """进入几何管线的第二级置信度下限."""
    depth_scale_unit: float
    """uint16 深度 × 本值 = 毫米（Percipio 常见 0.25）."""
    sync_slop_s: float
    """RGB-D ApproximateTime 允差 [s]."""
    tf_timeout_sec: float
    """精确 stamp TF 查询超时 [s]."""
    camera_optical_frame: str
    """光学系 frame_id；空串则用深度图 header."""
    output_frame: str
    """几何输出系；空串=保持相机系."""
    gravity_mode: str
    """'tf'=由 output←camera 反推；'fixed'=只用 gravity_hint."""
    publish_debug_image: bool
    """是否发 /peach/perception/debug_image."""
    publish_masks: bool
    """是否发 SAM 掩膜图."""
    publish_detection_cloud: bool
    """是否发检测框点云."""
    detection_cloud_stride: int
    """点云降采样步长."""
    sam_max_bboxes: int
    """单次 SAM 最大 prompt 框数."""
    sam_min_area: int
    """SAM 掩膜最小像素."""
    min_mask_points: int
    """收敛前景最少像素，不足 mask_unavailable."""
    detection_dedup_ios: float
    """重叠去重 IoS；≥1 关闭."""
    detection_dedup_area_ratio: float
    """碎片残枝面积比；≤0 关闭."""
    model_version: str
    """随结果发布的模型版本标识."""
    calibration_version: str
    """内外参版本标识."""
    gravity_hint_xyz: str
    """逗号分隔 x,y,z；空=算法默认相机系 +Y."""
    color_topic: str
    """彩色图话题（仅 yaml/launch 接线，代码不按参数改名）."""
    depth_topic: str
    """深度图话题."""
    camera_info_topic: str
    """彩色内参话题."""

    def __init__(self, raw, tool: ToolGeometry, gravity_hint, gravity_mode: str,
                 node=None):
        """Hold the yaml namespace and derived fields."""
        object.__setattr__(self, '_raw', raw)
        self.tool = tool
        self.gravity_hint = gravity_hint
        object.__setattr__(self, '_gravity_mode', gravity_mode)
        object.__setattr__(self, '_node', node)
        object.__setattr__(self, '_ready', False)

    def __getattr__(self, name):
        """Forward undeclared names to the yaml namespace."""
        if name == 'gravity_mode':
            return self._gravity_mode
        return getattr(self._raw, name)

    @classmethod
    def attach(cls, node) -> 'ScenePerceptionParams':
        """Declare yaml leaves, validate, and hook runtime refresh."""
        from peach_harvester.yaml_params import attach, package_yaml

        holder = []

        def _commit(_ns):
            if holder:
                holder[0]._refresh()

        raw = attach(
            node,
            package_yaml('peach_harvester', 'scene_perception.yaml'),
            validate=_validate,
            preview=cls._derive,
            on_commit=_commit)
        self = cls._build(raw, node=node)
        object.__setattr__(self, '_ready', True)
        holder.append(self)
        return self

    @classmethod
    def from_params(cls, snapshot) -> 'ScenePerceptionParams':
        """Pure-core constructor for tests (no ROS node)."""
        return cls._build(snapshot, node=None)

    @classmethod
    def _derive(cls, snapshot):
        """Parse gravity and assemble ToolGeometry; raise on illegal combos."""
        hint_text = str(snapshot.gravity_hint_xyz).strip()
        if hint_text:
            parts = [float(item) for item in hint_text.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    'gravity_hint_xyz needs 3 comma-separated floats, '
                    f'got {hint_text!r}')
            gravity_hint: Optional[np.ndarray] = np.asarray(parts, dtype=float)
        else:
            gravity_hint = None
        gravity_mode = str(snapshot.gravity_mode).strip()
        if gravity_mode not in ('fixed', 'tf'):
            gravity_mode = 'fixed'
        why = check_min_max(
            snapshot.pipeline.min_depth_m, snapshot.pipeline.max_depth_m,
            'pipeline.min_depth_m', 'pipeline.max_depth_m')
        if why:
            raise ValueError(why)
        tool = ToolGeometry(
            d_inner_m=float(snapshot.tool.D_inner),
            insert_length_m=float(snapshot.tool.L_insert),
            blade_offset_m=float(snapshot.tool.L_blade),
            entry_d_tool=float(snapshot.tool.entry_d_tool),
            entry_d_s=float(snapshot.tool.entry_d_s),
            entry_standoff=float(snapshot.tool.entry_d_tool)
            + float(snapshot.tool.entry_d_s),
            clearance_min=float(snapshot.tool.clearance_min),
            margin_neck=float(snapshot.tool.margin_neck),
            version=str(snapshot.tool.version),
        )
        return tool, gravity_hint, gravity_mode

    @classmethod
    def _build(cls, snapshot, node) -> 'ScenePerceptionParams':
        """Snapshot → ScenePerceptionParams."""
        tool, gravity_hint, gravity_mode = cls._derive(snapshot)
        return cls(snapshot, tool, gravity_hint, gravity_mode, node=node)

    def _refresh(self) -> None:
        """Rebuild derived fields after a successful ros2 param set."""
        tool, gravity_hint, gravity_mode = self._derive(self._raw)
        self.tool = tool
        self.gravity_hint = gravity_hint
        object.__setattr__(self, '_gravity_mode', gravity_mode)
        if self._ready and self._node is not None:
            self._node.get_logger().info(
                '参数热更新生效（逐帧读取键；构造期捕获键需重启）')
