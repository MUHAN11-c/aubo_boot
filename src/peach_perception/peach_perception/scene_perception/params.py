"""
ScenePerceptionParams：GPL py 官方生成参数之上的 frozen dataclass 装载层（含 tool/gravity_hint）.

声明 / 类型 / 默认值 / 中文描述 / 范围校验的权威源统一为
config/scene_perception_parameters.yaml（根键=节点名），由
generate_parameter_library_py 在构建期生成
peach_perception/scene_perception_parameters.py；本模块不再重复声明默认值
（旧 DECLARE_DEFAULTS/DESCRIPTIONS 双字典已移除，消除两处默认值漂移）。
静态装载语义不变：启动期装载，不做动态改参回调。

- ScenePerceptionParams.declare(node)：生成 ParamListener 并集中声明全部参数
  （类型/默认值/描述/校验来自参数库 yaml）；
- ScenePerceptionParams.from_params(p)：由生成的 Params 快照装载/解析/组装
  → frozen dataclass（gravity_hint 解析、gravity_mode 白名单回退、tool.* 组装）。

dataclass 用 eq=False：gravity_hint 为 ndarray 字段，默认逐字段 == 比较会触发
ndarray 多值歧义，按身份比较即可（无人比较 params 实例）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from peach_perception.scene_perception.interfaces import ToolGeometry


@dataclass(frozen=True, eq=False)
class ScenePerceptionParams:
    """
    ScenePerceptionNode 全部启动期参数装载后的不可变结构.

    标量字段名与参数键同名（target_memory.* 等的点号换下划线）；
    tool / gravity_hint 为 from_params 的派生字段（由 tool.* /
    gravity_hint_xyz 组装）；detector_impl 等 *_impl 为接口层实现注册名
    （节点装配用，yaml 原键 detector.impl 等）。
    """

    color_topic: str
    depth_topic: str
    camera_info_topic: str
    camera_optical_frame: str
    output_frame: str
    tf_timeout_sec: float            # 原值（秒）；Duration 转换由编排层做
    depth_scale_unit: float
    sync_slop_s: float
    min_detection_conf: float
    yolo_conf: float
    yolo_nms_iou: float
    sam_max_bboxes: int            # >=1（参数库校验）
    sam_min_area: int              # >=0（参数库校验）
    min_mask_points: int           # >=1（参数库校验）
    detection_dedup_ios: float
    publish_debug_image: bool
    publish_masks: bool
    publish_detection_cloud: bool
    detection_cloud_stride: int      # >=1（参数库校验）
    yolo_model_path: str
    sam_model_path: str
    model_version: str
    calibration_version: str
    gravity_hint_xyz: str
    gravity_mode: str                # 白名单 fixed/tf（from_params 回退）
    detector_impl: str               # 接口层实现注册名（2.14 装配）
    segmenter_impl: str
    pipeline_bag_impl: str
    pipeline_fruit_impl: str
    matcher_impl: str
    lock_impl: str
    tool: ToolGeometry = field(compare=False)
    # 派生字段（from_params 构建；不参与逐键同步测试的标量面）
    gravity_hint: Optional[np.ndarray] = field(compare=False)
    target_memory_enable: bool = True
    target_memory_match_radius_m: float = 0.06
    target_memory_max_targets: int = 50
    target_memory_position_ema: float = 0.3
    target_memory_recovery_scale: float = 1.0
    target_memory_cross_class_recovery: bool = True
    target_memory_confirm_frames: int = 5
    target_memory_tentative_ttl_frames: int = 8
    target_memory_anchor_max_age_s: float = 30.0
    target_memory_anchor_drop_s: float = 120.0
    target_memory_max_age_s: float = 600.0
    wind_swing_threshold_m: float = 0.03
    wind_swing_frames: int = 3
    lighting_min_depth_ratio: float = 0.35
    lighting_min_conf_mean: float = 0.3
    lighting_bad_frames: int = 5
    pipeline_min_depth_m: float = 0.3
    pipeline_max_depth_m: float = 2.5
    pipeline_min_points: int = 100
    pipeline_locked_only_segmentation: bool = True
    harvest_min_collect_frames: int = 10
    harvest_lock_settle_frames: int = 5
    harvest_max_collect_s: float = 25.0
    harvest_priority_prefer_lower_first: bool = True

    @staticmethod
    def declare(node) -> object:
        """
        生成 generate_parameter_library_py 的 ParamListener 并集中声明全部参数.

        Args:
            node: rclpy Node（声明参数+挂 on_set 校验，非法值在 declare 期拒绝）.

        Returns
        -------
            ParamListener：调用方持有并用于读取 Params 快照.

        """
        # 构建期由 setup.py 的 generate_parameter_module 生成.
        from peach_perception.scene_perception_parameters import (
            peach_scene_perception_node)
        return peach_scene_perception_node.ParamListener(node)

    @classmethod
    def from_params(cls, p) -> 'ScenePerceptionParams':
        """
        从生成的 Params 快照装载/解析/组装为 frozen dataclass.

        校验与组装规则（与节点原 _load_params 逐项一致）：
        gravity_hint_xyz 非空必须恰 3 个逗号分隔浮点（否则 ValueError）；
        gravity_mode 不在 {"fixed","tf"} 告警并回退 fixed；
        tool.* 组装 ToolGeometry（entry_standoff = entry_d_tool + entry_d_s）；
        数值范围校验已由参数库校验器承担（旧 from_node 的 max(1,..) 类 clamp 移除）。
        target_memory.* 与 *.impl 只作标量装载：匹配器/注册表/策略实例由节点按
        注册名装配（2.14）.

        Args:
            p: peach_scene_perception_node.Params（declare 后 get_params() 快照）.

        Returns
        -------
            ScenePerceptionParams 实例.

        """
        gh_s: str = p.gravity_hint_xyz.strip()
        if gh_s:
            parts = [float(x) for x in gh_s.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    f'gravity_hint_xyz needs 3 comma-separated floats, got {gh_s!r}')
            gravity_hint = np.asarray(parts, dtype=float)
        else:
            gravity_hint = None
        gravity_mode: str = p.gravity_mode.strip()
        if gravity_mode not in ('fixed', 'tf'):
            # 参数库无字符串白名单校验器，保留原语义：告警并回退 fixed。
            import rclpy.logging
            rclpy.logging.get_logger('peach_scene_perception_node').warning(
                f'未知 gravity_mode={gravity_mode!r}，回退 fixed')
            gravity_mode = 'fixed'
        # entry_standoff = 刀具伸出 + 安全间隙，与 contracts.ToolGeometry 一致
        tool = ToolGeometry(
            D_inner=float(p.tool.D_inner),
            L_insert=float(p.tool.L_insert),
            L_blade=float(p.tool.L_blade),
            entry_d_tool=float(p.tool.entry_d_tool),
            entry_d_s=float(p.tool.entry_d_s),
            entry_standoff=float(p.tool.entry_d_tool)
            + float(p.tool.entry_d_s),
            clearance_min=float(p.tool.clearance_min),
            margin_neck=float(p.tool.margin_neck),
            version=str(p.tool.version),
        )
        return cls(
            color_topic=p.color_topic,
            depth_topic=p.depth_topic,
            camera_info_topic=p.camera_info_topic,
            camera_optical_frame=p.camera_optical_frame.strip(),
            output_frame=p.output_frame.strip(),
            tf_timeout_sec=float(p.tf_timeout_sec),
            depth_scale_unit=float(p.depth_scale_unit),
            sync_slop_s=float(p.sync_slop_s),
            min_detection_conf=float(p.min_detection_conf),
            yolo_conf=float(p.yolo_conf),
            yolo_nms_iou=float(p.yolo_nms_iou),
            sam_max_bboxes=int(p.sam_max_bboxes),
            sam_min_area=int(p.sam_min_area),
            min_mask_points=int(p.min_mask_points),
            detection_dedup_ios=float(p.detection_dedup_ios),
            publish_debug_image=bool(p.publish_debug_image),
            publish_masks=bool(p.publish_masks),
            publish_detection_cloud=bool(p.publish_detection_cloud),
            detection_cloud_stride=int(p.detection_cloud_stride),
            yolo_model_path=p.yolo_model_path,
            sam_model_path=p.sam_model_path,
            model_version=p.model_version,
            calibration_version=p.calibration_version,
            gravity_hint_xyz=gh_s,
            gravity_mode=gravity_mode,
            detector_impl=p.detector.impl,
            segmenter_impl=p.segmenter.impl,
            pipeline_bag_impl=p.pipeline.bag_impl,
            pipeline_fruit_impl=p.pipeline.fruit_impl,
            matcher_impl=p.matcher.impl,
            lock_impl=p.lock.impl,
            tool=tool,
            gravity_hint=gravity_hint,
            target_memory_enable=bool(p.target_memory.enable),
            target_memory_match_radius_m=float(p.target_memory.match_radius_m),
            target_memory_max_targets=int(p.target_memory.max_targets),
            target_memory_position_ema=float(p.target_memory.position_ema),
            target_memory_recovery_scale=float(p.target_memory.recovery_scale),
            target_memory_cross_class_recovery=bool(
                p.target_memory.cross_class_recovery),
            target_memory_confirm_frames=int(p.target_memory.confirm_frames),
            target_memory_tentative_ttl_frames=int(
                p.target_memory.tentative_ttl_frames),
            target_memory_anchor_max_age_s=float(
                p.target_memory.anchor_max_age_s),
            target_memory_anchor_drop_s=float(p.target_memory.anchor_drop_s),
            target_memory_max_age_s=float(p.target_memory.max_age_s),
            wind_swing_threshold_m=float(p.wind.swing_threshold_m),
            wind_swing_frames=int(p.wind.swing_frames),
            lighting_min_depth_ratio=float(p.lighting.min_depth_ratio),
            lighting_min_conf_mean=float(p.lighting.min_conf_mean),
            lighting_bad_frames=int(p.lighting.bad_frames),
            pipeline_min_depth_m=float(p.pipeline.min_depth_m),
            pipeline_max_depth_m=float(p.pipeline.max_depth_m),
            pipeline_min_points=int(p.pipeline.min_points),
            pipeline_locked_only_segmentation=bool(
                p.pipeline.locked_only_segmentation),
            harvest_min_collect_frames=int(p.harvest.min_collect_frames),
            harvest_lock_settle_frames=int(p.harvest.lock_settle_frames),
            harvest_max_collect_s=float(p.harvest.max_collect_s),
            harvest_priority_prefer_lower_first=bool(
                p.harvest.priority_prefer_lower_first),
        )
