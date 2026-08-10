"""
参数层 — PeachPoseParams：32 个参数的集中声明/装载/校验（frozen dataclass）.

对标 nav2 ParameterHandler 的 Python 版（设计文档
docs/superpowers/specs/2026-08-10-peach-layered-architecture.md §2.1）：

- ``PeachPoseParams.declare(node)``：集中 declare 全部参数与中文 descriptor
  （由节点原 ``_declare_params`` 整体搬入，键名/默认值/说明不变）；
- ``PeachPoseParams.from_node(node)``：集中读取/解析/校验 → frozen dataclass
  （由节点原 ``_load_params`` 整体搬入：gravity_hint 解析、gravity_mode
  白名单、tool.* 组装 ToolGeometry、target_memory.* 构建 TargetRegistry）。

权威源哲学（AGENTS.md §7）：``config/peach_pose.yaml`` 是默认值权威源，
``DECLARE_DEFAULTS`` 与之逐项对齐，双向同步由 test/test_params.py 强制。
参数为启动期静态装载，不做动态改参回调（后续如需再加 on_set 校验）。

dataclass 用 ``eq=False``：gravity_hint 为 ndarray 字段，默认逐字段 ==
比较会触发 ndarray 多值歧义，按身份比较即可（无人比较 params 实例）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from peach_pose_ros2.peach_pose.contracts import ToolGeometry
from peach_pose_ros2.peach_pose.target_registry import TargetRegistry
from rcl_interfaces.msg import ParameterDescriptor

# 与 config/peach_pose.yaml 逐项对齐的默认值（test_params.py 双向同步强制）
DECLARE_DEFAULTS = {
    'color_topic': '/camera/color/image_raw',
    'depth_topic': '/camera/depth/image_raw',
    'camera_info_topic': '/camera/color/camera_info',
    # 手眼链挂在 color optical；深度 registration 后几何也在此系
    'camera_optical_frame': 'camera_color_optical_frame',
    'output_frame': 'base_link',
    'tf_timeout_sec': 0.5,
    # Percipio 原始深度常需 ×0.25 才是毫米量级（再 /1000→米）
    'depth_scale_unit': 0.25,
    'sync_slop_s': 0.05,
    'min_detection_conf': 0.5,
    'yolo_conf': 0.5,
    # 重叠检测框去重：IoS（交集/较小框面积）≥ 本值判同一目标，保留大框
    'detection_dedup_ios': 0.6,
    'publish_debug_image': True,
    'publish_masks': True,
    'publish_detection_cloud': True,
    'detection_cloud_stride': 2,
    'yolo_model_path': '',
    'sam_model_path': '',
    'model_version': 'yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f',
    'calibration_version':
        'percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006',
    # 空串 → 算法默认相机系 +Y；否则 "x,y,z" 重力提示
    'gravity_hint_xyz': '',
    # fixed=仅用 gravity_hint_xyz；tf=由本帧 TF 旋转反推相机系重力
    'gravity_mode': 'fixed',
    'tool.D_inner': 0.104,
    'tool.L_insert': 0.200,
    'tool.L_blade': 0.025,
    'tool.entry_d_tool': 0.030,
    'tool.entry_d_s': 0.040,
    'tool.clearance_min': 0.005,
    'tool.margin_neck': 0.015,
    'tool.version': '1.1',
    # 目标身份记忆：世界系（output_frame）最近邻匹配跨帧复用 target_id
    'target_memory.enable': True,
    'target_memory.match_radius_m': 0.06,
    'target_memory.max_targets': 50,
    'target_memory.position_ema': 0.3,
    # 恢复匹配：正常匹配未命中时半径 × 本倍率再试一次（1.0=关闭）
    'target_memory.recovery_scale': 2.0,
    # 恢复匹配允许跨类别（抗 bag/nobag 翻类；命中不改表项类别）
    'target_memory.cross_class_recovery': True,
    # 确认机制：累计命中 ≥ 本帧数才转正长期记录（1=立即确认）
    'target_memory.confirm_frames': 3,
    # 未确认目标存活时限 (s)：超期未再命中即清除（瞬时误检不留记录）
    'target_memory.tentative_ttl_sec': 1.0,
}

DESCRIPTIONS = {
    'color_topic': '彩色图话题（bgr8）',
    'depth_topic': '深度图话题（uint16，须与彩色图 registration 对齐）',
    'camera_info_topic': '彩色相机内参话题',
    'camera_optical_frame': '相机光学系 frame_id（手眼链所挂）；'
                            '空串则用深度图 header.frame_id',
    'output_frame': '输出坐标系：几何经 TF 变到此帧；空串=保持相机系',
    'tf_timeout_sec': 'TF 查询超时 (s)',
    'depth_scale_unit': '深度比例因子（仅 uint16 原始深度生效）：raw × 本值 = '
                        '毫米量级（Percipio 常见 0.25）；数据集回放（真毫米）'
                        '设 1.0；32FC1 浮点深度按「米」×1000 转毫米，本参数不生效',
    'sync_slop_s': 'RGB-D 近似同步允差 (s)',
    'min_detection_conf': '检测置信度下限，低于该值的目标不入管线',
    'yolo_conf': 'YOLO 推理置信度阈值',
    'detection_dedup_ios': '重叠检测框去重 IoS 阈值：交集/较小框面积 ≥ 本值'
                           '判同一物理目标，保留大框（跨类生效）；≥1.0 等效关闭',
    'publish_debug_image': '是否发布 debug 叠加图 (~/debug_image)',
    'publish_masks': '是否发布 SAM 掩膜图 (~/masks)',
    'publish_detection_cloud': '是否发布检测框内深度反投影彩色点云 '
                               '(~/detection_cloud)',
    'detection_cloud_stride': '点云降采样步长（>1 减轻 RViz 负载）',
    'yolo_model_path': 'YOLO 权重路径；空串=包内 model/best.pt',
    'sam_model_path': 'MobileSAM 权重路径；空串=包内 model/mobile_sam.pt',
    'model_version': '模型版本标识（随结果发布，便于追溯）',
    'calibration_version': '内外参版本标识（内参 color_camera_info.yaml；'
                           '外参 hand_eye/active.yaml）',
    'gravity_hint_xyz': '重力方向提示 "x,y,z"（相机系）；'
                        '空串=算法默认相机系 +Y',
    'gravity_mode': '重力来源：fixed=仅用 gravity_hint_xyz（默认，行为与旧版'
                    '一致）；tf=由本帧 output←camera 的 TF 旋转反推相机系重力'
                    '（output_frame 系重力约定 [0,0,-1]，只乘旋转不加平移，'
                    'TF 不可用的帧回退 gravity_hint_xyz）',
    'tool.D_inner': '工具圆柱内径 (m)，袋子必须能通过',
    'tool.L_insert': '最大插入深度 (m)',
    'tool.L_blade': '刀刃平面到圆柱入口平面的距离 (m)',
    'tool.entry_d_tool': '入口 standoff 的工具分量 (m)',
    'tool.entry_d_s': '入口 standoff 的安全裕量分量 (m)',
    'tool.clearance_min': '袋体与工具内壁的最小径向余量 (m)',
    'tool.margin_neck': '袋颈前方的安全停止距离 (m)',
    'tool.version': '工具几何配置的语义版本号',
    'target_memory.enable': '目标身份记忆开关：世界系最近邻匹配，同一物理'
                            '桃子消失再出现复用同一 target_id',
    'target_memory.match_radius_m': '目标匹配半径 (m)：同类且距离 ≤ 本值的'
                                    '最近历史目标命中复用 ID',
    'target_memory.max_targets': '目标表容量上限；超限按最久未见淘汰',
    'target_memory.position_ema': '命中后位置/轴/直径的 EMA 平滑系数 α '
                                  '(0,1]，越大越跟随最新观测',
    'target_memory.recovery_scale': '恢复匹配半径倍率 (≥1)：正常匹配未命中时'
                                    '放宽到 match_radius×本值再匹配一次，'
                                    '抗锚点跳变；1.0=关闭',
    'target_memory.cross_class_recovery': '恢复匹配允许跨类别（抗 bag/nobag '
                                          '翻类；半径不放大仍限 match_radius，'
                                          '命中只复用 ID 不改表项类别）',
    'target_memory.confirm_frames': '目标确认帧数：累计命中 ≥ 本值才转为长期'
                                    '记录，之前的短暂出现不长期保留；1=立即确认',
    'target_memory.tentative_ttl_sec': '未确认目标存活时限 (s)：超过本时间未再'
                                       '命中即从表中清除（瞬时误检不占身份）',
}


@dataclass(frozen=True, eq=False)
class PeachPoseParams:
    """
    PeachPoseNode 全部启动期参数（37 项 declare）装载后的不可变结构.

    标量字段名与参数键同名（``target_memory.*`` 的点号换下划线）；
    ``tool`` / ``gravity_hint`` / ``target_registry`` 为 from_node 的
    派生字段（由 tool.* / gravity_hint_xyz / target_memory.* 组装）。
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
    detection_dedup_ios: float
    publish_debug_image: bool
    publish_masks: bool
    publish_detection_cloud: bool
    detection_cloud_stride: int      # ≥1（from_node 已 clamp）
    yolo_model_path: str
    sam_model_path: str
    model_version: str
    calibration_version: str
    gravity_hint_xyz: str
    gravity_mode: str                # 白名单 {'fixed', 'tf'}（from_node 已校验）
    tool: ToolGeometry = field(compare=False)
    # 派生字段（from_node 构建；不参与逐键同步测试的标量面）
    gravity_hint: Optional[np.ndarray] = field(compare=False)
    target_memory_enable: bool = True
    target_memory_match_radius_m: float = 0.06
    target_memory_max_targets: int = 50
    target_memory_position_ema: float = 0.3
    target_memory_recovery_scale: float = 2.0
    target_memory_cross_class_recovery: bool = True
    target_memory_confirm_frames: int = 3
    target_memory_tentative_ttl_sec: float = 1.0
    target_registry: Optional[TargetRegistry] = field(default=None, compare=False)

    @staticmethod
    def declare(node) -> None:
        """
        在 node 上集中 declare 全部 32 个参数（中文 descriptor 同步附着）.

        Args:
            node: rclpy Node（测试可用带 declare_parameter 的替身）.

        Returns
        -------
            无返回值（None）.

        """
        for k, v in DECLARE_DEFAULTS.items():
            node.declare_parameter(
                k, v, ParameterDescriptor(description=DESCRIPTIONS[k]))

    @classmethod
    def from_node(cls, node) -> 'PeachPoseParams':
        """
        从 node 参数服务器集中读取/解析/校验，装载为 frozen dataclass.

        校验与组装规则（与节点原 _load_params 逐项一致）：
        gravity_hint_xyz 非空必须恰 3 个逗号分隔浮点（否则 ValueError）；
        gravity_mode 不在 {'fixed','tf'} 告警并回退 fixed；
        detection_cloud_stride clamp 到 ≥1；tool.* 组装 ToolGeometry
        （entry_standoff = entry_d_tool + entry_d_s）；target_memory.enable
        为真时按三个 target_memory.* 标量构建 TargetRegistry，否则 None。

        Args:
            node: rclpy Node（参数已 declare）.

        Returns
        -------
            PeachPoseParams 实例.

        """
        g = node.get_parameter
        gh_s = g('gravity_hint_xyz').get_parameter_value().string_value.strip()
        if gh_s:
            parts = [float(x) for x in gh_s.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    f'gravity_hint_xyz needs 3 comma-separated floats, got {gh_s!r}')
            gravity_hint = np.asarray(parts, dtype=float)
        else:
            gravity_hint = None
        gravity_mode = g('gravity_mode').get_parameter_value().string_value.strip()
        if gravity_mode not in ('fixed', 'tf'):
            node.get_logger().warning(
                f'未知 gravity_mode={gravity_mode!r}，回退 fixed')
            gravity_mode = 'fixed'
        # entry_standoff = 刀具伸出 + 安全间隙，与 contracts.ToolGeometry 一致
        tool = ToolGeometry(
            D_inner=float(g('tool.D_inner').value),
            L_insert=float(g('tool.L_insert').value),
            L_blade=float(g('tool.L_blade').value),
            entry_d_tool=float(g('tool.entry_d_tool').value),
            entry_d_s=float(g('tool.entry_d_s').value),
            entry_standoff=float(g('tool.entry_d_tool').value)
            + float(g('tool.entry_d_s').value),
            clearance_min=float(g('tool.clearance_min').value),
            margin_neck=float(g('tool.margin_neck').value),
            version=str(g('tool.version').value),
        )
        # 目标身份记忆（纯算法，无 ROS 依赖；语义见 target_registry.py 模块docstring）
        target_memory_enable = bool(g('target_memory.enable').value)
        if target_memory_enable:
            target_registry = TargetRegistry(
                match_radius=float(g('target_memory.match_radius_m').value),
                max_targets=int(g('target_memory.max_targets').value),
                position_ema=float(g('target_memory.position_ema').value),
                recovery_scale=float(g('target_memory.recovery_scale').value),
                cross_class_recovery=bool(
                    g('target_memory.cross_class_recovery').value),
                confirm_frames=int(g('target_memory.confirm_frames').value),
                tentative_ttl_sec=float(
                    g('target_memory.tentative_ttl_sec').value))
        else:
            target_registry = None
        return cls(
            color_topic=g('color_topic').get_parameter_value().string_value,
            depth_topic=g('depth_topic').get_parameter_value().string_value,
            camera_info_topic=g(
                'camera_info_topic').get_parameter_value().string_value,
            camera_optical_frame=g(
                'camera_optical_frame').get_parameter_value().string_value.strip(),
            output_frame=g('output_frame').get_parameter_value().string_value.strip(),
            tf_timeout_sec=float(g('tf_timeout_sec').value),
            depth_scale_unit=float(g('depth_scale_unit').value),
            sync_slop_s=float(g('sync_slop_s').value),
            min_detection_conf=float(g('min_detection_conf').value),
            yolo_conf=float(g('yolo_conf').value),
            detection_dedup_ios=float(g('detection_dedup_ios').value),
            publish_debug_image=bool(g('publish_debug_image').value),
            publish_masks=bool(g('publish_masks').value),
            publish_detection_cloud=bool(g('publish_detection_cloud').value),
            detection_cloud_stride=max(1, int(g('detection_cloud_stride').value)),
            yolo_model_path=g('yolo_model_path').get_parameter_value().string_value,
            sam_model_path=g('sam_model_path').get_parameter_value().string_value,
            model_version=g('model_version').get_parameter_value().string_value,
            calibration_version=g(
                'calibration_version').get_parameter_value().string_value,
            gravity_hint_xyz=gh_s,
            gravity_mode=gravity_mode,
            tool=tool,
            gravity_hint=gravity_hint,
            target_memory_enable=target_memory_enable,
            target_memory_match_radius_m=float(
                g('target_memory.match_radius_m').value),
            target_memory_max_targets=int(g('target_memory.max_targets').value),
            target_memory_position_ema=float(g('target_memory.position_ema').value),
            target_memory_recovery_scale=float(
                g('target_memory.recovery_scale').value),
            target_memory_cross_class_recovery=bool(
                g('target_memory.cross_class_recovery').value),
            target_memory_confirm_frames=int(
                g('target_memory.confirm_frames').value),
            target_memory_tentative_ttl_sec=float(
                g('target_memory.tentative_ttl_sec').value),
            target_registry=target_registry,
        )
