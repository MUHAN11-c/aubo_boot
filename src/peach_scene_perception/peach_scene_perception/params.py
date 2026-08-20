"""
参数层 — PeachPoseParams：63 个参数的集中声明/装载/校验（frozen dataclass）.

对标 nav2 ParameterHandler 的 Python 版（设计文档
docs/superpowers/specs/2026-08-10-peach-layered-architecture.md §2.1）：

- ``PeachPoseParams.declare(node)``：集中 declare 全部参数与中文 descriptor
  （由节点原 ``_declare_params`` 整体搬入，键名/默认值/说明不变）；
- ``PeachPoseParams.from_node(node)``：集中读取/解析/校验 → frozen dataclass
  （gravity_hint 解析、gravity_mode 白名单、tool.* 组装 ToolGeometry）。

装配规则（2.14，A3 起）：``detector.impl`` / ``segmenter.impl`` /
``pipeline.bag_impl`` / ``pipeline.fruit_impl`` / ``matcher.impl`` /
``lock.impl`` 六键按名选择接口层实现（注册名见 peach_pose/impls.py
显式注册清单），节点构造期经 Registry.create 注入；A3 前由本层直接
构建 TargetRegistry 的做法已移除（匹配器/注册表改由节点装配）。

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
from peach_scene_perception.peach_pose.contracts import ToolGeometry
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
    'min_detection_conf': 0.40,
    'yolo_conf': 0.35,
    # YOLO NMS IoU 阈值（阶段 D1 参数化；室外多果密排场景可调低防并框）
    'yolo_nms_iou': 0.5,
    # 单次 SAM 推理的最大 prompt 框数（阶段 D1 由硬编码 8 提为参数；室外
    # 多果场景一帧目标数常超 8，截断会让后排目标无掩膜被判 OCCLUDED）
    'sam_max_bboxes': 16,
    # SAM 掩膜最小像素数，过小丢弃（抗 SAM 边角碎掩膜）
    'sam_min_area': 100,
    # 收敛前景掩膜最小像素数，不足判 mask_unavailable（CandidateEstimator）
    'min_mask_points': 50,
    # 重叠检测框去重：IoS（交集/较小框面积）≥ 本值判同一目标，保留大框
    'detection_dedup_ios': 0.6,
    # 阶段 H（协议 2.13）：生产档 debug 叠加图默认关——关闭时零序列化零
    # 发布（不画不转不发，非发空图）；现场调图再显式开
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
    # 2.14 装配：接口层实现的注册名（默认实现见 peach_pose/impls.py）
    'detector.impl': 'yolo',
    'segmenter.impl': 'mobile_sam',
    'pipeline.bag_impl': 'robust_bag',
    'pipeline.fruit_impl': 'robust_fruit',
    'matcher.impl': 'spatial_ema',
    'lock.impl': 'collect_lock',
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
    # 恢复匹配：正常匹配未命中时半径 × 本倍率再试一次（1.0=关闭）；
    # 3.5（恢复半径 21cm）覆盖真机实测的跨视角锚点偏差 15.7cm
    'target_memory.recovery_scale': 1.0,
    # 恢复匹配允许跨类别（抗 bag/nobag 翻类；命中不改表项类别）
    'target_memory.cross_class_recovery': True,
    # 确认机制：累计命中 ≥ 本帧数才转正长期记录（1=立即确认）
    'target_memory.confirm_frames': 5,
    # 未确认目标存活时限（帧）：连续超本帧数未再命中即清除（瞬时误检不留
    # 记录）；按帧计、帧率以运行状态为准——须 ≥ confirm_frames + 余量，
    # 低帧率/卡顿时墙钟 TTL 会在确认攒满前误清进度，帧计数不受帧率影响
    'target_memory.tentative_ttl_frames': 8,
    # 锁定集 LOST 目标的记忆锚点新鲜度（阶段 D1，协议 2.4）：LOST 超本秒数
    # 打 anchor_stale 旗标并视为不可选（不移除）；上限语义——运行期节点按
    # 实测帧间隔 EMA 折算帧数判定（I4，帧率以运行状态为准）
    'target_memory.anchor_max_age_s': 30.0,
    # LOST 超本秒数从计划移除并记 target_dropped（编排侧按
    # SKIPPED_UNREACHABLE「目标丢失超时」入账）；同为上限语义按帧率折算
    'target_memory.anchor_drop_s': 120.0,
    # 全部表项（含已确认）的墙钟龄上限：超时未命中即淘汰（阶段 D1 语义
    # 变更——跨场景陈旧锚点会被恢复匹配误命中抢走新目标身份，协议 2.4）
    'target_memory.max_age_s': 600.0,
    # 摆动检测（阶段 D1 室外风动）：观测残差（当前观测锚点 − 注册表 EMA
    # 位置模长）连续 swing_frames 帧超阈值 → 目标打 target_swinging 旗标
    # （视为不可选，能力端 RECONFIRM 等平息）；平息判定对称连击清除
    'wind.swing_threshold_m': 0.03,
    'wind.swing_frames': 3,
    # 光照质量观测指标（阶段 D1；不打阻断旗标，仅 harvest_state 暴露 +
    # WARN 提示现场补光）：锁定集目标掩膜内有效深度占比 EMA / 置信度
    # EMA 任一连 bad_frames 帧低于阈值 → low_light_quality=true
    'lighting.min_depth_ratio': 0.35,
    'lighting.min_conf_mean': 0.3,
    'lighting.bad_frames': 5,
    # 位姿管线有效深度窗与前景点数下限（阶段 D1 由构造默认提为参数；
    # 室外远距目标/补深度场景可调）
    'pipeline.min_depth_m': 0.3,
    'pipeline.max_depth_m': 2.5,
    'pipeline.min_points': 100,
    # 阶段 H（协议 2.13-E1）：锁定后 SAM 只对锁定集目标的检测框推理
    # （锚点反投影预识别，见 peach_pose/segmentation_gate.py）；false 回退
    # 锁定后仍全量分割的旧行为
    'pipeline.locked_only_segmentation': True,
    # 全局采摘计划（收齐式窗口锁定）：窗口最少累积帧数，达到后才允许
    # 按静止条件关闭窗口锁定目标集合
    'harvest.min_collect_frames': 10,
    # 连续无新增确认 ID 的帧数，与最少帧数联合判定目标集合已稳定
    'harvest.lock_settle_frames': 5,
    # 收齐窗口最长时长 (s)，超时强制关闭兜底（空集也锁定）
    'harvest.max_collect_s': 25.0,
    # 优先级是否启用高度键：true=先低后高（避免摘高处时碰落低处果）
    'harvest.priority_prefer_lower_first': True,
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
    'min_detection_conf': '检测框进入几何管线的置信度下限（第二级）',
    'yolo_conf': 'YOLO 推理置信度阈值（第一级，低于此值的框不进入）',
    'yolo_nms_iou': 'YOLO NMS IoU 阈值：室外多果密排可调低防并框',
    'sam_max_bboxes': '单次 SAM 推理最大 prompt 框数（超出截断；室外多果'
                      '场景须覆盖一帧目标数，否则截断目标无掩膜判 OCCLUDED）',
    'sam_min_area': 'SAM 掩膜最小像素数，过小丢弃（抗边角碎掩膜）',
    'min_mask_points': '收敛前景掩膜最小像素数，不足判 mask_unavailable',
    'detection_dedup_ios': '重叠检测框去重 IoS 阈值：交集/较小框面积 ≥ 本值'
                           '判同一物理目标，保留大框（跨类生效）；≥1.0 等效关闭',
    'publish_debug_image': '是否发布 debug 叠加图 (/peach/perception/debug_image)；'
                           '阶段 H 起默认关：关闭时零序列化零发布，现场调图显式开',
    'publish_masks': '是否发布 SAM 掩膜图 (/peach/perception/masks)',
    'publish_detection_cloud': '是否发布检测框内深度反投影彩色点云 '
                               '(/peach/perception/single_cloud)',
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
    'detector.impl': '检测器实现注册名（Detector 接口；默认 yolo='
                     'UltralyticsYolo，可选名见 peach_pose/impls.py）',
    'segmenter.impl': '分割器实现注册名（Segmenter 接口；默认 mobile_sam='
                      'MobileSam）',
    'pipeline.bag_impl': '袋装线位姿管线实现注册名（PosePipeline 接口；'
                         '默认 robust_bag=RobustBagPosePipeline）',
    'pipeline.fruit_impl': '裸果线位姿管线实现注册名（PosePipeline 接口；'
                           '默认 robust_fruit=RobustFruitPosePipeline）',
    'matcher.impl': '目标匹配器实现注册名（TargetMatcher 接口；默认 '
                    'spatial_ema=SpatialEmaMatcher）',
    'lock.impl': '收齐窗口锁定策略实现注册名（LockPolicy 接口；默认 '
                 'collect_lock=CollectLockPolicy）',
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
    'target_memory.confirm_frames': '目标确认帧数：累计命中 ≥ 本值才计入锁定/'
                                    '候选话题/3D Marker；短暂闪现不转正；1=立即确认',
    'target_memory.tentative_ttl_frames': '未确认目标存活时限（帧）：连续超'
                                          '本帧数未再命中即清除（瞬时误检不占'
                                          '身份；按帧计，帧率以运行状态为准）',
    'target_memory.anchor_max_age_s': '锁定集 LOST 目标锚点新鲜度上限 (s)：'
                                      '超龄打 anchor_stale 旗标且视为不可选'
                                      '（不移除）；运行期按实测帧间隔 EMA 折算'
                                      '帧数判定（I4）',
    'target_memory.anchor_drop_s': '锁定集 LOST 目标移除上限 (s)：超龄从计划'
                                   '移除并记 target_dropped 事件（编排侧按'
                                   ' SKIPPED_UNREACHABLE 入账）；同按帧率折算',
    'target_memory.max_age_s': '目标表项（含已确认）墙钟龄上限 (s)：超时未命中'
                               '即淘汰，防跨场景陈旧锚点被恢复匹配误命中',
    'wind.swing_threshold_m': '摆动判定残差阈值 (m)：观测锚点 − 注册表 EMA 位置'
                              '的模长连续超本值判目标摆动（target_swinging）',
    'wind.swing_frames': '摆动判定连击帧数：连续本帧数超阈值置位、连续本帧数'
                         '低于阈值清除（对称判定）',
    'lighting.min_depth_ratio': '光照质量：锁定集目标掩膜内有效深度占比 EMA '
                                '下限，连续低质判 low_light_quality（观测指标，'
                                '不阻断）',
    'lighting.min_conf_mean': '光照质量：锁定集目标检测置信度均值 EMA 下限',
    'lighting.bad_frames': '光照质量低质判定的连续帧数',
    'pipeline.min_depth_m': '位姿管线有效深度下限 (m)，过近视为噪声',
    'pipeline.max_depth_m': '位姿管线有效深度上限 (m)，过远视为背景',
    'pipeline.min_points': '位姿管线有效前景点数下限，不足直接 REJECT',
    'pipeline.locked_only_segmentation':
        '锁定后 SAM 只对锁定集目标的检测框推理（2.13-E1；锚点反投影识别'
        '锁定框，TF 不可用帧自动回退全量）；false 回退锁定后全量分割',
    'harvest.min_collect_frames': '全局目标收齐窗口的最少累积帧数：达到后才允许'
                                  '按静止条件关闭窗口、锁定目标集合',
    'harvest.lock_settle_frames': '连续无新增确认 ID 的帧数：与最少帧数联合判定'
                                  '目标集合已稳定，关闭收齐窗口',
    'harvest.max_collect_s': '收齐窗口最长时长 (s)：超时强制关闭窗口兜底'
                             '（空集也锁定，target_count=0）',
    'harvest.priority_prefer_lower_first': '优先级排序启用高度键：先低后高，避免'
                                           '摘高处目标时碰落低处果实；false 则'
                                           '高度不参与排序',
}


@dataclass(frozen=True, eq=False)
class PeachPoseParams:
    """
    PeachPoseNode 全部启动期参数（63 项 declare）装载后的不可变结构.

    标量字段名与参数键同名（``target_memory.*`` 等的点号换下划线）；
    ``tool`` / ``gravity_hint`` 为 from_node 的派生字段（由 tool.* /
    gravity_hint_xyz 组装）；``*.impl`` 为接口层实现注册名（节点装配用）。
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
    sam_max_bboxes: int            # ≥1
    sam_min_area: int              # ≥0
    min_mask_points: int           # ≥1
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
    detector_impl: str               # 接口层实现注册名（2.14 装配）
    segmenter_impl: str
    pipeline_bag_impl: str
    pipeline_fruit_impl: str
    matcher_impl: str
    lock_impl: str
    tool: ToolGeometry = field(compare=False)
    # 派生字段（from_node 构建；不参与逐键同步测试的标量面）
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
    def declare(node) -> None:
        """
        在 node 上集中 declare 全部 63 个参数（中文 descriptor 同步附着）.

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
        （entry_standoff = entry_d_tool + entry_d_s）。target_memory.* 与
        *.impl 只作标量装载：匹配器/注册表/策略实例由节点按注册名装配
        （2.14，A3 起本层不再直接构建 TargetRegistry）。

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
            yolo_nms_iou=float(g('yolo_nms_iou').value),
            sam_max_bboxes=max(1, int(g('sam_max_bboxes').value)),
            sam_min_area=max(0, int(g('sam_min_area').value)),
            min_mask_points=max(1, int(g('min_mask_points').value)),
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
            detector_impl=g('detector.impl').get_parameter_value().string_value,
            segmenter_impl=g(
                'segmenter.impl').get_parameter_value().string_value,
            pipeline_bag_impl=g(
                'pipeline.bag_impl').get_parameter_value().string_value,
            pipeline_fruit_impl=g(
                'pipeline.fruit_impl').get_parameter_value().string_value,
            matcher_impl=g('matcher.impl').get_parameter_value().string_value,
            lock_impl=g('lock.impl').get_parameter_value().string_value,
            tool=tool,
            gravity_hint=gravity_hint,
            target_memory_enable=bool(g('target_memory.enable').value),
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
            target_memory_tentative_ttl_frames=int(
                g('target_memory.tentative_ttl_frames').value),
            target_memory_anchor_max_age_s=float(
                g('target_memory.anchor_max_age_s').value),
            target_memory_anchor_drop_s=float(
                g('target_memory.anchor_drop_s').value),
            target_memory_max_age_s=float(g('target_memory.max_age_s').value),
            wind_swing_threshold_m=float(g('wind.swing_threshold_m').value),
            wind_swing_frames=int(g('wind.swing_frames').value),
            lighting_min_depth_ratio=float(
                g('lighting.min_depth_ratio').value),
            lighting_min_conf_mean=float(g('lighting.min_conf_mean').value),
            lighting_bad_frames=int(g('lighting.bad_frames').value),
            pipeline_min_depth_m=float(g('pipeline.min_depth_m').value),
            pipeline_max_depth_m=float(g('pipeline.max_depth_m').value),
            pipeline_min_points=int(g('pipeline.min_points').value),
            pipeline_locked_only_segmentation=bool(
                g('pipeline.locked_only_segmentation').value),
            harvest_min_collect_frames=int(
                g('harvest.min_collect_frames').value),
            harvest_lock_settle_frames=int(
                g('harvest.lock_settle_frames').value),
            harvest_max_collect_s=float(g('harvest.max_collect_s').value),
            harvest_priority_prefer_lower_first=bool(
                g('harvest.priority_prefer_lower_first').value),
        )
