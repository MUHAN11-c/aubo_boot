"""
参数层 — ReconstructionParams：frozen dataclass + declare/from_node.

形态对标 nav2 ``ParameterHandler<ParamsT>`` 的 Python 版（设计文档
docs/superpowers/specs/2026-08-10-peach-layered-architecture.md §2.1）：
嵌套 frozen dataclass 按 ROS 参数名分组（frames/camera/capture/bind/
view_filter/icp/local_volume/tsdf/cloud_filter/refit/publish/session +
装配组 cloud_builder/refiner/volume/refitter/mask_gate，协议 2.14 的
``*.impl`` 实现选择键），字段名 =
ROS 参数名去组前缀，类型与 ROS 参数一致。

权威源哲学（本仓约定）：``config/reconstruction.yaml`` 是默认值权威源，
dataclass 字段默认值为代码侧唯一来源（declare 直接取字段默认值，
不再另存字典），两边由 test_params.py 的双向同步测试强制对齐。
参数为启动期静态装载（不做动态改参回调）；declare_parameter 本身即
类型校验，from_node 不另加业务校验（现状语义保持不变）。

本模块只依赖 rcl_interfaces（descriptor）与鸭子类型 node
（declare_parameter/get_parameter），不 import rclpy。
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Dict, Tuple

from rcl_interfaces.msg import ParameterDescriptor


@dataclass(frozen=True)
class FramesParams:
    """坐标系参数（frames.*）."""

    base_frame: str = 'base_link'  # 重建输出坐标系


@dataclass(frozen=True)
class CameraParams:
    """输入话题参数（camera.*）."""

    color_topic: str = '/camera/color/image_raw'
    depth_topic: str = '/camera/depth/image_raw'
    camera_info_topic: str = '/camera/color/camera_info'


@dataclass(frozen=True)
class CaptureParams:
    """采帧门禁与自动模式参数（capture.*）."""

    min_views: int = 4
    recommended_views: int = 5
    max_views: int = 24
    require_robot_static: bool = False
    static_joint_vel_thresh: float = 0.01
    max_frame_age_s: float = 2.0
    auto_mode: bool = True
    auto_finalize_at_max: bool = False
    auto_min_interval_s: float = 0.0
    require_target_mask: bool = True
    min_mask_pixels: int = 300
    min_mask_depth_ratio: float = 0.35
    max_target_drift_m: float = 0.04
    # 邻目标串扰门（E2）：绑定锚点与其他锁定目标锚点间距小于本值时拒帧，
    # 防邻近目标点云混入形成 TSDF 不可回滚双层表面（I6）；≤0 关闭
    min_neighbor_gap_m: float = 0.15
    build_timeout_s: float = 180.0  # BuildTargetModel 等 min_views 上限


@dataclass(frozen=True)
class BindParams:
    """目标绑定参数（bind.*，E2 会话防抖）."""

    # 防感知 selected 瞬态抖动销毁进行中会话：selected_target_id 变化
    # （含变空）须持续超过本时长才放弃旧会话重绑；holdoff 内切回原 ID
    # 取消挂起，旧会话继续采帧（同一果实的瞬态误切不浪费已攒视角）
    switch_holdoff_s: float = 2.0


@dataclass(frozen=True)
class ViewFilterParams:
    """视角过滤参数（view_filter.*，平移 [m] 旋转 [deg]）."""

    min_translation: float = 0.002
    max_translation: float = 0.080
    min_rotation_deg: float = 1.0
    max_rotation_deg: float = 25.0
    allow_duplicate_views: bool = True


@dataclass(frozen=True)
class IcpParams:
    """FK 初值约束下的帧到模型 ICP 参数（icp.*）."""

    enable: bool = True
    min_points: int = 300
    coarse_voxel: float = 0.006
    fine_voxel: float = 0.003
    coarse_correspondence: float = 0.015
    fine_correspondence: float = 0.007
    coarse_iterations: int = 20
    fine_iterations: int = 10
    min_fitness: float = 0.35
    max_rmse: float = 0.008
    max_translation: float = 0.010
    max_rotation_deg: float = 3.0
    # E4 ICP target 增量复用（icp_target_cache.IcpTargetCache）：两次全量
    # extract 之间复用「上次全量+已采帧修正后云增量拼接」做 ICP target；
    # 全量刷新周期 k [帧] 按近期修正量 EMA 在 [min,max] 内自适应伸缩
    # （min=max=1 退化为旧行为：每帧全量提取）
    target_refresh_min_period: int = 1
    target_refresh_max_period: int = 5
    # 漂移判定比：修正量 EMA ≥ max_translation×本值（或 fk 回退/拒帧）→
    # k 收回下限；EMA ≤ 1/4×该阈值 → k 拉长到上限；中间迟滞带保持
    target_refresh_drift_ratio: float = 0.5


@dataclass(frozen=True)
class LocalVolumeParams:
    """局部体素盒参数（local_volume.*，TSDF 云 ROI 裁剪 [m]）."""

    size_x: float = 0.30
    size_y: float = 0.30
    size_z: float = 0.40


@dataclass(frozen=True)
class TsdfParams:
    """TSDF 融合参数（tsdf.*，长度 [m]）."""

    enable: bool = True
    voxel_length: float = 0.003
    sdf_trunc: float = 0.012
    depth_trunc: float = 1.5


@dataclass(frozen=True)
class CloudFilterParams:
    """TSDF 提取云后处理参数（cloud_filter.*）."""

    voxel_size: float = 0.003
    enable_statistical_filter: bool = True


@dataclass(frozen=True)
class CloudBuilderParams:
    """点云构建器装配参数（cloud_builder.*，协议 2.14）."""

    impl: str = 'open3d_cloud'  # interfaces.CLOUD_BUILDERS 注册名


@dataclass(frozen=True)
class RefinerParams:
    """配准器装配参数（refiner.*，协议 2.14）."""

    impl: str = 'bounded_icp'  # interfaces.REFINERS 注册名


@dataclass(frozen=True)
class VolumeParams:
    """融合体积装配参数（volume.*，协议 2.14）."""

    impl: str = 'local_tsdf'  # interfaces.VOLUMES 注册名


@dataclass(frozen=True)
class RefitterParams:
    """几何精化器装配参数（refitter.*，协议 2.14，圆柱/球两线）."""

    cylinder_impl: str = 'cylinder_refit'  # interfaces.REFITTERS 注册名
    sphere_impl: str = 'sphere_refit'      # interfaces.REFITTERS 注册名


@dataclass(frozen=True)
class MaskGateParams:
    """掩膜门装配参数（mask_gate.*，协议 2.14）."""

    impl: str = 'strict_mask_gate'  # interfaces.MASK_GATES 注册名


@dataclass(frozen=True)
class RefitParams:
    """几何二次拟合参数（refit.*，长度 [m]）."""

    enable: bool = True
    cylinder_inlier_min: float = 0.35
    rmse_max_m: float = 0.005
    entry_standoff_m: float = 0.070


@dataclass(frozen=True)
class PublishParams:
    """发布节流参数（publish.*，E4 on-change + 最小间隔）."""

    # 点云/Marker 类大消息（local_cloud/tsdf_cloud/markers）仅内容版本
    # 变化才发（transient_local 闩锁保持，零变化抑制不丢 RViz 显示）；
    # false 回退逐次全发旧行为。心跳/状态/诊断/refit 三件套不经过本开关
    on_change_only: bool = True
    # 同一话题两次实际发布的最小间隔 [s]；间隔内的变化被抑制（不丢：
    # 下次发布触发时补发最新版本）；≤0 关闭间隔门（只留 on-change）
    min_interval_s: float = 0.2


@dataclass(frozen=True)
class SessionParams:
    """session 落盘参数（session.*）."""

    root_dir: str = ''  # 空 = <工作区>/peach_sessions


# 嵌套组（组字段名 → 组 dataclass 类型）；顺序即 declare/文档顺序
_GROUPS: Tuple[Tuple[str, type], ...] = (
    ('frames', FramesParams),
    ('camera', CameraParams),
    ('capture', CaptureParams),
    ('bind', BindParams),
    ('view_filter', ViewFilterParams),
    ('icp', IcpParams),
    ('local_volume', LocalVolumeParams),
    ('tsdf', TsdfParams),
    ('cloud_filter', CloudFilterParams),
    ('cloud_builder', CloudBuilderParams),
    ('refiner', RefinerParams),
    ('volume', VolumeParams),
    ('refitter', RefitterParams),
    ('mask_gate', MaskGateParams),
    ('refit', RefitParams),
    ('publish', PublishParams),
    ('session', SessionParams),
)

# 中文 descriptor（key 为完整 ROS 参数名；declare 时逐个取用）
_DESCRIPTIONS: Dict[str, str] = {
    'frames.base_frame': '重建输出坐标系（点云/Marker/诊断的 frame_id）',
    'camera.color_topic': '彩色图话题（bgr8）',
    'camera.depth_topic': '深度图话题（uint16 或 32FC1，须与彩图对齐）',
    'camera.camera_info_topic': '彩色相机内参话题',
    'sync_slop_s': 'RGB-D 近似同步允差 (s)',
    'tf_timeout_sec': '按 depth.header.stamp 精确查询 TF 的等待上限 (s)；'
                      '失败直接拒帧，禁止运动中回退最新 TF',
    'depth_scale_unit': '深度比例因子（仅 uint16 原始深度生效）：raw × 本值 = '
                        '毫米（Percipio 常见 0.25）；数据集回放设 1.0；'
                        '32FC1 浮点深度按「米」×1000 转毫米，本参数不生效',
    'capture.min_views': 'finalize 所需最少视角数',
    'capture.recommended_views': '推荐视角数（不足仅提示，不阻塞 finalize）',
    'capture.max_views': '帧栈上限（达到后拒采，先 remove_last 或 finalize）',
    'capture.require_robot_static': '采帧是否要求机器人静止（查 /joint_states）',
    'capture.static_joint_vel_thresh': '静止判定：最大关节速度阈值 [rad/s]',
    'capture.max_frame_age_s': '缓存帧龄期上限 (s)，超过视为陈帧拒采',
    'capture.auto_mode': '自动模式总开关：true=有候选自动开始、每个唯一'
                         '同步帧进入质量门并连续采集；'
                         'false=纯手动 Trigger 服务流',
    'capture.auto_finalize_at_max': '采满 max_views 自动 finalize'
                                    '（连续扫描默认关闭）',
    'capture.auto_min_interval_s': '两次自动采帧最小间隔 (s)，0 表示每个'
                                   '唯一相机时间戳均进入质量门',
    'capture.require_target_mask': '仅积分全局计划所选 target_id 的逐帧掩膜',
    'capture.min_mask_pixels': '目标掩膜最少像素数（过小视为远距或遮挡）',
    'capture.min_mask_depth_ratio': '掩膜内有效深度占比下限（强光/空洞门）',
    'capture.max_target_drift_m': '当前目标中心相对绑定中心最大漂移 [m]（风动门；'
                                  '室外风动场景建议调至 0.06，见 yaml 注释）',
    'capture.min_neighbor_gap_m': '绑定锚点与其他锁定目标锚点的最小间距 [m]；'
                                  '小于则拒帧，防邻近目标点云混入形成 TSDF '
                                  '不可回滚双层表面（I6）；≤0 关闭本门',
    'capture.build_timeout_s': 'BuildTargetModel 等待 min_views 的上限 (s)；超时不 finalize',
    'bind.switch_holdoff_s': 'selected 切换防抖 (s)：selected_target_id 变化'
                             '（含变空）须持续超过本时长才放弃进行中会话重绑，'
                             '防感知 selected 瞬态抖动销毁会话',
    'view_filter.min_translation': '与上一已采帧的最小平移 [m]（过近=重复视角）',
    'view_filter.max_translation': '与上一已采帧的最大平移 [m]（过远=跳变）',
    'view_filter.min_rotation_deg': '与上一已采帧的最小旋转 [deg]',
    'view_filter.max_rotation_deg': '与上一已采帧的最大旋转 [deg]',
    'view_filter.allow_duplicate_views': 'true 时重复视角仅告警仍采帧',
    'icp.enable': '启用 FK 初值约束下的帧到 TSDF 模型 ICP',
    'icp.min_points': 'ICP 源云和模型云各自最少点数',
    'icp.coarse_voxel': 'ICP 粗层体素边长 [m]',
    'icp.fine_voxel': 'ICP 细层体素边长 [m]',
    'icp.coarse_correspondence': 'ICP 粗层最大对应距离 [m]',
    'icp.fine_correspondence': 'ICP 细层最大对应距离 [m]',
    'icp.coarse_iterations': 'ICP 粗层最大迭代次数',
    'icp.fine_iterations': 'ICP 细层最大迭代次数',
    'icp.min_fitness': 'ICP/FK 预对齐最小重叠 fitness',
    'icp.max_rmse': 'ICP/FK 预对齐最大内点 RMSE [m]',
    'icp.max_translation': 'ICP 相对 FK 的最大平移修正 [m]',
    'icp.max_rotation_deg': 'ICP 相对 FK 的最大旋转修正 [deg]',
    'icp.target_refresh_min_period': 'ICP target 全量 extract 刷新周期下限 '
                                     '[帧]（E4）；1=每帧全量提取（旧行为）',
    'icp.target_refresh_max_period': 'ICP target 全量 extract 刷新周期上限 '
                                     '[帧]（E4）；k 在上下限间按修正量 EMA '
                                     '自适应伸缩',
    'icp.target_refresh_drift_ratio': '漂移判定比（E4）：修正量 EMA ≥ '
                                      'max_translation×本值（或 fk 回退/拒帧）'
                                      '→ k 收回下限；EMA ≤ 1/4×该阈值 → k '
                                      '拉长到上限；中间迟滞带保持',
    'local_volume.size_x': '局部体素盒 X 尺寸 [m]（TSDF 云 ROI 裁剪）',
    'local_volume.size_y': '局部体素盒 Y 尺寸 [m]（TSDF 云 ROI 裁剪）',
    'local_volume.size_z': '局部体素盒 Z 尺寸 [m]（TSDF 云 ROI 裁剪）',
    'tsdf.enable': 'TSDF 融合开关：true=采帧后立即在线积分并发布'
                   ' /peach/reconstruction/tsdf_cloud',
    'tsdf.voxel_length': 'TSDF 体素边长 [m]',
    'tsdf.sdf_trunc': 'TSDF 截断距离 [m]',
    'tsdf.depth_trunc': 'TSDF 深度截断 [m]（更远的深度不积分）',
    'cloud_filter.voxel_size': 'TSDF 提取云体素降采样边长 [m]（≤0 不降）',
    'cloud_filter.enable_statistical_filter': 'TSDF 提取云统计离群剔除'
                                              '（20 邻域 2σ）',
    'cloud_builder.impl': '点云构建器实现注册名（interfaces.CLOUD_BUILDERS，'
                          '协议 2.14）',
    'refiner.impl': '帧到模型配准器实现注册名（interfaces.REFINERS，'
                    '协议 2.14）',
    'volume.impl': '融合体积实现注册名（interfaces.VOLUMES，协议 2.14）',
    'refitter.cylinder_impl': '圆柱（袋桃）refit 实现注册名'
                              '（interfaces.REFITTERS，协议 2.14）',
    'refitter.sphere_impl': '球（裸桃）refit 实现注册名'
                            '（interfaces.REFITTERS，协议 2.14）',
    'mask_gate.impl': '目标掩膜门实现注册名（interfaces.MASK_GATES，'
                      '协议 2.14）',
    'refit.enable': '几何二次拟合（refit）开关：true=每次 TSDF 全量提取后'
                    '对 tsdf_cloud 做圆柱/球 RANSAC 精化并更新抓取示意；'
                    'finalize 后再拟一次为定稿。抓取许可仍仅 READY 后放行。'
                    '发 refined_pose/refined_axis/refined_diagnostics',
    'refit.cylinder_inlier_min': 'refit ACCEPT 门控：拟合内点率下限'
                                 '（圆柱/球共用，低于则 REOBSERVE）',
    'refit.rmse_max_m': 'refit ACCEPT 门控：拟合 RMSE 上限 [m]'
                        '（超过则 REOBSERVE）',
    'refit.entry_standoff_m': 'refined_pose 的 entry_pose 自 bottom 沿 '
                              '−axis 后撤量 [m]',
    'publish.on_change_only': '点云/Marker 类大消息（local_cloud/tsdf_cloud/'
                              'markers）仅内容版本变化才发（E4；闩锁保持，'
                              '零变化抑制不丢 RViz 显示）；false=逐次全发',
    'publish.min_interval_s': '点云/Marker 类话题两次实际发布的最小间隔 [s]'
                              '（E4）；间隔内变化被抑制、下次触发补发最新版；'
                              '≤0 关闭间隔门',
    'session.root_dir': 'session 落盘根目录；空 = <工作区>/peach_sessions'
                        '（按包 share 路径反推工作区根）',
}

# 无组前缀的顶层标量参数
_SCALARS: Tuple[str, ...] = ('sync_slop_s', 'tf_timeout_sec', 'depth_scale_unit')


@dataclass(frozen=True)
class ReconstructionParams:
    """全部 64 个节点参数的 frozen 装载形态（嵌套组 + 顶层标量）."""

    sync_slop_s: float = 0.05
    tf_timeout_sec: float = 1.0
    depth_scale_unit: float = 0.25
    frames: FramesParams = field(default_factory=FramesParams)
    camera: CameraParams = field(default_factory=CameraParams)
    capture: CaptureParams = field(default_factory=CaptureParams)
    bind: BindParams = field(default_factory=BindParams)
    view_filter: ViewFilterParams = field(default_factory=ViewFilterParams)
    icp: IcpParams = field(default_factory=IcpParams)
    local_volume: LocalVolumeParams = field(default_factory=LocalVolumeParams)
    tsdf: TsdfParams = field(default_factory=TsdfParams)
    cloud_filter: CloudFilterParams = field(default_factory=CloudFilterParams)
    cloud_builder: CloudBuilderParams = field(
        default_factory=CloudBuilderParams)
    refiner: RefinerParams = field(default_factory=RefinerParams)
    volume: VolumeParams = field(default_factory=VolumeParams)
    refitter: RefitterParams = field(default_factory=RefitterParams)
    mask_gate: MaskGateParams = field(default_factory=MaskGateParams)
    refit: RefitParams = field(default_factory=RefitParams)
    publish: PublishParams = field(default_factory=PublishParams)
    session: SessionParams = field(default_factory=SessionParams)

    @classmethod
    def defaults_flat(cls) -> Dict[str, object]:
        """
        全部参数的平坦 {ROS 名: 默认值} 映射（declare 与同步测试共用）.

        Returns
        -------
            dict：key 为完整 ROS 参数名（含组前缀），value 为字段默认值；
            遍历组 dataclass 即时构建，字段默认值是唯一代码来源.

        """
        inst = cls()
        out = {}
        for name in _SCALARS:
            out[name] = getattr(inst, name)
        for group_name, _group_cls in _GROUPS:
            group = getattr(inst, group_name)
            for f in fields(group):
                out[f'{group_name}.{f.name}'] = getattr(group, f.name)
        return out

    @classmethod
    def declare(cls, node) -> None:
        """
        在 node 上集中 declare 全部参数（中文 descriptor 逐个附带）.

        Args:
            node: 鸭子类型节点（提供 declare_parameter(name, default,
                descriptor)）.

        Returns
        -------
            无返回值（None）.

        """
        for name, default in cls.defaults_flat().items():
            node.declare_parameter(
                name, default,
                ParameterDescriptor(description=_DESCRIPTIONS[name]))

    @classmethod
    def from_node(cls, node) -> 'ReconstructionParams':
        """
        从 node 集中读取全部参数并组装为 frozen dataclass.

        字符串字段沿用现状语义（base_frame/root_dir 读入后 strip）；
        其余类型由 declare 的默认值类型保证，不再另加校验。

        Args:
            node: 鸭子类型节点（提供 get_parameter(name).value）.

        Returns
        -------
            ReconstructionParams（frozen，64 个参数全装载）.

        """
        g = node.get_parameter
        group_kwargs = {}
        for group_name, group_cls in _GROUPS:
            kwargs = {}
            for f in fields(group_cls):
                value = g(f'{group_name}.{f.name}').value
                if isinstance(value, str):
                    value = value.strip()
                kwargs[f.name] = value
            group_kwargs[group_name] = group_cls(**kwargs)
        return cls(
            sync_slop_s=float(g('sync_slop_s').value),
            tf_timeout_sec=float(g('tf_timeout_sec').value),
            depth_scale_unit=float(g('depth_scale_unit').value),
            **group_kwargs)
