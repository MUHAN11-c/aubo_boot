"""
TargetReconstructionParams：GPL py 官方生成参数之上的 frozen dataclass 装载层（嵌套组形态与参数库一致）.

声明 / 类型 / 默认值 / 中文描述 / 范围校验的权威源统一为
config/target_reconstruction_parameters.yaml（根键=节点名），由
generate_parameter_library_py 在构建期生成
peach_perception/target_reconstruction_parameters.py；本模块不再重复声明默认值
（旧 _DESCRIPTIONS + defaults_flat 维护面已移除，消除两处默认值漂移）。
参数为启动期静态装载（不做动态改参回调）；declare 期校验类型与范围，
from_params 不再另加业务校验（现状语义保持不变）。

本模块只依赖 stdlib 与鸭子类型 Params 快照，不 import rclpy。
"""
from __future__ import annotations

from dataclasses import dataclass, field


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

    min_views: int = 2  # 最少独立机位数（当前位+一次短 PTP）
    recommended_views: int = 5
    max_views: int = 24
    minimum_baseline_deg: float = 8.0
    minimum_mean_nearest_baseline_deg: float = 6.0
    minimum_mean_depth_ratio: float = 0.40
    require_robot_static: bool = True
    static_joint_vel_thresh: float = 0.03
    max_frame_age_s: float = 2.0
    auto_mode: bool = True
    auto_finalize_at_max: bool = False
    auto_min_interval_s: float = 0.0
    require_target_mask: bool = True
    min_mask_pixels: int = 300
    min_mask_depth_ratio: float = 0.35
    max_target_drift_m: float = 0.04
    # 邻目标串扰门（E2）：绑定锚点与其他锁定目标锚点间距小于本值时拒帧，
    # 防邻近目标点云混入形成 TSDF 不可回滚双层表面（I6）；<=0 关闭
    min_neighbor_gap_m: float = 0.15
    # 串扰门小框豁免比：邻居框面积×本值 < 绑定框面积时该邻居不计入间距
    # （叶片遮挡残片/误检）；<=0 关闭豁免（09-01 近距双检定夺）
    neighbor_gap_area_ratio: float = 2.0
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
    # 漂移判定比：修正量 EMA >= max_translation×本值（或 fk 回退/拒帧）→
    # k 收回下限；EMA <= 1/4×该阈值 → k 拉长到上限；中间迟滞带保持
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
class FrameStoreParams:
    """批级帧栈装配参数（frame_store.*，协议 2.14）."""

    impl: str = 'default'  # interfaces.FRAME_STORES 注册名


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
    entry_standoff_m: float = 0.0
    pregrasp_standoff_m: float = 0.0
    max_axis_angle_deg: float = 35.0


@dataclass(frozen=True)
class PublishParams:
    """发布节流参数（publish.*，E4 on-change + 最小间隔）."""

    # 点云/Marker 类大消息（local_cloud/tsdf_cloud/markers）仅内容版本
    # 变化才发（transient_local 闩锁保持，零变化抑制不丢 RViz 显示）；
    # false 回退逐次全发旧行为。心跳/状态/诊断/refit 三件套不经过本开关
    on_change_only: bool = True
    # 同一话题两次实际发布的最小间隔 [s]；间隔内的变化被抑制（不丢：
    # 下次发布触发时补发最新版本）；<=0 关闭间隔门（只留 on-change）
    min_interval_s: float = 0.2


@dataclass(frozen=True)
class SessionParams:
    """session 落盘参数（session.*）."""

    root_dir: str = ''  # 空 = 工作区 runs/


@dataclass(frozen=True)
class TargetReconstructionParams:
    """全部节点参数的 frozen 装载形态（嵌套组 + 顶层标量）."""

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
    frame_store: FrameStoreParams = field(default_factory=FrameStoreParams)
    cloud_builder: CloudBuilderParams = field(
        default_factory=CloudBuilderParams)
    refiner: RefinerParams = field(default_factory=RefinerParams)
    volume: VolumeParams = field(default_factory=VolumeParams)
    refitter: RefitterParams = field(default_factory=RefitterParams)
    mask_gate: MaskGateParams = field(default_factory=MaskGateParams)
    refit: RefitParams = field(default_factory=RefitParams)
    publish: PublishParams = field(default_factory=PublishParams)
    session: SessionParams = field(default_factory=SessionParams)

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
        from peach_perception.target_reconstruction_parameters import (
            peach_target_reconstruction_node)
        return peach_target_reconstruction_node.ParamListener(node)

    @classmethod
    def from_params(cls, p) -> 'TargetReconstructionParams':
        """
        从生成的 Params 快照集中装载为 frozen dataclass.

        字符串字段沿用现状语义（base_frame/root_dir 读入后 strip）；
        其余类型与范围由参数库校验器保证，不再另加校验。

        Args:
            p: peach_target_reconstruction_node.Params（declare 后
                get_params() 快照；嵌套组字段名与下列 dataclass 一一对应）.

        Returns
        -------
            TargetReconstructionParams（frozen，64 个参数全装载）.

        """
        def _strip(v):
            return v.strip() if isinstance(v, str) else v

        return cls(
            sync_slop_s=float(p.sync_slop_s),
            tf_timeout_sec=float(p.tf_timeout_sec),
            depth_scale_unit=float(p.depth_scale_unit),
            frames=FramesParams(base_frame=_strip(p.frames.base_frame)),
            camera=CameraParams(
                color_topic=p.camera.color_topic,
                depth_topic=p.camera.depth_topic,
                camera_info_topic=p.camera.camera_info_topic),
            capture=CaptureParams(
                min_views=int(p.capture.min_views),
                recommended_views=int(p.capture.recommended_views),
                max_views=int(p.capture.max_views),
                minimum_baseline_deg=float(p.capture.minimum_baseline_deg),
                minimum_mean_nearest_baseline_deg=float(
                    p.capture.minimum_mean_nearest_baseline_deg),
                minimum_mean_depth_ratio=float(
                    p.capture.minimum_mean_depth_ratio),
                require_robot_static=bool(p.capture.require_robot_static),
                static_joint_vel_thresh=float(
                    p.capture.static_joint_vel_thresh),
                max_frame_age_s=float(p.capture.max_frame_age_s),
                auto_mode=bool(p.capture.auto_mode),
                auto_finalize_at_max=bool(p.capture.auto_finalize_at_max),
                auto_min_interval_s=float(p.capture.auto_min_interval_s),
                require_target_mask=bool(p.capture.require_target_mask),
                min_mask_pixels=int(p.capture.min_mask_pixels),
                min_mask_depth_ratio=float(p.capture.min_mask_depth_ratio),
                max_target_drift_m=float(p.capture.max_target_drift_m),
                min_neighbor_gap_m=float(p.capture.min_neighbor_gap_m),
                neighbor_gap_area_ratio=float(
                    p.capture.neighbor_gap_area_ratio),
                build_timeout_s=float(p.capture.build_timeout_s)),
            bind=BindParams(switch_holdoff_s=float(p.bind.switch_holdoff_s)),
            view_filter=ViewFilterParams(
                min_translation=float(p.view_filter.min_translation),
                max_translation=float(p.view_filter.max_translation),
                min_rotation_deg=float(p.view_filter.min_rotation_deg),
                max_rotation_deg=float(p.view_filter.max_rotation_deg),
                allow_duplicate_views=bool(
                    p.view_filter.allow_duplicate_views)),
            icp=IcpParams(
                enable=bool(p.icp.enable),
                min_points=int(p.icp.min_points),
                coarse_voxel=float(p.icp.coarse_voxel),
                fine_voxel=float(p.icp.fine_voxel),
                coarse_correspondence=float(p.icp.coarse_correspondence),
                fine_correspondence=float(p.icp.fine_correspondence),
                coarse_iterations=int(p.icp.coarse_iterations),
                fine_iterations=int(p.icp.fine_iterations),
                min_fitness=float(p.icp.min_fitness),
                max_rmse=float(p.icp.max_rmse),
                max_translation=float(p.icp.max_translation),
                max_rotation_deg=float(p.icp.max_rotation_deg),
                target_refresh_min_period=int(
                    p.icp.target_refresh_min_period),
                target_refresh_max_period=int(
                    p.icp.target_refresh_max_period),
                target_refresh_drift_ratio=float(
                    p.icp.target_refresh_drift_ratio)),
            local_volume=LocalVolumeParams(
                size_x=float(p.local_volume.size_x),
                size_y=float(p.local_volume.size_y),
                size_z=float(p.local_volume.size_z)),
            tsdf=TsdfParams(
                enable=bool(p.tsdf.enable),
                voxel_length=float(p.tsdf.voxel_length),
                sdf_trunc=float(p.tsdf.sdf_trunc),
                depth_trunc=float(p.tsdf.depth_trunc)),
            cloud_filter=CloudFilterParams(
                voxel_size=float(p.cloud_filter.voxel_size),
                enable_statistical_filter=bool(
                    p.cloud_filter.enable_statistical_filter)),
            frame_store=FrameStoreParams(impl=p.frame_store.impl),
            cloud_builder=CloudBuilderParams(impl=p.cloud_builder.impl),
            refiner=RefinerParams(impl=p.refiner.impl),
            volume=VolumeParams(impl=p.volume.impl),
            refitter=RefitterParams(
                cylinder_impl=p.refitter.cylinder_impl,
                sphere_impl=p.refitter.sphere_impl),
            mask_gate=MaskGateParams(impl=p.mask_gate.impl),
            refit=RefitParams(
                enable=bool(p.refit.enable),
                cylinder_inlier_min=float(p.refit.cylinder_inlier_min),
                rmse_max_m=float(p.refit.rmse_max_m),
                entry_standoff_m=float(p.refit.entry_standoff_m),
                pregrasp_standoff_m=float(p.refit.pregrasp_standoff_m),
                max_axis_angle_deg=float(p.refit.max_axis_angle_deg)),
            publish=PublishParams(
                on_change_only=bool(p.publish.on_change_only),
                min_interval_s=float(p.publish.min_interval_s)),
            session=SessionParams(root_dir=_strip(p.session.root_dir)),
        )
