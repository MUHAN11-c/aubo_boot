"""
peach_perception handwritten parameter module (decision 0017 SNAPSHOT).

键名冻结：模块 DEFAULTS 与 config/<节点>.yaml 必须同键。GPL 形状的
`config/vision_contract.param.yaml` 是同一 schema 的声明式子集（跨字段合同），
不是第三套参数框架；Python 节点仍走本文件 ParamListener。C++ 新合同字段
走 generate_parameter_library。

两个感知节点的声明/兜底默认/校验/快照装载集中于本文件（各节点类持有
自己的 DEFAULTS/RULES）；部署值与中文描述的事实源是 config/scene_perception.yaml
与 target_reconstruction.yaml（nav2 式 ros__parameters 全量清单），launch 以
ParameterFile 装入。键名冻结；改默认值须模块与 yaml 各改一处。
接口与原 GPL 生成物一致：ParamListener(node).get_params()/is_old()。
"""

from __future__ import annotations

from copy import deepcopy

from peach_harvester.vision.param_rules import check as _check, check_min_max
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidParameterValueException


def _assign(params, key, value):
    """按点号键把 value 写入快照（逐段下钻嵌套组）."""
    parts = key.split('.')
    target = params
    for part in parts[:-1]:
        target = getattr(target, part)
    setattr(target, parts[-1], value)


class peach_scene_perception_node:
    """peach_scene_perception_node 的参数声明/装载/校验器（手写，决策 0017；原 GPL 生成物等价物）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/<节点>.yaml 为准
        'color_topic': ('string', '/camera/color/image_raw'),
        'depth_topic': ('string', '/camera/depth/image_raw'),
        'camera_info_topic': ('string', '/camera/color/camera_info'),
        'camera_optical_frame': ('string', 'camera_color_optical_frame'),
        'output_frame': ('string', 'base_link'),
        'tf_timeout_sec': ('double', 0.5),
        'depth_scale_unit': ('double', 0.25),
        'sync_slop_s': ('double', 0.05),
        'min_detection_conf': ('double', 0.4),
        'yolo_conf': ('double', 0.35),
        'yolo_nms_iou': ('double', 0.5),
        'sam_max_bboxes': ('int', 16),
        'sam_min_area': ('int', 100),
        'min_mask_points': ('int', 50),
        'detection_dedup_ios': ('double', 0.6),
        'detection_dedup_area_ratio': ('double', 0.5),
        'publish_debug_image': ('bool', True),
        'publish_masks': ('bool', True),
        'publish_detection_cloud': ('bool', True),
        'detection_cloud_stride': ('int', 2),
        'yolo_model_path': ('string', ''),
        'sam_model_path': ('string', ''),
        'model_version': ('string', 'yolo:6981750db67a726e|mobile_sam:6dbb90523a35330f'),
        'calibration_version': (
            'string', 'percipio-640x480-chessboard|hand_eye:import_humble_20260128T114006'),
        'gravity_hint_xyz': ('string', ''),
        'gravity_mode': ('string', 'tf'),
        'pipeline.bag_impl': ('string', 'robust_bag'),
        'pipeline.fruit_impl': ('string', 'robust_fruit'),
        'pipeline.min_depth_m': ('double', 0.3),
        'pipeline.max_depth_m': ('double', 2.5),
        'pipeline.min_points': ('int', 100),
        'pipeline.locked_only_segmentation': ('bool', True),
        'tool.D_inner': ('double', 0.104),
        'tool.L_insert': ('double', 0.2),
        'tool.L_blade': ('double', 0.0),
        'tool.entry_d_tool': ('double', 0.0),
        'tool.entry_d_s': ('double', 0.0),
        'tool.clearance_min': ('double', 0.005),
        'tool.margin_neck': ('double', 0.015),
        'tool.version': ('string', '1.1'),
        'target_memory.enable': ('bool', True),
        'target_memory.match_radius_m': ('double', 0.06),
        'target_memory.max_targets': ('int', 50),
        'target_memory.position_ema': ('double', 0.3),
        'target_memory.recovery_scale': ('double', 1.0),
        'target_memory.confirm_frames': ('int', 5),
        'target_memory.tentative_ttl_frames': ('int', 8),
        'target_memory.anchor_max_age_s': ('double', 30.0),
        'target_memory.anchor_drop_s': ('double', 120.0),
        'target_memory.max_age_s': ('double', 600.0),
        'wind.swing_threshold_m': ('double', 0.03),
        'wind.swing_frames': ('int', 3),
        'lighting.min_depth_ratio': ('double', 0.35),
        'lighting.min_conf_mean': ('double', 0.3),
        'lighting.bad_frames': ('int', 5),
        'harvest.min_collect_frames': ('int', 10),
        'harvest.lock_settle_frames': ('int', 5),
        'harvest.max_collect_s': ('double', 25.0),
        'harvest.priority_prefer_lower_first': ('bool', True),
    }

    RULES = {  # 手写校验规则；启动期非法覆盖即抛，运行期非法 set 即拒
        'tf_timeout_sec': (('gt_eq', 0.0),),
        'depth_scale_unit': (('gt', 0.0),),
        'sync_slop_s': (('gt_eq', 0.0),),
        'sam_max_bboxes': (('gt_eq', 1.0),),
        'sam_min_area': (('gt_eq', 0.0),),
        'min_mask_points': (('gt_eq', 1.0),),
        'detection_cloud_stride': (('gt_eq', 1.0),),
        'pipeline.min_depth_m': (('gt', 0.0),),
        'pipeline.max_depth_m': (('gt', 0.0),),
        'pipeline.min_points': (('gt_eq', 1.0),),
        'tool.D_inner': (('gt', 0.0),),
        'tool.L_insert': (('gt', 0.0),),
        'tool.entry_d_tool': (('gt_eq', 0.0),),
        'tool.entry_d_s': (('gt_eq', 0.0),),
        'tool.clearance_min': (('gt', 0.0),),
        'tool.margin_neck': (('gt_eq', 0.0),),
        'target_memory.match_radius_m': (('gt', 0.0),),
        'target_memory.max_targets': (('gt_eq', 1.0),),
        'target_memory.position_ema': (('gt', 0.0), ('lt_eq', 1.0),),
        'target_memory.recovery_scale': (('gt_eq', 1.0),),
        'target_memory.confirm_frames': (('gt_eq', 1.0),),
        'target_memory.tentative_ttl_frames': (('gt_eq', 1.0),),
        'target_memory.anchor_max_age_s': (('gt', 0.0),),
        'target_memory.anchor_drop_s': (('gt', 0.0),),
        'target_memory.max_age_s': (('gt', 0.0),),
        'wind.swing_threshold_m': (('gt', 0.0),),
        'wind.swing_frames': (('gt_eq', 1.0),),
        'lighting.min_depth_ratio': (('bounds', 0.0, 1.0),),
        'lighting.min_conf_mean': (('bounds', 0.0, 1.0),),
        'lighting.bad_frames': (('gt_eq', 1.0),),
        'harvest.min_collect_frames': (('gt_eq', 1.0),),
        'harvest.lock_settle_frames': (('gt_eq', 1.0),),
        'harvest.max_collect_s': (('gt', 0.0),),
    }

    class _Harvest:
        """组 harvest 的参数字段."""

        def __init__(self):
            self.min_collect_frames = 10
            self.lock_settle_frames = 5
            self.max_collect_s = 25.0
            self.priority_prefer_lower_first = True

    class _Lighting:
        """组 lighting 的参数字段."""

        def __init__(self):
            self.min_depth_ratio = 0.35
            self.min_conf_mean = 0.3
            self.bad_frames = 5

    class _Pipeline:
        """组 pipeline 的参数字段."""

        def __init__(self):
            self.bag_impl = 'robust_bag'
            self.fruit_impl = 'robust_fruit'
            self.min_depth_m = 0.3
            self.max_depth_m = 2.5
            self.min_points = 100
            self.locked_only_segmentation = True

    class _TargetMemory:
        """组 target_memory 的参数字段."""

        def __init__(self):
            self.enable = True
            self.match_radius_m = 0.06
            self.max_targets = 50
            self.position_ema = 0.3
            self.recovery_scale = 1.0
            self.confirm_frames = 5
            self.tentative_ttl_frames = 8
            self.anchor_max_age_s = 30.0
            self.anchor_drop_s = 120.0
            self.max_age_s = 600.0

    class _Tool:
        """组 tool 的参数字段."""

        def __init__(self):
            self.D_inner = 0.104
            self.L_insert = 0.2
            self.L_blade = 0.0
            self.entry_d_tool = 0.0
            self.entry_d_s = 0.0
            self.clearance_min = 0.005
            self.margin_neck = 0.015
            self.version = '1.1'

    class _Wind:
        """组 wind 的参数字段."""

        def __init__(self):
            self.swing_threshold_m = 0.03
            self.swing_frames = 3

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            self.harvest = peach_scene_perception_node._Harvest()
            self.lighting = peach_scene_perception_node._Lighting()
            self.pipeline = peach_scene_perception_node._Pipeline()
            self.target_memory = peach_scene_perception_node._TargetMemory()
            self.tool = peach_scene_perception_node._Tool()
            self.wind = peach_scene_perception_node._Wind()
            for key, (_, value) in peach_scene_perception_node.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node, prefix=''):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = peach_scene_perception_node.Params()
            for key, (_, default) in peach_scene_perception_node.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in peach_scene_perception_node.DEFAULTS:
                _assign(self.params_, key, node.get_parameter(key).value)
            self._validate_all()
            node.add_on_set_parameters_callback(self._on_set)

        def get_params(self):
            """当前快照深拷贝（含 stamp_ 变更戳）."""
            return deepcopy(self.params_)

        def is_old(self, other):
            """快照 other 是否落后于内部值（运行路径据此刷新）."""
            return self._stamp != other.stamp_

        def set_user_callback(self, callback):
            """注册快照刷新后的用户回调."""
            self.user_callback = callback

        def clear_user_callback(self):
            """注销用户回调."""
            self.user_callback = None

        def refresh_dynamic_parameters(self):
            """把内部快照推给用户回调（兼容旧刷新语义）."""
            if self.user_callback:
                self.user_callback(self.get_params())

        def _validate_all(self):
            """启动期全量校验：yaml 覆盖值非法即抛异常终止启动."""
            for key, rules in peach_scene_perception_node.RULES.items():
                group, _, leaf = key.rpartition('.')
                # 逐段下钻嵌套组（与 _assign 一致，支持任意层级）
                target = self.params_
                for part in group.split('.') if group else ():
                    target = getattr(target, part)
                value = getattr(target, leaf)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)
            why = check_min_max(
                self.params_.pipeline.min_depth_m,
                self.params_.pipeline.max_depth_m,
                'pipeline.min_depth_m', 'pipeline.max_depth_m')
            if why:
                raise InvalidParameterValueException(
                    'pipeline.min_depth_m', self.params_.pipeline.min_depth_m, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in peach_scene_perception_node.DEFAULTS:
                    continue
                for rule in peach_scene_perception_node.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in peach_scene_perception_node.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)


class peach_target_reconstruction_node:
    """peach_target_reconstruction_node 的参数声明/装载/校验器（手写，决策 0017；原 GPL 生成物等价物）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/<节点>.yaml 为准
        'frames.base_frame': ('string', 'base_link'),
        'camera.color_topic': ('string', '/camera/color/image_raw'),
        'camera.depth_topic': ('string', '/camera/depth/image_raw'),
        'camera.camera_info_topic': ('string', '/camera/color/camera_info'),
        'sync_slop_s': ('double', 0.05),
        'tf_timeout_sec': ('double', 1.0),
        'depth_scale_unit': ('double', 0.25),
        'capture.min_views': ('int', 2),
        'capture.recommended_views': ('int', 5),
        'capture.max_views': ('int', 24),
        'capture.minimum_baseline_deg': ('double', 8.0),
        'capture.minimum_mean_nearest_baseline_deg': ('double', 6.0),
        'capture.minimum_mean_depth_ratio': ('double', 0.4),
        'capture.require_robot_static': ('bool', True),
        'capture.static_joint_vel_thresh': ('double', 0.03),
        'capture.max_frame_age_s': ('double', 2.0),
        'capture.auto_mode': ('bool', True),
        'capture.auto_finalize_at_max': ('bool', False),
        'capture.auto_min_interval_s': ('double', 0.0),
        'capture.require_target_mask': ('bool', True),
        'capture.min_mask_pixels': ('int', 300),
        'capture.min_mask_depth_ratio': ('double', 0.35),
        'capture.max_target_drift_m': ('double', 0.04),
        'capture.min_neighbor_gap_m': ('double', 0.15),
        'capture.neighbor_gap_area_ratio': ('double', 2.0),
        'capture.build_timeout_s': ('double', 180.0),
        'bind.switch_holdoff_s': ('double', 2.0),
        'view_filter.min_translation': ('double', 0.002),
        'view_filter.min_rotation_deg': ('double', 1.0),
        'icp.enable': ('bool', True),
        'icp.min_points': ('int', 300),
        'icp.coarse_voxel': ('double', 0.006),
        'icp.fine_voxel': ('double', 0.003),
        'icp.coarse_correspondence': ('double', 0.015),
        'icp.fine_correspondence': ('double', 0.007),
        'icp.coarse_iterations': ('int', 20),
        'icp.fine_iterations': ('int', 10),
        'icp.min_fitness': ('double', 0.35),
        'icp.max_rmse': ('double', 0.008),
        'icp.max_translation': ('double', 0.01),
        'icp.max_rotation_deg': ('double', 3.0),
        'icp.target_refresh_min_period': ('int', 1),
        'icp.target_refresh_max_period': ('int', 5),
        'icp.target_refresh_drift_ratio': ('double', 0.5),
        'local_volume.size_x': ('double', 0.3),
        'local_volume.size_y': ('double', 0.3),
        'local_volume.size_z': ('double', 0.4),
        'tsdf.enable': ('bool', True),
        'tsdf.voxel_length': ('double', 0.003),
        'tsdf.sdf_trunc': ('double', 0.012),
        'tsdf.depth_trunc': ('double', 1.5),
        'cloud_filter.voxel_size': ('double', 0.003),
        'cloud_filter.enable_statistical_filter': ('bool', True),
        'refit.enable': ('bool', True),
        'refit.cylinder_inlier_min': ('double', 0.35),
        'refit.rmse_max_m': ('double', 0.005),
        'refit.entry_standoff_m': ('double', 0.0),
        'refit.pregrasp_standoff_m': ('double', 0.0),
        'refit.max_axis_angle_deg': ('double', 35.0),
        'publish.on_change_only': ('bool', True),
        'publish.min_interval_s': ('double', 0.2),
        'refitter.cylinder_impl': ('string', 'cylinder_refit'),
        'refitter.sphere_impl': ('string', 'sphere_refit'),
        'session.root_dir': ('string', ''),
        # 工具档案：基础值=固定圆柱；整栈由 launch tool_profile 档案注入覆盖
        'tool.budget.d_inner': ('double', 0.104),
        'tool.profile_id': ('string', 'hollow_cylinder_v1'),
    }

    RULES = {  # 手写校验规则；启动期非法覆盖即抛，运行期非法 set 即拒
        'sync_slop_s': (('gt_eq', 0.0),),
        'tf_timeout_sec': (('gt_eq', 0.0),),
        'depth_scale_unit': (('gt', 0.0),),
        'capture.min_views': (('gt_eq', 1.0),),
        'capture.recommended_views': (('gt_eq', 1.0),),
        'capture.max_views': (('gt_eq', 1.0),),
        'capture.minimum_baseline_deg': (('gt_eq', 0.0),),
        'capture.minimum_mean_nearest_baseline_deg': (('gt_eq', 0.0),),
        'capture.minimum_mean_depth_ratio': (('bounds', 0.0, 1.0),),
        'capture.static_joint_vel_thresh': (('gt_eq', 0.0),),
        'capture.max_frame_age_s': (('gt', 0.0),),
        'capture.auto_min_interval_s': (('gt_eq', 0.0),),
        'capture.min_mask_pixels': (('gt_eq', 1.0),),
        'capture.min_mask_depth_ratio': (('bounds', 0.0, 1.0),),
        'capture.max_target_drift_m': (('gt_eq', 0.0),),
        'capture.neighbor_gap_area_ratio': (('gt_eq', 0.0),),
        'capture.build_timeout_s': (('gt', 0.0),),
        'bind.switch_holdoff_s': (('gt_eq', 0.0),),
        'view_filter.min_translation': (('gt_eq', 0.0),),
        'view_filter.min_rotation_deg': (('gt_eq', 0.0),),
        'icp.min_points': (('gt_eq', 1.0),),
        'icp.coarse_voxel': (('gt', 0.0),),
        'icp.fine_voxel': (('gt', 0.0),),
        'icp.coarse_correspondence': (('gt', 0.0),),
        'icp.fine_correspondence': (('gt', 0.0),),
        'icp.coarse_iterations': (('gt_eq', 1.0),),
        'icp.fine_iterations': (('gt_eq', 1.0),),
        'icp.min_fitness': (('bounds', 0.0, 1.0),),
        'icp.max_rmse': (('gt', 0.0),),
        'icp.max_translation': (('gt_eq', 0.0),),
        'icp.max_rotation_deg': (('gt_eq', 0.0),),
        'icp.target_refresh_min_period': (('gt_eq', 1.0),),
        'icp.target_refresh_max_period': (('gt_eq', 1.0),),
        'icp.target_refresh_drift_ratio': (('gt', 0.0),),
        'local_volume.size_x': (('gt', 0.0),),
        'local_volume.size_y': (('gt', 0.0),),
        'local_volume.size_z': (('gt', 0.0),),
        'tsdf.voxel_length': (('gt', 0.0),),
        'tsdf.sdf_trunc': (('gt', 0.0),),
        'tsdf.depth_trunc': (('gt', 0.0),),
        'cloud_filter.voxel_size': (('gt_eq', 0.0),),
        'refit.cylinder_inlier_min': (('bounds', 0.0, 1.0),),
        'refit.rmse_max_m': (('gt', 0.0),),
        'refit.entry_standoff_m': (('gt_eq', 0.0),),
        'refit.pregrasp_standoff_m': (('gt_eq', 0.0),),
        'refit.max_axis_angle_deg': (('gt', 0.0),),
        'publish.min_interval_s': (('gt_eq', 0.0),),
        'tool.budget.d_inner': (('bounds', 0.01, 0.5),),
    }

    class _Bind:
        """组 bind 的参数字段."""

        def __init__(self):
            self.switch_holdoff_s = 2.0

    class _Camera:
        """组 camera 的参数字段."""

        def __init__(self):
            self.color_topic = '/camera/color/image_raw'
            self.depth_topic = '/camera/depth/image_raw'
            self.camera_info_topic = '/camera/color/camera_info'

    class _Capture:
        """组 capture 的参数字段."""

        def __init__(self):
            self.min_views = 2
            self.recommended_views = 5
            self.max_views = 24
            self.minimum_baseline_deg = 8.0
            self.minimum_mean_nearest_baseline_deg = 6.0
            self.minimum_mean_depth_ratio = 0.4
            self.require_robot_static = True
            self.static_joint_vel_thresh = 0.03
            self.max_frame_age_s = 2.0
            self.auto_mode = True
            self.auto_finalize_at_max = False
            self.auto_min_interval_s = 0.0
            self.require_target_mask = True
            self.min_mask_pixels = 300
            self.min_mask_depth_ratio = 0.35
            self.max_target_drift_m = 0.04
            self.min_neighbor_gap_m = 0.15
            self.neighbor_gap_area_ratio = 2.0
            self.build_timeout_s = 180.0

    class _CloudFilter:
        """组 cloud_filter 的参数字段."""

        def __init__(self):
            self.voxel_size = 0.003
            self.enable_statistical_filter = True

    class _Frames:
        """组 frames 的参数字段."""

        def __init__(self):
            self.base_frame = 'base_link'

    class _Icp:
        """组 icp 的参数字段."""

        def __init__(self):
            self.enable = True
            self.min_points = 300
            self.coarse_voxel = 0.006
            self.fine_voxel = 0.003
            self.coarse_correspondence = 0.015
            self.fine_correspondence = 0.007
            self.coarse_iterations = 20
            self.fine_iterations = 10
            self.min_fitness = 0.35
            self.max_rmse = 0.008
            self.max_translation = 0.01
            self.max_rotation_deg = 3.0
            self.target_refresh_min_period = 1
            self.target_refresh_max_period = 5
            self.target_refresh_drift_ratio = 0.5

    class _LocalVolume:
        """组 local_volume 的参数字段."""

        def __init__(self):
            self.size_x = 0.3
            self.size_y = 0.3
            self.size_z = 0.4

    class _Publish:
        """组 publish 的参数字段."""

        def __init__(self):
            self.on_change_only = True
            self.min_interval_s = 0.2

    class _Refit:
        """组 refit 的参数字段."""

        def __init__(self):
            self.enable = True
            self.cylinder_inlier_min = 0.35
            self.rmse_max_m = 0.005
            self.entry_standoff_m = 0.0
            self.pregrasp_standoff_m = 0.0
            self.max_axis_angle_deg = 35.0

    class _Refitter:
        """组 refitter 的参数字段."""

        def __init__(self):
            self.cylinder_impl = 'cylinder_refit'
            self.sphere_impl = 'sphere_refit'

    class _Session:
        """组 session 的参数字段."""

        def __init__(self):
            self.root_dir = ''

    class _ToolBudget:
        """组 tool.budget 的参数字段（GraspDecision 许可数学的内径）."""

        def __init__(self):
            self.d_inner = 0.104

    class _Tool:
        """组 tool 的参数字段（当前末端工具档案）."""

        def __init__(self):
            self.budget = peach_target_reconstruction_node._ToolBudget()
            self.profile_id = 'hollow_cylinder_v1'

    class _Tsdf:
        """组 tsdf 的参数字段."""

        def __init__(self):
            self.enable = True
            self.voxel_length = 0.003
            self.sdf_trunc = 0.012
            self.depth_trunc = 1.5

    class _ViewFilter:
        """组 view_filter 的参数字段."""

        def __init__(self):
            self.min_translation = 0.002
            self.min_rotation_deg = 1.0

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            self.bind = peach_target_reconstruction_node._Bind()
            self.camera = peach_target_reconstruction_node._Camera()
            self.capture = peach_target_reconstruction_node._Capture()
            self.cloud_filter = peach_target_reconstruction_node._CloudFilter()
            self.frames = peach_target_reconstruction_node._Frames()
            self.icp = peach_target_reconstruction_node._Icp()
            self.local_volume = peach_target_reconstruction_node._LocalVolume()
            self.publish = peach_target_reconstruction_node._Publish()
            self.refit = peach_target_reconstruction_node._Refit()
            self.refitter = peach_target_reconstruction_node._Refitter()
            self.session = peach_target_reconstruction_node._Session()
            self.tool = peach_target_reconstruction_node._Tool()
            self.tsdf = peach_target_reconstruction_node._Tsdf()
            self.view_filter = peach_target_reconstruction_node._ViewFilter()
            for key, (_, value) in peach_target_reconstruction_node.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node, prefix=''):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = peach_target_reconstruction_node.Params()
            for key, (_, default) in peach_target_reconstruction_node.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in peach_target_reconstruction_node.DEFAULTS:
                _assign(self.params_, key, node.get_parameter(key).value)
            self._validate_all()
            node.add_on_set_parameters_callback(self._on_set)

        def get_params(self):
            """当前快照深拷贝（含 stamp_ 变更戳）."""
            return deepcopy(self.params_)

        def is_old(self, other):
            """快照 other 是否落后于内部值（运行路径据此刷新）."""
            return self._stamp != other.stamp_

        def set_user_callback(self, callback):
            """注册快照刷新后的用户回调."""
            self.user_callback = callback

        def clear_user_callback(self):
            """注销用户回调."""
            self.user_callback = None

        def refresh_dynamic_parameters(self):
            """把内部快照推给用户回调（兼容旧刷新语义）."""
            if self.user_callback:
                self.user_callback(self.get_params())

        def _validate_all(self):
            """启动期全量校验：yaml 覆盖值非法即抛异常终止启动."""
            for key, rules in peach_target_reconstruction_node.RULES.items():
                group, _, leaf = key.rpartition('.')
                # 逐段下钻嵌套组（与 _assign 一致，支持任意层级如 tool.budget.*）
                target = self.params_
                for part in group.split('.') if group else ():
                    target = getattr(target, part)
                value = getattr(target, leaf)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in peach_target_reconstruction_node.DEFAULTS:
                    continue
                for rule in peach_target_reconstruction_node.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in peach_target_reconstruction_node.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)
