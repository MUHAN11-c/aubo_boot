"""
peach_supervisor handwritten parameter module (decision 0017 SNAPSHOT).

键名冻结。`config/supervisor_contract.param.yaml` 是同一 schema 的 GPL 形状子集，
不是第三套生成器。Python 仍走本文件 ParamListener。

调度/监控/生命周期管理三个节点的声明/兜底默认/校验/快照装载集中于
本文件（各节点类持有自己的 DEFAULTS/RULES）；部署值与中文描述的事实源是
config/peach_supervisor.yaml、observability.yaml 与 lifecycle_manager.yaml
（nav2 式全量清单），launch 以 ParameterFile 装入。键名冻结；改默认值须
模块与 yaml 各改一处。接口与原 GPL 生成物一致：ParamListener(node)
.get_params()/is_old()。
"""

from __future__ import annotations

from copy import deepcopy

from peach_harvester.supervisor.param_rules import check as _check, check_min_max
from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidParameterValueException


def _assign(params, key, value):
    """按点号键把 value 写入快照（逐段下钻嵌套组）."""
    parts = key.split('.')
    target = params
    for part in parts[:-1]:
        target = getattr(target, part)
    setattr(target, parts[-1], value)


class peach_supervisor:
    """peach_supervisor 的参数声明/装载/校验器（手写，决策 0017；原 GPL 生成物等价物）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/<节点>.yaml 为准
        'execution_enabled': ('bool', False),
        'survey_wait_s': ('double', 15.0),
        'survey_dwell_s': ('double', 2.0),
        'service_timeout_s': ('double', 30.0),
        'action_timeout_s': ('double', 180.0),
        'empty_survey_limit': ('int', 2),
        'persist_ledger': ('bool', True),
        'reconstruction_min_views': ('int', 2),
        'build_start_timeout_s': ('double', 2.0),
        'observe_build_grace_s': ('double', 3.0),
        'begin_scene_service': ('string', '/peach_scene_perception_node/begin_scene'),
        'survey_scene_action': ('string', '/peach_arm/survey_scene'),
        'execute_target_action': ('string', '/peach_arm/execute_target'),
        'build_target_model_action': (
            'string', '/peach_target_reconstruction_node/build_target_model'),
        'execute_pregrasp_only': ('bool', True),
        'selection_reach_min_m': ('double', 0.15),
        'selection_reach_max_m': ('double', 0.88),
        'check_reachability_service': ('string', '/peach_arm/check_reachability'),
        'selection_depth_min_m': ('double', 0.3),
        'selection_depth_max_m': ('double', 1.6),
        'require_managed_stack': ('bool', False),
        # 工具档案标签（ExecuteTarget goal.tool_profile_id）；
        # 基础值=固定圆柱，整栈由 launch tool_profile 档案注入覆盖
        'tool.profile_id': ('string', 'hollow_cylinder_v1'),
    }

    RULES = {  # 手写校验规则；启动期非法覆盖即抛，运行期非法 set 即拒
        'survey_wait_s': (('gt_eq', 0.0),),
        'survey_dwell_s': (('gt_eq', 0.0),),
        'service_timeout_s': (('gt', 0.0),),
        'action_timeout_s': (('gt', 0.0),),
        'empty_survey_limit': (('gt_eq', 1.0),),
        'reconstruction_min_views': (('gt_eq', 1.0),),
        'build_start_timeout_s': (('gt', 0.0),),
        'observe_build_grace_s': (('gt_eq', 0.0),),
    }

    class _Tool:
        """组 tool 的参数字段（当前末端工具档案标签）."""

        def __init__(self):
            self.profile_id = 'hollow_cylinder_v1'

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            self.tool = peach_supervisor._Tool()
            for key, (_, value) in peach_supervisor.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node, prefix=''):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = peach_supervisor.Params()
            for key, (_, default) in peach_supervisor.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in peach_supervisor.DEFAULTS:
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
            for key, rules in peach_supervisor.RULES.items():
                group, _, leaf = key.rpartition('.')
                target = getattr(self.params_, group) if group else self.params_
                value = getattr(target, leaf)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)
            why = check_min_max(
                self.params_.selection_reach_min_m,
                self.params_.selection_reach_max_m,
                'selection_reach_min_m', 'selection_reach_max_m')
            if why:
                raise InvalidParameterValueException(
                    'selection_reach_min_m',
                    self.params_.selection_reach_min_m, why)
            why = check_min_max(
                self.params_.selection_depth_min_m,
                self.params_.selection_depth_max_m,
                'selection_depth_min_m', 'selection_depth_max_m')
            if why:
                raise InvalidParameterValueException(
                    'selection_depth_min_m',
                    self.params_.selection_depth_min_m, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in peach_supervisor.DEFAULTS:
                    continue
                for rule in peach_supervisor.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in peach_supervisor.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)


class peach_observability:
    """peach_observability 的参数声明/装载/校验器（手写，决策 0017；原 GPL 生成物等价物）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/<节点>.yaml 为准
        'host': ('string', '127.0.0.1'),
        'port': ('int', 8090),
        'param_poll_period_s': ('double', 3.0),
        'target_observations_topic': ('string', '/peach/perception/target_observations'),
        'harvest_state_topic': ('string', '/peach/perception/harvest_state'),
        'reconstruction_status_topic': ('string', '/peach/reconstruction/status'),
        'reconstruction_diagnostics_topic': ('string', '/peach/reconstruction/diagnostics'),
        'reconstruction_diagnostics_debug_topic': (
            'string', '/peach/reconstruction/diagnostics_debug'),
        'grasp_decision_topic': ('string', '/peach/reconstruction/grasp_decision'),
        'refined_pose_topic': ('string', '/peach/reconstruction/refined_pose'),
        'refined_axis_topic': ('string', '/peach/reconstruction/refined_axis'),
        'refined_diagnostics_topic': ('string', '/peach/reconstruction/refined_diagnostics'),
        'manipulation_status_topic': ('string', '/peach_arm/status'),
        'grasp_hypothesis_topic': ('string', '/peach/manipulation/grasp_hypothesis'),
        'task_executor_state_topic': ('string', '/peach_supervisor/state'),
        'task_executor_events_topic': ('string', '/peach_supervisor/events'),
        'robot_status_topic': ('string', '/aubo_io_controller/robot_status'),
        'joint_states_topic': ('string', '/joint_states'),
        'joint_status_topic': ('string', '/aubo_io_controller/joint_status'),
        'tf_topic': ('string', '/tf'),
        'tf_static_topic': ('string', '/tf_static'),
        'scene_snapshot_topic': ('string', '/peach_supervisor/scene_snapshot'),
        'job_topic': ('string', '/peach/observability/job'),
        'metrics_topic': ('string', '/peach/observability/metrics'),
        'event_buffer_size': ('int', 100),
        'metrics_period_s': ('double', 1.0),
        'metrics_process_patterns': (
            'string_array', [
                'peach_scene_perception_node', 'peach_target_reconstruction_node',
                'peach_arm', 'peach_supervisor', 'ros2_control_node', 'percipio',
            ]),
        'record.enabled': ('bool', True),
        'record.root_dir': ('string', ''),
        'record.save_images': ('bool', True),
        'record.save_clouds': ('bool', True),
        # /rosout 进 bag：全部节点日志（stamp/level/logger）可回放——测试
        # 复盘的最低完备集（否则只剩 launch 终端输出，随会话丢失）
        'record.rosout': ('bool', True),
        # 全量录制档位：all=域内全部话题自动发现订阅、大流不限速（真全量，
        # 仿真复现/问题分析用，bag 体积大）；std=全话题但相机 raw 大流限
        # 1Hz（默认）；core=仅既定别名表（旧行为）
        'record.level': ('string', 'std'),
        'record.max_total_bag_gb': ('double', 20.0),
        # 会话 bag 录制别名清单（别名→话题/类型见 observability/bag_reader.py 注册表）；
        # debug_image/debug_image_raw/tsdf_cloud 另受 save_images/save_clouds 门控
        'record.bag_topics': ('string_array', [
            'events', 'state', 'scene_snapshot',
            'target_observations', 'harvest_state',
            'recon_status', 'recon_diagnostics', 'recon_debug',
            'grasp_decision', 'refined_pose', 'refined_axis',
            'refined_diagnostics',
            'manipulation_status', 'grasp_hypothesis',
            'tf', 'tf_static', 'joint_states', 'robot_status', 'joint_status',
            'job', 'metrics',
            'debug_image', 'debug_image_raw', 'tsdf_cloud',
        ]),
        'trajectory.enabled': ('bool', True),
        'trajectory.base_frame': ('string', 'base_link'),
        'trajectory.tip_frame': ('string', 'tcp'),
        'trajectory.period_s': ('double', 0.05),
        'trajectory.min_step_m': ('double', 0.003),
        'trajectory.max_points': ('int', 8000),
        'debug_image_topic': ('string', '/peach/perception/debug_image'),
        'debug_image_raw_topic': ('string', '/peach/perception/debug_image_raw'),
        'tsdf_cloud_topic': ('string', '/peach/reconstruction/tsdf_cloud'),
        'tcp_path_topic': ('string', '/peach/observability/tcp_path'),
        'tcp_markers_topic': ('string', '/peach/observability/markers'),
        'debug.enabled': ('bool', True),
        'debug.motion_enabled': ('bool', False),
        'debug.token': ('string', ''),
        'debug.action_timeout_s': ('double', 180.0),
        'debug.audit_enabled': ('bool', True),
        'debug.endpoints.run_harvest_action': ('string', '/peach_supervisor/run_harvest'),
        'debug.endpoints.control_service': ('string', '/peach_supervisor/control'),
        'debug.endpoints.begin_scene_service': (
            'string', '/peach_scene_perception_node/begin_scene'),
        'debug.endpoints.survey_action': ('string', '/peach_arm/survey_scene'),
        'debug.endpoints.execute_action': ('string', '/peach_arm/execute_target'),
        'debug.endpoints.build_action': (
            'string', '/peach_target_reconstruction_node/build_target_model'),
        'debug.endpoints.check_reachability_service': (
            'string', '/peach_arm/check_reachability'),
        'debug.endpoints.photo_pose_service': (
            'string', '/peach_arm/go_to_photo_pose'),
        'debug.endpoints.preview_approach_service': (
            'string', '/peach_arm/preview_approach_insert'),
        'debug.endpoints.preview_full_service': (
            'string', '/peach_arm/preview_full_contact'),
        'debug.endpoints.ack_recovery_service': (
            'string', '/peach_arm/acknowledge_recovery'),
        'debug.endpoints.arm_service': ('string', '/peach_arm/set_execution_armed'),
        'debug.endpoints.skill_cancel_service': (
            'string', '/peach_arm/cancel_cycle'),
        'debug.endpoints.recon_save_session_service': (
            'string', '/peach_target_reconstruction_node/save_session'),
        'debug.endpoints.recon_reset_service': (
            'string', '/peach_target_reconstruction_node/reset_reconstruction'),
        'debug.endpoints.recon_finalize_service': (
            'string', '/peach_target_reconstruction_node/finalize_reconstruction'),
        'debug.endpoints.recon_query_service': (
            'string', '/peach_target_reconstruction_node/query_reconstruction_state'),
        'debug.endpoints.manage_nodes_service': (
            'string', '/peach_lifecycle_manager/manage_nodes'),
    }

    RULES = {  # 手写校验规则；启动期非法覆盖即抛，运行期非法 set 即拒
        'port': (('bounds', 1.0, 65535.0),),
        'param_poll_period_s': (('gt', 0.0),),
        'event_buffer_size': (('gt_eq', 1.0),),
        'metrics_period_s': (('gt', 0.0),),
        'trajectory.period_s': (('gt', 0.0),),
        'trajectory.min_step_m': (('gt', 0.0),),
        'trajectory.max_points': (('gt_eq', 100.0),),
        'record.max_total_bag_gb': (('gt_eq', 0.0),),
        'debug.action_timeout_s': (('gt', 0.0),),
    }

    class _Debug:
        """组 debug 的参数字段."""

        def __init__(self):
            self.endpoints = peach_observability._DebugEndpoints()
            self.enabled = True
            self.motion_enabled = False
            self.token = ''
            self.action_timeout_s = 180.0
            self.audit_enabled = True

    class _DebugEndpoints:
        """组 debug.endpoints 的参数字段."""

        def __init__(self):
            self.run_harvest_action = '/peach_supervisor/run_harvest'
            self.control_service = '/peach_supervisor/control'
            self.begin_scene_service = '/peach_scene_perception_node/begin_scene'
            self.survey_action = '/peach_arm/survey_scene'
            self.execute_action = '/peach_arm/execute_target'
            self.build_action = '/peach_target_reconstruction_node/build_target_model'
            self.check_reachability_service = '/peach_arm/check_reachability'
            self.photo_pose_service = '/peach_arm/go_to_photo_pose'
            self.preview_approach_service = '/peach_arm/preview_approach_insert'
            self.preview_full_service = '/peach_arm/preview_full_contact'
            self.ack_recovery_service = '/peach_arm/acknowledge_recovery'
            self.arm_service = '/peach_arm/set_execution_armed'
            self.skill_cancel_service = '/peach_arm/cancel_cycle'
            self.recon_save_session_service = '/peach_target_reconstruction_node/save_session'
            self.recon_reset_service = '/peach_target_reconstruction_node/reset_reconstruction'
            self.recon_finalize_service = (
                '/peach_target_reconstruction_node/finalize_reconstruction')
            self.recon_query_service = (
                '/peach_target_reconstruction_node/query_reconstruction_state')
            self.manage_nodes_service = '/peach_lifecycle_manager/manage_nodes'

    class _Record:
        """组 record 的参数字段."""

        def __init__(self):
            self.enabled = True
            self.root_dir = ''
            self.save_images = True
            self.save_clouds = True
            self.rosout = True
            self.level = 'std'
            self.max_total_bag_gb = 20.0
            self.bag_topics = [
                'events', 'state', 'scene_snapshot',
                'target_observations', 'harvest_state',
                'recon_status', 'recon_diagnostics', 'recon_debug',
                'grasp_decision', 'refined_pose', 'refined_axis',
                'refined_diagnostics',
                'manipulation_status', 'grasp_hypothesis',
                'tf', 'tf_static', 'joint_states', 'robot_status',
                'joint_status',
                'job', 'metrics',
                'debug_image', 'debug_image_raw', 'tsdf_cloud',
            ]

    class _Trajectory:
        """组 trajectory 的参数字段."""

        def __init__(self):
            self.enabled = True
            self.base_frame = 'base_link'
            self.tip_frame = 'tcp'
            self.period_s = 0.05
            self.min_step_m = 0.003
            self.max_points = 8000

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            self.debug = peach_observability._Debug()
            self.debug.endpoints = peach_observability._DebugEndpoints()
            self.record = peach_observability._Record()
            self.trajectory = peach_observability._Trajectory()
            for key, (_, value) in peach_observability.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node, prefix=''):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = peach_observability.Params()
            for key, (_, default) in peach_observability.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in peach_observability.DEFAULTS:
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
            for key, rules in peach_observability.RULES.items():
                group, _, leaf = key.rpartition('.')
                target = getattr(self.params_, group) if group else self.params_
                value = getattr(target, leaf)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in peach_observability.DEFAULTS:
                    continue
                for rule in peach_observability.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in peach_observability.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)


class peach_lifecycle_manager:
    """peach_lifecycle_manager 的参数声明/装载/校验器（手写，决策 0017；原 GPL 生成物等价物）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/<节点>.yaml 为准
        'node_names': (
            'string_array', [
                'peach_scene_perception_node', 'peach_target_reconstruction_node',
                'peach_arm', 'peach_supervisor',
            ]),
        'startup_timeout_s': ('double', 60.0),
    }

    RULES = {  # 手写校验规则；启动期非法覆盖即抛，运行期非法 set 即拒
        'startup_timeout_s': (('gt', 0.0),),
    }

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            for key, (_, value) in peach_lifecycle_manager.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node, prefix=''):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = peach_lifecycle_manager.Params()
            for key, (_, default) in peach_lifecycle_manager.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in peach_lifecycle_manager.DEFAULTS:
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
            for key, rules in peach_lifecycle_manager.RULES.items():
                group, _, leaf = key.rpartition('.')
                target = getattr(self.params_, group) if group else self.params_
                value = getattr(target, leaf)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in peach_lifecycle_manager.DEFAULTS:
                    continue
                for rule in peach_lifecycle_manager.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in peach_lifecycle_manager.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)
