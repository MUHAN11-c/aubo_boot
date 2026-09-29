"""
imu_follow 手写参数模块（决策 0017 口径，与 peach 各包 params 模块同款写法）.

声明/兜底默认/校验/快照集中于本文件；部署值与中文描述的事实源是
config/imu_follow.yaml（nav2 式全量清单），launch 以 ParameterFile 装入。
键名冻结；改默认值须本模块与 yaml 各改一处。接口与 peach 生成物一致：
ParamListener(node).get_params()/is_old()。
"""

from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import InvalidParameterValueException

JOINT_NAMES = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


def _assign(target_obj, key, value):
    """按点号键写入快照（中间组缺失即建 SimpleNamespace）."""
    parts = key.split('.')
    target = target_obj
    for part in parts[:-1]:
        nxt = getattr(target, part, None)
        if nxt is None:
            nxt = SimpleNamespace()
            setattr(target, part, nxt)
        target = nxt
    setattr(target, parts[-1], value)


def _fetch(target_obj, key):
    """按点号键读快照值."""
    parts = key.split('.')
    target = target_obj
    for part in parts[:-1]:
        target = getattr(target, part)
    return getattr(target, parts[-1])


def _check(rule, value, key):
    """单条手写校验规则；合法返回 None，非法返回原因."""
    if rule and isinstance(rule[0], str):
        rule = (rule,)  # 单条规则写成 (kind, ...) 时自动包装
    kind, args = rule[0], rule[1:]
    if kind == 'gt' and not value > args[0]:
        return f'{key}: 须 > {args[0]}'
    if kind == 'gt_eq' and not value >= args[0]:
        return f'{key}: 须 >= {args[0]}'
    if kind == 'lt' and not value < args[0]:
        return f'{key}: 须 < {args[0]}'
    if kind == 'lt_eq' and not value <= args[0]:
        return f'{key}: 须 <= {args[0]}'
    if kind == 'bounds' and not args[0] <= value <= args[1]:
        return f'{key}: 须在 [{args[0]}, {args[1]}] 内'
    if kind == 'in' and value not in args[0]:
        return f'{key}: 须是 {args[0]} 之一'
    if kind == 'len' and not len(value) == args[0]:
        return f'{key}: 须含 {args[0]} 个关节'
    if kind == 'unique' and not len(set(value)) == len(value):
        return f'{key}: 关节名不得重复'
    return None


class imu_follow:
    """imu_follow 节点的参数声明/装载/校验器（手写，决策 0017 口径）."""

    DEFAULTS = {  # 点号键 -> (类型, 兜底默认)；部署值以 config/imu_follow.yaml 为准
        'motion.enabled': ('bool', False),
        'motion.backend': ('string', 'servo'),
        'rate.update_hz': ('double', 20.0),
        'topics.imu_topic': ('string', '/imu/data'),
        'topics.joint_states_topic': ('string', '/joint_states'),
        'frames.base_frame': ('string', 'base_link'),
        'frames.tip_frame': ('string', 'tcp'),
        'joints.joint_names': ('string_array', list(JOINT_NAMES)),
        'moveit.group': ('string', 'manipulator_e5'),
        'moveit.compute_ik_service': ('string', '/compute_ik'),
        'servo.twist_topic': ('string', '/moveit_servo/delta_twist_cmds'),
        'servo.command_type_service': (
            'string', '/moveit_servo/switch_command_type'),
        'servo.pause_service': ('string', '/moveit_servo/pause_servo'),
        'servo.orientation_gain': ('double', 4.0),
        'servo.position_gain': ('double', 2.0),
        'servo.position_deadband_m': ('double', 0.005),
        'execution.follow_joint_trajectory_action': (
            'string', '/joint_trajectory_controller/follow_joint_trajectory'),
        'execution.horizon_s': ('double', 0.15),
        'execution.max_joint_step_rad': ('double', 0.05),
        'execution.max_omega_rad_s': ('double', 0.5),
        'execution.max_lin_vel_m_s': ('double', 0.05),
        'follow.deadband_rad': ('double', 0.02),
        'follow.max_delta_rad': ('double', 0.35),
        'follow.smoothing_alpha': ('double', 0.3),
        'follow.invert_roll': ('bool', False),
        'follow.invert_pitch': ('bool', False),
        'follow.invert_yaw': ('bool', False),
        'insert.speed_m_s': ('double', 0.01),
        'insert.max_travel_m': ('double', 0.20),  # 兜底=旧圆柱行程；事实源=部署 yaml 0.09（=档案 L_insert）
        'safety.imu_timeout_s': ('double', 0.5),
        'safety.joint_states_timeout_s': ('double', 1.0),
        'safety.max_ik_failures': ('int', 10),
    }

    RULES = {  # 手写校验；启动期非法覆盖即抛，运行期非法 set 即拒
        'motion.backend': (('in', ('servo', 'fjt')),),
        'rate.update_hz': (('gt', 0.0), ('lt_eq', 100.0)),
        'servo.orientation_gain': (('gt', 0.0),),
        'servo.position_gain': (('gt', 0.0),),
        'servo.position_deadband_m': (('gt_eq', 0.0), ('lt', 0.05)),
        'execution.horizon_s': (('gt', 0.0), ('lt_eq', 1.0)),
        'execution.max_joint_step_rad': (('gt', 0.0), ('lt_eq', 0.5)),
        'execution.max_omega_rad_s': (('gt', 0.0), ('lt_eq', 2.0)),
        'execution.max_lin_vel_m_s': (('gt', 0.0), ('lt_eq', 0.5)),
        'follow.deadband_rad': (('gt_eq', 0.0), ('lt', 0.5)),
        'follow.max_delta_rad': (('gt', 0.0), ('lt_eq', 1.0)),
        'follow.smoothing_alpha': (('gt', 0.0), ('lt_eq', 1.0)),
        'insert.speed_m_s': (('gt', 0.0), ('lt_eq', 0.05)),
        'insert.max_travel_m': (('gt', 0.0), ('lt_eq', 0.5)),
        'safety.imu_timeout_s': (('gt', 0.0),),
        'safety.joint_states_timeout_s': (('gt', 0.0),),
        'safety.max_ik_failures': (('gt_eq', 1),),
        'joints.joint_names': (('len', 6), ('unique',)),
    }

    class Params:
        """参数快照：扁平键为属性，组为嵌套对象；stamp_ 为变更戳."""

        def __init__(self):
            self.stamp_ = 0
            for key, (_, value) in imu_follow.DEFAULTS.items():
                _assign(self, key, value)

    class ParamListener:
        """构造即声明+启动校验；on-set 校验拒绝非法值并刷新快照."""

        def __init__(self, node):
            """声明全部参数（yaml 覆盖值随后生效）并挂 on-set 回调."""
            self.node_ = node
            self.user_callback = None
            self._stamp = 0
            self.params_ = imu_follow.Params()
            for key, (_, default) in imu_follow.DEFAULTS.items():
                node.declare_parameter(key, default)
            for key in imu_follow.DEFAULTS:
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
            for key, rules in imu_follow.RULES.items():
                value = _fetch(self.params_, key)
                for rule in rules:
                    why = _check(rule, value, key)
                    if why:
                        raise InvalidParameterValueException(key, value, why)

        def _on_set(self, parameters):
            """运行期校验：全批合法才提交快照并递增变更戳."""
            for p in parameters:
                if p.name not in imu_follow.DEFAULTS:
                    continue
                for rule in imu_follow.RULES.get(p.name, ()):
                    why = _check(rule, p.value, p.name)
                    if why:
                        return SetParametersResult(successful=False, reason=why)
            for p in parameters:
                if p.name in imu_follow.DEFAULTS:
                    _assign(self.params_, p.name, p.value)
            self._stamp += 1
            self.params_.stamp_ = self._stamp
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)
