"""
Load tool profile archives from aubo_description (plain keys, not ROS ParameterFile format).

工具档案单一事实源：aubo_description/config/<profile_id>.yaml。
launch 期按 tool_profile 参数装载注入各包（模式同 grasp_standoffs 的 overlay 注入）：
  - scene:        tool.D_inner（径向走廊门）
  - reconstruction: tool.budget.d_inner（GraspDecision 许可数学）+ tool.profile_id（标签）
  - manipulation / executor: tool.profile_id（标签）
档案里其余字段（error_m、io 等登记项）运行期不注入。

新增工具：aubo_description 加档案 yaml + tcp_<profile>.xacro wrapper + 主 xacro
一个 if 块；本模块按 profile_id 动态读文件，无需改动。
launch/ament 依赖全部延迟到函数内：parse_tool_profile 可零 ROS 纯核测试。
"""

from __future__ import annotations

import os

import yaml

# 运行期有消费的档案字段（缺失即档案不完整，launch 期抛错）
_REQUIRED_FIELDS = ('profile_id', 'd_inner')


def parse_tool_profile(data, profile_id):
    """
    Validate archive dict and extract consumed fields (pure core, no ROS).

    Returns {'profile_id': str, 'd_inner': float}；档案缺字段或 profile_id
    与文件名不一致时抛 ValueError（launch 期 fail-fast，纯核测试同款入口）。
    """
    if not isinstance(data, dict):
        raise ValueError(f'tool profile archive must be a mapping, got {type(data)}')
    declared = data.get('profile_id')
    if declared != profile_id:
        raise ValueError(
            f'profile_id mismatch: asked {profile_id!r}, archive declares {declared!r}')
    geometry = data.get('geometry_m') or {}
    try:
        d_inner = float(geometry['D_inner'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'tool profile {profile_id!r}: geometry_m.D_inner missing/invalid') from exc
    if not 0.0 < d_inner < 0.5:
        raise ValueError(f'tool profile {profile_id!r}: D_inner {d_inner} out of range')
    return {'profile_id': declared, 'd_inner': d_inner}


_CACHE = {}


def load_tool_profile(profile_id: str):
    """Read aubo_description/config/<profile_id>.yaml and return parsed fields."""
    from ament_index_python.packages import get_package_share_directory
    if profile_id in _CACHE:
        return _CACHE[profile_id]
    path = os.path.join(
        get_package_share_directory('aubo_description'),
        'config', f'{profile_id}.yaml')
    if not os.path.isfile(path):
        raise RuntimeError(
            f'unknown tool_profile {profile_id!r}: no archive at {path} '
            f'(known: hollow_cylinder_v1, adaptive_cylinder_v1)')
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    parsed = parse_tool_profile(data, profile_id)
    _CACHE[profile_id] = parsed
    return parsed


def _tool_profile_value_cls():
    """构造（并缓存）launch 替换对象类；launch 仅在 launch 期被导入."""
    cls = _tool_profile_value_cls.cache
    if cls is None:
        from launch.substitution import Substitution

        class ToolProfileValue(Substitution):
            """
            按 tool_profile 参数值取档案字段（求值期装载、fail-fast）.

            ToolProfileValue(arg, 'd_inner') → float；'profile_id' → str。
            """

            def __init__(self, profile_id, field):
                self._profile_id = profile_id
                self._field = field

            def perform(self, context):
                # perform 必须返回 str（launch 对替换结果做 ''.join）；
                # 数值由 ParameterValue(value_type=float) 负责转回
                parsed = load_tool_profile(self._profile_id.perform(context))
                if self._field not in _REQUIRED_FIELDS:
                    raise RuntimeError(
                        f'ToolProfileValue: field {self._field!r} '
                        f'not in {_REQUIRED_FIELDS}')
                return str(parsed[self._field])

            def describe(self):
                return ('ToolProfileValue('
                        f'{self._profile_id.describe()}, {self._field!r})')

        cls = ToolProfileValue
        _tool_profile_value_cls.cache = cls
    return cls


_tool_profile_value_cls.cache = None


def scene_tool_params(profile_id):
    """场景感知注入：径向走廊门的工具内径."""
    from launch_ros.parameter_descriptions import ParameterValue
    value = _tool_profile_value_cls()(profile_id, 'd_inner')
    return {'tool.D_inner': ParameterValue(value, value_type=float)}


def reconstruction_tool_params(profile_id):
    """重建注入：许可数学内径 + 档案标签."""
    from launch_ros.parameter_descriptions import ParameterValue
    cls = _tool_profile_value_cls()
    return {
        'tool.budget.d_inner': ParameterValue(
            cls(profile_id, 'd_inner'), value_type=float),
        'tool.profile_id': ParameterValue(
            cls(profile_id, 'profile_id'), value_type=str),
    }


def tool_profile_id_params(profile_id):
    """标签注入（manipulation / executor）：当前工具档案名."""
    from launch_ros.parameter_descriptions import ParameterValue
    value = _tool_profile_value_cls()(profile_id, 'profile_id')
    return {'tool.profile_id': ParameterValue(value, value_type=str)}
