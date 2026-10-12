"""
peach_common：peach 参数 / QoS 设施单源（W1 起；角色对齐 Nav2 nav2_common）.

yaml_params（yaml 直读 + attach）、param_rules（三包规则并集）、qos（QoS
工厂）、lifecycle（名单外节点自转换）在此单源；各能力包直接导入本库，
原包内参数转发模块已退役（键名与行为不变）。
"""
from peach_common.lifecycle import break_bond, create_bond, ensure_lifecycle_active
from peach_common.yaml_params import (
    attach,
    CommitFn,
    dict_to_ns,
    expand_share,
    flatten_leaves,
    leaf_keys,
    load_ros_parameters,
    package_yaml,
    PreviewFn,
    set_dotted,
    snapshot,
    ValidateFn,
)

__all__ = [
    'attach',
    'break_bond',
    'CommitFn',
    'create_bond',
    'dict_to_ns',
    'ensure_lifecycle_active',
    'expand_share',
    'flatten_leaves',
    'leaf_keys',
    'load_ros_parameters',
    'package_yaml',
    'PreviewFn',
    'set_dotted',
    'snapshot',
    'ValidateFn',
]
