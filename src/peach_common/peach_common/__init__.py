"""
peach_common：peach 参数 / QoS 设施单源（W1 起；角色对齐 Nav2 nav2_common）.

yaml_params（yaml 直读 + attach）、param_rules（三包规则并集）、qos（QoS
工厂）在此单源；各能力包保留旧 import 路径作为转发 shim。
"""
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
    ValidateFn,
)

__all__ = [
    'attach',
    'CommitFn',
    'dict_to_ns',
    'expand_share',
    'flatten_leaves',
    'leaf_keys',
    'load_ros_parameters',
    'package_yaml',
    'PreviewFn',
    'set_dotted',
    'ValidateFn',
]
