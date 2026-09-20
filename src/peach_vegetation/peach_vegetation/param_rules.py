"""W1 单源 shim：实现已迁 ``peach_common.param_rules``（三包并集），旧路径可用."""
from peach_common.param_rules import *  # noqa: F401,F403
from peach_common.param_rules import (
    check,
    check_enable_deps,
    check_min_max,
)

__all__ = ['check', 'check_enable_deps', 'check_min_max']
