"""零 ROS 参数规则校验（executor；与 perception 同语义，禁止跨包 import）."""

from __future__ import annotations


def check(rule, value, key):
    """
    Validate one handwritten rule; return None if legal.

    RULES values are (('gt', 0.0), ...); the loop already passes
    ``('gt', 0.0)`` as ``(kind, *args)`` and must not wrap it again.
    """
    if not rule:
        return None
    if isinstance(rule[0], str):
        kind, args = rule[0], rule[1:]
    elif (isinstance(rule[0], tuple) and rule[0]
          and isinstance(rule[0][0], str)):
        kind, args = rule[0][0], rule[0][1:]
    else:
        return None
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
    return None


def check_min_max(min_value, max_value, min_key, max_key):
    """跨字段：min ≤ max."""
    if float(min_value) > float(max_value):
        return f'{min_key} <= {max_key} required'
    return None


def check_enable_deps(execution_enabled, grasp_enabled, tool_enabled):
    """Tool enable requires grasp then execution."""
    if tool_enabled and not grasp_enabled:
        return 'tool_enabled requires grasp_enabled'
    if grasp_enabled and not execution_enabled:
        return 'grasp_enabled requires execution_enabled'
    return None
