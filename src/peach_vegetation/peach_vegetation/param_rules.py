"""零 ROS 参数规则校验（与 peach_harvester param_rules 同语义，本包自持）."""

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
    if kind == 'one_of' and value not in args[0]:
        return f'{key}: 须为 {list(args[0])} 之一'
    if kind == 'seq_gt':
        if not value:
            return f'{key}: 不得为空'
        for item in value:
            if not item > args[0]:
                return f'{key}: 每项须 > {args[0]}'
        return None
    return None


def check_min_max(min_value, max_value, min_key, max_key):
    """跨字段：min ≤ max."""
    if float(min_value) > float(max_value):
        return f'{min_key} <= {max_key} required'
    return None
