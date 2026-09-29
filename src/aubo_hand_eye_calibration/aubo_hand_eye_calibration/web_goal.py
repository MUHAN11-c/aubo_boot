"""Web gateway goal parameter validation (pure core, no rclpy imports)."""

from .solver import VALID_METHODS

VALID_POSE_SOURCES = ('poses', 'auto')
VALID_SOLVE_TARGETS = ('hand_eye', 'joint')


def _normalize(value, choices, label):
    """空串/None → '' (回退服务端参数默认); 非法值抛 ValueError."""
    text = '' if value is None else str(value).strip()
    if text and text not in choices:
        raise ValueError(
            f'非法的{label}: {text}（可选: {"/".join(choices)}）')
    return text


def normalize_method(value):
    return _normalize(value, VALID_METHODS, '求解方法')


def normalize_pose_source(value):
    return _normalize(value, VALID_POSE_SOURCES, '位姿来源')


def normalize_solve_target(value):
    return _normalize(value, VALID_SOLVE_TARGETS, '求解目标')


def goal_params_from_body(body):
    """
    从 web POST body 提取并校验 goal 档位参数.

    返回 {'method','pose_source','solve_target'}, 空串 = 服务端参数默认
    (solver_method/pose_source/solve_target); 非法值抛 ValueError,
    由 HTTP 层转 400。body 非 dict 视为空。
    """
    body = body if isinstance(body, dict) else {}
    return {
        'method': normalize_method(body.get('method', '')),
        'pose_source': normalize_pose_source(body.get('pose_source', '')),
        'solve_target': normalize_solve_target(body.get('solve_target', '')),
    }
