"""
peach_bringup: yaml 直读 + 一行 attach.

部署事实源 ``config/bringup.yaml``（两节点块）。节点一行
``attach_flag_bridge(node)`` / ``attach_autostart_client(node)``：声明叶子
并挂规则校验（启动期非法即拒启、运行期非法 set 即拒），``ros2 param set``
原地刷新。
"""
from __future__ import annotations

from peach_bringup.yaml_params import attach, package_yaml

_POSITIVE = {  # 键 -> 下限（须严格大于）
    'poll_hz': 0.0,
    'wait_stack_timeout_s': 0.0,
}


def _validate(name, value):
    """数值下限校验；返回拒绝理由或 None."""
    if name in _POSITIVE and not value > _POSITIVE[name]:
        return f'{name} must be > {_POSITIVE[name]}'
    return None


class FlagBridgeParams:
    """peach_lifecycle_flag_bridge 参数."""

    is_active_service: str
    """查询托管节点是否 Active 的服务名."""
    poll_hz: float
    """轮询频率 [Hz]；须 >0."""


class AutostartClientParams:
    """peach_autostart_client 参数."""

    scene_key: str
    """BeginScene / RunHarvest 场景键."""
    intent: str
    """autostart 意图（部署参数，默认关）."""
    wait_stack_timeout_s: float
    """等栈 Active 超时 [s]；须 >0."""


def attach_flag_bridge(node) -> FlagBridgeParams:
    """peach_lifecycle_flag_bridge：is_active_service / poll_hz."""
    return attach(
        node, package_yaml('peach_bringup', 'bringup.yaml'),
        node_name='peach_lifecycle_flag_bridge', validate=_validate)


def attach_autostart_client(node) -> AutostartClientParams:
    """peach_autostart_client：scene_key / intent / wait_stack_timeout_s."""
    return attach(
        node, package_yaml('peach_bringup', 'bringup.yaml'),
        node_name='peach_autostart_client', validate=_validate)
