"""
不进 lifecycle 名单节点的自转换 helper（W11：observability / vegetation 共用）.

独立 launch 的生命周期 EmitEvent 经常匹配不到名单外节点；这类节点在 spin
前用本函数自行 configure/activate 兜底。依赖 rclpy LifecycleNode 的
``_state_machine.current_state``（无公开访问器，集中一处私访问胜过各节点
复制粘贴）。
"""
from __future__ import annotations


def ensure_lifecycle_active(node) -> None:
    """把 LifecycleNode 从 unconfigured/inactive 自行推进到 active."""
    # rclpy 无当前状态公开访问器；集中这一处私访问胜过各节点复制粘贴
    label = node._state_machine.current_state[1]
    if label == 'unconfigured':
        node.trigger_configure()
        label = node._state_machine.current_state[1]
    if label == 'inactive':
        node.trigger_activate()
