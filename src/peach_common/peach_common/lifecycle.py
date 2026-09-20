"""
不进 lifecycle 名单节点的自转换 helper（W11：observability / vegetation 共用）.

独立 launch 的生命周期 EmitEvent 经常匹配不到名单外节点；这类节点在 spin
前用本函数自行 configure/activate 兜底。依赖 rclpy LifecycleNode 的
``_state_machine.current_state``（无公开访问器，集中一处私访问胜过各节点
复制粘贴）。

W14 起另提供 ``create_bond``：托管节点在 on_configure 建心跳、拆除路径
断开。Python 侧依赖 ``ros-jazzy-bondpy``（本机未装时守卫降级并 WARN 一次，
装上即生效，无需改代码）。
"""
from __future__ import annotations

BOND_TOPIC = '/bond'


def ensure_lifecycle_active(node) -> None:
    """把 LifecycleNode 从 unconfigured/inactive 自行推进到 active."""
    # rclpy 无当前状态公开访问器；集中这一处私访问胜过各节点复制粘贴
    label = node._state_machine.current_state[1]
    if label == 'unconfigured':
        node.trigger_configure()
        label = node._state_machine.current_state[1]
    if label == 'inactive':
        node.trigger_activate()


def create_bond(node, bond_id: str, topic: str = BOND_TOPIC):
    """
    建进程存活心跳（bond 协议客户端半边）；返回 bond 或 None.

    on_configure 成功路径调用；on_deactivate / cleanup / shutdown 调
    ``break_bond()``。缺 ros-jazzy-bondpy 时守卫降级：WARN 一次并返回
    None（节点照常工作，仅 nav2_lm 的 bond 监测收不到该节点心跳）。
    bondpy 的心跳由其内部守护线程发送，收包依赖节点被 executor spin。
    """
    try:
        from bondpy import Bond
    except ImportError:
        node.get_logger().warning(
            'bond 未生效：缺 ros-jazzy-bondpy（sudo apt install 后重启'
            '节点即接入 nav2_lm 心跳监测）')
        return None
    bond = Bond(node, topic, bond_id)
    bond.start()
    return bond


def break_bond(bond) -> None:
    """拆除 ``create_bond`` 返回的心跳（None 安全；断开后关闭收发）."""
    if bond is None:
        return
    try:
        bond.break_bond()
        bond.shutdown()
    except Exception:  # noqa: BLE001 拆除失败不阻塞生命周期迁移
        pass
