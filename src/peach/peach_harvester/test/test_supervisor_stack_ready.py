"""托管栈就绪变化必须刷新闩锁状态，供启动检查解除阻塞."""
from types import SimpleNamespace

from peach_harvester.supervisor.executor_node import TaskExecutorNode
from peach_harvester.supervisor.harvest_fsm import (
    blockers_for, DISCOVERY,
)
from std_msgs.msg import Bool


def test_stack_ready_change_publishes_blockers_without_changing_fsm():
    """就绪上升/下降沿立即发布；相同心跳不重复发布或迁移批次状态."""
    published = []
    node = SimpleNamespace(_stack_ready=False, _batch_state=DISCOVERY)

    def publish(*, force=False):
        published.append((force, blockers_for(
            operation_mode=0, recovery_required=False,
            stack_ready=node._stack_ready, ledger_write_failures=0)))

    node._publish_state = publish
    for ready in (False, True, True, False, False):
        TaskExecutorNode._on_stack_ready(node, Bool(data=ready))
    assert published == [(True, []), (True, ['stack_not_ready'])]
    assert node._batch_state == DISCOVERY
