"""peach_common.lifecycle 自转换 helper 的单测（零 ROS，stub 节点）."""
from __future__ import annotations

from types import SimpleNamespace

from peach_common.lifecycle import ensure_lifecycle_active


def _stub_node(label: str) -> SimpleNamespace:
    calls = []

    def _transition(new_label):
        def trigger():
            calls.append(new_label)
            machine.current_state = ('x', new_label)
        return trigger

    machine = SimpleNamespace(current_state=('x', label))
    return SimpleNamespace(
        _state_machine=machine,
        trigger_configure=_transition('inactive'),
        trigger_activate=_transition('active'),
        _calls=calls,
    )


def test_transitions_from_unconfigured():
    """unconfigured 起步：先 configure 再 activate."""
    node = _stub_node('unconfigured')
    ensure_lifecycle_active(node)
    assert node._calls == ['inactive', 'active']


def test_transitions_from_inactive():
    """inactive 起步：只 activate."""
    node = _stub_node('inactive')
    ensure_lifecycle_active(node)
    assert node._calls == ['active']


def test_noop_when_active():
    """已 active：不重复触发."""
    node = _stub_node('active')
    ensure_lifecycle_active(node)
    assert node._calls == []
