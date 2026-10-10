"""peach_common.lifecycle 自转换 helper 的单测（零 ROS，stub 节点）."""
from __future__ import annotations

from types import SimpleNamespace

from peach_common.lifecycle import break_bond, ensure_lifecycle_active


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


def test_break_bond_none_safe_and_delegates():
    """None 直接返回；有 bond 时调 break_bond+shutdown（异常吞掉）."""
    break_bond(None)  # 不抛
    broken = []

    class _Bond:
        def break_bond(self):
            broken.append('break')

        def shutdown(self):
            broken.append('shutdown')

    break_bond(_Bond())
    assert broken == ['break', 'shutdown']

    class _Bad:
        def break_bond(self):
            raise RuntimeError('already dead')

        def shutdown(self):
            raise RuntimeError('already dead')

    break_bond(_Bad())  # 拆除异常不外抛
