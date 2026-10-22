"""调试操作的运动分类（不启节点）."""

from peach_observability.debug_actions import is_motion


def test_opening_an_enable_is_motion():
    assert is_motion('set_enables_service', {
        'execution': True, 'grasp': False, 'tool': False}) is True


def test_closing_all_enables_is_not_motion():
    assert is_motion('set_enables_service', {
        'execution': False, 'grasp': False, 'tool': False}) is False
