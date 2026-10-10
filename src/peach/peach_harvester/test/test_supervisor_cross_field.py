"""Executor enable-window and reach min/max."""
from peach_harvester.supervisor.param_rules import check_enable_deps, check_min_max


def test_reach_window():
    assert check_min_max(0.15, 0.88, 'min', 'max') is None
    assert check_min_max(0.88, 0.15, 'min', 'max') is not None


def test_default_enables_off():
    assert check_enable_deps(False, False, False) is None
    assert check_enable_deps(True, False, True) is not None
