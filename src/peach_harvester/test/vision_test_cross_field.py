"""Zero-ROS cross-field parameter checks."""
from peach_harvester.vision.domain.cross_field import (
    check_depth_window,
    check_enable_deps,
    check_min_max,
    cut_capability_report,
)


def test_min_max_rejects_inverted():
    assert check_min_max(2.5, 0.3, 'min', 'max') is not None
    assert check_depth_window(2.5, 0.3) is not None


def test_min_max_accepts_ordered():
    assert check_depth_window(0.3, 2.5) is None


def test_enable_deps_tool_requires_grasp():
    assert check_enable_deps(True, False, True) is not None
    assert check_enable_deps(False, True, False) is not None
    assert check_enable_deps(False, False, False) is None
    assert check_enable_deps(True, True, True) is None


def test_cut_capability_does_not_claim_cut():
    report = cut_capability_report()
    assert report['ready_ok'] is True
    assert report['cut_feasible'] is False
    assert report['claim_cut'] is False
    assert report['axial_margin_m'] < 0.0
