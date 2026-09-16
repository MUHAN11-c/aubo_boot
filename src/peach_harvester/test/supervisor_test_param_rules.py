"""Zero-ROS tests for executor parameter rule checking (F01)."""
from peach_harvester.supervisor.param_rules import check


def test_gt_rejects_non_positive_timeout():
    assert check(('gt', 0.0), -1.0, 'service_timeout_s') is not None
    assert check(('gt', 0.0), 0.0, 'service_timeout_s') is not None


def test_gt_accepts_positive_timeout():
    assert check(('gt', 0.0), 5.0, 'service_timeout_s') is None


def test_gt_eq_rejects_below_one():
    assert check(('gt_eq', 1.0), 0.0, 'empty_survey_limit') is not None
    assert check(('gt_eq', 1.0), 1.0, 'empty_survey_limit') is None
