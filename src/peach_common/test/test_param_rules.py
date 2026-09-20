"""
Zero-ROS tests for the merged parameter rule checking (W1 union).

并集用例：supervisor（gt / gt_eq）、vision（gt / bounds / nonempty /
gt+lt_eq 组合）、vegetation（one_of / seq_gt / check_min_max / bounds）。
全部指向 peach_common 单源。
"""
from peach_common.param_rules import check, check_enable_deps, check_min_max


def test_gt_rejects_non_positive_timeout():
    assert check(('gt', 0.0), -1.0, 'service_timeout_s') is not None
    assert check(('gt', 0.0), 0.0, 'service_timeout_s') is not None


def test_gt_accepts_positive_timeout():
    assert check(('gt', 0.0), 5.0, 'service_timeout_s') is None


def test_gt_eq_rejects_below_one():
    assert check(('gt_eq', 1.0), 0.0, 'empty_survey_limit') is not None
    assert check(('gt_eq', 1.0), 1.0, 'empty_survey_limit') is None


def test_gt_rejects_non_positive():
    assert check(('gt', 0.0), -1.0, 'depth_scale_unit') is not None
    assert check(('gt', 0.0), 0.0, 'depth_scale_unit') is not None


def test_gt_accepts_positive():
    assert check(('gt', 0.0), 0.25, 'depth_scale_unit') is None


def test_bounds_rejects_out_of_range():
    assert check(('bounds', 0.0, 1.0), 1.1, 'lighting.min_depth_ratio') is not None
    assert check(('bounds', 0.0, 1.0), -0.1, 'lighting.min_depth_ratio') is not None


def test_bounds_accepts_closed_interval():
    assert check(('bounds', 0.0, 1.0), 0.0, 'lighting.min_depth_ratio') is None
    assert check(('bounds', 0.0, 1.0), 1.0, 'lighting.min_depth_ratio') is None
    assert check(('bounds', 0.0, 1.0), 0.35, 'lighting.min_depth_ratio') is None


def test_nonempty_rejects_blank_path():
    assert check(('nonempty',), '', 'yolo_model_path') is not None
    assert check(('nonempty',), '  ', 'sam_model_path') is not None
    assert check(('nonempty',), '/tmp/best.pt', 'yolo_model_path') is None


def test_position_ema_gt_and_lt_eq():
    rules = (('gt', 0.0), ('lt_eq', 1.0))
    for value in (-0.1, 0.0, 1.1):
        reasons = [check(rule, value, 'target_memory.position_ema') for rule in rules]
        assert any(reason is not None for reason in reasons)
    for value in (0.3, 1.0):
        reasons = [check(rule, value, 'target_memory.position_ema') for rule in rules]
        assert all(reason is None for reason in reasons)


def test_device_one_of():
    rule = ('one_of', ('auto', 'cpu', 'cuda', 'cuda:0'))
    assert check(rule, 'auto', 'device') is None
    assert check(rule, 'gpu', 'device') is not None


def test_branch_sigmas_must_be_positive():
    rule = ('seq_gt', 0.0)
    assert check(rule, [1, 3, 5], 'branch.sigmas') is None
    assert check(rule, [], 'branch.sigmas') is not None
    assert check(rule, [1, 0], 'branch.sigmas') is not None


def test_leaf_h_window():
    assert check_min_max(35, 90, 'leaf.h_min', 'leaf.h_max') is None
    assert check_min_max(90, 35, 'leaf.h_min', 'leaf.h_max') is not None


def test_percentile_bounds():
    rule = ('bounds', 0.0, 100.0)
    assert check(rule, 88.0, 'branch.percentile') is None
    assert check(rule, 120.0, 'branch.percentile') is not None


def test_enable_deps_chain_and_all_off():
    assert check_enable_deps(False, False, False) is None
    assert check_enable_deps(True, False, False) is None
    assert check_enable_deps(True, True, False) is None
    assert check_enable_deps(True, True, True) is None
    assert check_enable_deps(False, True, True) is not None
    assert check_enable_deps(True, False, True) is not None
    assert check_enable_deps(False, False, True) is not None
