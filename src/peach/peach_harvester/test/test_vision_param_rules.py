"""Zero-ROS tests for parameter rule checking (F01)."""
from peach_harvester.vision.param_rules import check


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
