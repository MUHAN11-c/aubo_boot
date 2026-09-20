"""Zero-ROS tests for executor parameter rule checking (F01)."""
from peach_harvester.supervisor.param_rules import check, check_enable_deps


def test_gt_rejects_non_positive_timeout():
    assert check(('gt', 0.0), -1.0, 'service_timeout_s') is not None
    assert check(('gt', 0.0), 0.0, 'service_timeout_s') is not None


def test_gt_accepts_positive_timeout():
    assert check(('gt', 0.0), 5.0, 'service_timeout_s') is None


def test_gt_eq_rejects_below_one():
    assert check(('gt_eq', 1.0), 0.0, 'empty_survey_limit') is not None
    assert check(('gt_eq', 1.0), 1.0, 'empty_survey_limit') is None


def test_set_enables_dependency_combinations():
    """
    W6-B/S3 语义锁定：SetEnables 组合校验路径（纯函数级全组合）.

    依赖链 execution→grasp→tool：越级开刀拒绝且不落 override
    （executor _on_set_enables 的 check_enable_deps 调用）。
    """
    expected = {
        (False, False, False): None,   # 全关合法
        (True, False, False): None,    # 仅 execution 合法
        (True, True, False): None,     # execution+grasp 合法
        (True, True, True): None,      # 全链开合法
        (True, False, True): (  # tool 越过 grasp
            'tool_enabled requires grasp_enabled'),
        (False, True, False): (  # grasp 越过 execution
            'grasp_enabled requires execution_enabled'),
        (False, True, True): (
            'grasp_enabled requires execution_enabled'),
        (False, False, True): (
            'tool_enabled requires grasp_enabled'),
    }
    for combo, want in expected.items():
        got = check_enable_deps(*combo)
        assert got == want, (combo, got, want)
