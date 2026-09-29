"""启动自检纯核单测（零 ROS）：检查函数 + evaluate + RateProbe."""
from peach_observability.selfcheck import checks
from peach_observability.selfcheck.checks import (
    check_controllers,
    check_disk_free,
    check_fresh,
    check_joint_order,
    check_min_rate,
    check_param_consistency,
    check_present,
    check_tcp_norm,
    evaluate,
    RateProbe,
)


def test_joint_order_match_and_mismatch():
    ok = check_joint_order(list(checks.DEFAULT_JOINT_ORDER))
    assert ok.status == 'pass'
    # 字母序同集合也过（JSB 发布序；命令序在控制器侧）
    alpha = sorted(checks.DEFAULT_JOINT_ORDER)
    assert check_joint_order(alpha).status == 'pass'
    missing = [n for n in checks.DEFAULT_JOINT_ORDER
               if n != 'wrist1_joint']
    bad = check_joint_order(missing)
    assert bad.status == 'fail' and 'wrist1_joint' in bad.detail
    bad2 = check_joint_order(
        list(checks.DEFAULT_JOINT_ORDER) + ['wrist4_joint'])
    assert bad2.status == 'fail' and 'wrist4_joint' in bad2.detail
    none = check_joint_order(None)
    assert none.status == 'fail'


def test_min_rate_pass_fail_skip():
    assert check_min_rate(12, 5.0, 2.0, 'cam').status == 'pass'
    assert check_min_rate(3, 5.0, 2.0, 'cam').status == 'fail'  # 0.6Hz
    skip = check_min_rate(None, 5.0, 2.0, 'cam', 'camera_enabled=false')
    assert skip.status == 'skip'


def test_fresh_levels():
    assert check_fresh(0.5, 2.0, 'rs').status == 'pass'
    assert check_fresh(3.0, 2.0, 'rs').status == 'fail'
    assert check_fresh(None, 2.0, 'rs').status == 'fail'
    assert check_fresh(None, 2.0, 'rs', 'mock 不要求').status == 'skip'


def test_controllers_states_and_pending():
    states = {
        'joint_state_broadcaster': 'active',
        'joint_trajectory_controller': 'active'}
    assert check_controllers(
        states, checks.EXPECTED_CONTROLLERS['mock']).status == 'pass'
    inactive = dict(states, joint_trajectory_controller='inactive')
    result = check_controllers(inactive, checks.EXPECTED_CONTROLLERS['mock'])
    assert result.status == 'fail' and 'inactive' in result.detail
    assert check_controllers(
        None, checks.EXPECTED_CONTROLLERS['mock']).status == 'warn'


def test_disk_free_and_param_consistency():
    assert check_disk_free(50 << 30, 10 << 30).status == 'pass'
    assert check_disk_free(5 << 30, 10 << 30).status == 'fail'
    ok = check_param_consistency(
        {'hardware_mode': 'mock', 'require_robot_status': 'false'})
    assert ok.status == 'pass'
    bad = check_param_consistency(
        {'hardware_mode': 'mock', 'require_robot_status': 'true'})
    assert bad.status == 'fail'
    sim = check_param_consistency(
        {'hardware_mode': 'real', 'require_robot_status': 'true',
         'use_sim_time': 'true'})
    assert sim.status == 'fail'


def test_tcp_norm_and_present():
    assert check_tcp_norm(0.17509, 0.17509).status == 'pass'
    assert check_tcp_norm(0.166, 0.17509).status == 'fail'
    assert check_tcp_norm(None, 0.17509).status == 'fail'
    assert check_tcp_norm(0.1, -1.0).status == 'skip'
    assert check_present(True, 'x', 'ok', 'missing').status == 'pass'
    assert check_present(False, 'x', 'ok', 'missing').status == 'fail'
    assert check_present(None, 'x', 'ok', 'missing', '未启用').status == 'skip'


def test_evaluate_severity_ladder():
    def result(name, status):
        return checks.CheckResult(name, status)
    assert evaluate([result('a', 'pass'), result('b', 'skip')])['status'] \
        == 'pass'
    verdict = evaluate([result('a', 'pass'), result('b', 'warn')])
    assert verdict['status'] == 'warn' and verdict['passed'] is True
    verdict = evaluate([result('a', 'fail'), result('b', 'warn')])
    assert verdict['status'] == 'fail' and verdict['passed'] is False
    assert verdict['failed'] == ['a']


def test_rate_probe_window_count_and_age():
    probe = RateProbe(window_s=5.0)
    assert probe.count(100.0) == 0 and probe.age(100.0) is None
    probe.push(96.0)
    probe.push(97.0)
    probe.push(99.0)
    assert probe.count(100.0) == 3  # 全部落在 [95,100)
    probe.push(104.0)
    # 窗口随查询时刻滑动：count(105) 只统计 [100,105)，仅 104 在窗
    assert probe.count(105.0) == 1
    assert probe.count(103.0) == 2  # [98,103)：99 与 104
    assert abs(probe.age(105.0) - 1.0) < 1e-9
