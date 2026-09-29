"""startup_facts 纯核单测（零 ROS）：规范化 / 完整性 / 自检覆盖推导."""
from peach_observability.startup_facts import (
    facts_valid,
    normalize_facts,
    selfcheck_overrides_from_facts,
)


def test_normalize_filters_and_derives():
    facts = normalize_facts({
        'hardware_mode': 'mock', 'robot_ip': '1.2.3.4',
        'unknown_key': 'dropped', 'expected_tcp_norm_m': 0.17509,
        'git': {'branch': 'test'}, 'ros_domain_id': 77,
    })
    assert 'unknown_key' not in facts
    assert facts['require_robot_status'] == 'false'
    assert facts['expected_tcp_norm_m'] == 0.17509  # 数值保持数值
    assert facts['git'] == {'branch': 'test'}
    assert facts['ros_domain_id'] == 77  # 数值保持数值（JSON 原生类型）


def test_normalize_empty_and_real():
    assert normalize_facts(None) == {}
    facts = normalize_facts({'hardware_mode': 'real'})
    assert facts['require_robot_status'] == 'true'


def test_facts_valid_requires_mode():
    assert facts_valid({'hardware_mode': 'mock'}) is True
    assert facts_valid({}) is False
    assert facts_valid({'hardware_mode': 'weird'}) is False


def test_overrides_from_facts():
    overrides = selfcheck_overrides_from_facts({
        'camera_enabled': 'true', 'require_robot_status': 'false',
        'moveit_enabled': 'true', 'imu_enabled': 'false',
        'expected_tcp_norm_m': 0.17509})
    assert overrides == {
        'selfcheck_camera_probe_enabled': True,
        'robot_status_probe_enabled': False,
        'selfcheck_moveit_expected': True,
        'selfcheck_imu_expected': False,
        'selfcheck_expected_tcp_norm_m': 0.17509,
    }
    assert selfcheck_overrides_from_facts({}) == {}
    # 缺 tcp 期望时不覆盖（保持 yaml 的 -1 跳过档）
    assert 'selfcheck_expected_tcp_norm_m' not in \
        selfcheck_overrides_from_facts({'hardware_mode': 'mock'})
