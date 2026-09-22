"""ObservabilityParams 规则键对账与扁平派生（零 ROS）."""
from __future__ import annotations

from pathlib import Path

from peach_common.yaml_params import dict_to_ns, leaf_keys
from peach_observability.params import (
    _RULES,
    DEBUG_ENDPOINTS,
    from_params,
    TOPIC_NAMES,
)

# 部署清单随包走（W11 起从 peach_harvester/config 迁入本包）
DEPLOY = (
    Path(__file__).resolve().parents[1]
    / 'config' / 'observability.yaml')


def test_rules_reference_yaml_keys():
    keys = leaf_keys(DEPLOY, 'peach_observability')
    assert set(_RULES) <= keys
    assert 'debug.motion_enabled' in keys


def test_from_params_flattens_topics_and_endpoints():
    spec = {
        'host': '127.0.0.1',
        'port': 8090,
        'param_poll_period_s': 3.0,
        'event_buffer_size': 100,
        'metrics_period_s': 1.0,
        'metrics_process_patterns': ['peach_arm'],
        'record': {
            'enabled': True, 'root_dir': '', 'save_images': True,
            'save_clouds': True, 'rosout': True, 'level': 'std',
            'max_total_bag_gb': 100.0, 'queue_depth': 64},
        'trajectory': {
            'enabled': True, 'base_frame': 'base_link', 'tip_frame': 'tcp',
            'period_s': 0.05, 'min_step_m': 0.003, 'max_points': 8000},
        'debug': {
            'enabled': True, 'motion_enabled': False,
            'action_timeout_s': 180.0, 'audit_enabled': True,
            'endpoints': {name: '/x' for name in DEBUG_ENDPOINTS}},
    }
    spec.update({name: '/t' for name in TOPIC_NAMES})
    snapshot = from_params(dict_to_ns(spec))
    assert snapshot.port == 8090
    assert snapshot.record_level == 'std'
    assert snapshot.record_queue_depth == 64
    assert snapshot.topics['target_observations_topic'] == '/t'
    assert snapshot.debug_endpoints['run_harvest_action'] == '/x'
