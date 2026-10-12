"""Deployed supervisor YAML keys and actual validation rules."""
from __future__ import annotations

from pathlib import Path

from peach_common.yaml_params import leaf_keys, load_ros_parameters
from peach_harvester.supervisor import params as supervisor_params

ROOT = Path(__file__).resolve().parents[1]


def test_executor_yaml_loads():
    keys = leaf_keys(
        ROOT / 'config' / 'peach_supervisor.yaml', 'peach_supervisor')
    assert 'execution_enabled' in keys
    assert 'tool.profile_id' in keys
    assert 'skip_reconstruction' in keys
    deployed = load_ros_parameters(
        ROOT / 'config' / 'peach_supervisor.yaml', 'peach_supervisor')
    assert deployed['skip_reconstruction'] is False


def test_supervisor_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'peach_supervisor.yaml', 'peach_supervisor')
    assert set(supervisor_params._SUPERVISOR_RULES) <= keys
