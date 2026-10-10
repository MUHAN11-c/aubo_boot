"""Frozen executor yaml keys vs contract yaml; rules reference real keys."""
from __future__ import annotations

from pathlib import Path

from peach_harvester.supervisor import params as supervisor_params
from peach_harvester.yaml_params import leaf_keys, load_ros_parameters
import yaml

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


def test_lifecycle_manager_yaml_loads():
    keys = leaf_keys(
        ROOT / 'config' / 'lifecycle_manager.yaml',
        'peach_lifecycle_manager')
    assert 'node_names' in keys
    assert 'startup_timeout_s' in keys


def test_contract_param_keys_are_frozen_subset():
    keys = leaf_keys(
        ROOT / 'config' / 'peach_supervisor.yaml', 'peach_supervisor')
    document = yaml.safe_load(
        (ROOT / 'config' / 'supervisor_contract.param.yaml').read_text(
            encoding='utf-8'))
    schema = {
        name for name, value in document['peach_supervisor'].items()
        if isinstance(value, dict) and 'type' in value}
    assert schema <= keys


def test_supervisor_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'peach_supervisor.yaml', 'peach_supervisor')
    assert set(supervisor_params._SUPERVISOR_RULES) <= keys


def test_lifecycle_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'lifecycle_manager.yaml',
        'peach_lifecycle_manager')
    assert set(supervisor_params._LIFECYCLE_RULES) <= keys
