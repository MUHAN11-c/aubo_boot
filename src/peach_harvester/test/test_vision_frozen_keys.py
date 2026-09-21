"""Frozen parameter keys: contract yaml is a subset of deployment yaml."""
from __future__ import annotations

from pathlib import Path

from peach_harvester.vision.param_rules import check
from peach_harvester.vision.scene_perception import params as scene_params
from peach_harvester.vision.target_reconstruction import (
    params as recon_params,
)
from peach_harvester.yaml_params import leaf_keys, load_ros_parameters
import yaml

ROOT = Path(__file__).resolve().parents[1]


def _contract_keys(path: Path, node_name: str) -> set[str]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return {
        name for name, value in document[node_name].items()
        if isinstance(value, dict) and 'type' in value}


def test_scene_perception_yaml_loads():
    keys = leaf_keys(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    assert 'yolo_model_path' in keys
    assert 'pipeline.min_depth_m' in keys
    assert 'tool.D_inner' in keys


def test_target_reconstruction_yaml_loads():
    keys = leaf_keys(
        ROOT / 'config' / 'target_reconstruction.yaml',
        'peach_target_reconstruction_node')
    assert 'frames.base_frame' in keys
    assert 'capture.min_views' in keys


def test_contract_param_keys_are_frozen_subset():
    keys = leaf_keys(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    schema = _contract_keys(
        ROOT / 'config' / 'vision_contract.param.yaml',
        'peach_scene_perception_node')
    assert schema <= keys


def test_scene_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    assert set(scene_params._RULES) <= keys


def test_reconstruction_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'target_reconstruction.yaml',
        'peach_target_reconstruction_node')
    assert set(recon_params._RULES) <= keys


def test_decision_validity_key_default_and_rule():
    """G1 对账：decision.validity_s 键存在、部署默认 120s、gt 0 规则生效."""
    config = ROOT / 'config' / 'target_reconstruction.yaml'
    keys = leaf_keys(config, 'peach_target_reconstruction_node')
    assert 'decision.validity_s' in keys
    assert 'decision.validity_s' in recon_params._RULES
    deployed = load_ros_parameters(config, 'peach_target_reconstruction_node')
    assert float(deployed['decision']['validity_s']) == 120.0
    (rule,) = recon_params._RULES['decision.validity_s']
    assert check(rule, 0.0, 'decision.validity_s') is not None  # 非法拒启
    assert check(rule, -5.0, 'decision.validity_s') is not None
    assert check(rule, 120.0, 'decision.validity_s') is None  # 合法放行
