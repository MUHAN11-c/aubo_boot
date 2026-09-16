"""Frozen parameter keys: yaml ros__parameters match params.py DEFAULTS."""
from __future__ import annotations

import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def _defaults_keys(module_path: Path, class_name: str) -> set[str]:
    tree = ast.parse(module_path.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if not isinstance(item, ast.Assign):
                    continue
                target = item.targets[0]
                if isinstance(target, ast.Name) and target.id == 'DEFAULTS':
                    return {
                        key.value for key in item.value.keys
                        if isinstance(key, ast.Constant)}
    raise AssertionError(f'{class_name}.DEFAULTS not found')


def _yaml_keys(path: Path, node_name: str) -> set[str]:
    doc = yaml.safe_load(path.read_text(encoding='utf-8'))
    params = doc[node_name]['ros__parameters']

    def walk(prefix, value):
        if isinstance(value, dict):
            keys = set()
            for name, child in value.items():
                next_prefix = f'{prefix}{name}' if not prefix else f'{prefix}.{name}'
                if isinstance(child, dict):
                    keys |= walk(next_prefix, child)
                else:
                    keys.add(next_prefix)
            return keys
        return {prefix}

    return walk('', params)


def test_scene_perception_yaml_matches_defaults():
    keys = _defaults_keys(
        ROOT / 'peach_harvester' / 'vision' / 'params.py',
        'peach_scene_perception_node')
    yaml_keys = _yaml_keys(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    assert keys == yaml_keys


def test_target_reconstruction_yaml_matches_defaults():
    keys = _defaults_keys(
        ROOT / 'peach_harvester' / 'vision' / 'params.py',
        'peach_target_reconstruction_node')
    yaml_keys = _yaml_keys(
        ROOT / 'config' / 'target_reconstruction.yaml',
        'peach_target_reconstruction_node')
    assert keys == yaml_keys


def test_contract_param_keys_are_frozen_subset():
    keys = _defaults_keys(
        ROOT / 'peach_harvester' / 'vision' / 'params.py',
        'peach_scene_perception_node')
    doc = yaml.safe_load(
        (ROOT / 'config' / 'vision_contract.param.yaml').read_text(encoding='utf-8'))
    schema = {
        name for name, value in doc['peach_scene_perception_node'].items()
        if isinstance(value, dict) and 'type' in value}
    assert schema <= keys
