"""Frozen executor yaml keys vs params.py DEFAULTS."""
from __future__ import annotations

import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def _defaults_keys(class_name: str) -> set[str]:
    tree = ast.parse(
        (ROOT / 'peach_harvester' / 'supervisor' /
         'params.py').read_text(encoding='utf-8'))
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


def _yaml_keys(filename: str, node_name: str) -> set[str]:
    doc = yaml.safe_load(
        (ROOT / 'config' / filename).read_text(encoding='utf-8'))
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


def test_executor_yaml_matches_defaults():
    assert _defaults_keys('peach_supervisor') == _yaml_keys(
        'peach_supervisor.yaml', 'peach_supervisor')


def test_observability_yaml_matches_defaults():
    assert _defaults_keys('peach_observability') == _yaml_keys(
        'observability.yaml', 'peach_observability')


def test_lifecycle_manager_yaml_matches_defaults():
    assert _defaults_keys('peach_lifecycle_manager') == _yaml_keys(
        'lifecycle_manager.yaml', 'peach_lifecycle_manager')


def test_contract_param_keys_are_frozen_subset():
    keys = _defaults_keys('peach_supervisor')
    doc = yaml.safe_load(
        (ROOT / 'config' / 'supervisor_contract.param.yaml').read_text(encoding='utf-8'))
    schema = {
        name for name, value in doc['peach_supervisor'].items()
        if isinstance(value, dict) and 'type' in value}
    assert schema <= keys
