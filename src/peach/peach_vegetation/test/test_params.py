"""Zero-ROS tests for vegetation param rules and yaml keys."""
from __future__ import annotations

from pathlib import Path

from peach_vegetation.param_rules import check, check_min_max
from peach_vegetation.params import _RULES, _validate
from peach_vegetation.yaml_params import leaf_keys

ROOT = Path(__file__).resolve().parents[1]


def test_yaml_has_split_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'vegetation.yaml', 'peach_vegetation')
    assert 'device' in keys
    assert 'leaf.exg_min' in keys
    assert 'branch.sigmas' in keys


def test_device_one_of():
    rule = ('one_of', ('auto', 'cpu', 'cuda', 'cuda:0'))
    assert check(rule, 'auto', 'device') is None
    assert check(rule, 'gpu', 'device') is not None


def test_branch_sigmas_must_be_positive():
    rule = ('seq_gt', 0.0)
    assert check(rule, [1, 3, 5], 'branch.sigmas') is None
    assert check(rule, [], 'branch.sigmas') is not None
    assert check(rule, [1, 0], 'branch.sigmas') is not None


def test_leaf_h_window():
    assert check_min_max(35, 90, 'leaf.h_min', 'leaf.h_max') is None
    assert check_min_max(90, 35, 'leaf.h_min', 'leaf.h_max') is not None


def test_percentile_bounds():
    rule = ('bounds', 0.0, 100.0)
    assert check(rule, 88.0, 'branch.percentile') is None
    assert check(rule, 120.0, 'branch.percentile') is not None


def test_module_rules_reference_yaml_keys():
    keys = leaf_keys(
        ROOT / 'config' / 'vegetation.yaml', 'peach_vegetation')
    assert set(_RULES) <= keys


def test_module_validate_rejects_bad_device_and_sigmas():
    assert _validate('device', 'gpu') is not None
    assert _validate('device', 'cuda:0') is None
    assert _validate('branch.sigmas', []) is not None
    assert _validate('branch.sigmas', [1, 3]) is None
    assert _validate('branch.dilate_px', 2) is None
