"""yaml_params: flatten, dotted keys, load deployment yaml, snapshot (no ROS)."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from peach_common.yaml_params import (
    dict_to_ns,
    flatten_leaves,
    leaf_keys,
    load_ros_parameters,
    set_dotted,
    snapshot,
)


def test_flatten_nested_and_dotted_keys():
    spec = {
        'device': 'auto',
        'leaf.exg_min': 20.0,
        'pipeline': {'min_depth_m': 0.3, 'max_depth_m': 1.5},
    }
    keys = dict(flatten_leaves(spec))
    assert keys['device'] == 'auto'
    assert keys['leaf.exg_min'] == 20.0
    assert keys['pipeline.min_depth_m'] == 0.3
    assert keys['pipeline.max_depth_m'] == 1.5


def test_dict_to_ns_nested_access():
    ns = dict_to_ns({
        'leaf.exg_min': 20.0,
        'pipeline': {'min_depth_m': 0.3},
    })
    assert ns.leaf.exg_min == 20.0
    assert ns.pipeline.min_depth_m == 0.3


def test_set_dotted_overwrites_leaf():
    ns = NS()
    set_dotted(ns, 'tool.D_inner', 0.104)
    set_dotted(ns, 'tool.D_inner', 0.116)
    assert ns.tool.D_inner == 0.116


def test_load_ros_parameters_and_leaf_keys(tmp_path: Path):
    yaml_path = tmp_path / 'node.yaml'
    yaml_path.write_text(
        'peach_node:\n'
        '  ros__parameters:\n'
        '    device: auto\n'
        '    leaf.exg_min: 20.0\n'
        '    pipeline:\n'
        '      min_depth_m: 0.3\n',
        encoding='utf-8')
    spec = load_ros_parameters(yaml_path, 'peach_node')
    assert spec['device'] == 'auto'
    assert spec['pipeline'] == {'min_depth_m': 0.3}
    keys = leaf_keys(yaml_path, 'peach_node')
    assert keys == {'device', 'leaf.exg_min', 'pipeline.min_depth_m'}


def test_load_ros_parameters_single_block_fallback(tmp_path: Path):
    yaml_path = tmp_path / 'anon.yaml'
    yaml_path.write_text(
        'some_other_name:\n'
        '  ros__parameters:\n'
        '    value: 1\n',
        encoding='utf-8')
    spec = load_ros_parameters(yaml_path, 'missing_node')
    assert spec == {'value': 1}


def test_snapshot_flattens_live_namespace_with_native_types():
    ns = dict_to_ns({
        'device': 'auto',
        'pipeline': {'min_depth_m': 0.3},
        'leaf.exg_min': 20.0,
        'tags': [1, 2],
    })
    ns.pipeline.numpy_scalar = np.float64(2.5)
    data = snapshot(ns)
    assert data['device'] == 'auto'
    assert data['pipeline.min_depth_m'] == 0.3
    assert data['leaf.exg_min'] == 20.0
    assert data['tags'] == [1, 2]
    assert data['pipeline.numpy_scalar'] == 2.5
    assert type(data['pipeline.numpy_scalar']) is float
    # 与后续 set 解耦
    ns.pipeline.min_depth_m = 9.9
    assert data['pipeline.min_depth_m'] == 0.3
