"""yaml_params: flatten, dotted keys, load deployment yaml (no ROS)."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace as NS

from peach_harvester.yaml_params import (
    dict_to_ns,
    flatten_leaves,
    leaf_keys,
    load_ros_parameters,
    set_dotted,
)

ROOT = Path(__file__).resolve().parents[1]


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


def test_scene_yaml_has_model_paths():
    spec = load_ros_parameters(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    assert 'best.pt' in spec['yolo_model_path']
    assert spec['yolo_model_path']  # nonempty in deployment yaml
    keys = leaf_keys(
        ROOT / 'config' / 'scene_perception.yaml',
        'peach_scene_perception_node')
    assert 'sam_model_path' in keys
