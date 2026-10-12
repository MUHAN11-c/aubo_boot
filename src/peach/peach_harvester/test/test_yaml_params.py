"""Deployment model paths and YAML keys (shared helpers tested in peach_common)."""
from pathlib import Path

from peach_common.yaml_params import leaf_keys, load_ros_parameters

ROOT = Path(__file__).resolve().parents[1]


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
