from dataclasses import fields
import os

from peach2_scene import params as prm
from peach2_scene.scene_core import SceneParams
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'scene.yaml')


def _raw() -> dict:
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_shipped_config_values():
    p = prm.load_params(CONFIG)
    s = p.scene
    assert s.voxel_size_m == 0.03
    assert s.min_points_per_voxel == 5 and isinstance(s.min_points_per_voxel, int)
    assert s.workspace_radius_m == 1.5
    assert s.self_margin_m == 0.03
    assert s.target_margin_m == 0.05
    assert s.hard_min_thickness_m == 0.012
    assert s.max_objects == 400
    assert p.frame_timeout_s == 2.0
    assert p.base_frame == 'base_link' and p.tcp_frame == 'tcp'
    assert p.image_qos_reliability == prm.QOS_BEST_EFFORT
    assert p.camera_frame == 'camera_color_optical_frame'
    assert p.depth_unit_m == 0.00025


def test_empty_camera_frame_means_header_frame():
    for value in ('', None):
        raw = _raw()
        raw['camera_frame'] = value
        assert prm.params_from_dict(raw).camera_frame == ''


def test_rule_table_covers_every_field():
    names = {f.name for f in fields(SceneParams)} | \
        {f.name for f in fields(prm.NodeParams)} - {'scene'}
    assert names == set(prm._RULES)
    assert names == set(_raw())


@pytest.mark.parametrize('key,value', [
    ('voxel_size_m', 0.0), ('voxel_size_m', -0.03), ('min_points_per_voxel', 0),
    ('min_points_per_voxel', 2.5), ('max_objects', 0), ('workspace_radius_m', float('nan')),
    ('frame_timeout_s', 'fast'), ('self_margin_m', True), ('image_qos_reliability', 'maybe'),
    ('base_frame', ''), ('linearity_min', 1.5),
])
def test_bad_values_rejected(key, value):
    raw = _raw()
    raw[key] = value
    with pytest.raises(ValueError):
        prm.params_from_dict(raw)


def test_unknown_missing_and_cross_checks():
    raw = _raw()
    raw['voxel_sise_m'] = 0.03
    with pytest.raises(ValueError, match='unknown'):
        prm.params_from_dict(raw)
    raw = _raw()
    del raw['max_objects']
    with pytest.raises(ValueError, match='missing'):
        prm.params_from_dict(raw)
    for key, value in (('tf_timeout_s', 2.0), ('self_sample_spacing_m', 0.04),
                       ('leaf_mask_max_width_m', 0.012), ('soft_max_extent_m', 0.02),
                       ('min_depth_m', 1.5)):
        raw = _raw()
        raw[key] = value
        with pytest.raises(ValueError):
            prm.params_from_dict(raw)
    with pytest.raises(ValueError):
        prm.params_from_dict([1, 2])
