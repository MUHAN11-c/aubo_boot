from peach2_target_model.params import expand_dirs, params_from_dict
import pytest
from tm_fixtures import CALIB, CONFIG, params, PROFILES, resolve_share
import yaml


def _raw():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_shipped_config_values():
    p = params()
    assert p.validity_s == 120.0
    assert p.pregrasp_standoff_m == 0.03
    assert p.converge_sigma_lateral95_m == 0.006
    assert p.converge_theta95_deg == 4.0
    assert p.converge_sigma_axial95_m == 0.010
    assert p.remeasure_mismatch_m == 0.010
    assert p.frame_id == 'base_link'
    assert p.description_config_dir.rstrip('/') == PROFILES
    assert p.calibration_dir.rstrip('/') == CALIB
    assert isinstance(p.min_views, int) and isinstance(p.cut_requires_remeasure, bool)
    assert p.view_translation_change_m == 0.05 and p.view_rotation_change_deg == 10.0
    assert p.require_known_swing is True


def test_removed_distance_key_rejected():
    raw = _raw()
    raw['view_distance_change_m'] = 0.05
    with pytest.raises(ValueError, match='unknown'):
        params_from_dict(raw, resolve_share)


def test_expand_dirs():
    assert expand_dirs('$(find-pkg-share foo)/x', lambda pkg: '/s/' + pkg) == '/s/foo/x'
    assert expand_dirs('/abs/path', lambda pkg: 'unused') == '/abs/path'


@pytest.mark.parametrize('key,value', [
    ('validity_s', 0.0),
    ('validity_s', -1.0),
    ('validity_s', float('nan')),
    ('validity_s', 'long'),
    ('converge_sigma_lateral95_m', 0.0),
    ('converge_theta95_deg', 90.0),
    ('pregrasp_standoff_m', -0.01),
    ('min_views', 0),
    ('min_views', 1.5),
    ('min_views', True),
    ('cut_requires_remeasure', 1),
    ('frame_id', 'world'),
    ('frame_id', ''),
    ('remeasure_mismatch_m', 0.0),
    ('min_views', 9),
    ('default_max_views', 9),
    ('remeasure_min_frames', 31),
    ('view_translation_change_m', 0.0),
    ('view_rotation_change_deg', 0.0),
    ('view_rotation_change_deg', 200.0),
    ('require_known_swing', 'yes'),
])
def test_invalid_values_rejected(key, value):
    raw = _raw()
    raw[key] = value
    with pytest.raises(ValueError):
        params_from_dict(raw, resolve_share)


def test_missing_and_unknown_keys_rejected():
    raw = _raw()
    del raw['validity_s']
    with pytest.raises(ValueError, match='missing'):
        params_from_dict(raw, resolve_share)
    raw = _raw()
    raw['validty_s'] = 1.0
    with pytest.raises(ValueError, match='unknown'):
        params_from_dict(raw, resolve_share)
    with pytest.raises(ValueError):
        params_from_dict([], resolve_share)


def test_integer_valued_float_accepted():
    raw = _raw()
    raw['min_views'] = 2.0
    assert params_from_dict(raw, resolve_share).min_views == 2
