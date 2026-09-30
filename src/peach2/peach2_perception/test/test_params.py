import copy

from peach2_perception.params import load_params, params_from_dict
from perception_fixtures import CONFIG
import pytest
import yaml


def _raw():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _resolve(pkg):
    return f'/share/{pkg}'


def test_shipped_config_loads_and_expands_paths():
    p = load_params(CONFIG, _resolve)
    assert p.frame_id == 'base_link'
    assert p.depth_unit_m == pytest.approx(0.00025)
    assert p.tf_timeout_s == pytest.approx(0.2)
    assert p.sync_slop_s == pytest.approx(0.05)
    assert p.detector.model_path == '/share/peach_harvester/model/best.pt'
    assert p.segmenter.model_path == '/share/peach_harvester/model/mobile_sam.pt'
    assert p.detector.class_names == ('peach_bag', 'peach_nobag')
    assert p.detector.engine_path == ''


def test_absolute_model_path_kept():
    raw = _raw()
    raw['detector']['model_path'] = '/abs/best.pt'
    assert params_from_dict(raw, _resolve).detector.model_path == '/abs/best.pt'


@pytest.mark.parametrize('mutate, message', [
    (lambda r: r.update(extra=1), 'unknown keys'),
    (lambda r: r.pop('tf_timeout_s'), 'missing keys'),
    (lambda r: r['detector'].update(typo=1), 'unknown keys in detector'),
    (lambda r: r['lock'].pop('stable_s'), 'missing keys in lock'),
    (lambda r: r.update(frame_id='world'), 'base_link'),
    (lambda r: r.update(depth_unit_m=0.0), 'depth_unit_m'),
    (lambda r: r.update(min_depth_m=2.0), 'min_depth_m'),
    (lambda r: r['detector'].update(conf=1.5), 'detector.conf'),
    (lambda r: r['detector'].update(imgsz=650), 'multiple of 32'),
    (lambda r: r['detector'].update(bag_class_id=2), 'bag_class_id'),
    (lambda r: r['detector'].update(class_names=[]), 'class_names'),
    (lambda r: r['detector'].update(allow_cpu='yes'), 'allow_cpu'),
    (lambda r: r['segmenter'].update(morph_kernel_px=4), 'odd'),
    (lambda r: r['segmenter'].update(max_boxes=2.5), 'integer'),
    (lambda r: r['swing'].update(min_span_s=3.0), 'min_span_s'),
    (lambda r: r['surrogate_confidence'].update(jump_rel_lo=0.05), 'jump_rel_lo'),
    (lambda r: r['detector'].update(model_path=''), 'non-empty'),
    (lambda r: r.update(detector=[]), 'mapping'),
])
def test_invalid_config_rejected(mutate, message):
    raw = copy.deepcopy(_raw())
    mutate(raw)
    with pytest.raises(ValueError, match=message):
        params_from_dict(raw, _resolve)
