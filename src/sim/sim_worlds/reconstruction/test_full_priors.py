"""Check full-corpus measurement without sampling away valid frames."""

import measure_priors
import numpy as np
from PIL import Image


def test_full_measurement_uses_each_valid_frame(tmp_path, monkeypatch):
    split = tmp_path / 'Peach_bag'
    for folder in ('RGB', 'Depth', 'Annotations_VOC/VOC_4label'):
        (split / folder).mkdir(parents=True)
    for i in range(3):
        Image.fromarray(np.full((720, 1280, 3), (120, 60, 50), np.uint8)).save(
            split / 'RGB' / f'{i}.png')
        Image.fromarray(np.full((720, 1280), 500 + i * 100, np.uint16)).save(
            split / 'Depth' / f'{i}.png')
        (split / 'Annotations_VOC/VOC_4label' / f'{i}.xml').write_text(
            '<annotation><object><name>0</name><bndbox>'
            '<xmin>100</xmin><ymin>100</ymin><xmax>200</xmax><ymax>200</ymax>'
            '</bndbox></object></annotation>')
    monkeypatch.setattr(measure_priors, 'DATASET', tmp_path)
    result = measure_priors.measure_split('Peach_bag', limit=None)
    assert result['classes']['0']['depth_m']['n'] == 3
    assert result['classes']['0']['depth_m']['p50'] == .6

    (split / 'Depth' / '1.png').write_bytes(b'not an image')
    result = measure_priors.measure_split('Peach_bag', limit=None)
    assert result['classes']['0']['depth_m']['n'] == 2
    assert result['skipped_frames'][0]['id'] == '1'
