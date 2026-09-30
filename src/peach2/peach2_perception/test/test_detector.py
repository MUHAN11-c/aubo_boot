import numpy as np
from peach2_perception.detector import (box_iou, clip_box, dedup_overlapping, Detection,
                                        Detector, weights_path)
from perception_fixtures import FakeYolo
import pytest

NAMES = ('peach_bag', 'peach_nobag')


def _detector(rows, names=None):
    return Detector(FakeYolo(rows, names), 0.35, 0.5, NAMES, 0.6, 0.2, 0.5)


def test_clip_box():
    assert clip_box((-5.2, 10.4, 30.1, 500.0), (480, 640)) == (0, 10, 31, 480)
    assert clip_box((700, 10, 710, 20), (480, 640)) is None


def test_box_iou():
    assert box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert box_iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(50 / 150)
    assert box_iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


def test_detect_sorted_clipped_and_thresholded():
    det = _detector([[10, 10, 50, 90, 0.5, 0], [100, 10, 150, 90, 0.9, 0],
                     [200, 10, 250, 90, 0.2, 1], [600, 400, 700, 500, 0.8, 1]])
    out = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert [d.score for d in out] == [0.9, 0.8, 0.5]
    assert out[1].bbox == (600, 400, 640, 480)
    assert out[1].class_name == 'peach_nobag'


def test_cross_class_duplicate_removed():
    det = _detector([[100, 100, 200, 300, 0.9, 0], [102, 98, 198, 305, 0.6, 1]])
    out = det.detect(np.zeros((480, 640, 3), np.uint8))
    assert len(out) == 1 and out[0].class_id == 0


def test_fragment_inside_full_box_removed_but_neighbour_kept():
    full = Detection((100, 100, 200, 300), 0, 'peach_bag', 0.9)
    frag = Detection((150, 250, 230, 320), 0, 'peach_bag', 0.5)  # IoS ~0.31, area 0.28
    neighbour = Detection((190, 100, 290, 300), 0, 'peach_bag', 0.8)  # IoS 0.1
    kept = dedup_overlapping([frag, neighbour, full], 0.6, 0.2, 0.5)
    assert full in kept and neighbour in kept and frag not in kept
    kept = dedup_overlapping([frag, full], 0.6, 0.2, 0.0)
    assert frag in kept


def test_unknown_class_ignored_and_names_checked():
    det = _detector([[10, 10, 50, 90, 0.9, 5]])
    assert det.detect(np.zeros((480, 640, 3), np.uint8)) == []
    det.check_class_names()
    with pytest.raises(ValueError):
        _detector([], names={0: 'bag', 1: 'nobag'}).check_class_names()


def test_weights_path_prefers_existing_engine(tmp_path):
    pt = tmp_path / 'best.pt'
    pt.write_bytes(b'x')
    engine = tmp_path / 'best.engine'
    assert weights_path(str(pt), str(engine)) == str(pt)
    engine.write_bytes(b'x')
    assert weights_path(str(pt), str(engine)) == str(engine)
    assert weights_path(str(pt), '') == str(pt)
    with pytest.raises(FileNotFoundError):
        weights_path(str(tmp_path / 'missing.pt'), '')
