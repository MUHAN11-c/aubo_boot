import numpy as np
from peach2_perception.detector import Detector
from peach2_perception.frame_pipeline import (builder_params, CONFIDENCE_PUBLISHED,
                                              CONFIDENCE_SURROGATE, FrameInput, FramePipeline)
from peach2_perception.observation_builder import (CATEGORY_BAG, CATEGORY_NOBAG,
                                                   ObservationBuilder)
from peach2_perception.params import load_params
from peach2_perception.segmenter import RefineParams, Segmenter
from perception_fixtures import (bag_points, camera_pose, colour_image, CONFIG, FakeSam,
                                 FakeYolo, K, mask_bbox, render, to_raw)
import pytest


def _pipeline(rows, masks):
    p = load_params(CONFIG, lambda pkg: f'/opt/{pkg}')
    d, s = p.detector, p.segmenter
    yolo, sam = FakeYolo(rows), FakeSam(masks)
    det = Detector(yolo, d.conf, d.iou, d.class_names, d.dedup_ios, d.dedup_frag_ios,
                   d.dedup_area_ratio)
    seg = Segmenter(sam, s.max_boxes, s.box_expand_frac, s.neg_point_offset_px,
                    RefineParams(s.depth_jump_rel, s.depth_jump_abs_m, s.seed_frac,
                                 s.morph_kernel_px, s.min_area_px))
    return FramePipeline(p, det, seg, ObservationBuilder(builder_params(p))), yolo, sam


def test_end_to_end_bag_and_nobag():
    T = camera_pose()
    depth, lab = render([bag_points(bottom=(0.6, 0.08, 0.35)),
                         bag_points(bottom=(0.6, -0.08, 0.35))], T)
    b0, b1 = mask_bbox(lab == 0), mask_bbox(lab == 1)
    pipe, yolo, sam = _pipeline([[*b1, 0.7, 1], [*b0, 0.9, 0]], [lab == 0, lab == 1])
    res = pipe.run(FrameInput(1.0, colour_image(lab), to_raw(depth), K, T))
    assert res.confidence_source == CONFIDENCE_SURROGATE
    assert yolo.calls == 1 and sam.set_image_calls == 1
    # Nobag boxes are not segmented (segment_nobag: false); bags come first.
    assert sam.last_prompts[0].shape[0] == 1
    cats = [m.category for m in res.measurements]
    assert cats == [CATEGORY_BAG, CATEGORY_NOBAG]
    bag = res.measurements[0]
    assert bag.bottom.valid and np.linalg.norm(bag.bottom.position - [0.6, 0.08, 0.35]) < 0.012
    assert res.measurements[1].mask is None
    assert set(res.timings_ms) >= {'detect', 'segment', 'geometry', 'total'}


def test_published_confidence_used_and_size_checks():
    T = camera_pose()
    depth, lab = render([bag_points()], T)
    pipe, _, _ = _pipeline([[*mask_bbox(lab == 0), 0.9, 0]], [lab == 0])
    conf = np.full(depth.shape, 255, np.uint8)
    res = pipe.run(FrameInput(0.0, colour_image(lab), to_raw(depth), K, T, conf))
    assert res.confidence_source == CONFIDENCE_PUBLISHED
    assert res.measurements[0].bottom.valid
    with pytest.raises(ValueError):
        pipe.run(FrameInput(0.0, colour_image(lab)[:-1], to_raw(depth), K, T))
    with pytest.raises(ValueError):
        pipe.run(FrameInput(0.0, colour_image(lab), to_raw(depth), K, T, conf[:-1]))


def test_low_published_confidence_removes_depth():
    T = camera_pose()
    depth, lab = render([bag_points()], T)
    pipe, _, _ = _pipeline([[*mask_bbox(lab == 0), 0.9, 0]], [lab == 0])
    conf = np.zeros(depth.shape, np.uint8)
    res = pipe.run(FrameInput(0.0, colour_image(lab), to_raw(depth), K, T, conf))
    m = res.measurements[0]
    assert m.mask is None and 'mask_no_depth' in m.flags and not m.bottom.valid
