import numpy as np
from peach2_perception.depth_quality import (confident_depth, surrogate_confidence,
                                             valid_depth_mask)
from peach2_perception.segmenter import (build_prompt, expand_box, LABEL_NEGATIVE,
                                         LABEL_PADDING, LABEL_POSITIVE, refine_mask,
                                         RefineParams, Segmenter)
from perception_fixtures import (bag_points, camera_pose, FakeSam, mask_bbox, plane_points,
                                 render)

RP = RefineParams(depth_jump_rel=0.03, depth_jump_abs_m=0.01, seed_frac=0.34,
                  morph_kernel_px=5, min_area_px=100)


def _depth_ok(depth):
    valid = valid_depth_mask(depth, 0.3, 1.5)
    conf = surrogate_confidence(depth, valid, 0.01, 0.04)
    dc = confident_depth(depth, valid, conf, 0.5)
    return dc, dc > 0.0


def test_prompt_layout():
    p = build_prompt((100, 100, 200, 300), (480, 640), 0.1, 8)
    assert p.box.tolist() == [90, 80, 210, 320]
    assert p.labels.tolist() == [LABEL_POSITIVE] + [LABEL_NEGATIVE] * 4
    assert p.points[0].tolist() == [150.0, 200.0]
    for (u, v) in p.points[1:]:
        assert not (90 <= u < 210 and 80 <= v < 320)


def test_prompt_negatives_clipped_into_box_are_padding():
    p = build_prompt((0, 0, 100, 200), (480, 640), 0.1, 8)
    assert p.labels[0] == LABEL_POSITIVE
    assert LABEL_PADDING in p.labels.tolist()
    assert p.points.shape == (5, 2)
    assert expand_box((0, 0, 100, 200), (480, 640), 0.1) == (0, 0, 110, 220)


def test_one_image_encoding_per_frame_for_all_boxes():
    T = camera_pose()
    b1, b2 = bag_points(bottom=(0.6, 0.08, 0.35)), bag_points(bottom=(0.6, -0.08, 0.35))
    depth, lab = render([b1, b2], T)
    masks = [lab == 0, lab == 1]
    sam = FakeSam(masks)
    seg = Segmenter(sam, 16, 0.1, 8, RP)
    dc, ok = _depth_ok(depth)
    out, truncated = seg.segment(np.zeros(depth.shape + (3,), np.uint8),
                                 [mask_bbox(m) for m in masks], dc, ok)
    assert sam.set_image_calls == 1 and sam.predict_calls == 1
    assert truncated == 0 and all(r.mask is not None for r in out)
    boxes, points, labels = sam.last_prompts
    assert boxes.shape == (2, 4) and points.shape == (2, 5, 2) and labels.shape == (2, 5)


def test_no_boxes_no_inference():
    sam = FakeSam([np.zeros((4, 4), bool)])
    out, n = Segmenter(sam, 16, 0.1, 8, RP).segment(np.zeros((4, 4, 3), np.uint8), [],
                                                    np.zeros((4, 4)), np.zeros((4, 4), bool))
    assert out == [] and n == 0 and sam.set_image_calls == 0


def test_truncation_is_reported_not_silent():
    T = camera_pose()
    depth, lab = render([bag_points()], T)
    mask = lab == 0
    dc, ok = _depth_ok(depth)
    seg = Segmenter(FakeSam([mask]), 1, 0.1, 8, RP)
    bb = mask_bbox(mask)
    out, truncated = seg.segment(np.zeros(depth.shape + (3,), np.uint8), [bb, bb, bb], dc, ok)
    assert truncated == 2 and len(out) == 3
    assert out[0].mask is not None
    assert out[1].mask is None and 'sam_truncated' in out[1].flags


def test_tilted_bag_is_not_cut_by_a_depth_band():
    # The old fixed +-25 mm band around the median depth cut bags whose depth spans > 50 mm.
    T = camera_pose()
    depth, lab = render([bag_points(axis=(-0.6, 0.0, 0.8))], T)
    mask = lab == 0
    assert depth[mask].max() - depth[mask].min() > 0.08
    dc, ok = _depth_ok(depth)
    bb = mask_bbox(mask)
    r = refine_mask(mask, bb, expand_box(bb, depth.shape, 0.1), dc, ok, RP)
    assert r.mask is not None
    assert (r.mask & mask).sum() / mask.sum() > 0.9


def test_leaf_in_front_is_split_off_by_depth():
    T = camera_pose()
    leaf = plane_points((0.5, 0.03, 0.47), (1.0, 0.0, 0.0), 0.03, 0.03)
    depth, lab = render([bag_points(), leaf], T)
    bag, lf = lab == 0, lab == 1
    dc, ok = _depth_ok(depth)
    bb = mask_bbox(bag)
    r = refine_mask(bag | lf, bb, expand_box(bb, depth.shape, 0.1), dc, ok, RP)
    assert r.mask is not None and 'depth_split' in r.flags
    assert (r.mask & lf).sum() == 0
    iou = (r.mask & bag).sum() / (r.mask | bag).sum()
    assert iou > 0.9


def test_mask_without_depth_rejected():
    mask = np.zeros((50, 50), bool)
    mask[10:40, 10:40] = True
    r = refine_mask(mask, (10, 10, 40, 40), (5, 5, 45, 45), np.zeros((50, 50), np.float32),
                    np.zeros((50, 50), bool), RP)
    assert r.mask is None and 'mask_no_depth' in r.flags


def test_small_mask_rejected():
    mask = np.zeros((50, 50), bool)
    mask[20:26, 20:26] = True
    depth = np.full((50, 50), 0.6, np.float32)
    r = refine_mask(mask, (20, 20, 26, 26), (18, 18, 28, 28), depth, depth > 0, RP)
    assert r.mask is None and 'mask_too_small' in r.flags
