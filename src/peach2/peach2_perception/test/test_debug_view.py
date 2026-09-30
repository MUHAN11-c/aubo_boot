import numpy as np
from peach2_perception.debug_view import draw_debug
from peach2_perception.observation_builder import ObservationBuilder
from peach2_perception.stationary import Motion
from perception_fixtures import (bag_instance, bag_points, BUILDER_PARAMS, camera_pose,
                                 colour_image, make_frame)
import pytest


def test_overlay_draws_and_downscales():
    frame, lab = make_frame(camera_pose(), [bag_points()])
    b = ObservationBuilder(BUILDER_PARAMS)
    ms = b.measure(frame, [bag_instance(lab)])
    recs = b.associate(0.0, ms, Motion.STATIONARY)
    img = colour_image(lab)
    out = draw_debug(img, recs, [], 0.5, 'epoch 1')
    assert out.shape == (240, 320, 3) and out.dtype == np.uint8
    full = draw_debug(img, recs, ms, 1.0)
    assert full.shape == img.shape and not np.array_equal(full, img)
    with pytest.raises(ValueError):
        draw_debug(img, recs, [], 0.0)
