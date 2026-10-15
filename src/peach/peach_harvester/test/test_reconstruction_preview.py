"""Visualization output has a hard budget; algorithm clouds stay complete."""
from types import SimpleNamespace

import numpy as np
from peach_harvester.vision.target_reconstruction import publish
from peach_harvester.vision.target_reconstruction.reconstruction_core import ReconstructionCore
import pytest
from std_msgs.msg import Header


@pytest.mark.parametrize('count', [0, 50000, 50001, 200003])
def test_preview_is_bounded_deterministic_and_keeps_xyz_and_rgb(count):
    xyz = np.arange(count * 3, dtype=np.float32).reshape(-1, 3)
    colors = np.tile(np.array([[7, 31, 251]], dtype=np.uint8), (count, 1))
    colors[:, 0] = np.arange(count, dtype=np.uint32) % 256
    header = Header(frame_id='base_link')
    preview = publish.preview_cloud_msg(xyz, colors, header)
    again = publish.preview_cloud_msg(xyz, colors, header)
    assert preview.width * preview.height <= 50000
    assert preview.data == again.data
    assert preview.header == header
    points = np.frombuffer(preview.data, dtype=np.float32).reshape(-1, 4)
    stride = max(1, (count + 49999) // 50000)
    np.testing.assert_array_equal(points[:, :3], xyz[::stride])
    rgb = points[:, 3].view(np.uint32)
    np.testing.assert_array_equal(rgb & 255, colors[::stride, 0])
    np.testing.assert_array_equal(rgb >> 8,
                                  np.full(len(points), 0xfb1f, dtype=np.uint32))
    # Serialization of the original algorithm output still includes every point.
    assert publish.xyzrgb_to_cloud_msg(xyz, colors, header).width == count


def test_preview_prefers_tsdf_and_clears_on_reset_without_mutating_sources():
    local = np.ones((120003, 3), dtype=np.float32)
    tsdf = np.full((60003, 3), 2., dtype=np.float32)
    messages = []
    node = SimpleNamespace(
        _tsdf_cloud_cache=(tsdf, None),
        collector=SimpleNamespace(accumulated_cloud=lambda: local, accumulated_rgb=lambda: None),
        pub_preview_cloud=SimpleNamespace(publish=messages.append))
    ReconstructionCore._publish_preview(node, Header(), (1, 1., 1), None, False)
    assert messages[-1].width <= 50000
    assert np.frombuffer(messages[-1].data, np.float32)[0] == 2.
    node._tsdf_cloud_cache = None
    ReconstructionCore._publish_preview(node, Header(), (1, 1., 2), None, False)
    assert np.frombuffer(messages[-1].data, np.float32)[0] == 1.
    node.collector.accumulated_cloud = lambda: np.empty((0, 3))
    ReconstructionCore._publish_preview(node, Header(), (0, -1., 3), None, True)
    assert messages[-1].width == 0
    assert len(local) == 120003 and len(tsdf) == 60003


def test_preview_throttle_still_forces_an_empty_reset():
    messages = []
    throttle = publish.PublishThrottle(min_interval_s=1., now=lambda: 0.)
    node = SimpleNamespace(
        _tsdf_cloud_cache=None,
        collector=SimpleNamespace(accumulated_cloud=lambda: np.ones((10, 3)),
                                  accumulated_rgb=lambda: None),
        pub_preview_cloud=SimpleNamespace(publish=messages.append))
    ReconstructionCore._publish_preview(node, Header(), (1, 1., 1), throttle, False)
    ReconstructionCore._publish_preview(node, Header(), (1, 1., 1), throttle, False)
    assert len(messages) == 1
    node.collector.accumulated_cloud = lambda: np.empty((0, 3))
    ReconstructionCore._publish_preview(node, Header(), (0, -1., 2), throttle, True)
    assert len(messages) == 2 and messages[-1].width == 0
