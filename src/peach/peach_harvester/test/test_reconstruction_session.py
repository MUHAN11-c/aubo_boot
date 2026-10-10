"""Zero-ROS tests for ReconstructionSession.process ingest gates."""
import numpy as np

from peach_harvester.vision.target_reconstruction.session import (
    ReconstructionSession,
)


class _P:
    depth_scale_unit = 1.0


def _session():
    return ReconstructionSession(params=_P(), refitters={})


def test_process_accepts_uint16_mm_and_valid_k():
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    depth = np.full((8, 8), 800, dtype=np.uint16)
    k = [100.0, 0.0, 4.0, 0.0, 100.0, 4.0, 0.0, 0.0, 1.0]
    out = _session().process(rgb, depth, k)
    assert out.ok
    assert out.frame.depth_mm.dtype == np.uint16
    assert out.frame.K['fx'] == 100.0
    assert out.frame.K['width'] == 8


def test_process_drops_resolution_mismatch():
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    depth = np.full((4, 4), 800, dtype=np.uint16)
    k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]
    out = _session().process(rgb, depth, k)
    assert not out.ok
    assert '分辨率不一致' in out.reason


def test_process_drops_nonpositive_fx():
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    depth = np.full((8, 8), 800, dtype=np.uint16)
    k = [0.0, 0.0, 4.0, 0.0, 100.0, 4.0, 0.0, 0.0, 1.0]
    out = _session().process(rgb, depth, k)
    assert not out.ok
    assert 'fx/fy' in out.reason
