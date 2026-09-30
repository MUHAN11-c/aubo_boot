"""Check projection and discontinuity handling in observed foliage surfaces."""

import numpy as np
from observed_foliage import surface_mesh


def test_observed_plane_reprojects_and_faces_camera():
    rgb = np.full((32, 32, 3), [30, 110, 45], dtype=np.uint8)
    depth = np.full((32, 32), 500, dtype=np.uint16)
    verts, faces = surface_mesh(rgb, depth)
    assert len(faces) > 0
    u = verts[:, 0] * 640 / verts[:, 1] + 640
    v = 360 - (verts[:, 2] - 1.6) * 640 / verts[:, 1]
    assert np.allclose(u % 4, 0)
    assert np.allclose(v, np.round(v))
    normals = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]],
                       verts[faces[:, 2]] - verts[faces[:, 0]])
    assert np.all(normals[:, 1] < 0)


def test_missing_depth_and_depth_edges_are_not_bridged():
    rgb = np.full((32, 32, 3), [30, 110, 45], dtype=np.uint8)
    depth = np.full((32, 32), 500, dtype=np.uint16)
    depth[:, 16:] = 900
    depth[:8] = 0
    verts, faces = surface_mesh(rgb, depth)
    assert len(faces) > 0
    assert np.all(np.ptp(verts[faces, 1], axis=1) <= .025)
    _, empty_faces = surface_mesh(rgb, np.zeros_like(depth))
    assert len(empty_faces) == 0
