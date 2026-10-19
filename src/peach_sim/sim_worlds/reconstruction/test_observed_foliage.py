"""Check projection and support limits for reference foliage completion."""

import numpy as np
from observed_foliage import leaf_surface


def test_supported_leaf_reprojects_and_faces_camera():
    result = leaf_surface(np.ones((32, 32), dtype=bool),
                          np.full((32, 32), 500, dtype=np.uint16))
    verts, faces = result['vertices'], result['faces']
    u = verts[:, 0] * 640 / verts[:, 1] + 640
    v = 360 - (verts[:, 2] - 1.6) * 640 / verts[:, 1]
    assert np.allclose(u, np.round(u))
    assert np.allclose(v, np.round(v))
    normals = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]],
                       verts[faces[:, 2]] - verts[faces[:, 0]])
    assert np.all(normals[:, 1] < 0)


def test_missing_leaf_depth_is_not_silently_invented():
    mask = np.ones((32, 32), dtype=bool)
    assert leaf_surface(mask, np.zeros((32, 32), dtype=np.uint16)) is None
    depth = np.full((32, 32), 500, dtype=np.uint16)
    depth[:16] = 0
    result = leaf_surface(mask, depth)
    assert result['valid_depth_fraction'] == .5
    assert result['depth_m'] == .5
