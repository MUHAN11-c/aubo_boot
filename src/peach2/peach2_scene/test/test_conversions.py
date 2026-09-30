import math

import numpy as np
from peach2_scene.conversions import depth_to_metres, pose_matrix, target_capsule
import pytest


def test_uint16_depth_scaled_and_invalid_zeroed():
    raw = np.array([[0, 4000, 65535]], dtype=np.uint16)
    out = depth_to_metres(raw, '16UC1', 0.00025)
    assert out.dtype == np.float32
    assert np.allclose(out, [[0.0, 1.0, 0.0]])
    assert depth_to_metres(raw, 'mono16', 0.001)[0, 1] == pytest.approx(4.0)


def test_float_depth_is_metres_and_nan_zeroed():
    raw = np.array([[0.5, np.nan, np.inf, -1.0]], dtype=np.float32)
    out = depth_to_metres(raw, '32FC1', 123.0)
    assert np.allclose(out, [[0.5, 0.0, 0.0, 0.0]])
    assert np.isnan(raw[0, 1])
    with pytest.raises(ValueError):
        depth_to_metres(raw, 'rgb8', 0.001)


def test_pose_matrix_normalises_quaternion():
    s = math.sqrt(0.5)
    T = pose_matrix([1.0, 2.0, 3.0], [0.0, 0.0, 2 * s, 2 * s])
    assert np.allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(T[:3, 3], [1, 2, 3])
    with pytest.raises(ValueError):
        pose_matrix([0, 0, 0], [0, 0, 0, 0])


def test_target_capsule_validation():
    ok = target_capsule('t1', True, (0, 0, 0), True, (0, 0, 0.1), 0.08)
    assert ok is not None and ok.d95_m == 0.08 and np.allclose(ok.neck, [0, 0, 0.1])
    assert target_capsule('t1', False, (0, 0, 0), True, (0, 0, 0.1), 0.08) is None
    assert target_capsule('t1', True, (0, 0, np.nan), True, (0, 0, 0.1), 0.08) is None
    assert target_capsule('t1', True, (0, 0, 0), True, (0, 0, 0.1), 0.0) is None
    assert target_capsule('t1', True, (0, 0, 0), True, (0, 0, 0.1), math.inf) is None
    assert target_capsule('', True, (0, 0, 0), True, (0, 0, 0.1), 0.08) is None
