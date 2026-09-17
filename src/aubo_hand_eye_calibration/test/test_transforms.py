"""Zero-ROS tests for the SE(3) helpers."""

from aubo_hand_eye_calibration.transforms import (
    from_se3_vector,
    inverse,
    make_transform,
    mean_transform,
    se3_vector,
    transform_error,
    transform_from_xyz_quat,
    transform_to_xyz_quat,
)
import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def random_transform(rng):
    rotation = Rotation.random(random_state=rng).as_matrix()
    return make_transform(rotation, rng.uniform(-1.0, 1.0, 3))


@pytest.mark.parametrize('seed', [1, 2, 3])
def test_xyz_quat_round_trip(seed):
    rng = np.random.default_rng(seed)
    transform = random_transform(rng)
    xyz, quaternion = transform_to_xyz_quat(transform)
    restored = transform_from_xyz_quat(xyz, quaternion)
    translation_m, rotation_deg = transform_error(transform, restored)
    assert translation_m < 1e-12
    assert rotation_deg < 1e-9


@pytest.mark.parametrize('seed', [4, 5])
def test_inverse_round_trip(seed):
    rng = np.random.default_rng(seed)
    transform = random_transform(rng)
    composition = transform @ inverse(transform)
    assert np.allclose(composition, np.eye(4), atol=1e-12)


def test_inverse_rejects_scaled_rotation():
    scaled = np.eye(4)
    scaled[:3, :3] = 2.0 * np.eye(3)
    with pytest.raises(ValueError):
        inverse(scaled)


def test_se3_vector_round_trip():
    rng = np.random.default_rng(6)
    transform = random_transform(rng)
    restored = from_se3_vector(se3_vector(transform))
    translation_m, rotation_deg = transform_error(transform, restored)
    assert translation_m < 1e-12
    assert rotation_deg < 1e-9


def test_mean_transform_of_identical_is_identity_member():
    rng = np.random.default_rng(7)
    transform = random_transform(rng)
    average = mean_transform([transform, transform, transform])
    translation_m, rotation_deg = transform_error(transform, average)
    assert translation_m < 1e-12
    assert rotation_deg < 1e-9


def test_mean_transform_requires_input():
    with pytest.raises(ValueError):
        mean_transform([])


def test_transform_error_units():
    reference = np.eye(4)
    estimate = make_transform(
        Rotation.from_euler('z', 90.0, degrees=True).as_matrix(),
        [0.1, 0.0, 0.0],
    )
    translation_m, rotation_deg = transform_error(reference, estimate)
    assert translation_m == pytest.approx(0.1)
    assert rotation_deg == pytest.approx(90.0)
