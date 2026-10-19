"""Analytical oracles for geometry, independent of legacy implementation."""
import numpy as np

from sim_perception.vision.common.bag_landmarks import estimate_bag_landmarks
from sim_perception.vision.common.geometry import (
    estimate_normals, fit_sphere_robust,
)
from sim_perception.vision.target_reconstruction.refine import RefitResult
import pytest


def test_off_axis_pinhole_plane_normals_are_orthogonal_to_the_plane():
    intrinsics = {'fx': 460.0, 'fy': 461.0, 'cx': 320.0, 'cy': 240.0}
    xoff, yoff = 500, 340
    u, v = np.meshgrid(np.arange(43) + xoff, np.arange(41) + yoff)
    rays_x = (u - intrinsics['cx']) / intrinsics['fx']
    rays_y = (v - intrinsics['cy']) / intrinsics['fy']
    # -0.7 X + 0.6 Y + Z = 0.8 m. Float depths isolate projection math.
    depth_mm = 800.0 / (1.0 - 0.7 * rays_x + 0.6 * rays_y)
    normals, valid = estimate_normals(depth_mm, xoff, yoff, intrinsics)
    expected = np.array([0.7, -0.6, -1.0])
    expected /= np.linalg.norm(expected)
    assert valid[1:-1, 1:-1].all()
    np.testing.assert_allclose(normals[valid], np.broadcast_to(
        expected, normals[valid].shape), atol=1e-10)


def test_normals_reject_invalid_neighbors_even_when_jump_gate_is_disabled():
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    depth[2, 1] = 0
    k = {'fx': 400.0, 'fy': 400.0, 'cx': 2.0, 'cy': 2.0}
    _, valid = estimate_normals(depth, 0, 0, k, depth_jump_mm=100000)
    assert not valid[2, 2]


def test_sphere_radius_prior_is_the_given_radius_without_normals():
    rng = np.random.default_rng(8)
    directions = rng.normal(size=(300, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    points = np.array([0.2, -0.1, 0.6]) + 0.030 * directions
    fixed = fit_sphere_robust(points, radius_prior=0.040, thresh=0.02)
    assert fixed is not None
    assert fixed['radius'] == pytest.approx(0.040)
    free = fit_sphere_robust(points, radius_prior=None)
    assert free is not None
    assert free['radius'] == pytest.approx(0.030, abs=1e-8)


def test_bag_auxiliary_sphere_does_not_change_when_cloud_order_changes():
    angles, z = np.meshgrid(np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False),
                            np.linspace(0.4, 0.56, 30))
    points = np.column_stack((0.035 * np.cos(angles).ravel(),
                              0.035 * np.sin(angles).ravel(), z.ravel()))
    first = estimate_bag_landmarks(points, np.array([0.0, 0.0, -1.0]))
    assert first.fruit_prior_center is not None
    for seed in (8, 18, 26):
        order = np.random.default_rng(seed).permutation(len(points))
        other = estimate_bag_landmarks(points[order], np.array([0.0, 0.0, -1.0]))
        np.testing.assert_allclose(other.fruit_prior_center,
                                   first.fruit_prior_center, atol=1e-10)
        assert other.fruit_prior_radius_m == pytest.approx(
            first.fruit_prior_radius_m, abs=1e-10)


def test_cut_distance_diagnostics_label_auxiliary_source_and_missing_data():
    unavailable = RefitResult().as_dict()
    assert unavailable['cut_distance_source'] == 'unavailable'
    assert unavailable['cut_to_fruit_m'] is None
    estimate = RefitResult(
        model_revision='target_0:1:2', fruit_prior_radius_m=0.035,
        cut_to_fruit_m=0.0089).as_dict()
    assert estimate['cut_distance_source'] == 'bag_surface_sphere_auxiliary'
    assert estimate['model_revision'] == 'target_0:1:2'
    assert estimate['cut_to_fruit_m'] == pytest.approx(0.0089)


@pytest.mark.parametrize('polish,radius_range', [
    (True, (0.015, 0.12)),
    (False, (0.0236, 0.02365)),
])
def test_final_cylinder_radius_respects_requested_range(polish, radius_range):
    from sim_perception.vision.common.geometry import fit_cylinder_robust

    theta, z = np.meshgrid(np.linspace(-np.pi / 2, np.pi / 2, 20),
                           np.linspace(-0.03, 0.03, 10))
    points = np.column_stack((0.003 * np.cos(theta.ravel()),
                              0.003 * np.sin(theta.ravel()), z.ravel()))
    # Unreliable normals can admit an initial larger-radius RANSAC model.
    # The final geometry must still honor the caller's range in both modes.
    normals = np.column_stack((np.cos(0.1 * theta.ravel()),
                               np.sin(0.1 * theta.ravel()), np.zeros(theta.size)))
    fitted = fit_cylinder_robust(
        points, normals, radius_range=radius_range, polish=polish)
    assert fitted is None
