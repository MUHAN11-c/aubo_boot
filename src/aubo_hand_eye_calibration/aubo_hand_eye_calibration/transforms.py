# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Small, explicit SE(3) helpers using T_parent_child notation."""

import numpy as np
from scipy.spatial.transform import Rotation


def make_transform(rotation, translation):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def transform_from_xyz_quat(xyz, quat_xyzw):
    return make_transform(
        Rotation.from_quat(np.asarray(quat_xyzw, dtype=np.float64)).as_matrix(),
        xyz,
    )


def transform_to_xyz_quat(transform):
    transform = np.asarray(transform, dtype=np.float64)
    return (
        transform[:3, 3].tolist(),
        Rotation.from_matrix(transform[:3, :3]).as_quat().tolist(),
    )


def inverse(transform):
    transform = np.asarray(transform, dtype=np.float64)
    rotation = transform[:3, :3]
    if abs(np.linalg.det(rotation) - 1.0) > 1e-3:
        raise ValueError('not a valid rotation matrix (det != 1)')
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation.T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def se3_vector(transform):
    transform = np.asarray(transform, dtype=np.float64)
    return np.r_[
        transform[:3, 3],
        Rotation.from_matrix(transform[:3, :3]).as_rotvec(),
    ]


def from_se3_vector(vector):
    vector = np.asarray(vector, dtype=np.float64)
    return make_transform(Rotation.from_rotvec(vector[3:]).as_matrix(), vector[:3])


def mean_transform(transforms):
    transforms = [np.asarray(t, dtype=np.float64) for t in transforms]
    if not transforms:
        raise ValueError('at least one transform is required')
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    result[:3, :3] = Rotation.from_matrix(
        [t[:3, :3] for t in transforms]).mean().as_matrix()
    return result


def transform_error(reference, estimate):
    delta = inverse(reference) @ estimate
    translation_m = float(np.linalg.norm(delta[:3, 3]))
    rotation_deg = float(np.degrees(
        np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
    return translation_m, rotation_deg
