"""Re-export split modules (behavior-preserving shim)."""
from __future__ import annotations

from peach_perception.common.depth_geometry import normalize_depth_to_uint16_mm
from peach_perception.common.fitting import (
    angle_between_deg,
    axis_radial_distance,
    estimate_normals,
    fit_cylinder_robust,
    fit_sphere_robust,
    polish_cylinder_axis,
    polish_sphere_lm,
    ransac_cylinder,
    ransac_sphere,
    unit_vector,
)
from peach_perception.common.tf_utils import (
    gravity_camera_from_R,
    invert_transform,
    pack_rgb_bgr,
    QuaternionValue,
    relative_motion,
    rotation_to_quat,
    transform_direction,
    transform_msg_to_matrix,
    transform_point,
    transform_points,
)

__all__ = [
    'QuaternionValue',
    'angle_between_deg',
    'axis_radial_distance',
    'estimate_normals',
    'fit_cylinder_robust',
    'fit_sphere_robust',
    'gravity_camera_from_R',
    'invert_transform',
    'normalize_depth_to_uint16_mm',
    'pack_rgb_bgr',
    'polish_cylinder_axis',
    'polish_sphere_lm',
    'ransac_cylinder',
    'ransac_sphere',
    'relative_motion',
    'rotation_to_quat',
    'transform_direction',
    'transform_msg_to_matrix',
    'transform_point',
    'transform_points',
    'unit_vector',
]
