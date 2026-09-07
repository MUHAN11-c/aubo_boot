"""Re-export split modules (behavior-preserving shim)."""
from __future__ import annotations

from peach_perception.target_reconstruction.candidate_contract import (
    axis_from_vector3,
    candidate_axis_hint,
    select_reconstruction_candidate,
    TargetKindMemory,
)
from peach_perception.target_reconstruction.geometry_refiner import (
    _apply_axis_consistency,
    _cylinder_ends,
    _fail,
    _gated_result,
    _normalize_axis_hint,
    _precheck,
    axis_angle_deg,
    CylinderRefitter,
    estimate_normals_knn,
    make_refitter,
    orient_axis_bottom_to_neck,
    refine_geometry,
    RefitConfig,
    select_refitter,
    SphereRefitter,
    STATUS_ACCEPT,
    STATUS_REJECT,
    STATUS_REOBSERVE,
)

__all__ = [
    'CylinderRefitter',
    'RefitConfig',
    'STATUS_ACCEPT',
    'STATUS_REJECT',
    'STATUS_REOBSERVE',
    'SphereRefitter',
    'TargetKindMemory',
    '_apply_axis_consistency',
    '_cylinder_ends',
    '_fail',
    '_gated_result',
    '_normalize_axis_hint',
    '_precheck',
    'axis_angle_deg',
    'axis_from_vector3',
    'candidate_axis_hint',
    'estimate_normals_knn',
    'make_refitter',
    'orient_axis_bottom_to_neck',
    'refine_geometry',
    'select_reconstruction_candidate',
    'select_refitter',
]
