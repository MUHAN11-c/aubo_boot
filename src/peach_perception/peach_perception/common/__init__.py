"""peach_perception.common — 拟合/深度/TF/时钟/注册表/runs（纯核）."""
from peach_perception.common.geometry import (
    gravity_camera_from_R,
    invert_transform,
    normalize_depth_to_uint16_mm,
    QuaternionValue,
    relative_motion,
    rotation_to_quat,
    transform_direction,
    transform_msg_to_matrix,
    transform_point,
)
from peach_perception.common.runtime import (
    BoundedWorker,
    Clock,
    default_harvest_root,
    default_runs_root,
    HarvestDataStore,
    ManualClock,
    Registry,
    resolve_runs_root,
)

__all__ = [
    'BoundedWorker',
    'Clock',
    'HarvestDataStore',
    'ManualClock',
    'QuaternionValue',
    'Registry',
    'default_harvest_root',
    'default_runs_root',
    'resolve_runs_root',
    'gravity_camera_from_R',
    'invert_transform',
    'normalize_depth_to_uint16_mm',
    'relative_motion',
    'rotation_to_quat',
    'transform_direction',
    'transform_msg_to_matrix',
    'transform_point',
]
