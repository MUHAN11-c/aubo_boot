# 默认旋转转换使用 scipy.spatial.transform（rotation_conversion.py）
# 可选无 scipy 分支：from .rotation_conversion_numpy import ... 同 API，仅 NumPy 实现
from .coordinate_systems import (
    CameraIntrinsics,
    project_3d_to_2d,
    project_3d_to_2d_batch,
    unproject_2d_to_3d,
    unproject_2d_to_3d_batch,
    reprojection_error,
    reprojection_error_batch,
    reprojection_error_rms,
)
from .rotation_conversion import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    rpy_to_quaternion,
    quaternion_to_rpy,
    rpy_to_rotation_matrix,
    rotation_matrix_to_rpy,
)
from .point_transform import (
    transform_point,
    transform_direction,
    transform_points,
    transform_pose,
    transform_frame,
    transform_directions,
    transform_line,
    handedness_transform,
    right_to_left,
    left_to_right,
)

__all__ = [
    'CameraIntrinsics', 'project_3d_to_2d', 'project_3d_to_2d_batch',
    'unproject_2d_to_3d', 'unproject_2d_to_3d_batch',
    'reprojection_error', 'reprojection_error_batch', 'reprojection_error_rms',
    'quaternion_to_rotation_matrix', 'rotation_matrix_to_quaternion',
    'rpy_to_quaternion', 'quaternion_to_rpy',
    'rpy_to_rotation_matrix', 'rotation_matrix_to_rpy',
    'transform_point', 'transform_direction', 'transform_points',
    'transform_pose', 'transform_frame', 'transform_directions', 'transform_line',
    'handedness_transform', 'right_to_left', 'left_to_right',
]
