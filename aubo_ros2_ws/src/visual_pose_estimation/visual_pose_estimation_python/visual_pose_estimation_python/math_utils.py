"""
数学工具模块（向后兼容重导出）

原有实现已迁移至 ivg_utils.math，本文件保留以兼容已有导入。
"""

from ivg_utils.math import (
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
    rotation_matrix_to_euler_rpy,
    normalize_angle_to_180,
    normalize_angle_to_pi,
    filter_components_by_params,
)

__all__ = [
    'quaternion_to_rotation_matrix',
    'rotation_matrix_to_quaternion',
    'rotation_matrix_to_euler_rpy',
    'normalize_angle_to_180',
    'normalize_angle_to_pi',
    'filter_components_by_params',
]
