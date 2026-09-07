"""Re-export split modules (behavior-preserving shim)."""
from __future__ import annotations

from peach_perception.target_reconstruction.cloud_builder import (
    _depth_image_o3d,
    _intrinsic_o3d,
    apply_target_mask,
    backproject_depth,
    build_cloud_base,
    Open3dCloudBuilder,
    valid_depth_mask,
    valid_depth_ratio,
)
from peach_perception.target_reconstruction.icp_refiner import (
    _quality_ok,
    BoundedIcp,
    IcpConfig,
    IcpResult,
)
from peach_perception.target_reconstruction.icp_target_cache import (
    IcpTargetCache,
    IcpTargetRefreshConfig,
)
from peach_perception.target_reconstruction.overlap import (
    assembly_overlap_metrics,
    cloud_centroid,
    nn_distance_stats_mm,
    subsample_points,
    summarize_pairs_mm,
)
from peach_perception.target_reconstruction.tsdf_volume import LocalTsdf, require_open3d
from peach_perception.target_reconstruction.view_coverage import (
    _angle_deg,
    summarize_view_coverage,
)

__all__ = [
    'BoundedIcp',
    'IcpConfig',
    'IcpResult',
    'IcpTargetCache',
    'IcpTargetRefreshConfig',
    'LocalTsdf',
    'Open3dCloudBuilder',
    '_angle_deg',
    '_depth_image_o3d',
    '_intrinsic_o3d',
    '_quality_ok',
    'apply_target_mask',
    'assembly_overlap_metrics',
    'backproject_depth',
    'build_cloud_base',
    'cloud_centroid',
    'nn_distance_stats_mm',
    'require_open3d',
    'subsample_points',
    'summarize_pairs_mm',
    'summarize_view_coverage',
    'valid_depth_mask',
    'valid_depth_ratio',
]
