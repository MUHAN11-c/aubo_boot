"""backends 工厂."""

from aubo_scene_recon.backends.base import FusionBackend
from aubo_scene_recon.backends.cloud_accum import CloudAccumBackend
from aubo_scene_recon.backends.tsdf_volume import TsdfBackend


def create_backend(
    name: str,
    voxel_size: float,
    max_map_points: int,
    **kwargs,
) -> FusionBackend:
    key = (name or 'open3d').strip().lower()
    if key in ('open3d', 'cloud', 'o3d'):
        return CloudAccumBackend(
            voxel_size=voxel_size,
            max_map_points=max_map_points,
            outlier_every_n=int(kwargs.get('outlier_every_n', 5)),
            outlier_nb_neighbors=int(kwargs.get('outlier_nb_neighbors', 20)),
            outlier_std_ratio=float(kwargs.get('outlier_std_ratio', 2.0)),
        )
    if key == 'tsdf':
        return TsdfBackend(
            voxel_size=voxel_size,
            max_map_points=max_map_points,
            sdf_trunc=float(kwargs.get('sdf_trunc', 0.04)),
            depth_scale=float(kwargs.get('depth_scale', 4000.0)),
            depth_max=float(kwargs.get('depth_max', 1.5)),
        )
    raise ValueError(f'未知 backend: {name!r}（支持 open3d|tsdf）')


__all__ = ['FusionBackend', 'CloudAccumBackend', 'TsdfBackend', 'create_backend']
