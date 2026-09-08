# -*- coding: utf-8 -*-
"""纯 torch 点云算子与分组器（替代 graspnet-baseline 的 pointnet2/knn CUDA 扩展）.

算子部分语义与 CUDA kernel 逐条对齐（见上游 _ext_src/src/*.cu）：
- ball_query/cylinder_query：距离严格小于半径（d2 < r*r）才入选；按点索引升序取
  前 nsample 个；命中不足时用首个命中索引填充；一个都没有则保持全 0。
- cylinder_query：先减中心、再用旋转矩阵把偏移转到圆柱局部系（x=approach），
  判据为 y^2+z^2 < r^2 且 hmin < x < hmax（均为严格不等号）。
- furthest_point_sample：标准最远点采样，初始距离 1e10、起点为索引 0。

与 CUDA 版的差异仅在浮点求和顺序与并列邻居的次序，不影响推理结果的有效性。
分组器部分（QueryAndGroup/CylinderQueryAndGroup）为推理所需的裁剪版，
去掉 sample_uniformly/ret_unique_cnt 等推理用不到的选项路径。
"""

from __future__ import annotations

import torch
import torch.nn as nn

# new_xyz 每次分块的行数：限制 ball/cylinder 查询的峰值显存/内存
_QUERY_CHUNK_ROWS = 256


def furthest_point_sample(xyz: torch.Tensor, npoint: int) -> torch.Tensor:
    """最远点采样.

    Args:
        xyz: (B, N, 3) 点坐标。
        npoint: 采样点数。

    Returns:
        (B, npoint) long，被选中的点索引。
    """
    device = xyz.device
    centroids = torch.zeros(xyz.shape[0], npoint, dtype=torch.long, device=device)
    distance = torch.full((xyz.shape[0], xyz.shape[1]), 1e10, device=device)
    farthest = torch.zeros(xyz.shape[0], dtype=torch.long, device=device)
    batch_indices = torch.arange(xyz.shape[0], device=device)
    for i in range(npoint):
        centroids[:, i] = farthest
        centroid = xyz[batch_indices, farthest].view(-1, 1, 3)
        dist = ((xyz - centroid) ** 2).sum(-1)
        closer = dist < distance
        distance[closer] = dist[closer]
        farthest = torch.max(distance, -1)[1]
    return centroids


def gather_operation(features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """按索引在点维gather特征.

    Args:
        features: (B, C, N) 特征。
        idx: (B, npoint) long 索引。

    Returns:
        (B, C, npoint)。
    """
    idx_exp = idx.unsqueeze(1).expand(-1, features.shape[1], -1)
    return torch.gather(features, 2, idx_exp)


def three_nn(unknown: torch.Tensor, known: torch.Tensor):
    """对 unknown 中每点找 known 中最近 3 邻居.

    Args:
        unknown: (B, n, 3)。
        known: (B, m, 3)。

    Returns:
        (dist (B, n, 3), idx (B, n, 3))，dist 为欧氏距离。
    """
    with torch.no_grad():
        # torch.cdist 直接返回欧氏距离（非平方），无需再开方
        dist = torch.cdist(unknown, known)
        dist, idx = torch.topk(dist, 3, dim=-1, largest=False)
    return dist, idx


def three_interpolate(
    features: torch.Tensor, idx: torch.Tensor, weight: torch.Tensor
) -> torch.Tensor:
    """三邻居加权插值.

    Args:
        features: (B, c, m) 源特征。
        idx: (B, n, 3) 邻居索引。
        weight: (B, n, 3) 权重（和为 1）。

    Returns:
        (B, c, n) 插值结果。
    """
    b, c, _ = features.shape
    n = idx.shape[1]
    idx_flat = idx.reshape(b, 1, n * 3).expand(b, c, n * 3)
    gathered = torch.gather(features, 2, idx_flat).view(b, c, n, 3)
    return (gathered * weight.unsqueeze(1)).sum(dim=-1)


def group_points(features: torch.Tensor, idx: torch.Tensor) -> torch.Tensor:
    """按 (npoint, nsample) 索引网格 gather 特征.

    Args:
        features: (B, C, N)。
        idx: (B, npoint, nsample) long。

    Returns:
        (B, C, npoint, nsample)。
    """
    b, c, _ = features.shape
    npoint, nsample = idx.shape[1], idx.shape[2]
    idx_flat = idx.reshape(b, 1, npoint * nsample).expand(b, c, npoint * nsample)
    return torch.gather(features, 2, idx_flat).view(b, c, npoint, nsample)


def _first_hit_fill(
    mask: torch.Tensor, nsample: int, sentinel_count: torch.Tensor
) -> torch.Tensor:
    """把布尔命中掩码 (B, m, N) 转成 (B, m, nsample) 索引.

    语义对齐 CUDA kernel：升序取前 nsample 个命中；不足时用首个命中填充；
    全无命中则为 0。sentinel_count 为每个中心的总命中数（用于区分 0 命中）。
    """
    b, m, n = mask.shape
    device = mask.device
    k_idx = torch.arange(n, device=device).view(1, 1, n).expand(b, m, n)
    val = torch.where(mask, k_idx, torch.full_like(k_idx, n))
    top, _ = torch.topk(val, nsample, dim=-1, largest=False)  # 升序有效索引在前，哨兵 n 在尾
    first = top[..., :1]
    filled = torch.where(top >= n, first, top)
    # 无任何命中：top 全为哨兵 → 归零（kernel 初始化为 0）
    zero_rows = sentinel_count == 0  # (b, m, 1)，可直接广播到 (b, m, nsample)
    return torch.where(zero_rows, torch.zeros_like(filled), filled)


def ball_query(
    radius: float, nsample: int, xyz: torch.Tensor, new_xyz: torch.Tensor
) -> torch.Tensor:
    """球形邻域查询.

    Args:
        radius: 球半径（米）。
        nsample: 每个中心最多取的点数。
        xyz: (B, N, 3) 原始点。
        new_xyz: (B, M, 3) 查询中心。

    Returns:
        (B, M, nsample) long 索引。
    """
    b, n, _ = xyz.shape
    m_total = new_xyz.shape[1]
    device = xyz.device
    out = torch.zeros(b, m_total, nsample, dtype=torch.long, device=device)
    radius2 = radius * radius
    for start in range(0, m_total, _QUERY_CHUNK_ROWS):
        end = min(start + _QUERY_CHUNK_ROWS, m_total)
        centers = new_xyz[:, start:end]
        d2 = ((centers.unsqueeze(2) - xyz.unsqueeze(1)) ** 2).sum(-1)  # (B, chunk, N)
        mask = d2 < radius2
        counts = mask.sum(-1, keepdim=True)
        out[:, start:end] = _first_hit_fill(mask, nsample, counts)
    return out


def cylinder_query(
    radius: float,
    hmin: float,
    hmax: float,
    nsample: int,
    xyz: torch.Tensor,
    new_xyz: torch.Tensor,
    rot: torch.Tensor,
) -> torch.Tensor:
    """圆柱邻域查询（GraspNet CloudCrop 用）.

    把 (点 - 中心) 旋转到圆柱局部系（x=approach 方向），判据：
    y^2 + z^2 < radius^2 且 hmin < x < hmax。

    Args:
        radius: 圆柱半径。
        hmin/hmax: 圆柱沿 x 的下/上界。
        nsample: 每个中心最多取的点数。
        xyz: (B, N, 3) 原始点。
        new_xyz: (B, M, 3) 查询中心。
        rot: (B, M, 9) 展平的旋转矩阵（行优先，圆柱局部系到世界系）。

    Returns:
        (B, M, nsample) long 索引。
    """
    b, n, _ = xyz.shape
    m_total = new_xyz.shape[1]
    device = xyz.device
    out = torch.zeros(b, m_total, nsample, dtype=torch.long, device=device)
    radius2 = radius * radius
    for start in range(0, m_total, _QUERY_CHUNK_ROWS):
        end = min(start + _QUERY_CHUNK_ROWS, m_total)
        centers = new_xyz[:, start:end]
        rots = rot[:, start:end].reshape(-1, end - start, 3, 3)
        offset = xyz.unsqueeze(1) - centers.unsqueeze(2)  # (B, chunk, N, 3)
        local = torch.matmul(offset, rots)  # (B, chunk, N, 3)，分量 = 与旋转矩阵列的点积
        d2 = local[..., 1] ** 2 + local[..., 2] ** 2
        mask = (d2 < radius2) & (local[..., 0] > hmin) & (local[..., 0] < hmax)
        counts = mask.sum(-1, keepdim=True)
        out[:, start:end] = _first_hit_fill(mask, nsample, counts)
    return out


class QueryAndGroup(nn.Module):
    """球形邻域分组（backbone SA 层用）.

    Args:
        radius: 球半径。
        nsample: 每组最多点数。
        use_xyz: 是否把坐标拼进特征。
        ret_grouped_xyz: 是否额外返回分组坐标。
        normalize_xyz: 是否把分组坐标除以 radius。
    """

    def __init__(self, radius, nsample, use_xyz=True, ret_grouped_xyz=False,
                 normalize_xyz=False):
        super().__init__()
        self.radius, self.nsample, self.use_xyz = radius, nsample, use_xyz
        self.ret_grouped_xyz = ret_grouped_xyz
        self.normalize_xyz = normalize_xyz

    def forward(self, xyz, new_xyz, features=None):
        """xyz (B,N,3)、new_xyz (B,npoint,3)、features (B,C,N) → (B,3+C,npoint,nsample)."""
        idx = ball_query(self.radius, self.nsample, xyz, new_xyz)

        xyz_trans = xyz.transpose(1, 2).contiguous()
        grouped_xyz = group_points(xyz_trans, idx)  # (B,3,npoint,nsample)
        grouped_xyz -= new_xyz.transpose(1, 2).unsqueeze(-1)
        if self.normalize_xyz:
            grouped_xyz /= self.radius

        if features is not None:
            grouped_features = group_points(features, idx)
            if self.use_xyz:
                new_features = torch.cat([grouped_xyz, grouped_features], dim=1)
            else:
                new_features = grouped_features
        else:
            assert self.use_xyz, '无特征时必须 use_xyz'
            new_features = grouped_xyz

        if self.ret_grouped_xyz:
            return new_features, grouped_xyz
        return new_features


class CylinderQueryAndGroup(nn.Module):
    """圆柱邻域分组（GraspNet CloudCrop 用）.

    Args:
        radius: 圆柱半径。
        hmin/hmax: 圆柱沿局部 x（approach）的下/上界。
        nsample: 每组最多点数。
        rotate_xyz: 是否把分组坐标转到圆柱局部系。
    """

    def __init__(self, radius, hmin, hmax, nsample, use_xyz=True, rotate_xyz=True):
        super().__init__()
        self.radius, self.nsample, self.hmin, self.hmax = radius, nsample, hmin, hmax
        self.use_xyz = use_xyz
        self.rotate_xyz = rotate_xyz

    def forward(self, xyz, new_xyz, rot, features=None):
        """xyz (B,N,3)、new_xyz (B,npoint,3)、rot (B,npoint,3,3) → (B,3+C,npoint,nsample)."""
        b, npoint, _ = new_xyz.size()
        idx = cylinder_query(
            self.radius, self.hmin, self.hmax, self.nsample,
            xyz, new_xyz, rot.reshape(b, npoint, 9),
        )

        xyz_trans = xyz.transpose(1, 2).contiguous()
        grouped_xyz = group_points(xyz_trans, idx)  # (B,3,npoint,nsample)
        grouped_xyz -= new_xyz.transpose(1, 2).unsqueeze(-1)
        if self.rotate_xyz:
            grouped_xyz_ = grouped_xyz.permute(0, 2, 3, 1).contiguous()  # (B,npoint,nsample,3)
            # 与上游一致：局部系旋转在 CPU 上做 matmul 再搬回，绕开部分
            # CUDA cuBLAS strided-batched 的兼容问题
            dev = grouped_xyz_.device
            if dev.type == 'cuda':
                grouped_xyz_ = torch.matmul(grouped_xyz_.cpu(), rot.cpu()).to(device=dev)
            else:
                grouped_xyz_ = torch.matmul(grouped_xyz_, rot)
            grouped_xyz = grouped_xyz_.permute(0, 3, 1, 2).contiguous()

        if features is not None:
            grouped_features = group_points(features, idx)
            if self.use_xyz:
                new_features = torch.cat([grouped_xyz, grouped_features], dim=1)
            else:
                new_features = grouped_features
        else:
            assert self.use_xyz, '无特征时必须 use_xyz'
            new_features = grouped_xyz
        return new_features
