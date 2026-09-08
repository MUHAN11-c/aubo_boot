# -*- coding: utf-8 -*-
"""Pointnet++ SA/FP 层（vendored 裁剪版，仅保留 backbone 用到的两个模块）.

基于 graspnet-baseline pointnet2/pointnet2_modules.py（原作者 Facebook, Inc.，
MIT）；MSG 变体、RBF/avg 池化、sample_uniformly 等训练辅助未随附。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import List

from graspnet_ros2.graspnet_lib import pointnet2_ops as pointnet2_utils
from graspnet_ros2.graspnet_lib import pytorch_utils as pt_utils


class PointnetSAModuleVotes(nn.Module):
    """Set Abstraction：FPS 采样 + 球邻域分组 + SharedMLP + max 池化."""

    def __init__(
            self,
            *,
            mlp: List[int],
            npoint: int,
            radius: float,
            nsample: int,
            bn: bool = True,
            use_xyz: bool = True,
            normalize_xyz: bool = False,
    ):
        super().__init__()

        self.npoint = npoint
        self.radius = radius
        self.nsample = nsample
        self.use_xyz = use_xyz
        self.normalize_xyz = normalize_xyz

        self.grouper = pointnet2_utils.QueryAndGroup(
            radius, nsample, use_xyz=use_xyz,
            ret_grouped_xyz=True, normalize_xyz=normalize_xyz,
        )

        mlp_spec = list(mlp)
        if use_xyz and len(mlp_spec) > 0:
            mlp_spec[0] += 3
        self.mlp_module = pt_utils.SharedMLP(mlp_spec, bn=bn)

    def forward(self, xyz: torch.Tensor,
                features: torch.Tensor = None) -> (torch.Tensor, torch.Tensor):
        """
        xyz: (B, N, 3)；features: (B, C, N)。
        返回 (new_xyz (B, npoint, 3), new_features (B, mlp[-1], npoint), inds (B, npoint))。
        """
        xyz_flipped = xyz.transpose(1, 2).contiguous()
        inds = pointnet2_utils.furthest_point_sample(xyz, self.npoint)
        new_xyz = pointnet2_utils.gather_operation(
            xyz_flipped, inds
        ).transpose(1, 2).contiguous()

        grouped_features, _ = self.grouper(xyz, new_xyz, features)

        new_features = self.mlp_module(grouped_features)
        new_features = F.max_pool2d(
            new_features, kernel_size=[1, new_features.size(3)]
        )  # (B, mlp[-1], npoint, 1)
        new_features = new_features.squeeze(-1)  # (B, mlp[-1], npoint)

        return new_xyz, new_features, inds


class PointnetFPModule(nn.Module):
    """Feature Propagation：三邻居加权插值 + 特征拼接 + SharedMLP."""

    def __init__(self, *, mlp: List[int], bn: bool = True):
        super().__init__()
        self.mlp = pt_utils.SharedMLP(mlp, bn=bn)

    def forward(
            self, unknown: torch.Tensor, known: torch.Tensor,
            unknow_feats: torch.Tensor, known_feats: torch.Tensor
    ) -> torch.Tensor:
        """
        unknown: (B, n, 3)；known: (B, m, 3)；
        unknow_feats: (B, C1, n)；known_feats: (B, C2, m)。
        返回 (B, mlp[-1], n)。
        """
        if known is not None:
            dist, idx = pointnet2_utils.three_nn(unknown, known)
            dist_recip = 1.0 / (dist + 1e-8)
            norm = torch.sum(dist_recip, dim=2, keepdim=True)
            weight = dist_recip / norm

            interpolated_feats = pointnet2_utils.three_interpolate(
                known_feats, idx, weight
            )
        else:
            interpolated_feats = known_feats.expand(
                *known_feats.size()[0:2], unknown.size(1)
            )

        if unknow_feats is not None:
            new_features = torch.cat([interpolated_feats, unknow_feats],
                                     dim=1)  # (B, C2 + C1, n)
        else:
            new_features = interpolated_feats

        new_features = new_features.unsqueeze(-1)
        new_features = self.mlp(new_features)

        return new_features.squeeze(-1)
