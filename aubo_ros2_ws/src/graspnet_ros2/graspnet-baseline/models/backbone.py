# -*- coding: utf-8 -*-
""" PointNet++ backbone：点云特征学习，基于 Pointnet++ 单尺度分组。
    Author: Charles R. Qi
"""
import os
import sys
import torch
import torch.nn as nn

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'pointnet2'))

from pointnet2_modules import PointnetSAModuleVotes, PointnetFPModule


class Pointnet2Backbone(nn.Module):
    """ 点云 backbone：4 层 Set Abstraction（SA）下采样 + 2 层 Feature Propagation（FP）上采样，输出 fp2 的坐标与特征。 """

    def __init__(self, input_feature_dim=0):
        super().__init__()
        # SA1: 2048 点，半径 0.04，每组 64 点，MLP 出 128 维
        self.sa1 = PointnetSAModuleVotes(
                npoint=2048,
                radius=0.04,
                nsample=64,
                mlp=[input_feature_dim, 64, 64, 128],
                use_xyz=True,
                normalize_xyz=True
            )
        # SA2: 1024 点，半径 0.1，每组 32 点，MLP 出 256 维
        self.sa2 = PointnetSAModuleVotes(
                npoint=1024,
                radius=0.1,
                nsample=32,
                mlp=[128, 128, 128, 256],
                use_xyz=True,
                normalize_xyz=True
            )
        # SA3: 512 点，半径 0.2，每组 16 点，MLP 出 256 维
        self.sa3 = PointnetSAModuleVotes(
                npoint=512,
                radius=0.2,
                nsample=16,
                mlp=[256, 128, 128, 256],
                use_xyz=True,
                normalize_xyz=True
            )
        # SA4: 256 点，半径 0.3，每组 16 点，MLP 出 256 维
        self.sa4 = PointnetSAModuleVotes(
                npoint=256,
                radius=0.3,
                nsample=16,
                mlp=[256, 128, 128, 256],
                use_xyz=True,
                normalize_xyz=True
            )
        # FP1: 从 sa4 上采样到 sa3 的点数，通道 256+256 → 256
        self.fp1 = PointnetFPModule(mlp=[256+256, 256, 256])
        # FP2: 从 sa3 上采样到 sa2 的点数，通道 256+256 → 256；输出即 fp2
        self.fp2 = PointnetFPModule(mlp=[256+256, 256, 256])

    def _break_up_pc(self, pc):
        """ 将 (B, N, 3+C) 拆成 xyz (B, N, 3) 与 features (B, C, N)；若 C=0 则 features 为 None。 """
        xyz = pc[..., 0:3].contiguous()
        features = (
            pc[..., 3:].transpose(1, 2).contiguous()
            if pc.size(-1) > 3 else None
        )
        return xyz, features

    def forward(self, pointcloud: torch.cuda.FloatTensor, end_points=None):
        """ 前向：4 层 SA 下采样，2 层 FP 上采样，返回 fp2 特征、fp2 坐标、end_points。 """
        if not end_points:
            end_points = {}
        batch_size = pointcloud.shape[0]
        xyz, features = self._break_up_pc(pointcloud)
        end_points['input_xyz'] = xyz
        end_points['input_features'] = features

        # --------- 4 层 Set Abstraction ---------
        xyz, features, fps_inds = self.sa1(xyz, features)
        end_points['sa1_inds'] = fps_inds
        end_points['sa1_xyz'] = xyz
        end_points['sa1_features'] = features

        xyz, features, fps_inds = self.sa2(xyz, features)
        end_points['sa2_inds'] = fps_inds
        end_points['sa2_xyz'] = xyz
        end_points['sa2_features'] = features

        xyz, features, fps_inds = self.sa3(xyz, features)
        end_points['sa3_xyz'] = xyz
        end_points['sa3_features'] = features

        xyz, features, fps_inds = self.sa4(xyz, features)
        end_points['sa4_xyz'] = xyz
        end_points['sa4_features'] = features

        # --------- 2 层 Feature Propagation（上采样）---------
        features = self.fp1(end_points['sa3_xyz'], end_points['sa4_xyz'], end_points['sa3_features'], end_points['sa4_features'])
        features = self.fp2(end_points['sa2_xyz'], end_points['sa3_xyz'], end_points['sa2_features'], features)
        end_points['fp2_features'] = features
        end_points['fp2_xyz'] = end_points['sa2_xyz']
        num_seed = end_points['fp2_xyz'].shape[1]
        # fp2 各点对应回原始输入点云的索引（用 sa1 的 inds 前 num_seed 个）
        end_points['fp2_inds'] = end_points['sa1_inds'][:, 0:num_seed]

        return features, end_points['fp2_xyz'], end_points
