# -*- coding: utf-8 -*-
"""
视角与旋转工具（vendored 自 graspnet-baseline utils/loss_utils.py，作者 chenxi-wang）.

推理子集：只保留视角采样与 approach→旋转矩阵换算（训练损失函数未随附）。
"""

import numpy as np
import torch

# 抓取宽度上限（米）
GRASP_MAX_WIDTH = 0.1
# 抓取容差上限（米）
GRASP_MAX_TOLERANCE = 0.05


def generate_grasp_views(N=300, phi=(np.sqrt(5)-1)/2, center=np.zeros(3), r=1):
    """
    在单位球面上用 Fibonacci 格点均匀采样 N 个视角方向.
    参考: https://arxiv.org/abs/0912.4540
    输入: N 视角数, phi 黄金角常数, center 球心, r 半径
    输出: views (N,3) torch，每个为单位方向（乘 r 后）
    """
    views = []
    for i in range(N):
        # z 在 [-1,1] 上均匀： (2i+1)/N - 1
        zi = (2 * i + 1) / N - 1
        # 球面坐标：水平半径 sqrt(1-z^2)，角度 2*pi*i*phi
        xi = np.sqrt(1 - zi**2) * np.cos(2 * i * np.pi * phi)
        yi = np.sqrt(1 - zi**2) * np.sin(2 * i * np.pi * phi)
        views.append([xi, yi, zi])
    # 缩放到半径 r 并平移到 center
    views = r * np.array(views) + center
    # 转为 float32 的 torch 张量
    return torch.from_numpy(views.astype(np.float32))


def batch_viewpoint_params_to_matrix(batch_towards, batch_angle):
    """
    将“接近方向向量 + 绕该方向的旋转角”批量转换为 3x3 旋转矩阵.
    输入: batch_towards (N,3) approach 方向, batch_angle (N,) 绕 approach 的角（弧度）
    输出: batch_matrix (N,3,3)
    """
    # 局部 x 轴取为 approach 方向
    axis_x = batch_towards
    # 全 1、全 0 向量，与 axis_x 同 dtype、device，长度 N
    ones = torch.ones(axis_x.shape[0], dtype=axis_x.dtype, device=axis_x.device)
    zeros = torch.zeros(axis_x.shape[0], dtype=axis_x.dtype, device=axis_x.device)
    # 构造与 x 正交的 y：取 (-x1, x0, 0)，即与 z 轴叉乘方向
    axis_y = torch.stack([-axis_x[:, 1], axis_x[:, 0], zeros], dim=-1)
    # 若 axis_x 与 z 平行则 axis_y 为零向量，将 y 分量置 1 避免退化
    mask_y = (torch.norm(axis_y, dim=-1) == 0)
    axis_y[mask_y, 1] = 1
    # x 方向单位化（避免除零导致 NaN，进而触发 CUBLAS_STATUS_INVALID_VALUE）
    norm_x = torch.norm(axis_x, dim=-1, keepdim=True).clamp(min=1e-6)
    axis_x = axis_x / norm_x
    # y 方向单位化
    norm_y = torch.norm(axis_y, dim=-1, keepdim=True).clamp(min=1e-6)
    axis_y = axis_y / norm_y
    # z = x × y，构成右手系
    axis_z = torch.linalg.cross(axis_x, axis_y, dim=-1)
    # 绕 x 轴的旋转角 batch_angle 的 sin, cos
    sin = torch.sin(batch_angle)
    cos = torch.cos(batch_angle)
    # 绕 x 轴旋转矩阵 R1 的 9 个元素（行优先）：[1,0,0; 0,cos,-sin; 0,sin,cos]
    R1 = torch.stack([ones, zeros, zeros, zeros, cos, -sin, zeros, sin, cos], dim=-1)
    # 变为 (N, 3, 3) 并保证连续，避免 CUDA batched matmul 的 stride 问题
    R1 = R1.reshape([-1, 3, 3]).contiguous()
    # 局部轴到世界轴：列向量为 x,y,z，即 R2 的列是基向量
    R2 = torch.stack([axis_x, axis_y, axis_z], dim=-1).contiguous()
    # 先绕局部 x 转 batch_angle，再转到世界系：R = R2 @ R1
    # 在 CUDA 上 cublasSgemmStridedBatched 可能报 CUBLAS_STATUS_INVALID_VALUE，改为在 CPU 上做 matmul 再搬回
    device = R1.device
    if device.type == 'cuda':
        R1_cpu, R2_cpu = R1.cpu(), R2.cpu()
        batch_matrix = torch.matmul(R2_cpu, R1_cpu).to(device=device)
    else:
        batch_matrix = torch.matmul(R2, R1)
    return batch_matrix
