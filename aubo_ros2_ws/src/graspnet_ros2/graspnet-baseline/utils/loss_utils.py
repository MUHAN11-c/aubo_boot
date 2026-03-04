# -*- coding: utf-8 -*-
"""
损失计算与视角相关工具：点云变换（PyTorch）、球面视角采样、朝向转旋转矩阵、Huber 损失等。
用于 GraspNet 训练流程（如 models/graspnet.py、models/loss.py）。
作者: chenxi-wang
"""

import torch
import numpy as np

# 抓取宽度上限（米），标签中宽度超过此值的抓取在训练时会被忽略
GRASP_MAX_WIDTH = 0.1
# 抓取容差上限（米）
GRASP_MAX_TOLERANCE = 0.05
# 正样本阈值：质量得分 >= 此值视为好抓取
THRESH_GOOD = 0.7
# 负样本阈值：质量得分 <= 此值视为坏抓取
THRESH_BAD = 0.1


def transform_point_cloud(cloud, transform, format='4x4'):
    """
    使用变换矩阵将点云从原坐标系变换到新坐标系（PyTorch 版本）。
    输入: cloud (N,3), transform (3,3)/(3,4)/(4,4), format 指定形式
    输出: cloud_transformed (N,3)
    """
    # 检查 format 是否合法
    if not (format == '3x3' or format == '4x4' or format == '3x4'):
        raise ValueError('Unknown transformation format, only support \'3x3\' or \'4x4\' or \'3x4\'.')
    if format == '3x3':
        # 仅旋转：transform @ cloud.T 得 (3,N)，再 .T 得 (N,3)
        cloud_transformed = torch.matmul(transform, cloud.T).T
    elif format == '4x4' or format == '3x4':
        # 与 cloud 同设备、同 dtype 的 (N,1) 全 1
        ones = cloud.new_ones(cloud.size(0), device=cloud.device).unsqueeze(-1)
        # 拼成齐次坐标 (N, 4)
        cloud_ = torch.cat([cloud, ones], dim=1)
        # 左乘变换矩阵
        cloud_transformed = torch.matmul(transform, cloud_.T).T
        # 只取前三维
        cloud_transformed = cloud_transformed[:, :3]
    return cloud_transformed


def generate_grasp_views(N=300, phi=(np.sqrt(5)-1)/2, center=np.zeros(3), r=1):
    """
    在单位球面上用 Fibonacci 格点均匀采样 N 个视角方向。
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
    将“接近方向向量 + 绕该方向的旋转角”批量转换为 3x3 旋转矩阵。
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


def huber_loss(error, delta=1.0):
    """
    Huber 损失：|error|<=delta 时为 0.5*error^2，否则为 0.5*delta^2 + delta*(|error|-delta)。
    Ref: Frustum PointNets model_util.py
    输入: error 任意形状, delta 拐点
    输出: loss 与 error 同形状
    """
    # 误差绝对值
    abs_error = torch.abs(error)
    # 在 delta 处截断，小于等于 delta 的部分保留
    quadratic = torch.clamp(abs_error, max=delta)
    # 超出 delta 的线性部分：abs_error - quadratic
    linear = (abs_error - quadratic)
    # 二次部分 0.5*quadratic^2 + 线性部分 delta*linear
    loss = 0.5 * quadratic**2 + delta * linear
    return loss
