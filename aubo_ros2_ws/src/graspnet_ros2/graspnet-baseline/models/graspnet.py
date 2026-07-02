# -*- coding: utf-8 -*-
""" GraspNet baseline 模型定义：Stage1 视角估计 + Stage2 抓取参数预测，以及 pred_decode 解码。
    Author: chenxi-wang
"""

import os
import sys
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

# 当前文件所在目录（models）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 项目根目录（graspnet-baseline）
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'pointnet2'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from backbone import Pointnet2Backbone
from modules import ApproachNet, CloudCrop, OperationNet, ToleranceNet
from loss import get_loss
from loss_utils import GRASP_MAX_WIDTH, GRASP_MAX_TOLERANCE
from label_generation import process_grasp_labels, match_grasp_view_and_label, batch_viewpoint_params_to_matrix


class GraspNetStage1(nn.Module):
    """ 第一阶段：点云特征提取 + 每个 seed 点的最佳 approach 视角估计。 """

    def __init__(self, input_feature_dim=0, num_view=300):
        super().__init__()
        # 点云 backbone：PointNet++ 多尺度 SA + FP，输出 seed 点坐标与特征
        self.backbone = Pointnet2Backbone(input_feature_dim)
        # 视角模块：根据 seed 特征预测 objectness 与 num_view 个视角的得分，取 top 视角
        self.vpmodule = ApproachNet(num_view, 256)

    def forward(self, end_points):
        # 输入点云 (B, N, 3) 或 (B, N, 3+feature_dim)
        pointcloud = end_points['point_clouds']
        # backbone 前向：得到 seed 特征、seed 坐标，并写入 end_points（如 fp2_xyz, fp2_features 等）
        seed_features, seed_xyz, end_points = self.backbone(pointcloud, end_points)
        # 视角网络：为每个 seed 预测最佳 approach 方向，写入 grasp_top_view_* 等
        end_points = self.vpmodule(seed_xyz, seed_features, end_points)
        return end_points


class GraspNetStage2(nn.Module):
    """ 第二阶段：在 Stage1 的 top 视角下，对每个 seed 预测抓取分数、角度类、宽度、容差。 """

    def __init__(self, num_angle=12, num_depth=4, cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=True):
        super().__init__()
        self.num_angle = num_angle
        self.num_depth = num_depth
        self.is_training = is_training
        # 圆柱裁剪：以 seed 为中心、按 approach 旋转后，在不同深度 hmax 下采点并提特征
        self.crop = CloudCrop(64, 3, cylinder_radius, hmin, hmax_list)
        # 抓取操作头：预测 score/angle_cls/width（每个 angle、每个 depth）
        self.operation = OperationNet(num_angle, num_depth)
        # 容差头：预测 tolerance（每个 angle、每个 depth）
        self.tolerance = ToleranceNet(num_angle, num_depth)

    def forward(self, end_points):
        # 原始点云（与 backbone 的 input_xyz 一致）
        pointcloud = end_points['input_xyz']
        if self.is_training:
            # 训练：用标签中的 top 视角与 batch_grasp_point 作为 seed
            grasp_top_views_rot, _, _, _, end_points = match_grasp_view_and_label(end_points)
            seed_xyz = end_points['batch_grasp_point']
        else:
            # 推理：用 Stage1 预测的 top 视角旋转与 fp2_xyz 作为 seed
            grasp_top_views_rot = end_points['grasp_top_view_rot']
            seed_xyz = end_points['fp2_xyz']
        # 圆柱裁剪 + MLP：得到每个 seed、每个深度的局部特征
        vp_features = self.crop(seed_xyz, pointcloud, grasp_top_views_rot)
        # 操作头：写入 grasp_score_pred、grasp_angle_cls_pred、grasp_width_pred
        end_points = self.operation(vp_features, end_points)
        # 容差头：写入 grasp_tolerance_pred
        end_points = self.tolerance(vp_features, end_points)
        return end_points


class GraspNet(nn.Module):
    """ 完整 GraspNet：Stage1 +（训练时 process_grasp_labels）+ Stage2。 """

    def __init__(self, input_feature_dim=0, num_view=300, num_angle=12, num_depth=4, cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=True):
        super().__init__()
        self.is_training = is_training
        self.view_estimator = GraspNetStage1(input_feature_dim, num_view)
        self.grasp_generator = GraspNetStage2(num_angle, num_depth, cylinder_radius, hmin, hmax_list, is_training)

    def forward(self, end_points):
        # Stage1：点云 → seed 特征与坐标 + 每个 seed 的 top 视角
        end_points = self.view_estimator(end_points)
        # 训练时：根据标签生成 batch_grasp_*，供 Stage2 与 loss 使用
        if self.is_training:
            end_points = process_grasp_labels(end_points)
        # Stage2：在 top 视角下预测抓取分数、角度类、宽度、容差
        end_points = self.grasp_generator(end_points)
        return end_points


def pred_decode(end_points):
    """
    将 end_points 中的各头预测解码为 (B, M, 17) 的抓取数组列表，每样本 (M, 17) 可构造 GraspGroup。
    流程：按角度 argmax → 按深度 argmax → objectness 过滤 → 转旋转矩阵 → 拼 17 维。
    """
    # 获取 batch 大小：point_clouds 是 list 或 tensor，len 得到 batch_size
    batch_size = len(end_points['point_clouds'])
    # 存放每个样本解码后的抓取数组（每个元素是 (M, 17) 的 tensor）
    grasp_preds = []
    # 逐样本处理
    for i in range(batch_size):
        # ========== 第 1 步：从 end_points 取出第 i 个样本的所有预测 ==========
        # objectness_score: (2, num_seed) 二类 logits，0=无物体，1=有物体
        objectness_score = end_points['objectness_score'][i].float()
        # grasp_score_pred: (num_angle, num_seed, num_depth) 抓取分数
        grasp_score = end_points['grasp_score_pred'][i].float()
        # fp2_xyz: (num_seed, 3) seed 点坐标，作为抓取中心
        grasp_center = end_points['fp2_xyz'][i].float()
        # grasp_top_view_xyz: (num_seed, 3) Stage1 预测的 top 视角方向（单位向量）
        # approach 方向 = -视角方向（夹爪从视角反方向接近）
        approaching = -end_points['grasp_top_view_xyz'][i].float()
        # grasp_angle_cls_pred: (num_angle, num_seed, num_depth) 角度类得分
        grasp_angle_class_score = end_points['grasp_angle_cls_pred'][i]
        # grasp_width_pred: (num_angle, num_seed, num_depth) 宽度预测，乘 1.2 后截断到 [0, GRASP_MAX_WIDTH]
        grasp_width = 1.2 * end_points['grasp_width_pred'][i]
        grasp_width = torch.clamp(grasp_width, min=0, max=GRASP_MAX_WIDTH)
        # grasp_tolerance_pred: (num_angle, num_seed, num_depth) 容差预测
        grasp_tolerance = end_points['grasp_tolerance_pred'][i]

        # ========== 第 2 步：按角度维取 argmax，每个 seed 选得分最高的角度类 ==========
        # grasp_angle_class_score 形状 (num_angle, num_seed, num_depth)，在 dim=0（角度维）取 argmax
        # 结果 grasp_angle_class: (num_seed, num_depth)，每个值在 [0, num_angle-1]
        grasp_angle_class = torch.argmax(grasp_angle_class_score, 0)
        # 角度类转弧度：第 i 类对应 i*π/12（假设 num_angle=12）
        grasp_angle = grasp_angle_class.float() / 12 * np.pi
        # 扩展维度以便 gather：(num_seed, num_depth) → (1, num_seed, num_depth)
        grasp_angle_class_ = grasp_angle_class.unsqueeze(0)
        # 用 grasp_angle_class_ 在 dim=0 上 gather，从 (num_angle, num_seed, num_depth) 中取对应角度类的值
        # grasp_score: (num_seed, num_depth)，每个 seed、每个深度取最优角度的分数
        grasp_score = torch.gather(grasp_score, 0, grasp_angle_class_).squeeze(0)
        # 同理取最优角度的宽度
        grasp_width = torch.gather(grasp_width, 0, grasp_angle_class_).squeeze(0)
        # 同理取最优角度的容差
        grasp_tolerance = torch.gather(grasp_tolerance, 0, grasp_angle_class_).squeeze(0)

        # ========== 第 3 步：按深度维取 argmax，每个 seed 选得分最高的深度类 ==========
        # grasp_score 现在形状 (num_seed, num_depth)，在 dim=1（深度维）取 argmax
        # grasp_depth_class: (num_seed, 1)，每个值在 [0, num_depth-1]
        grasp_depth_class = torch.argmax(grasp_score, 1, keepdims=True)
        # 深度类转米：第 i 类对应 (i+1)*0.01 米（如 0.01, 0.02, 0.03, 0.04）
        grasp_depth = (grasp_depth_class.float()+1) * 0.01
        # 用 grasp_depth_class 在 dim=1 上 gather，从 (num_seed, num_depth) 中取对应深度类的值
        # grasp_score: (num_seed, 1) → squeeze(1) → (num_seed,)，每个 seed 的最优深度分数
        grasp_score = torch.gather(grasp_score, 1, grasp_depth_class)
        # 同理取最优深度的角度（弧度）
        grasp_angle = torch.gather(grasp_angle, 1, grasp_depth_class)
        # 同理取最优深度的宽度
        grasp_width = torch.gather(grasp_width, 1, grasp_depth_class)
        # 同理取最优深度的容差
        grasp_tolerance = torch.gather(grasp_tolerance, 1, grasp_depth_class)

        # ========== 第 4 步：按 objectness 过滤，只保留预测为"有物体"的 seed ==========
        # objectness_score 形状 (2, num_seed)，在 dim=0 取 argmax，得到每个 seed 的预测类别
        objectness_pred = torch.argmax(objectness_score, 0)
        # objectness_mask: (num_seed,)，True 表示该 seed 预测为"有物体"（class=1）
        objectness_mask = (objectness_pred == 1)
        # 用布尔掩码过滤：只保留 objectness_mask 为 True 的 seed
        # grasp_score: (num_seed,) → (M,) 其中 M = objectness_mask 中 True 的数量
        grasp_score = grasp_score[objectness_mask]
        grasp_width = grasp_width[objectness_mask]
        grasp_depth = grasp_depth[objectness_mask]
        approaching = approaching[objectness_mask]
        grasp_angle = grasp_angle[objectness_mask]
        grasp_center = grasp_center[objectness_mask]
        grasp_tolerance = grasp_tolerance[objectness_mask]
        # 用容差加权分数：分数 = 原始分数 * tolerance / GRASP_MAX_TOLERANCE
        grasp_score = grasp_score * grasp_tolerance / GRASP_MAX_TOLERANCE

        # ========== 第 5 步：将 approach 方向 + in-plane 角转为 3x3 旋转矩阵 ==========
        # Ns：过滤后剩余抓取数量
        Ns = grasp_angle.size(0)
        # approaching: (M, 3) approach 方向向量
        approaching_ = approaching.view(Ns, 3)
        # grasp_angle: (M,) in-plane 旋转角（弧度）
        grasp_angle_ = grasp_angle.view(Ns)
        # 调用工具函数：由 approach 方向与绕该方向的旋转角得到 3x3 旋转矩阵
        # rotation_matrix: (M, 3, 3)
        rotation_matrix = batch_viewpoint_params_to_matrix(approaching_, grasp_angle_)
        # 展平为 (M, 9)，行优先：R[0,0], R[0,1], R[0,2], R[1,0], ..., R[2,2]
        rotation_matrix = rotation_matrix.view(Ns, 9)

        # ========== 第 6 步：拼成 17 维数组，格式为 [score, width, height, depth, rot(9), trans(3), object_id] ==========
        # grasp_height: (M,) 固定为 0.02 米
        grasp_height = 0.02 * torch.ones_like(grasp_score)
        # obj_ids: (M,) 推理时设为 -1（表示未知物体）
        obj_ids = -1 * torch.ones_like(grasp_score)
        # 在最后一维拼接：grasp_score(M,1) + width(M,1) + height(M,1) + depth(M,1) + rot(M,9) + trans(M,3) + obj_id(M,1) = (M, 17)
        grasp_preds.append(torch.cat([grasp_score, grasp_width, grasp_height, grasp_depth, rotation_matrix, grasp_center, obj_ids], axis=-1))
    # 返回 list，每个元素是 (M_i, 17) 的 tensor，M_i 为该样本过滤后的抓取数量
    return grasp_preds
