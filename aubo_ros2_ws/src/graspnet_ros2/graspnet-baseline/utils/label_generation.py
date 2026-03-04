# -*- coding: utf-8 -*-
"""
训练时动态生成抓取标签：根据场景点、物体位姿和模板抓取，将标签对齐到网络预测的 seed 点与视角。
作者: chenxi-wang
"""

import os   # 路径操作
import sys  # 用于 sys.path
import torch

# 当前文件所在目录（utils）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 上一级目录（graspnet-baseline）
ROOT_DIR = os.path.dirname(BASE_DIR)
# 将项目根目录加入路径，便于 import 项目内模块
sys.path.append(ROOT_DIR)
# knn 模块在 ROOT_DIR/knn 下
sys.path.append(os.path.join(ROOT_DIR, 'knn'))

from knn_modules import knn  # K 近邻，用于视角与 seed 的匹配
from loss_utils import GRASP_MAX_WIDTH, batch_viewpoint_params_to_matrix,\
                       transform_point_cloud, generate_grasp_views


def process_grasp_labels(end_points):
    """
    根据场景点云与物体位姿，将各物体的抓取点/视角/标签变换到场景系，
    并与 fp2_xyz（seed 点）做最近邻匹配，得到 batch 维度的抓取标签；并对标签做 log 与 view 得分处理。
    会改写 end_points，写入 batch_grasp_* 等键。
    """
    # 输入点云 (B, N, 3)
    clouds = end_points['input_xyz']
    # 网络第二层 FP 后的采样点，即 seed 点 (B, Ns, 3)
    seed_xyzs = end_points['fp2_xyz']
    # 取出 batch 大小与 seed 数量
    batch_size, num_samples, _ = seed_xyzs.size()

    # 每个 batch 的抓取点、视角、旋转、标签、偏移、容差列表，最后再 stack
    batch_grasp_points = []
    batch_grasp_views = []
    batch_grasp_views_rot = []
    batch_grasp_labels = []
    batch_grasp_offsets = []
    batch_grasp_tolerance = []
    # 逐样本处理
    for i in range(len(clouds)):
        # 当前样本的 seed 点 (Ns, 3)
        seed_xyz = seed_xyzs[i]
        # 当前样本中所有物体的位姿列表，每个元素 (3, 4)
        poses = end_points['object_poses_list'][i]

        # 当前样本内多物体抓取合并用的临时列表
        grasp_points_merged = []
        grasp_views_merged = []
        grasp_views_rot_merged = []
        grasp_labels_merged = []
        grasp_offsets_merged = []
        grasp_tolerance_merged = []
        # 遍历该样本的每个物体
        for obj_idx, pose in enumerate(poses):
            # 该物体上的抓取点，物体坐标系 (Np, 3)
            grasp_points = end_points['grasp_points_list'][i][obj_idx]
            # 该物体的抓取标签 (Np, V, A, D)：V 视角, A 角度, D 深度
            grasp_labels = end_points['grasp_labels_list'][i][obj_idx]
            # 抓取偏移 (Np, V, A, D, 3)
            grasp_offsets = end_points['grasp_offsets_list'][i][obj_idx]
            # 抓取容差 (Np, V, A, D)
            grasp_tolerance = end_points['grasp_tolerance_list'][i][obj_idx]
            # 取 V, A, D 维度大小
            _, V, A, D = grasp_labels.size()
            num_grasp_points = grasp_points.size(0)
            # 生成 V 个模板视角方向 (V, 3)，并放到与 pose 相同设备
            grasp_views = generate_grasp_views(V).to(pose.device)
            # 抓取点从物体系变换到场景系 (Np, 3)
            grasp_points_trans = transform_point_cloud(grasp_points, pose, '3x4')
            # 视角方向仅旋转，变换到场景系 (V, 3)
            grasp_views_trans = transform_point_cloud(grasp_views, pose[:3, :3], '3x3')
            # 模板视角的 in-plane 角先设为 0
            angles = torch.zeros(grasp_views.size(0), dtype=grasp_views.dtype, device=grasp_views.device)
            # 由视角方向得到旋转矩阵：-grasp_views 为 approach 方向，(V, 3, 3)
            grasp_views_rot = batch_viewpoint_params_to_matrix(-grasp_views, angles)
            # 旋转矩阵变换到场景系：左乘 pose 的旋转部分 (V, 3, 3)
            grasp_views_rot_trans = torch.matmul(pose[:3, :3], grasp_views_rot)

            # 用 KNN 将“变换后的视角”与“模板视角”对齐，得到视角索引重排
            # knn 输入格式为 (1, 3, N)，故 transpose 后 unsqueeze
            grasp_views_ = grasp_views.transpose(0, 1).contiguous().unsqueeze(0)
            grasp_views_trans_ = grasp_views_trans.transpose(0, 1).contiguous().unsqueeze(0)
            # 对每个变换后视角找最近的模板视角，k=1，squeeze 后减 1 得 0-based 索引 (V,)
            view_inds = knn(grasp_views_trans_, grasp_views_, k=1).squeeze() - 1
            # 按 view_inds 重排变换后的视角，再扩展为 (Np, V, 3)
            grasp_views_trans = torch.index_select(grasp_views_trans, 0, view_inds)
            grasp_views_trans = grasp_views_trans.unsqueeze(0).expand(num_grasp_points, -1, -1)
            # 重排旋转矩阵并扩展为 (Np, V, 3, 3)
            grasp_views_rot_trans = torch.index_select(grasp_views_rot_trans, 0, view_inds)
            grasp_views_rot_trans = grasp_views_rot_trans.unsqueeze(0).expand(num_grasp_points, -1, -1, -1)
            # 标签、偏移、容差在 V 维上按 view_inds 重排
            grasp_labels = torch.index_select(grasp_labels, 1, view_inds)
            grasp_offsets = torch.index_select(grasp_offsets, 1, view_inds)
            grasp_tolerance = torch.index_select(grasp_tolerance, 1, view_inds)
            # 加入合并列表
            grasp_points_merged.append(grasp_points_trans)
            grasp_views_merged.append(grasp_views_trans)
            grasp_views_rot_merged.append(grasp_views_rot_trans)
            grasp_labels_merged.append(grasp_labels)
            grasp_offsets_merged.append(grasp_offsets)
            grasp_tolerance_merged.append(grasp_tolerance)

        # 在 dim=0 上拼接所有物体，得到 (Np', 3) 等，Np' 为当前样本总抓取点数
        grasp_points_merged = torch.cat(grasp_points_merged, dim=0)
        grasp_views_merged = torch.cat(grasp_views_merged, dim=0)
        grasp_views_rot_merged = torch.cat(grasp_views_rot_merged, dim=0)
        grasp_labels_merged = torch.cat(grasp_labels_merged, dim=0)
        grasp_offsets_merged = torch.cat(grasp_offsets_merged, dim=0)
        grasp_tolerance_merged = torch.cat(grasp_tolerance_merged, dim=0)

        # 每个 seed 点找“合并抓取点”中最近的一个，用其标签作为该 seed 的标签
        # knn 输入 (1, 3, N)，故 transpose
        seed_xyz_ = seed_xyz.transpose(0, 1).contiguous().unsqueeze(0)
        grasp_points_merged_ = grasp_points_merged.transpose(0, 1).contiguous().unsqueeze(0)
        # 对每个 seed 找最近的 1 个抓取点索引，(Ns,) 0-based
        nn_inds = knn(grasp_points_merged_, seed_xyz_, k=1).squeeze() - 1

        # 用 nn_inds 从合并的抓取中取出与每个 seed 对应的项，得到 (Ns, ...)
        grasp_points_merged = torch.index_select(grasp_points_merged, 0, nn_inds)
        grasp_views_merged = torch.index_select(grasp_views_merged, 0, nn_inds)
        grasp_views_rot_merged = torch.index_select(grasp_views_rot_merged, 0, nn_inds)
        grasp_labels_merged = torch.index_select(grasp_labels_merged, 0, nn_inds)
        grasp_offsets_merged = torch.index_select(grasp_offsets_merged, 0, nn_inds)
        grasp_tolerance_merged = torch.index_select(grasp_tolerance_merged, 0, nn_inds)

        # 加入 batch 列表
        batch_grasp_points.append(grasp_points_merged)
        batch_grasp_views.append(grasp_views_merged)
        batch_grasp_views_rot.append(grasp_views_rot_merged)
        batch_grasp_labels.append(grasp_labels_merged)
        batch_grasp_offsets.append(grasp_offsets_merged)
        batch_grasp_tolerance.append(grasp_tolerance_merged)

    # 在 batch 维 stack，得到 (B, Ns, ...)
    batch_grasp_points = torch.stack(batch_grasp_points, 0)
    batch_grasp_views = torch.stack(batch_grasp_views, 0)
    batch_grasp_views_rot = torch.stack(batch_grasp_views_rot, 0)
    batch_grasp_labels = torch.stack(batch_grasp_labels, 0)
    batch_grasp_offsets = torch.stack(batch_grasp_offsets, 0)
    batch_grasp_tolerance = torch.stack(batch_grasp_tolerance, 0)

    # 标签后处理：从 offset 最后一维取宽度（通常为 z 分量）
    batch_grasp_widths = batch_grasp_offsets[:, :, :, :, :, 2]
    # 有效标签：得分>0 且 宽度<=GRASP_MAX_WIDTH
    label_mask = (batch_grasp_labels > 0) & (batch_grasp_widths <= GRASP_MAX_WIDTH)
    # 全局最大得分，用于 log 变换
    u_max = batch_grasp_labels.max()
    # 有效位置：标签改为 log(u_max / label)，无效位置置 0
    batch_grasp_labels[label_mask] = torch.log(u_max / batch_grasp_labels[label_mask])
    batch_grasp_labels[~label_mask] = 0
    # 将 (B, Ns, V, A, D) 视为 (B, Ns, V, A*D)，在最后一维取最大值作为该 view 的得分 (B, Ns, V)
    batch_grasp_view_scores, _ = batch_grasp_labels.view(batch_size, num_samples, V, A*D).max(dim=-1)

    # 写回 end_points
    end_points['batch_grasp_point'] = batch_grasp_points
    end_points['batch_grasp_view'] = batch_grasp_views
    end_points['batch_grasp_view_rot'] = batch_grasp_views_rot
    end_points['batch_grasp_label'] = batch_grasp_labels
    end_points['batch_grasp_offset'] = batch_grasp_offsets
    end_points['batch_grasp_tolerance'] = batch_grasp_tolerance
    end_points['batch_grasp_view_label'] = batch_grasp_view_scores.float()

    return end_points


def match_grasp_view_and_label(end_points):
    """
    根据网络预测的 top 视角索引，从 (V, A, D) 标签中取出该视角的 label/offset/tolerance/旋转，
    用于后续只对预测视角计算 loss。
    会更新 end_points 中 batch_grasp_view_rot、batch_grasp_label、batch_grasp_offset、batch_grasp_tolerance。
    """
    # 网络预测的每个 seed 的 top 视角索引 (B, Ns)
    top_view_inds = end_points['grasp_top_view_inds']
    # 所有视角的模板旋转 (B, Ns, V, 3, 3)
    template_views_rot = end_points['batch_grasp_view_rot']
    # 抓取标签 (B, Ns, V, A, D)
    grasp_labels = end_points['batch_grasp_label']
    # 抓取偏移 (B, Ns, V, A, D, 3)
    grasp_offsets = end_points['batch_grasp_offset']
    # 抓取容差 (B, Ns, V, A, D)
    grasp_tolerance = end_points['batch_grasp_tolerance']

    B, Ns, V, A, D = grasp_labels.size()
    # 将 top_view_inds 扩展为 (B, Ns, 1, 1, 1) 再 expand 到 (B, Ns, 1, 3, 3)，用于在 dim=2(V) 上 gather 3x3 矩阵
    top_view_inds_ = top_view_inds.view(B, Ns, 1, 1, 1).expand(-1, -1, -1, 3, 3)
    # 在 dim=2 上按索引取，squeeze(2) 后得 (B, Ns, 3, 3)
    top_template_views_rot = torch.gather(template_views_rot, 2, top_view_inds_).squeeze(2)
    # 对 label 在 V 维 gather：索引扩展为 (B, Ns, 1, A, D)
    top_view_inds_ = top_view_inds.view(B, Ns, 1, 1, 1).expand(-1, -1, -1, A, D)
    top_view_grasp_labels = torch.gather(grasp_labels, 2, top_view_inds_).squeeze(2)
    top_view_grasp_tolerance = torch.gather(grasp_tolerance, 2, top_view_inds_).squeeze(2)
    # 对 offset 在 V 维 gather：索引扩展为 (B, Ns, 1, A, D, 3)
    top_view_inds_ = top_view_inds.view(B, Ns, 1, 1, 1, 1).expand(-1, -1, -1, A, D, 3)
    top_view_grasp_offsets = torch.gather(grasp_offsets, 2, top_view_inds_).squeeze(2)

    # 用“top 视角”的结果覆盖 end_points，后续 loss 只用到这些
    end_points['batch_grasp_view_rot'] = top_template_views_rot
    end_points['batch_grasp_label'] = top_view_grasp_labels
    end_points['batch_grasp_offset'] = top_view_grasp_offsets
    end_points['batch_grasp_tolerance'] = top_view_grasp_tolerance

    return top_template_views_rot, top_view_grasp_labels, top_view_grasp_offsets, top_view_grasp_tolerance, end_points
