# -*- coding: utf-8 -*-
"""
无模型碰撞检测：在无物体标签的场景中，根据点云判断抓取是否会与物体/桌面碰撞。
作者: chenxi-wang
"""

import os
import sys
import numpy as np   # 点云与矩阵运算
import open3d as o3d # 体素下采样与点云结构


class ModelFreeCollisionDetector():
    """
    无模型碰撞检测器：不依赖物体 mesh/标签，仅根据场景点云判断抓取是否与点云相交。
    夹爪几何固定：finger_width=0.01m，finger_length=0.06m。
    输入: scene_points (N,3), voxel_size 体素边长
    """

    def __init__(self, scene_points, voxel_size=0.005):
        """初始化：固定夹爪尺寸，对场景点云做体素下采样并保存。"""
        # 夹爪单指宽度（米）
        self.finger_width = 0.01
        # 夹爪单指长度（米）
        self.finger_length = 0.06
        # 体素边长（米），用于下采样与体积归一化
        self.voxel_size = voxel_size
        # 创建 Open3D 点云对象
        scene_cloud = o3d.geometry.PointCloud()
        # 将 numpy 点坐标传入
        scene_cloud.points = o3d.utility.Vector3dVector(scene_points)
        # 体素下采样，同一体素内只保留一个代表点，减少点数
        scene_cloud = scene_cloud.voxel_down_sample(voxel_size)
        # 转回 numpy 并保存，供 detect 中做矩阵运算
        self.scene_points = np.array(scene_cloud.points)

    def detect(self, grasp_group, approach_dist=0.03, collision_thresh=0.05, return_empty_grasp=False, empty_thresh=0.01, return_ious=False):
        """
        检测每条抓取是否与场景点云发生碰撞。
        输出: collision_mask (M,) bool；可选 empty_mask、iou_list
        """
        # 接近距离至少为指宽，避免除零或无效区间
        approach_dist = max(approach_dist, self.finger_width)
        # 抓取平移 (M, 3)
        T = grasp_group.translations
        # 抓取旋转矩阵 (M, 3, 3)，世界系到抓取局部系
        R = grasp_group.rotation_matrices
        # 夹爪开口高度 (M, 1)
        heights = grasp_group.heights[:, np.newaxis]
        # 夹爪深度方向长度 (M, 1)
        depths = grasp_group.depths[:, np.newaxis]
        # 夹爪开口宽度 (M, 1)
        widths = grasp_group.widths[:, np.newaxis]
        # 场景点减平移：每个点相对每条抓取中心的位置 (M, N, 3)
        targets = self.scene_points[np.newaxis, :, :] - T[:, np.newaxis, :]
        # 乘旋转矩阵：将点变换到抓取局部坐标系（局部 x 为 approach 方向）
        targets = np.matmul(targets, R)

        # ----- 在局部系下用 AABB 判断点是否落在夹爪各部位 -----
        # mask1: 高度方向 z 在 [-height/2, height/2]
        mask1 = ((targets[:, :, 2] > -heights/2) & (targets[:, :, 2] < heights/2))
        # mask2: 深度方向 x 在指尖到掌根之间 [depths - finger_length, depths]
        mask2 = ((targets[:, :, 0] > depths - self.finger_length) & (targets[:, :, 0] < depths))
        # mask3: 左指 y 方向下界，y > -(width/2 + finger_width)
        mask3 = (targets[:, :, 1] > -(widths/2 + self.finger_width))
        # mask4: 左指 y 方向上界，y < -width/2
        mask4 = (targets[:, :, 1] < -widths/2)
        # mask5: 右指 y 方向下界，y < width/2 + finger_width
        mask5 = (targets[:, :, 1] < (widths/2 + self.finger_width))
        # mask6: 右指 y 方向上界，y > width/2
        mask6 = (targets[:, :, 1] > widths/2)
        # mask7: 底部横梁在 x 方向的范围（掌根再往 approach 反方向 finger_width）
        mask7 = ((targets[:, :, 0] <= depths - self.finger_length)
                & (targets[:, :, 0] > depths - self.finger_length - self.finger_width))
        # mask8: 接近通道（approach 方向预留空间），再往 approach 反方向 approach_dist
        mask8 = ((targets[:, :, 0] <= depths - self.finger_length - self.finger_width)
                & (targets[:, :, 0] > depths - self.finger_length - self.finger_width - approach_dist))

        # 左指区域：同时满足 mask1~4
        left_mask = (mask1 & mask2 & mask3 & mask4)
        # 右指区域：同时满足 mask1,2,5,6
        right_mask = (mask1 & mask2 & mask5 & mask6)
        # 底部区域：高度 + 左右范围内 + mask7
        bottom_mask = (mask1 & mask3 & mask5 & mask7)
        # 接近通道内的点
        shifting_mask = (mask1 & mask3 & mask5 & mask8)
        # 任意一部分有点即认为该抓取与场景相交
        global_mask = (left_mask | right_mask | bottom_mask | shifting_mask)

        # 左/右指体积（体素个数）：高*长*宽 / voxel_size^3，展平为 (M,)
        left_right_volume = (heights * self.finger_length * self.finger_width / (self.voxel_size**3)).reshape(-1)
        # 底部体积：高 * (开口宽+两指宽) * 指宽
        bottom_volume = (heights * (widths+2*self.finger_width) * self.finger_width / (self.voxel_size**3)).reshape(-1)
        # 接近通道体积：高 * (开口宽+两指宽) * approach_dist
        shifting_volume = (heights * (widths+2*self.finger_width) * approach_dist / (self.voxel_size**3)).reshape(-1)
        # 总体积（左右各一份）
        volume = left_right_volume*2 + bottom_volume + shifting_volume

        # 全局 IoU：落入夹爪内的点数 / 总体积，避免除零加 1e-6
        global_iou = global_mask.sum(axis=1) / (volume + 1e-6)
        # 超过阈值则判定为碰撞
        collision_mask = (global_iou > collision_thresh)

        # 不需要额外输出时直接返回
        if not (return_empty_grasp or return_ious):
            return collision_mask

        # 返回值列表，第一个元素恒为碰撞掩码
        ret_value = [collision_mask,]
        if return_empty_grasp:
            # 抓取“内部”区域：两指之间、沿深度方向，即非左指非右指但在指尖框内
            inner_mask = (mask1 & mask2 & (~mask4) & (~mask6))
            # 内部体积：高 * 长 * 开口宽
            inner_volume = (heights * self.finger_length * widths / (self.voxel_size**3)).reshape(-1)
            # 内部点数/体积 小于 empty_thresh 则认为“空抓”（没有物体）
            empty_mask = (inner_mask.sum(axis=-1) / inner_volume < empty_thresh)
            ret_value.append(empty_mask)
        if return_ious:
            # 左指 IoU
            left_iou = left_mask.sum(axis=1) / (left_right_volume + 1e-6)
            # 右指 IoU
            right_iou = right_mask.sum(axis=1) / (left_right_volume + 1e-6)
            # 底部 IoU
            bottom_iou = bottom_mask.sum(axis=1) / (bottom_volume + 1e-6)
            # 接近通道 IoU
            shifting_iou = shifting_mask.sum(axis=1) / (shifting_volume + 1e-6)
            ret_value.append([global_iou, left_iou, right_iou, bottom_iou, shifting_iou])
        return ret_value
