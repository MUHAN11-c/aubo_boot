# -*- coding: utf-8 -*-
"""
数据处理工具：相机内参、深度图反投影、点云变换、距离计算、工作空间掩码等。
作者: chenxi-wang
"""

import numpy as np  # 数值计算，用于点云与矩阵运算


class CameraInfo():
    """
    相机内参封装类，用于从深度图生成点云。
    存储图像尺寸、焦距、主点、深度缩放因子。
    """

    def __init__(self, width, height, fx, fy, cx, cy, scale):
        """初始化相机参数。"""
        self.width = width    # 图像宽度（像素）
        self.height = height  # 图像高度（像素）
        self.fx = fx          # x 方向焦距（像素单位）
        self.fy = fy          # y 方向焦距（像素单位）
        self.cx = cx          # 主点 x 坐标（光轴与像平面交点）
        self.cy = cy          # 主点 y 坐标
        self.scale = scale    # 深度缩放因子：原始深度值 / scale = 真实深度（米）


def create_point_cloud_from_depth_image(depth, camera, organized=True):
    """
    仅根据深度图与相机内参生成 3D 点云（无颜色）。
    输入: depth (H,W), camera [CameraInfo], organized [bool]
    输出: cloud (H,W,3) 或 (H*W,3)，单位米
    """
    # 检查深度图尺寸与相机内参一致
    assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    # 生成像素坐标网格：x 取 0 到 width-1
    xmap = np.arange(camera.width)
    # y 取 0 到 height-1
    ymap = np.arange(camera.height)
    # 网格化，得到每个像素的 (u,v)，xmap/ymap 形状 (H, W)
    xmap, ymap = np.meshgrid(xmap, ymap)
    # 深度值转物理深度（米）：像素值除以 scale
    points_z = depth / camera.scale
    # 针孔模型反投影：X = (u - cx) * Z / fx
    points_x = (xmap - camera.cx) * points_z / camera.fx
    # Y = (v - cy) * Z / fy
    points_y = (ymap - camera.cy) * points_z / camera.fy
    # 在最后一维堆叠成 (H, W, 3)，顺序为 x,y,z
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    # 若不需要保持图像形状，展平为 (H*W, 3)
    if not organized:
        cloud = cloud.reshape([-1, 3])
    return cloud


def transform_point_cloud(cloud, transform, format='4x4'):
    """
    用变换矩阵将点云从原坐标系变换到新坐标系。
    输入: cloud (N,3), transform (3,3)/(3,4)/(4,4), format 指定矩阵形式
    输出: cloud_transformed (N,3)
    """
    # 只支持 3x3 / 3x4 / 4x4 三种格式
    if not (format == '3x3' or format == '4x4' or format == '3x4'):
        raise ValueError('Unknown transformation format, only support \'3x3\' or \'4x4\' or \'3x4\'.')
    if format == '3x3':
        # 仅旋转：对每点 p 做 p' = R @ p；.T 转置后 (3,N)，乘完再转回 (N,3)
        cloud_transformed = np.dot(transform, cloud.T).T
    elif format == '4x4' or format == '3x4':
        # 齐次坐标：先给每点补一维 1
        ones = np.ones(cloud.shape[0])[:, np.newaxis]
        # 得到 (N, 4)
        cloud_ = np.concatenate([cloud, ones], axis=1)
        # 左乘变换矩阵 [R|t]，得到 (N, 4)
        cloud_transformed = np.dot(transform, cloud_.T).T
        # 只取前三维作为坐标
        cloud_transformed = cloud_transformed[:, :3]
    return cloud_transformed


def compute_point_dists(A, B):
    """
    计算两组点之间的两两欧氏距离。
    输入: A (N,3), B (M,3)
    输出: dists (N,M)，dists[i,j] = ||A[i]-B[j]||
    """
    # 扩展维度以便广播：A 变为 (N, 1, 3)
    A = A[:, np.newaxis, :]
    # B 变为 (1, M, 3)
    B = B[np.newaxis, :, :]
    # A-B 得 (N, M, 3)，对最后一维求 2-范数得 (N, M)
    dists = np.linalg.norm(A - B, axis=-1)
    return dists


def remove_invisible_grasp_points(cloud, grasp_points, pose, th=0.01):
    """
    根据场景点云剔除“不可见”的抓取点（如物体背面）。
    若某抓取点到场景点的最小距离 > th，则认为不可见。
    输入: cloud 场景点云, grasp_points 物体系抓取点, pose 物体到世界的 4x4 变换, th 距离阈值
    输出: visible_mask (M,) bool，True 表示可见
    """
    # 将抓取点变换到世界（相机）坐标系
    grasp_points_trans = transform_point_cloud(grasp_points, pose)
    # 计算每个抓取点到场景点的距离矩阵 (M, N)
    dists = compute_point_dists(grasp_points_trans, cloud)
    # 每个抓取点到最近场景点的距离 (M,)
    min_dists = dists.min(axis=1)
    # 距离小于阈值则视为可见
    visible_mask = (min_dists < th)
    return visible_mask


def get_workspace_mask(cloud, seg, trans=None, organized=True, outlier=0):
    """
    根据分割结果得到工作空间掩码：保留前景物体 3D 包围盒内的点（可外扩 outlier）。
    输入: cloud (H,W,3), seg (H,W) 分割, trans 可选变换, organized 是否保持形状, outlier 外扩量
    输出: workspace_mask (H,W) 或 (H*W,)，bool
    """
    # 若需要保持图像形状，先记下 h,w
    if organized:
        h, w, _ = cloud.shape
        # 展平为 (H*W, 3) 便于后续按坐标筛选
        cloud = cloud.reshape([h*w, 3])
        seg = seg.reshape(h*w)
    # 若提供了变换矩阵，将点云变换后再算包围盒
    if trans is not None:
        cloud = transform_point_cloud(cloud, trans)
    # 取分割值大于 0 的点作为前景
    foreground = cloud[seg > 0]
    # 前景在 x,y,z 三个方向的最小值
    xmin, ymin, zmin = foreground.min(axis=0)
    # 最大值
    xmax, ymax, zmax = foreground.max(axis=0)
    # x 在 [xmin-outlier, xmax+outlier] 内为 True
    mask_x = ((cloud[:, 0] > xmin - outlier) & (cloud[:, 0] < xmax + outlier))
    # y 方向
    mask_y = ((cloud[:, 1] > ymin - outlier) & (cloud[:, 1] < ymax + outlier))
    # z 方向
    mask_z = ((cloud[:, 2] > zmin - outlier) & (cloud[:, 2] < zmax + outlier))
    # 三个方向同时满足则在工作空间内
    workspace_mask = (mask_x & mask_y & mask_z)
    # 若输入是有序的，恢复为 (H, W) 形状
    if organized:
        workspace_mask = workspace_mask.reshape([h, w])
    return workspace_mask
