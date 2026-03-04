#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GraspNet 抓取预测演示脚本

功能概述：
  1. 从指定目录加载 RGB 图、深度图、工作空间掩码和相机内参（meta.mat）
  2. 将深度图反投影为点云，并按工作空间掩码筛选、采样固定数量点
  3. 使用预训练 GraspNet 模型预测抓取位姿（GraspGroup）
  4. 可选：对预测抓取做无模型碰撞检测，过滤与场景碰撞的抓取
  5. 对抓取做 NMS、按分数排序后取最优一个，用 Open3D 可视化

数据目录要求：
  - color.png：RGB 彩色图
  - depth.png：深度图（与 color 对齐）
  - workspace_mask.png：工作空间二值掩码（有效区域为真）
  - meta.mat：含 intrinsic_matrix、factor_depth 等相机参数

作者: chenxi-wang
"""

import os  # 与操作系统路径、环境相关的标准库
import sys  # Python 解释器相关的系统接口（路径、参数等）
import numpy as np  # 数值计算库，用于数组和矩阵运算
import open3d as o3d  # 3D 点云处理与可视化库
import scipy.io as scio  # 读写 MATLAB .mat 文件
from PIL import Image  # 图像读写库，用于读取 PNG 图像

import torch  # PyTorch 深度学习框架
from graspnetAPI import GraspGroup  # GraspNetAPI 中定义的抓取集合数据结构

# ---------------------------------------------------------------------------
# 路径与模块：将本项目 models / dataset / utils 加入 Python 搜索路径，
# 以便可以直接使用 import models.xxx、dataset.xxx、utils.xxx 的形式导入
# ---------------------------------------------------------------------------
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))  # 当前 demo.py 所在目录的绝对路径
sys.path.insert(0, os.path.join(ROOT_DIR, 'models'))   # 优先搜索 models 目录
sys.path.insert(0, os.path.join(ROOT_DIR, 'dataset'))  # 优先搜索 dataset 目录
sys.path.append(os.path.join(ROOT_DIR, 'utils'))       # 将 utils 目录加入搜索路径末尾
# 以下导入依赖上面设置的路径，顺序不能颠倒
from models.graspnet import GraspNet, pred_decode  # 抓取网络模型与解码函数
from dataset.graspnet_dataset import GraspNetDataset  # 训练/评估用数据集类（demo 中未直接使用）
from utils.collision_detector import ModelFreeCollisionDetector  # 无模型碰撞检测器
from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image  # 相机信息封装与点云生成工具

# ---------------------------------------------------------------------------
# 默认参数配置：根据 command_demo.sh 的默认参数设置（不再使用命令行参数）
# ---------------------------------------------------------------------------
# 模型权重路径（command_demo.sh 中指定），相对于项目根目录
checkpoint_path = os.path.join(ROOT_DIR, 'logs', 'log_kn', 'checkpoint-rs.tar')
# 数据目录路径（原默认值），相对于项目根目录
data_dir = os.path.join(ROOT_DIR, 'doc', 'pose_1')
# 输入网络的点数（原默认值）
num_point = 20000
# GraspNet 的视角数量（原默认值，需与训练时保持一致）
num_view = 300
# 碰撞检测阈值（米），大于 0 时启用碰撞检测，设为 0 可关闭（原默认值）
collision_thresh = 0.01
# 体素下采样大小（米），用于加速碰撞检测（原默认值）
voxel_size = 0.01
# GPU 设备编号（原默认值）
gpu = 0

# 根据默认参数选择 GPU 设备，例如 cuda:0、cuda:1 等
DEVICE = torch.device("cuda:%d" % gpu)
# 打印当前使用的设备名称和 GPU 型号
print("-> Using device: %s (%s)" % (DEVICE, torch.cuda.get_device_name(gpu)))


def get_net():
    """
    创建 GraspNet 模型、加载权重并设为评估模式。

    模型参数简要说明：
      - input_feature_dim=0：点云不附带额外特征，仅 xyz
      - num_view：多视角编码的视角数，与 --num_view 一致
      - num_angle=12：抓取角度离散化档位
      - num_depth=4：抓取深度档位
      - cylinder_radius / hmin / hmax_list：定义抓取空间与夹爪几何

    返回:
        net: 已加载权重、处于 eval 模式的 GraspNet 模型（在 DEVICE 上）
    """
    # 初始化网络结构（与论文/训练配置一致）
    # 这些超参数必须与训练 checkpoint 的配置保持一致
    net = GraspNet(
        input_feature_dim=0,           # 仅使用 xyz 三维坐标，不添加额外特征
        num_view=num_view,            # 视角数量，使用默认值 300
        num_angle=12,                 # 平面内角度离散为 12 档（0~165°）
        num_depth=4,                  # 抓取深度离散为 4 档
        cylinder_radius=0.05,         # 以种子点为中心的圆柱半径 5cm
        hmin=-0.02,                   # 圆柱下底面 x 坐标（相对种子点），-2cm
        hmax_list=[0.01, 0.02, 0.03, 0.04],  # 对应 4 个深度档位（1~4cm）
        is_training=False             # 推理模式（不做标签处理）
    )
    net.to(DEVICE)                    # 将模型移动到 GPU 设备

    # 检查 checkpoint 文件是否存在
    if not os.path.exists(checkpoint_path):
        print("错误：找不到 checkpoint 文件: %s" % checkpoint_path)
        print("请确保已下载预训练权重并放在正确位置，或修改 checkpoint_path 变量指向正确的文件路径。")
        raise FileNotFoundError("Checkpoint file not found: %s" % checkpoint_path)
    
    # 权重先加载到 CPU 再转到 GPU，避免在环境不一致时反序列化到错误设备
    checkpoint = torch.load(checkpoint_path, weights_only=True)  # 从文件加载 checkpoint
    net.load_state_dict(checkpoint['model_state_dict'])                # 恢复模型参数
    start_epoch = checkpoint['epoch']                                  # 记录训练到的 epoch
    print("-> loaded checkpoint %s (epoch: %d)" % (checkpoint_path, start_epoch))

    net.eval()                          # 切换到评估模式（关闭 Dropout、BN 统计更新）
    return net                          # 返回已准备好的模型


def get_and_process_data(data_dir):
    """
    从 data_dir 读取相机数据，生成点云并整理为模型输入格式。

    步骤概要：
      1. 读取 color.png、depth.png、workspace_mask.png、meta.mat
      2. 用相机内参和深度图反投影得到有序点云
      3. 用工作空间掩码和有效深度筛出有效点
      4. 对有效点采样（或重复采样补足）到 num_point 个点，作为网络输入
      5. 同时保留“全部有效点”的 Open3D 点云，供后续碰撞检测和可视化使用

    参数:
        data_dir: 字符串，数据所在目录路径

    返回:
        end_points: dict，键包括 'point_clouds'（GPU 上的 tensor）、'cloud_colors'
        cloud: Open3D 点云，为全部有效点（未采样），用于碰撞检测与可视化
    """
    # ----- 1. 加载图像与相机参数 -----
    # 读取彩色图像，并归一化到 [0,1] 的 float32
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    # 读取深度图（灰度或 16-bit），保持原始数值
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    # 读取工作空间掩码：非零区域为有效工作空间
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
    # 读取相机内参和深度缩放因子（factor_depth）
    meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    intrinsic = meta['intrinsic_matrix']  # 3x3 相机内参矩阵
    factor_depth = meta['factor_depth']   # 深度缩放因子（depth / factor_depth = 米）

    # ----- 2. 由深度图反投影为 3D 点云（organized=True 保持与图像一致的 HxW 布局） -----
    height, width = depth.shape[0], depth.shape[1]  # 深度图高度和宽度
    # 封装相机参数到 CameraInfo 结构中
    camera = CameraInfo(
        width, height,
        intrinsic[0][0], intrinsic[1][1],  # fx, fy
        intrinsic[0][2], intrinsic[1][2],  # cx, cy
        factor_depth                       # 深度缩放因子
    )
    # 用深度图和相机内参生成有序点云 cloud，形状 (H, W, 3)，单位：米
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # ----- 3. 有效点：既在工作空间内，又有有效深度（非零） -----
    # workspace_mask 非零 且 depth > 0 的像素为有效点
    mask = (workspace_mask & (depth > 0))
    # 使用掩码从 cloud 中取出有效 3D 点
    cloud_masked = cloud[mask]
    # 同时取出对应颜色
    color_masked = color[mask]

    # ----- 4. 采样到固定点数：若点数足够则无放回采样，不足则用有放回采样补足 -----
    # 固定随机种子，使与 ROS2 graspnet_demo_node 使用同一数据时得到一致结果
    np.random.seed(1)
    if len(cloud_masked) >= num_point:
        # 有足够多的点：随机无放回采样 num_point 个点，把有效点云变成模型要求的点数
        idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
    else:
        # 点数不足：先取所有点，再有放回采样补足到 num_point
        idxs1 = np.arange(len(cloud_masked))  # 所有现有点的索引
        idxs2 = np.random.choice(
            len(cloud_masked),
            num_point - len(cloud_masked),
            replace=True
        )
        # 拼接索引，得到最终采样索引
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    # 根据索引采样点云和颜色
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # ----- 5. 组装：Open3D 点云（全部有效点） + 模型输入字典 -----
    # Open3D 点云：使用全部有效点（未采样），用于可视化和碰撞检测
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))   # 设置点坐标
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))   # 设置点颜色（0~1）

    # 模型输入：将采样后的点云转为 [1, N, 3] 的 tensor，并移动到 GPU
    end_points = dict()  # 用于存放输入和中间结果的字典
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))  # (1, N, 3)
    cloud_sampled = cloud_sampled.to(DEVICE)  # 放到 GPU
    end_points['point_clouds'] = cloud_sampled   # GraspNetStage1 期望的键名
    end_points['cloud_colors'] = color_sampled   # 颜色信息（目前 demo 未直接使用）

    return end_points, cloud  # 返回网络输入和完整点云


def get_grasps(net, end_points):
    """
    执行 GraspNet 前向推理，并将输出解码为 GraspGroup。

    流程：
      - net(end_points) 得到原始预测（各角度/深度/宽度的分数等）
      - pred_decode 将原始输出解码为 6 自由度的抓取位姿 + 宽度等，组成 grasp 数组
      - 取第一个 batch 的结果转为 numpy，再封装为 GraspGroup 对象

    参数:
        net: GraspNet 模型
        end_points: 模型输入字典（含 point_clouds 等）

    返回:
        gg: GraspGroup，一组抓取位姿，每个抓取包含位姿、宽度、分数等
    """
    # 在推理阶段关闭梯度计算，加快速度并节省显存
    with torch.no_grad():
        # 前向传播：更新 end_points，包括中间特征和最终预测结果
        end_points = net(end_points)
        # 将网络输出解码为 (B, M, 17) 的抓取数组列表
        grasp_preds = pred_decode(end_points)
    # 取 batch 中第 0 个样本（本 demo 默认 batch_size=1）
    gg_array = grasp_preds[0].detach().cpu().numpy()  # 从 GPU tensor 转为 CPU numpy 数组
    # 将 numpy 数组封装为 GraspGroup，内部为 (M, 17) 的抓取参数
    gg = GraspGroup(gg_array)
    return gg


def collision_detection(gg, cloud):
    """
    无模型碰撞检测：在场景点云上检测哪些抓取会与物体/桌面发生碰撞并剔除。

    原理简述：
      - 使用 ModelFreeCollisionDetector，将场景点云体素化后，模拟夹爪沿 approach 方向
        靠近抓取点，检查夹爪与体素是否相交；相交则视为碰撞。
      - approach_dist=0.05 表示沿 approach 方向检查的距离（米）
      - collision_thresh 为判定碰撞的距离/厚度阈值

    参数:
        gg: GraspGroup，待检测的抓取组
        cloud: 场景点云坐标，shape 为 (N, 3) 的 numpy 数组（如 np.array(cloud.points)）

    返回:
        过滤掉碰撞抓取后的新 GraspGroup
    """
    # 构造无模型碰撞检测器：对场景点云做体素下采样，存储到 self.scene_points 中
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=voxel_size)
    # 调用 detect：返回布尔掩码 collision_mask，True 表示该抓取发生碰撞
    collision_mask = mfcdetector.detect(
        gg,
        approach_dist=0.05,             # 夹爪沿 approach 方向提前空出 5cm 空间
        collision_thresh=collision_thresh
    )
    # 使用布尔索引过滤掉碰撞抓取，只保留 collision_mask 为 False 的抓取
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud):
    """
    对抓取做 NMS、按分数排序，取分数最高的一条抓取并可视化。

    NMS（非极大值抑制）：去掉与当前最优抓取重叠度高的其他抓取，避免重复显示相似抓取。
    排序后只保留一条，转为 Open3D 的几何体（夹爪示意），与点云、坐标系一起显示。

    参数:
        gg: GraspGroup，待可视化的抓取组
        cloud: Open3D 点云，场景点云
    """
    # NMS：去重相近的抓取（注意：nms 返回新的 GraspGroup，需要接收返回值）
    gg = gg.nms()
    # 按 score 从高到低排序
    gg.sort_by_score()

    # Open3D 可视化：只保留最高分的一条抓取
    gg_vis = gg[:15]
    print(gg_vis)  # 打印抓取信息，便于调试
    # 转为 Open3D 的几何体列表（每条抓取对应一组夹爪网格）
    grippers = gg_vis.to_open3d_geometry_list()
    # 创建坐标系用于可视化
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    # 绘制场景点云 + 抓取夹爪 + 坐标系
    o3d.visualization.draw_geometries([cloud, *grippers, frame])


def demo(data_dir):
    """
    演示主流程：加载模型 -> 加载并处理数据 -> 预测抓取 -> 碰撞检测（可选）-> 可视化。

    参数:
        data_dir: 数据目录路径，与 --data_dir 含义相同
    """
    # 创建并加载 GraspNet 模型
    net = get_net()
    # 从 data_dir 读取图像和相机内参，生成网络输入和完整点云
    end_points, cloud = get_and_process_data(data_dir)
    # 执行网络前向推理并解码为 GraspGroup
    gg = get_grasps(net, end_points)
    # 碰撞前：打印 width <= 0.018 的抓取
    for i, g in enumerate(gg):
        if g.width <= 0.018:
            print(f'[碰撞前] width<=0.018: Grasp {i} score={g.score:.4f} width={g.width:.4f} trans={g.translation}')
    # 若启用碰撞检测（collision_thresh > 0），则对抓取做碰撞过滤
    if collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    # NMS 前：打印 width <= 0.018 的抓取
    for i, g in enumerate(gg):
        if g.width <= 0.018:
            print(f'[NMS前] width<=0.018: Grasp {i} score={g.score:.4f} width={g.width:.4f} trans={g.translation}')
    # 对过滤后的抓取做 NMS、排序，并在 Open3D 中可视化
    vis_grasps(gg, cloud)


if __name__ == '__main__':
    # 作为脚本直接运行时，使用默认 data_dir 执行 demo
    demo(data_dir)
