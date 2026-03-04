#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
零基础：一步一步生成 demo 所需的「模拟数据」

目的：
  demo.py 的 get_and_process_data(data_dir) 需要从 data_dir 读取 4 样东西：
    1. color.png       — 一张彩色图
    2. depth.png       — 一张深度图（每个像素一个“深度值”）
    3. workspace_mask.png — 一张掩码图（标出“工作空间”在哪里）
    4. meta.mat        — 相机参数（内参 + 深度缩放因子）

本脚本从零生成这 4 样东西并保存到指定目录，不依赖真实相机或真实场景。
运行后可用：python demo.py --data_dir <输出目录> --checkpoint_path <权重路径> 做推理。

依赖：numpy, PIL, scipy（与 demo 一致）

本文件还包含 get_and_process_data 的「复刻」：get_and_process_data_replica()，
与 demo.py 中的逻辑完全一致，并带有每一步的详细解析注释。
依赖：torch, open3d（复刻函数需要）；可选从 utils.data_utils 导入或使用内联实现。
"""

import os
import numpy as np
from PIL import Image
import scipy.io as scio

# 复刻 get_and_process_data 时需要
try:
    import torch
    import open3d as o3d
except ImportError:
    torch = None
    o3d = None


# =============================================================================
# 第 0 步：先想清楚「图像有多大」
# =============================================================================
# 相机拍出来的图是「高×宽」个像素，例如 480 行、640 列。
# 我们模拟的数据也必须是「同一尺寸」：深度图、彩色图、掩码图都要一样大。
HEIGHT = 480   # 图像高度（行数）
WIDTH = 640    # 图像宽度（列数）


def create_fake_data(out_dir, height=HEIGHT, width=WIDTH):
    """
    在 out_dir 下生成 color.png、depth.png、workspace_mask.png、meta.mat。
    height, width：图像尺寸，所有图都是 (height, width)。
    """
    os.makedirs(out_dir, exist_ok=True)

    # =========================================================================
    # 第 1 步：生成「彩色图」color.png
    # =========================================================================
    # 彩色图就是 H×W 个像素，每个像素有 R、G、B 三个通道，值 0~255。
    # 我们用 numpy 做一个 (H, W, 3) 的数组，再保存成 PNG。

    # 先做一个全零的数组，再按区域填上不同颜色（方便看出“工作空间”在哪）
    color = np.zeros((height, width, 3), dtype=np.uint8)
    # 上半部分：偏蓝（模拟天空/远处）
    color[0 : height // 2, :, 2] = 200   # B
    color[0 : height // 2, :, 0] = 80   # R
    color[0 : height // 2, :, 1] = 120  # G
    # 下半部分：偏绿（模拟桌面/地面）
    color[height // 2 :, :, 1] = 180    # G
    color[height // 2 :, :, 0] = 60     # R
    color[height // 2 :, :, 2] = 80     # B
    # 中间一块：放一个“假物体”（灰色矩形），后面深度图里这里会近一些
    y1, y2 = height // 3, 2 * height // 3
    x1, x2 = width // 4, 3 * width // 4
    color[y1:y2, x1:x2, :] = 140  # 灰色

    # 保存：PIL 要的是 (H, W, 3) 的 uint8
    Image.fromarray(color).save(os.path.join(out_dir, 'color.png'))
    # 到这里：color.png = 一张 640×480 的彩色图，已经存到 out_dir 里了。

    # =========================================================================
    # 第 2 步：生成「深度图」depth.png
    # =========================================================================
    # 深度图：每个像素一个数，表示“这个像素对应的 3D 点离相机有多远”。
    # 在 GraspNet 里，深度图存的往往不是“米”，而是“原始值”；真实深度(米) = 原始值 / factor_depth。
    # 所以我们先按“米”想，再乘一个因子得到“原始值”，保存成 PNG（通常用 16 位整数）。

    # 2.1 先做一个“按米来”的深度（后面会乘 factor_depth 再存）
    depth_meters = np.ones((height, width), dtype=np.float32) * 0.8   # 默认 0.8 米
    # 中间“假物体”区域：设成更近，比如 0.4 米
    depth_meters[y1:y2, x1:x2] = 0.4
    # 边缘可以再远一点（模拟背景）
    depth_meters[0 : height // 6, :] = 1.2
    depth_meters[5 * height // 6 :, :] = 1.0

    # 2.2 选一个“深度缩放因子”（和后面 meta.mat 里 factor_depth 一致）
    factor_depth = 1000.0
    # 含义：depth.png 里存的值 = 真实深度(米) × factor_depth，这样 0.4 米 → 400，0.8 米 → 800。
    depth_raw = (depth_meters * factor_depth).astype(np.float32)

    # 2.3 保存成 PNG：常见做法是 16 位无符号整数，范围 0~65535
    depth_raw_clip = np.clip(depth_raw, 0, 65535).astype(np.uint16)
    Image.fromarray(depth_raw_clip).save(os.path.join(out_dir, 'depth.png'))
    # 注意：demo 里读进来后可能是 float，这里保存成 uint16 和很多数据集一致；若 demo 期望 float 再改读入方式即可。

    # =========================================================================
    # 第 3 步：生成「工作空间掩码」workspace_mask.png
    # =========================================================================
    # 掩码：和图像一样大，每个像素一个数。通常“非零”表示“这个像素在工作空间内，要参与抓取”。
    # demo 里用 workspace_mask & (depth > 0) 得到有效点，所以掩码和深度图要同一尺寸。

    workspace_mask = np.zeros((height, width), dtype=np.uint8)
    # 我们定义：图像中间一个矩形是工作空间（和假物体区域差不多，略大一点）
    margin = 20
    workspace_mask[y1 - margin : y2 + margin, x1 - margin : x2 + margin] = 255

    Image.fromarray(workspace_mask).save(os.path.join(out_dir, 'workspace_mask.png'))

    # =========================================================================
    # 第 4 步：生成「相机参数」meta.mat
    # =========================================================================
    # 相机内参是一个 3×3 矩阵，通常长这样（齐次坐标下）：
    #   [ fx  0  cx ]
    #   [ 0  fy  cy ]
    #   [ 0   0   1 ]
    # fx, fy：焦距（像素单位）；cx, cy：主点（光轴与图像交点，通常接近图像中心）。
    # 深度图反投影公式：X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy，Z = depth 像素值 / factor_depth（米）。

    fx = 600.0   # 随便取一个合理值，仿真用
    fy = 600.0
    cx = width / 2.0   # 主点在图像中心
    cy = height / 2.0
    intrinsic_matrix = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float64)

    # factor_depth 必须和 depth 图一致：depth 里存的是 深度(米)*factor_depth
    meta = {
        'intrinsic_matrix': intrinsic_matrix,
        'factor_depth': np.array([[factor_depth]], dtype=np.float64)  # 有的读法期望 2 维
    }
    scio.savemat(os.path.join(out_dir, 'meta.mat'), meta)

    # =========================================================================
    # 第 5 步：自检——用和 demo 相同的公式把深度图变点云（可选）
    # =========================================================================
    # 不依赖 graspnet 的 utils，直接按公式算一遍，验证“模拟数据”是否自洽。
    depth_read = np.array(Image.open(os.path.join(out_dir, 'depth.png')))
    if depth_read.dtype == np.uint16:
        depth_read = depth_read.astype(np.float32)
    z = depth_read / factor_depth
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    u, v = np.meshgrid(u, v)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    cloud = np.stack([x, y, z], axis=-1)   # (H, W, 3)
    # 取工作空间内、深度>0 的点
    mask = (workspace_mask > 0) & (depth_read > 0)
    n_valid = np.sum(mask)
    print("[create_fake_data] 已生成: %s" % out_dir)
    print("  color.png, depth.png, workspace_mask.png, meta.mat")
    print("  有效点数（工作空间内且深度>0）: %d" % n_valid)
    print("  若 num_point=20000，需 有效点>=20000 才能无放回采样；当前 %s" % ("满足" if n_valid >= 20000 else "不满足（会重复采样补足）"))
    return out_dir


# =============================================================================
# 复刻 demo.py 中的 get_and_process_data：逻辑完全一致，每一步带详细解析
# =============================================================================

def get_and_process_data_replica(data_dir, num_point=20000, device=None):
    """
    复刻 demo.py 的 get_and_process_data(data_dir) 逻辑。
    从 data_dir 读取 4 样数据 → 反投影点云 → 筛有效点 → 采样到固定点数 → 组装 end_points 与 Open3D cloud。

    参数:
        data_dir: 字符串，目录路径，内含 color.png, depth.png, workspace_mask.png, meta.mat
        num_point: 整数，网络输入要求的点数（demo 里为 cfgs.num_point，默认 20000）
        device: torch 设备，如 torch.device('cuda:0')；None 时自动选 cuda:0 或 cpu

    返回:
        end_points: dict，含 'point_clouds'(1,N,3 tensor)、'cloud_colors'
        cloud: open3d.geometry.PointCloud，全部有效点（未采样），用于碰撞检测与可视化
    """
    if device is None:
        device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

    # -------------------------------------------------------------------------
    # 第 1 步：加载图像与相机参数（与 demo 第 1 步一一对应）
    # -------------------------------------------------------------------------
    # 1.1 彩色图：读入后转 float32 并除以 255，得到 [0,1] 范围，形状 (H, W, 3)
    color_path = os.path.join(data_dir, 'color.png')
    color = np.array(Image.open(color_path), dtype=np.float32) / 255.0

    # 1.2 深度图：保持磁盘上的数值类型（常见为 uint16），形状 (H, W)
    depth_path = os.path.join(data_dir, 'depth.png')
    depth = np.array(Image.open(depth_path))

    # 1.3 工作空间掩码：非零表示该像素在工作空间内，形状 (H, W)
    mask_path = os.path.join(data_dir, 'workspace_mask.png')
    workspace_mask = np.array(Image.open(mask_path))

    # 1.4 相机参数：从 meta.mat 读 3x3 内参矩阵和深度缩放因子
    meta_path = os.path.join(data_dir, 'meta.mat')
    meta = scio.loadmat(meta_path)
    # 内参矩阵 [fx 0 cx; 0 fy cy; 0 0 1]，用于反投影
    intrinsic = meta['intrinsic_matrix']
    # 深度图像素值 / factor_depth = 真实深度（米）；meta 里可能是 (1,1) 数组，统一转为标量
    factor_depth = float(np.asarray(meta['factor_depth']).flat[0])

    # -------------------------------------------------------------------------
    # 第 2 步：由深度图反投影为 3D 点云（与 demo 第 2 步一致，此处内联实现）
    # -------------------------------------------------------------------------
    # 2.1 图像尺寸
    height, width = depth.shape[0], depth.shape[1]

    # 2.2 反投影公式（与 utils/data_utils.py create_point_cloud_from_depth_image 一致）：
    #     Z = depth / factor_depth（米）
    #     X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy
    # 其中 u 为列索引、v 为行索引；fx=intrinsic[0,0], fy=intrinsic[1,1], cx=intrinsic[0,2], cy=intrinsic[1,2]
    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    # 深度转米：像素值 / factor_depth → Z（相机坐标系下深度），形状 (H, W)
    points_z = np.asarray(depth, dtype=np.float32) / factor_depth
    # 像素坐标：u 为列索引 [0..width-1]，v 为行索引 [0..height-1]
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    # 网格化后 u,v 均为 (H, W)，同一位置 [v,u] 对应像素 (列u, 行v)
    u, v = np.meshgrid(u, v)
    # 针孔反投影：X = (u - cx) * Z / fx，Y = (v - cy) * Z / fy，形状均为 (H, W)，单位米
    points_x = (u - cx) * points_z / fx
    points_y = (v - cy) * points_z / fy
    # 在最后一维堆叠：每个像素 [v,u] 得到 [x,y,z]，cloud 形状 (H, W, 3)
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    print("cloud.shape:", cloud.shape)
    # print("cloud:", cloud)

    # -------------------------------------------------------------------------
    # 第 3 步：筛出有效点（在工作空间内且深度>0）
    # -------------------------------------------------------------------------
    # 3.1 有效像素条件：工作空间掩码非零 且 深度大于 0（排除无效/缺失深度）
    mask = (workspace_mask != 0) & (depth > 0)
    # print("mask.shape:", mask.shape)
    # 3.2 用布尔掩码从有序点云中取出有效点，形状 (N_valid, 3)
    cloud_masked = cloud[mask ]
    # print("cloud", cloud[mask])
    print("cloud_masked.shape:", cloud_masked.shape)
    # 3.3 同样按掩码取出有效像素的颜色，形状 (N_valid, 3)
    color_masked = color[mask]
    print("color_masked.shape:", color_masked.shape)
    # -------------------------------------------------------------------------
    # 第 4 步：采样到固定点数（与 demo 第 4 步完全一致）
    # -------------------------------------------------------------------------
    n_valid = len(cloud_masked)
    if n_valid >= num_point:
        # 点数足够：在 [0, n_valid-1] 中无放回随机抽取 num_point 个索引
        idxs = np.random.choice(n_valid, num_point, replace=False)
    else:
        # 点数不足：先取全部 n_valid 个索引，再在 [0, n_valid-1] 中有放回抽取 (num_point - n_valid) 个，拼成 num_point 个
        idxs1 = np.arange(n_valid)
        idxs2 = np.random.choice(n_valid, num_point - n_valid, replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    # 按索引取点云和颜色，得到 (num_point, 3) 与 (num_point, 3)
    print("idxs.shape:", idxs.shape)
    print("idxs:", idxs)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]
    print("cloud_sampled.shape:", cloud_sampled.shape)
    print("color_sampled.shape:", color_sampled.shape)

    # -------------------------------------------------------------------------
    # 第 5 步：组装返回值（与 demo 第 5 步一致）
    # -------------------------------------------------------------------------
    # 5.1 Open3D 点云：用「全部有效点」（未采样），供碰撞检测与可视化
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    # 5.2 模型输入字典：采样后的点云送进网络
    end_points = dict()
    # 增加 batch 维：(num_point, 3) → (1, num_point, 3)，再转 tensor 并放到 device
    cloud_sampled_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)
    end_points['point_clouds'] = cloud_sampled_tensor   # GraspNet 期望的键名
    end_points['cloud_colors'] = color_sampled         # 颜色（demo 中未直接参与前向）

    return end_points, cloud_o3d


if __name__ == '__main__':
    # 直接使用默认：输出目录、图像尺寸、点数，不解析命令行参数
    out_dir = 'doc/fake_pose_1'
    create_fake_data(out_dir, height=HEIGHT, width=WIDTH)
    if torch is not None and o3d is not None:
        print("[run_replica] 对 %s 执行复刻逻辑..." % out_dir)
        end_pts, cloud = get_and_process_data_replica(out_dir, num_point=20000)
        print("  end_points['point_clouds'].shape:", end_pts['point_clouds'].shape)
        print("  end_points['cloud_colors'].shape:", end_pts['cloud_colors'].shape)
        print("  cloud (Open3D) 点数:", np.asarray(cloud.points).shape[0])
        # 可视化：弹出窗口显示“全部有效点”的点云（带颜色），关闭窗口后脚本继续/退出
        print("  正在打开点云可视化窗口，关闭窗口后程序结束...")
        o3d.visualization.draw_geometries([cloud], window_name="点云（全部有效点）")
    else:
        print("  [跳过复刻] 需要安装 torch 与 open3d 才能运行 get_and_process_data_replica")
