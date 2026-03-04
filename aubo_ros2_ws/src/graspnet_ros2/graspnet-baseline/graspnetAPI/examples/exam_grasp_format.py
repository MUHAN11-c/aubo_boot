# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：抓取格式与数据结构。
演示如何加载 6D/矩形抓取、访问 Grasp/GraspGroup 属性、索引与切片、以及位姿变换。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

from graspnetAPI import GraspNet, Grasp, GraspGroup
import open3d as o3d
import cv2
import numpy as np

####################################################################
graspnet_root = '/disk1/graspnet'  # GraspNet 数据集根路径，请按需修改
####################################################################

sceneId = 1
annId = 3

# 初始化 GraspNet 实例（kinect，train 划分）
g = GraspNet(graspnet_root, camera='kinect', split='train')

# 加载 6D 抓取：场景 1，标注 3，kinect，摩擦系数阈值 0.2
_6d_grasp = g.loadGrasp(sceneId=sceneId, annId=annId, format='6d', camera='kinect', fric_coef_thresh=0.2)
print('6d grasp:\n{}'.format(_6d_grasp))

# _6d_grasp 为 grasp.py 中定义的 GraspGroup 实例
print('_6d_grasp:\n{}'.format(_6d_grasp))

# 索引：取第一个抓取
grasp = _6d_grasp[0]
print('_6d_grasp[0](grasp):\n{}'.format(grasp))

# 切片
print('_6d_grasp[0:2]:\n{}'.format(_6d_grasp[0:2]))
print('_6d_grasp[[0,1]]:\n{}'.format(_6d_grasp[[0, 1]]))

# grasp 为 grasp.py 中定义的 Grasp 实例：访问与修改属性
print('grasp.translation={}'.format(grasp.translation))
grasp.translation = np.array([1.0, 2.0, 3.0])
print('After modification, grasp.translation={}'.format(grasp.translation))
print('grasp.rotation_matrix={}'.format(grasp.rotation_matrix))
grasp.rotation_matrix = np.eye(3).reshape((9))
print('After modification, grasp.rotation_matrix={}'.format(grasp.rotation_matrix))
print('grasp.width={}, height:{}, depth:{}, score:{}'.format(grasp.width, grasp.height, grasp.depth, grasp.score))
print('More operation on Grasp and GraspGroup can be seen in the API document')

# 加载矩形抓取：场景 1，标注 3，realsense，摩擦系数阈值 0.2
rect_grasp_group = g.loadGrasp(sceneId=sceneId, annId=annId, format='rect', camera='realsense', fric_coef_thresh=0.2)
print('rectangle grasp group:\n{}'.format(rect_grasp_group))

# rect_grasp_group 为 grasp.py 中定义的 RectGraspGroup 实例
print('rect_grasp_group:\n{}'.format(rect_grasp_group))

# 索引
rect_grasp = rect_grasp_group[0]
print('rect_grasp_group[0](rect_grasp):\n{}'.format(rect_grasp))

# 切片
print('rect_grasp_group[0:2]:\n{}'.format(rect_grasp_group[0:2]))
print('rect_grasp_group[[0,1]]:\n{}'.format(rect_grasp_group[[0, 1]]))

# 矩形抓取属性：中心点、张开点、高度、分数
print('rect_grasp.center_point:{}, open_point:{}, height:{}, score:{}'.format(
    rect_grasp.center_point, rect_grasp.open_point, rect_grasp.height, rect_grasp.score))

# ========== 抓取位姿变换 ==========
g = Grasp()  # 创建一个简单 Grasp
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)

# 变换前的抓取
o3d.visualization.draw_geometries([g.to_open3d_geometry(), frame])
g.translation = np.array((0, 0, 0.01))

# 构造 4x4 变换矩阵 T
T = np.eye(4)
T[:3, 3] = np.array((0.01, 0.02, 0.03))
T[:3, :3] = np.array([[0, 0, 1.0], [1, 0, 0], [0, 1, 0]])
g.transform(T)

# 变换后的抓取
o3d.visualization.draw_geometries([g.to_open3d_geometry(), frame])

# GraspGroup 的构造与变换
g1 = Grasp()
gg = GraspGroup()
gg.add(g)
gg.add(g1)

# 变换前的 GraspGroup
o3d.visualization.draw_geometries([*gg.to_open3d_geometry_list(), frame])

gg.transform(T)

# 变换后的 GraspGroup
o3d.visualization.draw_geometries([*gg.to_open3d_geometry_list(), frame])
