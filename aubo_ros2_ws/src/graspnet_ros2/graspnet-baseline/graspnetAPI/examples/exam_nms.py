# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：抓取 NMS（非极大值抑制）。
对 6D 抓取做 NMS，去除位置与姿态过于接近的重复抓取，便于可视化和后续筛选。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

####################################################################
graspnet_root = '/home/gmh/graspnet'  # GraspNet 数据集根路径，请按需修改
####################################################################

sceneId = 1
annId = 3

from graspnetAPI import GraspNet
import open3d as o3d
import cv2

# 初始化 GraspNet 实例（kinect，train 划分）
g = GraspNet(graspnet_root, camera='kinect', split='train')

# 加载 6D 抓取：场景 1，标注 3，kinect，摩擦系数阈值 0.2
_6d_grasp = g.loadGrasp(sceneId=sceneId, annId=annId, format='6d', camera='kinect', fric_coef_thresh=0.2)
print('6d grasp:\n{}'.format(_6d_grasp))

# 可视化 NMS 前：场景点云 + 随机 1000 个抓取
geometries = []
geometries.append(g.loadScenePointCloud(sceneId=sceneId, annId=annId, camera='kinect'))
geometries += _6d_grasp.random_sample(numGrasp=1000).to_open3d_geometry_list()
o3d.visualization.draw_geometries(geometries)

# NMS：平移阈值 0.1 m，旋转阈值 30 度（弧度）
nms_grasp = _6d_grasp.nms(translation_thresh=0.1, rotation_thresh=30 / 180.0 * 3.1416)
print('grasp after nms:\n{}'.format(nms_grasp))

# 可视化 NMS 后的抓取
geometries = []
geometries.append(g.loadScenePointCloud(sceneId=sceneId, annId=annId, camera='kinect'))
geometries += nms_grasp.to_open3d_geometry_list()
o3d.visualization.draw_geometries(geometries)
