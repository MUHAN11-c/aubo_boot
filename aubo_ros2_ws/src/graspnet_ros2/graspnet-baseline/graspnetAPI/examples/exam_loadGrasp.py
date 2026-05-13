# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：加载场景抓取并可视化。
加载 6D 与矩形抓取，分别用 Open3D（点云+抓取）和 OpenCV（矩形叠加图）展示。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

from graspnetAPI import GraspNet
import open3d as o3d
import cv2

####################################################################
graspnet_root = '/home/gmh/graspnet'  # GraspNet 数据集根路径，请按需修改
####################################################################

sceneId = 1
annId = 3

# 初始化 GraspNet 实例（kinect，train 划分）
g = GraspNet(graspnet_root, camera='kinect', split='train')

# 加载 6D 抓取：场景 1，标注 3，kinect，摩擦系数阈值 0.2
_6d_grasp = g.loadGrasp(sceneId=sceneId, annId=annId, format='6d', camera='kinect', fric_coef_thresh=0.2)
print('6d grasp:\n{}'.format(_6d_grasp))

# 使用 Open3D 可视化：场景点云 + 随机 20 个 6D 抓取
geometries = []
geometries.append(g.loadScenePointCloud(sceneId=sceneId, annId=annId, camera='kinect'))
geometries += _6d_grasp.random_sample(numGrasp=20).to_open3d_geometry_list()
o3d.visualization.draw_geometries(geometries)

# 加载矩形抓取：场景 1，标注 3，realsense，摩擦系数阈值 0.2
rect_grasp = g.loadGrasp(sceneId=sceneId, annId=annId, format='rect', camera='realsense', fric_coef_thresh=0.2)
print('rectangle grasp:\n{}'.format(rect_grasp))

# 使用 OpenCV 可视化：在 BGR 图上绘制矩形抓取（随机 20 个）
bgr = g.loadBGR(sceneId=sceneId, annId=annId, camera='realsense')
img = rect_grasp.to_opencv_image(bgr, numGrasp=20)
cv2.imshow('rectangle grasps', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
