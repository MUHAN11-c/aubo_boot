# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：抓取格式转换。
演示矩形抓取(Rect)与 6D 抓取之间的相互转换，以及 OpenCV/Open3D 可视化。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

from graspnetAPI import GraspNet
import cv2
import open3d as o3d

# 配置：相机类型、场景 ID、标注 ID
camera = 'kinect'
sceneId = 5
annId = 3

####################################################################
graspnet_root = '/home/gmh/graspnet'  # GraspNet 数据集根路径，请按需修改
####################################################################

g = GraspNet(graspnet_root, camera=camera, split='all')

# 加载 BGR 彩色图与深度图
bgr = g.loadBGR(sceneId=sceneId, camera=camera, annId=annId)
depth = g.loadDepth(sceneId=sceneId, camera=camera, annId=annId)

# ========== Rect 转 6D ==========
# 加载矩形格式抓取（摩擦系数阈值 0.2）
rect_grasp_group = g.loadGrasp(sceneId=sceneId, camera=camera, annId=annId, fric_coef_thresh=0.2, format='rect')

# 单个 RectGrasp 转 Grasp（6D）
rect_grasp = rect_grasp_group.random_sample(1)[0]
img = rect_grasp.to_opencv_image(bgr)

cv2.imshow('rect grasp', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 将矩形抓取转换为 6D 抓取（需深度图）
grasp = rect_grasp.to_grasp(camera, depth)
if grasp is not None:
    geometry = []
    geometry.append(g.loadScenePointCloud(sceneId, camera, annId))
    geometry.append(grasp.to_open3d_geometry())
    o3d.visualization.draw_geometries(geometry)
else:
    print('No result because the depth is invalid, please try again!')  # 深度无效，无结果

# RectGraspGroup 转 GraspGroup（批量）
sample_rect_grasp_group = rect_grasp_group.random_sample(20)
img = sample_rect_grasp_group.to_opencv_image(bgr)
cv2.imshow('rect grasp', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

grasp_group = sample_rect_grasp_group.to_grasp_group(camera, depth)
if grasp_group is not None:
    geometry = []
    geometry.append(g.loadScenePointCloud(sceneId, camera, annId))
    geometry += grasp_group.to_open3d_geometry_list()
    o3d.visualization.draw_geometries(geometry)

# ========== 6D 转 Rect ==========
# 加载 6D 格式抓取
_6d_grasp_group = g.loadGrasp(sceneId=sceneId, camera=camera, annId=annId, fric_coef_thresh=0.2, format='6d')

# 注：Grasp 转 RectGrasp 不常用，因仅少数 6D 抓取可转为矩形抓取

# GraspGroup 转 RectGraspGroup
sample_6d_grasp_group = _6d_grasp_group.random_sample(20)
geometry = []
geometry.append(g.loadScenePointCloud(sceneId, camera, annId))
geometry += sample_6d_grasp_group.to_open3d_geometry_list()
o3d.visualization.draw_geometries(geometry)

rect_grasp_group = _6d_grasp_group.to_rect_grasp_group(camera)
img = rect_grasp_group.to_opencv_image(bgr)

cv2.imshow('rect grasps', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
