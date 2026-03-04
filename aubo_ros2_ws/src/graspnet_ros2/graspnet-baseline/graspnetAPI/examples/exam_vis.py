# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：可视化。
展示物体抓取、6D 位姿、场景矩形抓取与场景 6D 抓取的可视化接口。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

####################################################################
graspnet_root = '/home/gmh/graspnet'  # GraspNet 数据集根路径，请按需修改
####################################################################

from graspnetAPI import GraspNet

# 初始化 GraspNet 实例（kinect，train 划分）
g = GraspNet(graspnet_root, camera='kinect', split='train')

# 显示指定物体的抓取（objIds=0）
g.showObjGrasp(objIds=0, show=True)

# 显示 6D 位姿（场景 0）
g.show6DPose(sceneIds=0, show=True)

# 显示场景矩形抓取：场景 0，realsense，标注 0，矩形格式，数量 20
g.showSceneGrasp(sceneId=0, camera='realsense', annId=0, format='rect', numGrasp=20)

# 显示场景 6D 抓取（场景 4，kinect，标注 2；可能需等待数分钟）
g.showSceneGrasp(sceneId=4, camera='kinect', annId=2, format='6d')
