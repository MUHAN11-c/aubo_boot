# -*- coding: utf-8 -*-
"""
GraspNetAPI 示例：检查数据完整性。
检查 GraspNet 数据集中 kinect 与 realsense 数据是否完整。
使用前请修改 graspnet_root 路径。
"""
__author__ = 'mhgou'
__version__ = '1.0'

from graspnetAPI import GraspNet

if __name__ == '__main__':

    ####################################################################
    graspnet_root = '/home/gmh/graspnet'  ### GraspNet 数据集根路径，请按需修改 ###
    ####################################################################

    # 使用 kinect 相机数据检查
    g = GraspNet(graspnet_root, 'kinect', 'all')
    if g.checkDataCompleteness():
        print('Check for kinect passed')  # kinect 数据检查通过

    # 使用 realsense 相机数据检查
    g = GraspNet(graspnet_root, 'realsense', 'all')
    if g.checkDataCompleteness():
        print('Check for realsense passed')  # realsense 数据检查通过
