# -*- coding: utf-8 -*-
# GraspNet 工具包（utils）的包初始化文件
#
# 本包包含以下子模块：
#   - collision_detector：无模型碰撞检测，过滤与场景碰撞的抓取
#   - data_utils：相机内参、深度图反投影、点云变换、工作空间掩码等
#   - label_generation：训练时动态生成抓取标签（process_grasp_labels、match_grasp_view_and_label）
#   - loss_utils：损失与视角工具（transform_point_cloud、generate_grasp_views、huber_loss 等）
#
# 使用方式：from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
#           from utils.collision_detector import ModelFreeCollisionDetector
#           from utils.loss_utils import transform_point_cloud, generate_grasp_views
#           from utils.label_generation import process_grasp_labels, match_grasp_view_and_label
