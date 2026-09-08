# -*- coding: utf-8 -*-
"""
graspnet_ros2：GraspNet 6-DOF 抓取检测（点云 → 抓取位姿）ROS 2 包.

架构参考 anygrasp_with_ros 的分层思路（采集 → 推理核心 → ROS 发布互相解耦），
推理后端使用本地 GraspNet-baseline 权重（vendored 纯 torch 子集），
不依赖 AnyGrasp SDK（许可证限制）与 open3d/graspnetAPI/pointnet2 CUDA 扩展。
"""

__version__ = '1.0.0'
