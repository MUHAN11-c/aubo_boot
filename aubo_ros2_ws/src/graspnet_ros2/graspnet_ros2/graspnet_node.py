#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet ROS2 节点：接收点云话题，预测抓取位姿并发布 MarkerArray。

功能：
  1. 订阅 PointCloud2 话题或从文件读取数据
  2. 使用预训练 GraspNet 模型预测抓取位姿
  3. 碰撞检测、NMS、排序
  4. 发布 MarkerArray 到 RViz2
"""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
import numpy as np
import torch
import time
from scipy.spatial.transform import Rotation

# 路径设置函数（ROS2 标准方式）
def setup_baseline_paths(baseline_dir):
    """设置 graspnet-baseline 的 Python 路径
    
    需要同时添加 baseline_dir 和 models 目录到 sys.path：
    - baseline_dir: 用于 from models.graspnet import ...
    - models 目录: 用于 graspnet.py 内部的 from backbone import ...
    """
    if baseline_dir and os.path.exists(baseline_dir):
        # 添加 baseline_dir 本身（用于 from models.graspnet import ...）
        if baseline_dir not in sys.path:
            sys.path.insert(0, baseline_dir)
        # 添加 models 目录（用于 graspnet.py 内部的 from backbone import ...）
        models_dir = os.path.join(baseline_dir, 'models')
        if os.path.exists(models_dir) and models_dir not in sys.path:
            sys.path.insert(0, models_dir)
        return True
    return False


class GraspNetNode(Node):
    """GraspNet ROS2 节点：处理点云输入，预测并发布抓取位姿"""
    
    def __init__(self):
        super().__init__('graspnet_node')
        
        # 声明参数（baseline_dir 从 launch 文件传入）
        self.declare_parameter('baseline_dir', '')
        self.declare_parameter('model_path', '')
        
        # 获取参数
        baseline_dir = self.get_parameter('baseline_dir').get_parameter_value().string_value
        
        # 设置 Python 路径（使用从 launch 文件传入的 baseline_dir）
        if baseline_dir and os.path.exists(baseline_dir):
            if not setup_baseline_paths(baseline_dir):
                self.get_logger().error(f'无法设置 baseline 路径: {baseline_dir}')
                raise RuntimeError(f'无法设置 baseline 路径: {baseline_dir}')
            self.get_logger().info(f'已设置 baseline 路径: {baseline_dir}')
        else:
            self.get_logger().error(f'baseline_dir 参数无效或目录不存在: {baseline_dir}')
            raise RuntimeError(f'baseline_dir 参数无效或目录不存在: {baseline_dir}')
        
        # 导入 graspnet-baseline 模块（必须在路径设置后）
        from models.graspnet import GraspNet, pred_decode
        from utils.collision_detector import ModelFreeCollisionDetector
        from graspnetAPI import GraspGroup
        
        # 保存到实例变量
        self.GraspNet = GraspNet
        self.pred_decode = pred_decode
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self.GraspGroup = GraspGroup
        
        # 获取 model_path 参数
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.declare_parameter('max_point_num', 20000)
        self.declare_parameter('num_view', 300)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('max_grasps_num', 20)
        self.declare_parameter('gpu', 0)
        self.declare_parameter('pointcloud_topic', 'pointcloud')
        self.declare_parameter('marker_topic', 'grasp_markers')
        self.declare_parameter('frame_id', 'camera_frame')
        
        # 获取参数
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.max_point_num = self.get_parameter('max_point_num').get_parameter_value().integer_value
        self.num_view = self.get_parameter('num_view').get_parameter_value().integer_value
        self.collision_thresh = self.get_parameter('collision_thresh').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.max_grasps_num = self.get_parameter('max_grasps_num').get_parameter_value().integer_value
        self.gpu = self.get_parameter('gpu').get_parameter_value().integer_value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 设置设备
        self.device = torch.device(f"cuda:{self.gpu}" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"使用设备: {self.device}")
        
        # 加载模型
        self.net = self._load_model()
        
        # 创建发布器
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        
        # 创建订阅器
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )
        
        # 标记 ID 计数器
        self.draw_grasp_id = 0
        
        self.get_logger().info(f'GraspNet 节点已启动')
        self.get_logger().info(f'订阅话题: {self.pointcloud_topic}')
        self.get_logger().info(f'发布话题: {self.marker_topic}')
    
    def _load_model(self):
        """加载 GraspNet 模型"""
        self.get_logger().info(f'正在加载模型: {self.model_path}')
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'模型文件不存在: {self.model_path}')
            raise FileNotFoundError(f'模型文件不存在: {self.model_path}')
        
        # 创建模型
        net = self.GraspNet(
            input_feature_dim=0,
            num_view=self.num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        net.to(self.device)
        
        # 加载权重
        checkpoint = torch.load(self.model_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])
        net.eval()
        
        self.get_logger().info('模型加载完成')
        return net
    
    def pointcloud_callback(self, msg):
        """点云回调函数"""
        try:
            # 将 PointCloud2 转换为 numpy 数组
            # 注意：这里需要 ros_numpy 或类似工具
            # 简化版本：假设点云已经转换为 numpy
            self.get_logger().info('收到点云消息')
            
            # TODO: 实现 PointCloud2 到 numpy 的转换
            # 这里需要 ros_numpy 或手动解析 PointCloud2
            
        except Exception as e:
            self.get_logger().error(f'处理点云时出错: {str(e)}')
    
    def predict_grasps_from_points(self, points):
        """
        从点云预测抓取
        
        参数:
            points: np.ndarray, shape=(N, 3)，点云坐标
            
        返回:
            gg: GraspGroup，预测的抓取组
        """
        # 采样点云
        if len(points) > self.max_point_num:
            idxs = np.random.choice(len(points), self.max_point_num, replace=False)
        else:
            idxs = np.arange(len(points))
        
        points_sampled = points[idxs]
        
        # 转换为 tensor
        points_torch = torch.from_numpy(points_sampled[np.newaxis].astype(np.float32)).to(self.device)
        end_points = {'point_clouds': points_torch}
        
        # 预测
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = self.pred_decode(end_points)
        
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = self.GraspGroup(gg_array)
        
        return gg
    
    def collision_detection(self, gg, cloud):
        """碰撞检测"""
        if self.collision_thresh <= 0:
            return gg
        
        mfcdetector = self.ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(
            gg, approach_dist=0.05, collision_thresh=self.collision_thresh
        )
        gg = gg[~collision_mask]
        self.get_logger().info(f'碰撞检测后剩余 {len(gg)} 个抓取')
        return gg
    
    def publish_marker_array(self, gg):
        """发布 MarkerArray"""
        # 清除之前的标记
        self.clear_markers()
        
        # NMS 和排序
        gg = gg.nms()
        gg.sort_by_score()
        gg = gg[:self.max_grasps_num]
        
        if len(gg) == 0:
            self.get_logger().warn('没有可发布的抓取')
            return
        
        # 颜色映射函数
        def color_map(score):
            return tuple([float(1 - score), float(score), float(0), float(1)])
        
        vmin, vmax = 0, 1
        self.draw_grasp_id = 0
        
        # 发布每个抓取
        for i in range(len(gg)):
            grasp = gg[i]
            score_normalized = (grasp.score - vmin) / (vmax - vmin) if vmax > vmin else 0.5
            color = color_map(score_normalized)
            
            markers = self.create_grasp_markers(grasp, color, self.draw_grasp_id)
            marker_array = MarkerArray()
            marker_array.markers = markers
            self.marker_pub.publish(marker_array)
            
            self.draw_grasp_id += 4
            time.sleep(0.05)
        
        self.get_logger().info(f'已发布 {len(gg)} 个抓取')
    
    def clear_markers(self):
        """清除所有标记"""
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        
        marker_array = MarkerArray()
        marker_array.markers = [delete_marker]
        self.marker_pub.publish(marker_array)
    
    def create_grasp_markers(self, grasp, color, id_start=0):
        """创建单个抓取的标记"""
        pose_mat = np.eye(4)
        pose_mat[:3, 3] = grasp.translation
        pose_mat[:3, :3] = grasp.rotation_matrix
        
        w = grasp.width
        d = grasp.depth
        radius = 0.005
        stamp = self.get_clock().now().to_msg()
        
        markers = []
        
        # 左指
        left_point = np.dot(pose_mat, np.array([-w / 2, 0, -d / 2, 1]))
        left_pose = pose_mat.copy()
        left_pose[:3, 3] = left_point[:3]
        markers.append(self._create_marker(
            Marker.CYLINDER, left_pose, [radius, radius, d], color, id_start, stamp
        ))
        
        # 右指
        right_point = np.dot(pose_mat, np.array([w / 2, 0, -d / 2, 1]))
        right_pose = pose_mat.copy()
        right_pose[:3, 3] = right_point[:3]
        markers.append(self._create_marker(
            Marker.CYLINDER, right_pose, [radius, radius, d], color, id_start + 1, stamp
        ))
        
        # 手腕
        wrist_point = np.dot(pose_mat, np.array([0.0, 0.0, -d * 5 / 4, 1]))
        wrist_pose = pose_mat.copy()
        wrist_pose[:3, 3] = wrist_point[:3]
        markers.append(self._create_marker(
            Marker.CYLINDER, wrist_pose, [radius, radius, d / 2], color, id_start + 2, stamp
        ))
        
        # 手掌
        palm_point = np.dot(pose_mat, np.array([0.0, 0.0, -d, 1]))
        palm_pose = pose_mat.copy()
        palm_pose[:3, 3] = palm_point[:3]
        rot = np.eye(4)
        rot[:3, :3] = Rotation.from_rotvec([0, np.pi / 2, 0]).as_matrix()
        palm_pose = np.dot(palm_pose, rot)
        markers.append(self._create_marker(
            Marker.CYLINDER, palm_pose, [radius, radius, w], color, id_start + 3, stamp
        ))
        
        return markers
    
    def _create_marker(self, marker_type, pose_mat, scale, color, id, stamp):
        """创建单个 Marker"""
        if np.isscalar(scale):
            scale = [scale, scale, scale]
        
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "grasp"
        marker.id = id
        marker.type = marker_type
        marker.action = Marker.ADD
        
        # 转换位姿
        marker.pose = self._matrix_to_pose(pose_mat)
        
        # 设置尺寸和颜色
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        
        return marker
    
    def _matrix_to_pose(self, pose_mat):
        """将变换矩阵转换为 Pose"""
        pose = Pose()
        quat = Rotation.from_matrix(pose_mat[:3, :3]).as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        pose.position.x = float(pose_mat[0, 3])
        pose.position.y = float(pose_mat[1, 3])
        pose.position.z = float(pose_mat[2, 3])
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = GraspNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
