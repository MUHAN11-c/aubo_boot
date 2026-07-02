#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手眼标定结果 TF 发布节点
从标定结果文件读取变换矩阵，发布相机与末端执行器之间的静态 TF 变换
"""

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory


class HandEyeCalibrationTfPublisher(Node):
    """手眼标定 TF 发布节点"""
    
    def __init__(self):
        super().__init__('hand_eye_calibration_tf_publisher')
        
        # 声明参数
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('parent_frame', 'ee_link')
        self.declare_parameter('child_frame', 'camera_link')
        self.declare_parameter('publish_rate', 1.0)
        
        calibration_file = self.get_parameter('calibration_file').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 如果没有指定文件，使用默认路径
        if not calibration_file:
            # 首先尝试从源码目录查找
            current_file = os.path.abspath(__file__)
            file_dir = os.path.dirname(current_file)
            # 从 hand_eye_calibration/hand_eye_calibration/ 回到 src/hand_eye_calibration/
            src_dir = os.path.dirname(os.path.dirname(file_dir))
            calibration_file = os.path.join(
                src_dir,
                'config',
                'calibration_results',
                'hand_eye_calibration_20260112_155451.yaml'
            )
            calibration_file = os.path.abspath(calibration_file)
            
            # 如果源码目录不存在，尝试从 install 目录查找
            if not os.path.exists(calibration_file):
                try:
                    package_share = get_package_share_directory('hand_eye_calibration')
                    calibration_file = os.path.join(
                        package_share,
                        'config',
                        'calibration_results',
                        'hand_eye_calibration_20260112_155451.yaml'
                    )
                except:
                    pass
        
        self.get_logger().info(f'📁 标定文件路径: {calibration_file}')
        
        # 检查文件是否存在
        if not os.path.exists(calibration_file):
            self.get_logger().error(f'❌ 标定文件不存在: {calibration_file}')
            raise FileNotFoundError(f'标定文件不存在: {calibration_file}')
        
        # 读取标定结果
        try:
            with open(calibration_file, 'r', encoding='utf-8') as f:
                calib_data = yaml.safe_load(f)
            
            # 提取变换矩阵
            hand_eye_data = calib_data.get('hand_eye_calibration', {})
            transformation_matrix = hand_eye_data.get('transformation_matrix')
            
            if transformation_matrix is None:
                raise ValueError('标定文件中未找到 transformation_matrix')
            
            # 转换为 numpy 数组
            T_camera_to_ee = np.array(transformation_matrix, dtype=np.float64)
            
            self.get_logger().info(f'✅ 成功加载标定结果')
            self.get_logger().info(f'   标定类型: {hand_eye_data.get("calibration_type", "Unknown")}')
            self.get_logger().info(f'   标定方法: {hand_eye_data.get("calibration_method", "Unknown")}')
            self.get_logger().info(f'   标定日期: {hand_eye_data.get("calibration_date", "Unknown")}')
            
            # 提取旋转矩阵和平移向量
            R_matrix = T_camera_to_ee[:3, :3]
            t_vector = T_camera_to_ee[:3, 3]  # 单位：毫米
            
            self.get_logger().info(f'   旋转矩阵行列式: {np.linalg.det(R_matrix):.6f} (应该≈1.0)')
            self.get_logger().info(f'   平移向量 (mm): [{t_vector[0]:.3f}, {t_vector[1]:.3f}, {t_vector[2]:.3f}]')
            
            # 标定结果说明：
            # OpenCV模式的手眼标定结果是 T_camera_to_ee（相机坐标系 → 末端执行器/夹爪坐标系）
            # 这是从「相机坐标系（Camera Coordinate System）」到「机器人末端执行器（夹爪/工具）坐标系
            # （Gripper/End-effector/Tool Coordinate System）」的刚性变换
            #
            # ROS TF 需要：
            # 由于相机安装在末端执行器上，camera_link 是 ee_link 的子坐标系
            # TF 发布的是从父坐标系到子坐标系的变换，即 ee_link -> camera_link
            # 因此需要对标定结果取逆：T_ee_to_camera = inv(T_camera_to_ee)
            T_ee_to_camera = np.linalg.inv(T_camera_to_ee)
            R_ee_to_camera = T_ee_to_camera[:3, :3]
            t_ee_to_camera = T_ee_to_camera[:3, 3]  # 单位：毫米
            
            # 转换为米（ROS tf 使用米）
            t_ee_to_camera_m = t_ee_to_camera / 1000.0
            
            self.get_logger().info(f'   发布变换: {self.parent_frame} -> {self.child_frame}')
            self.get_logger().info(f'   平移 (m): [{t_ee_to_camera_m[0]:.6f}, {t_ee_to_camera_m[1]:.6f}, {t_ee_to_camera_m[2]:.6f}]')
            
            # 将旋转矩阵转换为四元数（兼容不同版本的 scipy）
            try:
                # 新版本 scipy (>=1.2.0) 使用 from_matrix
                if hasattr(R, 'from_matrix'):
                    rotation = R.from_matrix(R_ee_to_camera)
                else:
                    # 旧版本 scipy 使用 from_dcm
                    rotation = R.from_dcm(R_ee_to_camera)
                quaternion = rotation.as_quat()  # [x, y, z, w]
            except Exception as e:
                # 如果都失败，使用手动转换方法
                self.get_logger().warn(f'使用 scipy 转换失败: {e}，改用手动转换方法')
                quaternion = self._rotation_matrix_to_quaternion(R_ee_to_camera)
            
            self.get_logger().info(f'   四元数: [{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]')
            
            # 创建静态 TF 广播器
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            
            # 创建变换消息
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = self.parent_frame
            transform_stamped.child_frame_id = self.child_frame
            
            # 设置平移（米）
            transform_stamped.transform.translation.x = float(t_ee_to_camera_m[0])
            transform_stamped.transform.translation.y = float(t_ee_to_camera_m[1])
            transform_stamped.transform.translation.z = float(t_ee_to_camera_m[2])
            
            # 设置旋转（四元数）
            transform_stamped.transform.rotation.x = float(quaternion[0])
            transform_stamped.transform.rotation.y = float(quaternion[1])
            transform_stamped.transform.rotation.z = float(quaternion[2])
            transform_stamped.transform.rotation.w = float(quaternion[3])
            
            # 发布静态变换
            self.tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().info(f'✅ 已发布静态 TF 变换: {self.parent_frame} -> {self.child_frame}')
            
            # 如果有标定精度信息，也打印出来
            accuracy = hand_eye_data.get('calibration_accuracy', {})
            if accuracy:
                mean_error = accuracy.get('mean_error_mm', 0)
                max_error = accuracy.get('max_error_mm', 0)
                self.get_logger().info(f'   标定精度: 平均误差 {mean_error:.3f} mm, 最大误差 {max_error:.3f} mm')
            
        except Exception as e:
            self.get_logger().error(f'❌ 读取或发布标定结果失败: {str(e)}')
            import traceback
            self.get_logger().error(f'   堆栈信息: {traceback.format_exc()}')
            raise
    
    def _rotation_matrix_to_quaternion(self, R):
        """
        将旋转矩阵转换为四元数（手动实现，兼容所有版本）
        使用 Shepperd's method
        
        Args:
            R: 3x3 旋转矩阵
            
        Returns:
            quaternion: [x, y, z, w]
        """
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandEyeCalibrationTfPublisher()
        # 静态变换只需要发布一次，不需要持续运行
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('✅ TF 发布节点已完成，静态变换将持续存在')
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'❌ 节点运行失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
