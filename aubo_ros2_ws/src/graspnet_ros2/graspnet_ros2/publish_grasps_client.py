#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 发布客户端：手动触发抓取结果发布并控制机械臂运动

功能：
  - 调用 /publish_grasps 服务发布抓取位姿、MarkerArray 和点云
  - 从 TF 树中获取最佳抓取位姿（grasp_pose_0）
  - 调用 /move_to_pose 服务控制机械臂运动到抓取位姿

使用方法：
  ros2 run graspnet_ros2 publish_grasps_client
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from demo_interface.srv import MoveToPose
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
import csv
import os


class PublishGraspsClient(Node):
    """抓取发布客户端节点"""
    
    def __init__(self):
        super().__init__('publish_grasps_client')
        
        # 声明参数
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('grasp_frame', 'grasp_pose_0')
        self.declare_parameter('velocity_factor', 0.3)
        self.declare_parameter('acceleration_factor', 0.3)
        self.declare_parameter('use_joints', True)
        self.declare_parameter('grasp_z_offset', 0.185)  # gripper_tip_link 相对 wrist3_Link 的 z 偏移补偿（沿末端z轴）
        
        # 数据保存参数
        self.declare_parameter('save_grasp_data', True)  # 是否保存抓取数据到文件
        self.declare_parameter('save_dir', '/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/resource')  # 数据保存目录（建议用绝对路径，如 /tmp/grasp_data 或 ~/grasp_data）
        
        # 抓取过滤参数：只保留适合从上方竖直下压的抓取
        self.declare_parameter('filter_vertical_approach', True)  # 是否按 approach 与竖直方向夹角过滤
        self.declare_parameter('vertical_angle_deg', 30.0)  # approach 与竖直方向夹角上限（度），±此角度内保留
        self.declare_parameter('max_yaw_deg', 180.0)  # 绕世界 z 轴旋转（yaw）上限（度），|yaw| <= 此值保留
        
        # 获取参数
        self.base_frame = self.get_parameter('base_frame').value
        self.grasp_frame = self.get_parameter('grasp_frame').value
        self.velocity_factor = self.get_parameter('velocity_factor').value
        self.acceleration_factor = self.get_parameter('acceleration_factor').value
        self.use_joints = self.get_parameter('use_joints').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value
        
        # 获取数据保存参数
        self.save_grasp_data = self.get_parameter('save_grasp_data').value
        self.save_dir = self.get_parameter('save_dir').value
        
        # 获取抓取过滤参数
        self.filter_vertical_approach = self.get_parameter('filter_vertical_approach').value
        self.vertical_angle_deg = self.get_parameter('vertical_angle_deg').value
        self.max_yaw_deg = self.get_parameter('max_yaw_deg').value
        
        # 存储抓取数据
        self.grasp_data = None
        
        # 创建发布服务客户端
        self.publish_client = self.create_client(Trigger, 'publish_grasps')
        
        # 创建运动控制服务客户端
        self.move_client = self.create_client(MoveToPose, '/move_to_pose')
        
        # 创建 TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 等待服务可用
        self.get_logger().info('等待 /publish_grasps 服务...')
        while not self.publish_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/publish_grasps 服务不可用，继续等待...')
        
        self.get_logger().info('/publish_grasps 服务已连接')
        
        self.get_logger().info('等待 /move_to_pose 服务...')
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/move_to_pose 服务不可用，继续等待...')
        
        self.get_logger().info('/move_to_pose 服务已连接')
    
    def filter_grasps_by_approach_and_yaw(self, grasps_data):
        """
        只保留 approach 方向与竖直方向夹角在 ±vertical_angle_deg 内的抓取（适合从上方竖直下压抓），
        且绕世界 z 轴旋转（yaw）在 ±max_yaw_deg 内。
        
        GraspNet 的 rotation_matrix 第一列为 approach 方向；竖直方向取世界 z 轴 (0,0,1)。
        
        Args:
            grasps_data: 抓取列表，每项含 'rotation_matrix'(9 元列表), 'translation', 'score' 等
        
        Returns:
            过滤后的抓取列表（保持原有字典结构）
        """
        if not grasps_data or not self.filter_vertical_approach:
            return grasps_data
        
        vertical = np.array([0.0, 0.0, 1.0])
        cos_thresh = np.cos(np.deg2rad(self.vertical_angle_deg))
        max_yaw_rad = np.deg2rad(self.max_yaw_deg)
        filtered = []
        
        for g in grasps_data:
            R_flat = g.get('rotation_matrix')
            if not R_flat or len(R_flat) != 9:
                continue
            R = np.array(R_flat, dtype=np.float64).reshape(3, 3)
            # approach 方向 = 第一列
            approach = R[:, 0]
            # 与竖直方向夹角：|cos(angle)| >= cos_thresh 即 angle <= vertical_angle_deg
            cos_angle = np.dot(approach, vertical)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            if np.abs(cos_angle) < cos_thresh:
                continue
            # 绕世界 z 的 yaw：atan2(R[1,0], R[0,0])
            yaw = np.arctan2(R[1, 0], R[0, 0])
            if np.abs(yaw) > max_yaw_rad:
                continue
            filtered.append(g)
        
        self.get_logger().info(
            f'抓取过滤: 竖直 approach ±{self.vertical_angle_deg}°、yaw ±{self.max_yaw_deg}° → '
            f'{len(grasps_data)} → {len(filtered)} 个'
        )
        return filtered
    
    def save_grasps_to_csv(self, grasps_data):
        """
        将抓取数据保存到 CSV 文件
        
        Args:
            grasps_data: 抓取数据列表
        """
        if not self.save_grasp_data or not grasps_data or not self.save_dir:
            return
        
        try:
            # 转为绝对路径，避免相对路径导致文件找不到
            save_dir_abs = os.path.abspath(os.path.expanduser(self.save_dir))
            
            # 创建保存目录
            os.makedirs(save_dir_abs, exist_ok=True)  # type: ignore
            
            # 固定文件名，每次覆盖保存最新数据
            csv_file = os.path.join(save_dir_abs, 'grasps.csv')
            
            # 写入 CSV 文件
            with open(csv_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 写入中文含义说明行（# 开头，便于理解各列；pandas 读时可 comment='#' 忽略）
                f.write('# 原始抓取数据 列含义: 索引, 分数, 宽度(m), 高度(m), 深度(m), X(m), Y(m), Z(m), 物体ID, 旋转矩阵R00~R22, 到中心距离(m)\n')
                
                # 写入表头（英文列名+中文注释）
                header = [
                    'Index(索引)', 'Score(分数)', 'Width(m)(宽度)', 'Height(m)(高度)', 'Depth(m)(深度)',
                    'X(m)', 'Y(m)', 'Z(m)', 'Object_ID(物体ID)',
                    'R00', 'R01', 'R02', 'R10', 'R11', 'R12', 'R20', 'R21', 'R22',
                    'Distance_to_Center(m)(到中心距离)'
                ]
                writer.writerow(header)
                
                # 写入原始抓取数据行
                for grasp in grasps_data:
                    rot_matrix = grasp['rotation_matrix']
                    dist_to_center = np.sqrt(grasp['translation'][0]**2 + grasp['translation'][1]**2)
                    row = [
                        grasp['index'],
                        f"{grasp['score']:.4f}",
                        f"{grasp['width']:.4f}",
                        f"{grasp['height']:.4f}",
                        f"{grasp['depth']:.4f}",
                        f"{grasp['translation'][0]:.4f}",
                        f"{grasp['translation'][1]:.4f}",
                        f"{grasp['translation'][2]:.4f}",
                        grasp['object_id'],
                        f"{rot_matrix[0]:.6f}", f"{rot_matrix[1]:.6f}", f"{rot_matrix[2]:.6f}",
                        f"{rot_matrix[3]:.6f}", f"{rot_matrix[4]:.6f}", f"{rot_matrix[5]:.6f}",
                        f"{rot_matrix[6]:.6f}", f"{rot_matrix[7]:.6f}", f"{rot_matrix[8]:.6f}",
                        f"{dist_to_center:.4f}"
                    ]
                    writer.writerow(row)
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'✓ 抓取数据已保存到（绝对路径）:')
            self.get_logger().info(f'  {csv_file}')
            self.get_logger().info(f'  总数: {len(grasps_data)} 个抓取')
            self.get_logger().info('=' * 60)
            
            # 同时保存一个简化版本（更易读），固定文件名覆盖
            summary_file = os.path.join(save_dir_abs, 'grasps_summary.csv')
            with open(summary_file, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 简化表头（英文+中文，仅原始数据）
                header = [
                    'Index(索引)', 'Score(分数)', 'Width(m)(宽度)', 'Height(m)(高度)', 'Depth(m)(深度)',
                    'X(m)', 'Y(m)', 'Z(m)',
                    'Dist_Center(m)(到中心距离)'
                ]
                writer.writerow(header)
                
                # 写入原始抓取数据
                for grasp in grasps_data:
                    dist_to_center = np.sqrt(grasp['translation'][0]**2 + grasp['translation'][1]**2)
                    row = [
                        grasp['index'],
                        f"{grasp['score']:.4f}",
                        f"{grasp['width']:.4f}",
                        f"{grasp['height']:.4f}",
                        f"{grasp['depth']:.4f}",
                        f"{grasp['translation'][0]:.4f}",
                        f"{grasp['translation'][1]:.4f}",
                        f"{grasp['translation'][2]:.4f}",
                        f"{dist_to_center:.4f}"
                    ]
                    writer.writerow(row)
            
            self.get_logger().info(f'✓ 简化版数据已保存到:')
            self.get_logger().info(f'  {summary_file}')
            
        except Exception as e:
            self.get_logger().error(f'保存抓取数据失败: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def build_grasp_to_end_effector_transform(self):
        """
        构建从 gripper_tip（抓取位姿）到 end_effector（wrist3/tool_tcp）的局部变换矩阵。

        变换在末端（抓取）坐标系的 z 轴上应用：
        - 抓取位姿的 Z 轴 = approach 方向（手指伸出方向）
        - gripper_tip 在 wrist3 前方 grasp_z_offset 处
        - 沿抓取 Z 轴反向平移 grasp_z_offset，得到 wrist3 的目标位姿

        Returns:
            4x4 齐次变换矩阵（在抓取局部坐标系下：沿 z 轴平移 -grasp_z_offset）
        """
        # 局部坐标系下的平移：沿 z 轴反向移动 offset（gripper_tip -> wrist3）
        T_local = np.eye(4)
        T_local[2, 3] = -self.grasp_z_offset  # z 轴负方向

        self.get_logger().info(f'构建抓取局部变换: 沿末端 z 轴平移 -{self.grasp_z_offset:.3f} m')
        return T_local

    def apply_transformation_to_pose(self, pose, transform_local):
        """
        将局部变换矩阵应用到位姿上。

        将 gripper_tip 的抓取位姿变换为 end_effector 的目标位姿：
        T_base_target = T_base_grasp @ T_grasp_local
        其中 T_grasp_local 为沿抓取 z 轴的反向平移（含 gripper_tip 补偿）

        Args:
            pose: geometry_msgs.msg.Pose，gripper_tip 的抓取位姿（base_link 下）
            transform_local: 4x4 齐次变换矩阵，抓取局部坐标系下的平移

        Returns:
            变换后的 Pose，end_effector 目标位姿
        """
        # 构建 base -> grasp 的齐次变换
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R_grasp = R.from_quat(quat).as_matrix()
        T_base_grasp = np.eye(4)
        T_base_grasp[:3, :3] = R_grasp
        T_base_grasp[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]

        # 应用局部变换：沿抓取 z 轴平移
        T_base_target = T_base_grasp @ transform_local

        transformed_pose = Pose()
        transformed_pose.position.x = float(T_base_target[0, 3])
        transformed_pose.position.y = float(T_base_target[1, 3])
        transformed_pose.position.z = float(T_base_target[2, 3])

        quat_target = R.from_matrix(T_base_target[:3, :3]).as_quat()
        transformed_pose.orientation.x = float(quat_target[0])
        transformed_pose.orientation.y = float(quat_target[1])
        transformed_pose.orientation.z = float(quat_target[2])
        transformed_pose.orientation.w = float(quat_target[3])

        return transformed_pose
    
    def call_publish_service(self):
        """调用发布服务"""
        request = Trigger.Request()
        
        self.get_logger().info('发送发布请求...')
        future = self.publish_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 发布成功')
                
                # 尝试解析返回的抓取数据
                try:
                    data = json.loads(response.message)
                    if isinstance(data, dict) and 'grasps' in data:
                        self.get_logger().info(f"收到 {data['num_grasps']} 个抓取位姿数据")
                        
                        # 按 approach 竖直夹角与绕 z 旋转过滤
                        raw_grasps = data['grasps']
                        self.grasp_data = self.filter_grasps_by_approach_and_yaw(raw_grasps)
                        
                        # 保存过滤后的抓取数据到 CSV 文件
                        self.save_grasps_to_csv(self.grasp_data)
                        
                        return True
                except json.JSONDecodeError:
                    # 如果不是 JSON 格式，就是普通消息
                    self.get_logger().info(f"消息: {response.message}")
                    return True
                
                return True
            else:
                self.get_logger().error(f'✗ 发布失败: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'发布服务调用异常: {str(e)}')
            return False
    
    def get_grasp_pose(self):
        """从 TF 树中获取抓取位姿"""
        self.get_logger().info(f'等待 TF 变换: {self.base_frame} -> {self.grasp_frame}...')
        
        # 等待 TF 变换可用（最多等待 5 秒）
        max_attempts = 50
        for attempt in range(max_attempts):
            try:
                # 尝试获取变换
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.grasp_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # 转换为 Pose 消息
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                
                self.get_logger().info(f'✓ 获取到抓取位姿:')
                self.get_logger().info(f'  位置 (position): x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}')
                self.get_logger().info(f'  姿态 (orientation): x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}')
                return pose
                
            except Exception as e:
                if attempt < max_attempts - 1:
                    time.sleep(0.1)
                else:
                    self.get_logger().error(f'✗ 无法获取 TF 变换: {str(e)}')
                    return None
        
        return None
    
    def call_move_service(self, target_pose):
        """调用运动控制服务"""
        request = MoveToPose.Request()
        request.target_pose = target_pose
        request.use_joints = self.use_joints
        request.velocity_factor = self.velocity_factor
        request.acceleration_factor = self.acceleration_factor
        
        self.get_logger().info(f'发送运动控制请求...')
        self.get_logger().info(f'  目标位姿:')
        self.get_logger().info(f'    位置: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
        self.get_logger().info(f'    姿态: x={target_pose.orientation.x:.3f}, y={target_pose.orientation.y:.3f}, z={target_pose.orientation.z:.3f}, w={target_pose.orientation.w:.3f}')
        self.get_logger().info(f'  速度因子: {self.velocity_factor}, 加速度因子: {self.acceleration_factor}')
        self.get_logger().info(f'  使用关节空间规划: {self.use_joints}')
        
        future = self.move_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 运动控制成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'✗ 运动控制失败 (错误码 {response.error_code}): {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'运动控制服务调用异常: {str(e)}')
            return False
    
    def run(self):
        """执行完整流程：发布抓取 -> 保存数据 -> 获取位姿 -> 控制运动"""
        # 步骤 1: 调用发布服务（保存抓取数据到 CSV）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 1: 调用发布服务')
        self.get_logger().info('=' * 60)
        if not self.call_publish_service():
            return False
        
        # 检查是否有抓取数据
        if not self.grasp_data:
            self.get_logger().error('没有抓取数据')
            return False
        
        # 等待一段时间让 TF 发布
        time.sleep(0.5)
        
        # 步骤 2: 从 TF 获取抓取位姿（使用 grasp_pose_0，即最佳抓取）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 2: 从 TF 获取抓取位姿 (grasp_pose_0)')
        self.get_logger().info('=' * 60)
        grasp_pose = self.get_grasp_pose()
        if grasp_pose is None:
            return False
        
        # 步骤 2.1: 构建变换矩阵（沿末端 z 轴平移，含 gripper_tip 补偿）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 2.1: 构建变换矩阵')
        self.get_logger().info('=' * 60)
        transform_local = self.build_grasp_to_end_effector_transform()
        
        # 步骤 2.2: 应用变换矩阵到抓取位姿（gripper_tip -> end_effector）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 2.2: 应用变换矩阵到抓取位姿')
        self.get_logger().info('=' * 60)
        self.get_logger().info('原始抓取位姿 (gripper_tip):')
        self.get_logger().info(f'  位置: x={grasp_pose.position.x:.3f}, y={grasp_pose.position.y:.3f}, z={grasp_pose.position.z:.3f}')
        self.get_logger().info(f'  姿态: x={grasp_pose.orientation.x:.3f}, y={grasp_pose.orientation.y:.3f}, z={grasp_pose.orientation.z:.3f}, w={grasp_pose.orientation.w:.3f}')
        
        transformed_pose = self.apply_transformation_to_pose(grasp_pose, transform_local)
        
        self.get_logger().info('变换后的目标位姿 (end_effector，沿末端 z 轴补偿):')
        self.get_logger().info(f'  位置: x={transformed_pose.position.x:.3f}, y={transformed_pose.position.y:.3f}, z={transformed_pose.position.z:.3f}')
        self.get_logger().info(f'  姿态: x={transformed_pose.orientation.x:.3f}, y={transformed_pose.orientation.y:.3f}, z={transformed_pose.orientation.z:.3f}, w={transformed_pose.orientation.w:.3f}')
        
        # 步骤 3: 调用运动控制服务
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 3: 控制机械臂运动')
        self.get_logger().info('=' * 60)
        if not self.call_move_service(transformed_pose):
            return False

        self.get_logger().info('=' * 60)
        self.get_logger().info('✓ 所有步骤完成')
        self.get_logger().info('=' * 60)
        return True


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        client = PublishGraspsClient()
        success = client.run()
        
        client.destroy_node()
        rclpy.shutdown()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print('\n用户中断')
        sys.exit(1)
    except Exception as e:
        print(f'错误: {str(e)}')
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
