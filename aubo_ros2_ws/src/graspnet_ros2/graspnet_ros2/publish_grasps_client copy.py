#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 发布客户端：手动触发抓取结果发布并控制机械臂运动

功能：
  - 调用 /publish_grasps 服务发布抓取位姿、MarkerArray 和点云
  - 从 TF 树中获取所有抓取位姿（grasp_pose_0, grasp_pose_1, ...），挑选尽量垂直的抓取
  - 调用 /move_to_pose 服务控制机械臂运动到该抓取位姿

与 graspnet_demo_points_node 配合时：该节点最多发布 5 个抓取（max_grasps_num=5），
TF 父坐标系为 frame_id（默认 camera_frame），子坐标系为 grasp_pose_0..grasp_pose_4。

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


class PublishGraspsClient(Node):
    """抓取发布客户端节点"""
    
    def __init__(self):
        super().__init__('publish_grasps_client')
        
        # 声明参数
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('grasp_frame', 'grasp_pose_0')
        self.declare_parameter('max_grasp_candidates', 5)   # 与 graspnet_demo_points_node 的 max_grasps_num 一致（最多 5 个）
        self.declare_parameter('prefer_vertical', True)
        self.declare_parameter('velocity_factor', 0.3)
        self.declare_parameter('acceleration_factor', 0.3)
        self.declare_parameter('use_joints', True)
        self.declare_parameter('grasp_z_offset', 0.185+0.02)  # gripper_tip_link 相对 wrist3_Link 的 z 偏移补偿（沿末端z轴）
        
        # 获取参数
        self.base_frame = self.get_parameter('base_frame').value
        self.grasp_frame = self.get_parameter('grasp_frame').value
        self.max_grasp_candidates = self.get_parameter('max_grasp_candidates').value
        self.prefer_vertical = self.get_parameter('prefer_vertical').value
        self.velocity_factor = self.get_parameter('velocity_factor').value
        self.acceleration_factor = self.get_parameter('acceleration_factor').value
        self.use_joints = self.get_parameter('use_joints').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value
        
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
                self.get_logger().info(f'✓ 发布成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'✗ 发布失败: {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'发布服务调用异常: {str(e)}')
            return False
    
    def get_grasp_pose(self, grasp_frame=None):
        """从 TF 树中获取指定抓取帧的位姿。grasp_frame 为 None 时使用 self.grasp_frame。"""
        frame = grasp_frame if grasp_frame is not None else self.grasp_frame
        self.get_logger().info(f'等待 TF 变换: {self.base_frame} -> {frame}...')
        
        # 等待 TF 变换可用（最多等待 5 秒）
        max_attempts = 50
        for attempt in range(max_attempts):
            try:
                # 尝试获取变换
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # 转换为 Pose 消息
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                
                self.get_logger().info(f'✓ 获取到抓取位姿 ({frame}):')
                self.get_logger().info(f'  位置 (position): x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}')
                self.get_logger().info(f'  姿态 (orientation): x={pose.orientation.x:.3f}, y={pose.orientation.y:.3f}, z={pose.orientation.z:.3f}, w={pose.orientation.w:.3f}')
                return pose
                
            except Exception as e:
                if attempt < max_attempts - 1:
                    time.sleep(0.1)
                else:
                    self.get_logger().error(f'✗ 无法获取 TF 变换 {self.base_frame} -> {frame}: {str(e)}')
                    return None
        
        return None

    def get_all_grasp_poses(self):
        """
        从 TF 树中获取所有可用的抓取位姿 (grasp_pose_0, grasp_pose_1, ...)。
        与 graspnet_demo_points_node 一致：该节点最多发布 max_grasps_num=5 个抓取。
        Returns:
            list of (frame_id: str, pose: Pose)，按帧序号排序。
        """
        results = []
        for i in range(self.max_grasp_candidates):
            frame_id = f'grasp_pose_{i}'
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                )
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                results.append((frame_id, pose))
            except Exception:
                break
        return results

    def _verticality_score(self, pose):
        """
        计算抓取位姿的“垂直度”得分：抓取坐标系 Z 轴（approach 方向）与世界 -Z 的对齐程度。
        垂直自上而下抓取时 approach 沿 -Z，得分为 1；水平抓取得分接近 0。
        Returns:
            float in [0, 1]，越大越垂直（自上而下）。
        """
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R_base_grasp = R.from_quat(quat).as_matrix()
        z_axis = R_base_grasp[:3, 2]  # 抓取坐标系 z 轴在 base 下的方向
        world_down = np.array([0.0, 0.0, -1.0])
        # 点积：1 表示完全沿 -Z（垂直向下），-1 表示完全沿 +Z（垂直向上），0 表示水平
        alignment = float(np.dot(z_axis, world_down))
        # 取绝对值并映射到 [0,1]，使“尽量垂直”即得分高（向下或向上都算垂直）
        return abs(alignment)

    def select_most_vertical_grasp(self, frame_pose_list):
        """
        从 (frame_id, pose) 列表中挑选垂直度得分最高的抓取。
        Returns:
            (frame_id: str, pose: Pose) 或 None（列表为空时）。
        """
        if not frame_pose_list:
            return None
        best = max(
            frame_pose_list,
            key=lambda item: self._verticality_score(item[1])
        )
        score = self._verticality_score(best[1])
        self.get_logger().info(f'从 {len(frame_pose_list)} 个抓取中挑选最垂直: {best[0]}, 垂直度得分={score:.3f}')
        return best
    
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
        
        # 等待一段时间让 TF 发布
        time.sleep(0.5)
        
        # 步骤 2: 从 TF 获取抓取位姿（获取所有后挑选尽量垂直的）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 2: 从 TF 获取抓取位姿（获取所有，挑选尽量垂直）')
        self.get_logger().info('=' * 60)
        if self.prefer_vertical:
            all_grasps = self.get_all_grasp_poses()
            if not all_grasps:
                self.get_logger().error('未找到任何抓取 TF (grasp_pose_0, grasp_pose_1, ...)')
                return False
            self.get_logger().info(f'共获取 {len(all_grasps)} 个抓取位姿: {[f for f, _ in all_grasps]}')
            selected = self.select_most_vertical_grasp(all_grasps)
            if selected is None:
                return False
            grasp_frame_id, grasp_pose = selected
            self.get_logger().info(f'选用抓取: {grasp_frame_id}')
        else:
            grasp_pose = self.get_grasp_pose()
            if grasp_pose is None:
                return False
            grasp_frame_id = self.grasp_frame
        
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
