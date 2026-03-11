#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 发布客户端：使用抓取结果并控制机械臂运动

功能：
  - 订阅 graspnet_demo_points_node 发布的抓取位姿话题（PoseArray，base_link 下），挑选尽量垂直的抓取
  - 通过 MoveIt2 笛卡尔路径（grasp_motion_controller）执行：XY → 姿态旋转 → Z 垂直抓取（一条轨迹一次执行）

与 graspnet_demo_points_node 配合时：该节点循环发布 PoseArray 到 grasp_poses_base（默认），
本客户端订阅该话题，等待非空位姿列表后选最垂直、做 gripper_tip 补偿并执行运动。抓取位姿全部来自话题，不再从 TF 查询。

使用方法：
  ros2 run graspnet_ros2 publish_grasps_client

前置条件：
  1. 已启动 graspnet_demo_points_node（含 move_group 的 launch，如 graspnet_demo_points.launch.py）
  2. 节点已收到点云并开始循环发布抓取 TF
  3. 本客户端与 move_group 需 source 同一工作空间（aubo_ros2_ws/install/setup.bash）
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray
from tf2_ros import Buffer, TransformListener
import sys
import time
import threading
from typing import Optional
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R

from graspnet_ros2.grasp_motion_controller import GraspMotionController


class PublishGraspsClient(Node):
    """抓取发布客户端节点"""
    
    def __init__(self):
        super().__init__('publish_grasps_client')
        
        # 声明参数
        self.declare_parameter('prefer_vertical', True)
        self.declare_parameter('grasp_z_offset', 0.15)  # gripper_tip_link 相对 wrist3_Link 的 z 偏移补偿（沿末端z轴）
        self.declare_parameter('height_above', 0.05)  # 抓取点上方安全高度 (m)，用于笛卡尔路径
        self.declare_parameter('joint_velocity_scaling', 0.15)  # 关节空间回退速度缩放（0~1）
        self.declare_parameter('joint_acceleration_scaling', 0.1)  # 关节空间回退加速度缩放（0~1）
        
        # 获取参数
        self.prefer_vertical = self.get_parameter('prefer_vertical').value
        self.grasp_z_offset = self.get_parameter('grasp_z_offset').value
        self.height_above = self.get_parameter('height_above').value
        self.joint_velocity_scaling = float(self.get_parameter('joint_velocity_scaling').value)
        self.joint_acceleration_scaling = float(self.get_parameter('joint_acceleration_scaling').value)

        # 抓取位姿话题（与 graspnet_demo_points_node 的 grasp_poses_topic 一致）
        self.declare_parameter('grasp_poses_topic', 'grasp_poses_base')
        self.grasp_poses_topic = self.get_parameter('grasp_poses_topic').value

        # 等待话题非空的最长时间（秒）
        self.declare_parameter('wait_poses_timeout_sec', 30.0)
        self.wait_poses_timeout_sec = self.get_parameter('wait_poses_timeout_sec').value
        # 选择策略：缓存最近 N 组抓取位姿，至少攒够 M 组后再选最优
        self.declare_parameter('grasp_window_size', 5)
        self.declare_parameter('min_groups_before_pick', 3)
        self.grasp_window_size = int(self.get_parameter('grasp_window_size').value)
        self.min_groups_before_pick = int(self.get_parameter('min_groups_before_pick').value)

        # 订阅抓取位姿话题（base_link 下 PoseArray）
        self._latest_grasp_poses: Optional[PoseArray] = None
        self._grasp_groups_window = deque(maxlen=max(1, self.grasp_window_size))
        self.create_subscription(PoseArray, self.grasp_poses_topic, self._grasp_poses_callback, 10)

        # TF 监听器（仅供 run_grasp_approach 获取当前末端位姿 base_link -> tool_tcp）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.motion_controller = GraspMotionController(self)

        self.get_logger().info(f'订阅抓取位姿话题: {self.grasp_poses_topic}（由 graspnet_demo_points_node 发布）')

    def _grasp_poses_callback(self, msg: PoseArray):
        """缓存最新抓取位姿（base_link 下）。"""
        if len(msg.poses) == 0:
            return
        self._latest_grasp_poses = msg
        self._grasp_groups_window.append(msg)

    def wait_for_grasp_window_ready(self) -> bool:
        """等待缓存窗口达到最少组数。"""
        deadline = time.time() + self.wait_poses_timeout_sec
        while time.time() < deadline:
            if len(self._grasp_groups_window) >= self.min_groups_before_pick:
                self.get_logger().info(
                    f'抓取窗口已就绪: {len(self._grasp_groups_window)}/{self.grasp_window_size} 组'
                )
                return True
            time.sleep(0.1)
        self.get_logger().error(
            f'等待抓取窗口超时 ({self.wait_poses_timeout_sec} s), '
            f'当前仅 {len(self._grasp_groups_window)} 组'
        )
        return False

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

        quat_target = R.from_matrix(T_base_target[:3, :3]).as_quat(canonical=False)
        transformed_pose.orientation.x = float(quat_target[0])
        transformed_pose.orientation.y = float(quat_target[1])
        transformed_pose.orientation.z = float(quat_target[2])
        transformed_pose.orientation.w = float(quat_target[3])

        return transformed_pose

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

    def select_best_from_window(self):
        """
        从最近 N 组抓取位姿中挑选垂直度最高的抓取。
        Returns:
            (tag: str, pose: Pose) 或 None
        """
        best = None
        best_score = -1.0
        for group_idx, group in enumerate(self._grasp_groups_window):
            for pose_idx, pose in enumerate(group.poses):
                score = self._verticality_score(pose)
                if score > best_score:
                    best_score = score
                    best = (f'group{group_idx}_grasp{pose_idx}', pose)
        if best is not None:
            self.get_logger().info(
                f'从最近 {len(self._grasp_groups_window)} 组中挑选最垂直: {best[0]}, 垂直度得分={best_score:.3f}'
            )
        return best

    def run_grasp_motion(self, target_pose: Pose) -> bool:
        """通过 MoveIt2 笛卡尔路径执行抓取接近（XY → 姿态旋转 → Z），一条轨迹一次执行。"""
        return self.motion_controller.run_grasp_approach(
            target_pose,
            height_above=self.height_above,
            velocity_scaling=self.joint_velocity_scaling,
            acceleration_scaling=self.joint_acceleration_scaling,
        )
    
    def run(self):
        """执行完整流程：等待抓取窗口 → 选最垂直 → 控制运动"""
        # 步骤 1: 等待抓取窗口就绪（最近多组）
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 1: 等待抓取位姿窗口')
        self.get_logger().info('=' * 60)
        if not self.wait_for_grasp_window_ready():
            return False

        # 步骤 2: 从窗口中挑选尽量垂直的抓取
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 2: 从抓取窗口中挑选尽量垂直')
        self.get_logger().info('=' * 60)
        if self.prefer_vertical:
            selected = self.select_best_from_window()
            if selected is None:
                return False
            grasp_frame_id, grasp_pose = selected
            self.get_logger().info(f'选用抓取: {grasp_frame_id}')
        else:
            if self._latest_grasp_poses is None or len(self._latest_grasp_poses.poses) == 0:
                self.get_logger().error('最新抓取位姿为空')
                return False
            grasp_frame_id, grasp_pose = ('latest_grasp_0', self._latest_grasp_poses.poses[0])
            self.get_logger().info(f'选用抓取: {grasp_frame_id}（第一个）')
        
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
        
        # 步骤 3: 通过 MoveIt2 笛卡尔路径执行抓取接近
        self.get_logger().info('=' * 60)
        self.get_logger().info('步骤 3: 控制机械臂运动 (MoveIt2 笛卡尔路径)')
        self.get_logger().info('=' * 60)
        if not self.run_grasp_motion(transformed_pose):
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
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(client)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        success = client.run()
        executor.shutdown()
        spin_thread.join(timeout=1.0)
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
