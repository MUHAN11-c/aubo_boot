#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GraspNet 抓取执行客户端：订阅抓取位姿 → 选优 → TCP 补偿 → MoveIt 接近.

订阅 graspnet_demo_points_node 发布的 PoseArray（base 系），在最近
grasp_window_size 组中选垂直度最高的抓取，沿抓取 Z 轴做 gripper_tip→末端
补偿（-grasp_z_offset）后调 run_grasp_approach 执行。

用法（需 move_group 已由外部启动）：
  ros2 run graspnet_ros2 publish_grasps_client
"""

from __future__ import annotations

from collections import deque
import sys
import threading
import time
from typing import Optional

from geometry_msgs.msg import Pose, PoseArray
from graspnet_ros2.motion_controller import GraspMotionController
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R


class PublishGraspsClient(GraspMotionController):
    """抓取位姿订阅 + 选优 + 运动执行."""

    def __init__(self):
        super().__init__('publish_grasps_client')

        self.declare_parameter('prefer_vertical', True)
        self.declare_parameter('grasp_z_offset', 0.15)  # gripper_tip→末端沿抓取 Z 的补偿（米）
        self.declare_parameter('height_above', 0.05)
        self.declare_parameter('joint_velocity_scaling', 0.15)
        self.declare_parameter('joint_acceleration_scaling', 0.1)
        self.declare_parameter('grasp_poses_topic', 'grasp_poses_base')
        self.declare_parameter('wait_poses_timeout_sec', 30.0)
        self.declare_parameter('grasp_window_size', 5)
        self.declare_parameter('min_groups_before_pick', 3)

        self.prefer_vertical = bool(self.get_parameter('prefer_vertical').value)
        self.grasp_z_offset = float(self.get_parameter('grasp_z_offset').value)
        self.height_above = float(self.get_parameter('height_above').value)
        self.joint_velocity_scaling = float(
            self.get_parameter('joint_velocity_scaling').value
        )
        self.joint_acceleration_scaling = float(
            self.get_parameter('joint_acceleration_scaling').value
        )
        self.grasp_poses_topic = self.get_parameter('grasp_poses_topic').value
        self.wait_poses_timeout_sec = float(
            self.get_parameter('wait_poses_timeout_sec').value
        )
        self.grasp_window_size = max(1, int(self.get_parameter('grasp_window_size').value))
        self.min_groups_before_pick = int(
            self.get_parameter('min_groups_before_pick').value
        )

        self._latest_grasp_poses: Optional[PoseArray] = None
        self._grasp_groups_window: deque = deque(maxlen=self.grasp_window_size)
        self.create_subscription(PoseArray, self.grasp_poses_topic, self._grasp_poses_callback, 10)
        self.get_logger().info(f'订阅抓取位姿话题: {self.grasp_poses_topic}')

    def _grasp_poses_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return
        self._latest_grasp_poses = msg
        self._grasp_groups_window.append(msg)

    def wait_for_grasp_window_ready(self) -> bool:
        deadline = time.time() + self.wait_poses_timeout_sec
        while time.time() < deadline:
            if len(self._grasp_groups_window) >= self.min_groups_before_pick:
                self.get_logger().info(
                    f'抓取窗口已就绪: {len(self._grasp_groups_window)}/{self.grasp_window_size} 组'
                )
                return True
            time.sleep(0.1)
        self.get_logger().error(
            f'等待抓取窗口超时 ({self.wait_poses_timeout_sec} s)，'
            f'当前仅 {len(self._grasp_groups_window)} 组'
        )
        return False

    def _verticality_score(self, pose: Pose) -> float:
        """抓取 Z（approach）与世界 -Z 的对齐度，越接近垂直越大."""
        quat = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]
        rot = R.from_quat(quat).as_matrix()
        return abs(float(np.dot(rot[:3, 2], np.array([0.0, 0.0, -1.0]))))

    def select_best_from_window(self):
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
                f'从最近 {len(self._grasp_groups_window)} 组中选最垂直: '
                f'{best[0]}, 得分={best_score:.3f}'
            )
        return best

    def apply_tcp_offset(self, pose: Pose) -> Pose:
        """gripper_tip 抓取位姿 → 末端目标位姿：沿抓取 Z 反向平移 grasp_z_offset."""
        quat = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]
        t_base_grasp = np.eye(4)
        t_base_grasp[:3, :3] = R.from_quat(quat).as_matrix()
        t_base_grasp[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        t_local = np.eye(4)
        t_local[2, 3] = -self.grasp_z_offset
        t_base_target = t_base_grasp @ t_local

        transformed = Pose()
        transformed.position.x = float(t_base_target[0, 3])
        transformed.position.y = float(t_base_target[1, 3])
        transformed.position.z = float(t_base_target[2, 3])
        quat_target = R.from_matrix(t_base_target[:3, :3]).as_quat(canonical=False)
        transformed.orientation.x = float(quat_target[0])
        transformed.orientation.y = float(quat_target[1])
        transformed.orientation.z = float(quat_target[2])
        transformed.orientation.w = float(quat_target[3])
        return transformed

    def run(self) -> bool:
        """等待窗口 → 选优 → TCP 补偿 → 抓取接近运动."""
        self.get_logger().info('步骤 1: 等待抓取位姿窗口')
        if not self.wait_for_grasp_window_ready():
            return False

        self.get_logger().info('步骤 2: 选择抓取')
        if self.prefer_vertical:
            selected = self.select_best_from_window()
            if selected is None:
                return False
            grasp_tag, grasp_pose = selected
        else:
            if self._latest_grasp_poses is None or len(self._latest_grasp_poses.poses) == 0:
                self.get_logger().error('最新抓取位姿为空')
                return False
            grasp_tag, grasp_pose = 'latest_grasp_0', self._latest_grasp_poses.poses[0]
        self.get_logger().info(f'选用抓取: {grasp_tag}')

        self.get_logger().info(
            f'步骤 3: TCP 补偿（沿抓取 Z 平移 -{self.grasp_z_offset:.3f} m）'
        )
        target_pose = self.apply_tcp_offset(grasp_pose)
        self.get_logger().info(
            f'末端目标位姿: pos=({target_pose.position.x:.3f}, '
            f'{target_pose.position.y:.3f}, {target_pose.position.z:.3f})'
        )

        self.get_logger().info('步骤 4: MoveIt 笛卡尔抓取接近')
        if not self.run_grasp_approach(
            target_pose,
            height_above=self.height_above,
            velocity_scaling=self.joint_velocity_scaling,
            acceleration_scaling=self.joint_acceleration_scaling,
        ):
            return False

        self.get_logger().info('全部步骤完成')
        return True


def main(args=None):
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
        print(f'错误: {e}')
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
