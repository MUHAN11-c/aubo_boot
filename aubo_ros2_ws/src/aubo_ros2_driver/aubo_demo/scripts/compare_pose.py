#!/usr/bin/env python3
"""
位姿对比脚本
对比从 /demo_robot_status 主题获取的位姿和 MoveIt2 控制机械臂到位后的位姿
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from demo_interface.msg import RobotStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

import numpy as np
import time
import math
from typing import Optional, Tuple


class PoseComparator(Node):
    """位姿对比器节点"""
    
    def __init__(self):
        super().__init__('pose_comparator')
        
        # 参数
        self.declare_parameter('planning_group', 'manipulator_e5')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('position_tolerance', 0.001)  # 米
        self.declare_parameter('orientation_tolerance', 0.01)  # 弧度
        
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        
        # 订阅机器人状态
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        
        # 当前机器人状态
        self.current_robot_status: Optional[RobotStatus] = None
        self.status_received = False
        
        self.get_logger().info('位姿对比器节点已启动')
        self.get_logger().info(f'规划组: {self.planning_group}')
        self.get_logger().info(f'基础坐标系: {self.base_frame}')
        self.get_logger().info(f'位置容差: {self.position_tolerance} m')
        self.get_logger().info(f'姿态容差: {self.orientation_tolerance} rad')
        
    def robot_status_callback(self, msg: RobotStatus):
        """机器人状态回调函数"""
        self.current_robot_status = msg
        if not self.status_received:
            self.status_received = True
            self.get_logger().info('已接收到机器人状态数据')
    
    def wait_for_robot_status(self, timeout: float = 5.0) -> bool:
        """等待接收机器人状态"""
        self.get_logger().info('等待接收机器人状态...')
        start_time = time.time()
        
        while not self.status_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.status_received:
            self.get_logger().info('✓ 已接收到机器人状态')
            return True
        else:
            self.get_logger().error('✗ 等待机器人状态超时')
            return False
    
    def get_current_pose_from_status(self) -> Optional[Pose]:
        """从机器人状态获取当前位姿"""
        if self.current_robot_status is None:
            self.get_logger().error('机器人状态未接收')
            return None
        
        return self.current_robot_status.cartesian_position
    
    def quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        """四元数转欧拉角 (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def compute_pose_difference(self, pose1: Pose, pose2: Pose) -> dict:
        """计算两个位姿之间的差异"""
        # 位置差异
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        position_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 姿态差异 (欧拉角)
        roll1, pitch1, yaw1 = self.quaternion_to_euler(
            pose1.orientation.x, pose1.orientation.y,
            pose1.orientation.z, pose1.orientation.w
        )
        roll2, pitch2, yaw2 = self.quaternion_to_euler(
            pose2.orientation.x, pose2.orientation.y,
            pose2.orientation.z, pose2.orientation.w
        )
        
        droll = roll2 - roll1
        dpitch = pitch2 - pitch1
        dyaw = yaw2 - yaw1
        
        # 归一化角度到 [-π, π]
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle
        
        droll = normalize_angle(droll)
        dpitch = normalize_angle(dpitch)
        dyaw = normalize_angle(dyaw)
        
        orientation_distance = math.sqrt(droll**2 + dpitch**2 + dyaw**2)
        
        # 四元数差异
        quat_dot = (pose1.orientation.w * pose2.orientation.w +
                    pose1.orientation.x * pose2.orientation.x +
                    pose1.orientation.y * pose2.orientation.y +
                    pose1.orientation.z * pose2.orientation.z)
        
        # 夹角 (弧度)
        quaternion_angle = 2 * math.acos(min(abs(quat_dot), 1.0))
        
        return {
            'position': {
                'dx': dx,
                'dy': dy,
                'dz': dz,
                'distance': position_distance
            },
            'orientation_euler': {
                'droll': droll,
                'dpitch': dpitch,
                'dyaw': dyaw,
                'distance': orientation_distance
            },
            'quaternion_angle': quaternion_angle
        }
    
    def print_pose(self, pose: Pose, title: str = "位姿"):
        """打印位姿信息"""
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"  {title}")
        self.get_logger().info(f"{'='*70}")
        
        self.get_logger().info(f"\n📍 位置 (米):")
        self.get_logger().info(f"    x: {pose.position.x:>10.6f}")
        self.get_logger().info(f"    y: {pose.position.y:>10.6f}")
        self.get_logger().info(f"    z: {pose.position.z:>10.6f}")
        
        self.get_logger().info(f"\n🔄 姿态 (四元数):")
        self.get_logger().info(f"    w: {pose.orientation.w:>10.6f}")
        self.get_logger().info(f"    x: {pose.orientation.x:>10.6f}")
        self.get_logger().info(f"    y: {pose.orientation.y:>10.6f}")
        self.get_logger().info(f"    z: {pose.orientation.z:>10.6f}")
        
        # 欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        )
        self.get_logger().info(f"\n🔄 姿态 (欧拉角，弧度):")
        self.get_logger().info(f"    Roll:  {roll:>10.6f} rad ({math.degrees(roll):>8.2f}°)")
        self.get_logger().info(f"    Pitch: {pitch:>10.6f} rad ({math.degrees(pitch):>8.2f}°)")
        self.get_logger().info(f"    Yaw:   {yaw:>10.6f} rad ({math.degrees(yaw):>8.2f}°)")
    
    def print_pose_difference(self, diff: dict):
        """打印位姿差异"""
        self.get_logger().info(f"\n{'='*70}")
        self.get_logger().info(f"  📊 位姿差异分析")
        self.get_logger().info(f"{'='*70}")
        
        pos_diff = diff['position']
        self.get_logger().info(f"\n📍 位置差异 (米):")
        self.get_logger().info(f"    Δx: {pos_diff['dx']:>10.6f} m")
        self.get_logger().info(f"    Δy: {pos_diff['dy']:>10.6f} m")
        self.get_logger().info(f"    Δz: {pos_diff['dz']:>10.6f} m")
        self.get_logger().info(f"    总距离: {pos_diff['distance']:>10.6f} m ({pos_diff['distance']*1000:.3f} mm)")
        
        ori_diff = diff['orientation_euler']
        self.get_logger().info(f"\n🔄 姿态差异 (欧拉角):")
        self.get_logger().info(f"    ΔRoll:  {ori_diff['droll']:>10.6f} rad ({math.degrees(ori_diff['droll']):>8.3f}°)")
        self.get_logger().info(f"    ΔPitch: {ori_diff['dpitch']:>10.6f} rad ({math.degrees(ori_diff['dpitch']):>8.3f}°)")
        self.get_logger().info(f"    ΔYaw:   {ori_diff['dyaw']:>10.6f} rad ({math.degrees(ori_diff['dyaw']):>8.3f}°)")
        self.get_logger().info(f"    总距离: {ori_diff['distance']:>10.6f} rad ({math.degrees(ori_diff['distance']):>8.3f}°)")
        
        self.get_logger().info(f"\n🔄 四元数旋转角度:")
        self.get_logger().info(f"    角度: {diff['quaternion_angle']:>10.6f} rad ({math.degrees(diff['quaternion_angle']):>8.3f}°)")
        
        # 判断是否在容差范围内
        pos_ok = pos_diff['distance'] <= self.position_tolerance
        ori_ok = diff['quaternion_angle'] <= self.orientation_tolerance
        
        self.get_logger().info(f"\n{'='*70}")
        if pos_ok and ori_ok:
            self.get_logger().info("✅ 位姿匹配：在容差范围内")
        else:
            self.get_logger().info("⚠️  位姿不匹配：超出容差范围")
            if not pos_ok:
                self.get_logger().info(f"   位置差异 {pos_diff['distance']*1000:.3f} mm > {self.position_tolerance*1000:.3f} mm")
            if not ori_ok:
                self.get_logger().info(f"   姿态差异 {math.degrees(diff['quaternion_angle']):.3f}° > {math.degrees(self.orientation_tolerance):.3f}°")
        self.get_logger().info(f"{'='*70}\n")
    
    def wait_for_motion_complete(self, timeout: float = 30.0) -> bool:
        """等待机械臂运动完成"""
        self.get_logger().info('等待机械臂运动完成...')
        start_time = time.time()
        
        # 等待开始运动
        motion_started = False
        while not motion_started and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_robot_status and self.current_robot_status.in_motion:
                motion_started = True
                self.get_logger().info('机械臂开始运动')
                break
        
        if not motion_started:
            self.get_logger().warn('未检测到机械臂运动，继续等待静止状态')
        
        # 等待停止运动
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_robot_status and not self.current_robot_status.in_motion:
                # 额外等待一段时间确保完全稳定
                time.sleep(0.5)
                self.get_logger().info('✓ 机械臂已到位')
                return True
        
        self.get_logger().error('✗ 等待机械臂到位超时')
        return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    comparator = PoseComparator()
    
    try:
        # 等待接收机器人状态
        if not comparator.wait_for_robot_status(timeout=10.0):
            comparator.get_logger().error('无法接收机器人状态，退出')
            return
        
        print("\n" + "="*70)
        print("  🤖 位姿对比工具")
        print("="*70)
        print("\n此工具将对比以下两个位姿：")
        print("  1. 从 /demo_robot_status 主题获取的当前位姿")
        print("  2. MoveIt2 控制机械臂到达目标位置后的位姿")
        print("\n使用方法：")
        print("  1. 确保机器人已连接并上电")
        print("  2. 确保 MoveIt2 已启动")
        print("  3. 使用 MoveIt2 或其他方式移动机械臂到目标位置")
        print("  4. 运行此脚本进行位姿对比")
        print("\n" + "="*70)
        
        # 获取移动前的位姿
        input("\n📍 准备记录【初始位姿】，按 Enter 继续...")
        rclpy.spin_once(comparator, timeout_sec=0.5)
        
        pose_before = comparator.get_current_pose_from_status()
        if pose_before is None:
            comparator.get_logger().error('无法获取初始位姿')
            return
        
        comparator.print_pose(pose_before, "初始位姿 (从 /demo_robot_status)")
        
        # 提示用户移动机械臂
        print("\n" + "="*70)
        input("\n🎯 请使用 MoveIt2 或其他方式移动机械臂，完成后按 Enter 继续...")
        
        # 等待机械臂运动完成
        comparator.wait_for_motion_complete(timeout=60.0)
        
        # 获取移动后的位姿
        input("\n📍 准备记录【目标位姿】，按 Enter 继续...")
        rclpy.spin_once(comparator, timeout_sec=0.5)
        
        pose_after = comparator.get_current_pose_from_status()
        if pose_after is None:
            comparator.get_logger().error('无法获取目标位姿')
            return
        
        comparator.print_pose(pose_after, "目标位姿 (从 /demo_robot_status)")
        
        # 计算并显示位姿差异
        diff = comparator.compute_pose_difference(pose_before, pose_after)
        comparator.print_pose_difference(diff)
        
        # 可选：对比当前两个位姿
        print("\n" + "="*70)
        choice = input("\n🔄 是否对比当前位姿与目标位姿? (y/n): ")
        if choice.lower() == 'y':
            rclpy.spin_once(comparator, timeout_sec=0.5)
            pose_current = comparator.get_current_pose_from_status()
            if pose_current is not None:
                comparator.print_pose(pose_current, "当前位姿 (从 /demo_robot_status)")
                diff2 = comparator.compute_pose_difference(pose_current, pose_after)
                comparator.print_pose_difference(diff2)
        
        comparator.get_logger().info('\n✓ 位姿对比完成')
        
    except KeyboardInterrupt:
        comparator.get_logger().info('用户中断')
    except Exception as e:
        comparator.get_logger().error(f'发生错误: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        comparator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
