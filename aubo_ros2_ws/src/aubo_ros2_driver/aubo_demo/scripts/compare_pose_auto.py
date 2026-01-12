#!/usr/bin/env python3
"""
自动位姿对比脚本
自动使用 MoveIt2 控制机械臂移动到目标位姿，然后对比实际到达的位姿与目标位姿
"""

import rclpy
from rclpy.node import Node
from demo_interface.msg import RobotStatus
from demo_interface.srv import MoveToPose
from geometry_msgs.msg import Pose

import math
import time
from typing import Optional, Tuple


class AutoPoseComparator(Node):
    """自动位姿对比器节点"""
    
    def __init__(self):
        super().__init__('auto_pose_comparator')
        
        # 参数
        self.declare_parameter('position_tolerance', 0.001)  # 米
        self.declare_parameter('orientation_tolerance', 0.01)  # 弧度
        self.declare_parameter('velocity_factor', 0.3)
        self.declare_parameter('acceleration_factor', 0.3)
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.velocity_factor = self.get_parameter('velocity_factor').value
        self.acceleration_factor = self.get_parameter('acceleration_factor').value
        
        # 订阅机器人状态
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        
        # 创建服务客户端
        self.move_to_pose_client = self.create_client(MoveToPose, '/move_to_pose')
        
        # 当前机器人状态
        self.current_robot_status: Optional[RobotStatus] = None
        self.status_received = False
        
        self.get_logger().info('自动位姿对比器节点已启动')
        self.get_logger().info(f'位置容差: {self.position_tolerance} m')
        self.get_logger().info(f'姿态容差: {self.orientation_tolerance} rad')
        self.get_logger().info(f'速度因子: {self.velocity_factor}')
        self.get_logger().info(f'加速度因子: {self.acceleration_factor}')
        
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
    
    def wait_for_service(self, timeout: float = 5.0) -> bool:
        """等待服务可用"""
        self.get_logger().info('等待 /move_to_pose 服务...')
        if not self.move_to_pose_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('✗ 服务 /move_to_pose 不可用')
            return False
        self.get_logger().info('✓ 服务 /move_to_pose 已就绪')
        return True
    
    def get_current_pose_from_status(self) -> Optional[Pose]:
        """从机器人状态获取当前位姿"""
        if self.current_robot_status is None:
            self.get_logger().error('机器人状态未接收')
            return None
        
        return self.current_robot_status.cartesian_position
    
    def move_to_pose(self, target_pose: Pose, use_joints: bool = False) -> bool:
        """移动到目标位姿"""
        self.get_logger().info(f'发送移动请求到目标位姿...')
        self.get_logger().info(f'  位置: x={target_pose.position.x:.4f}, '
                              f'y={target_pose.position.y:.4f}, '
                              f'z={target_pose.position.z:.4f}')
        
        request = MoveToPose.Request()
        request.target_pose = target_pose
        request.use_joints = use_joints
        request.velocity_factor = self.velocity_factor
        request.acceleration_factor = self.acceleration_factor
        
        future = self.move_to_pose_client.call_async(request)
        
        # 等待服务响应
        timeout = 60.0  # 60秒超时
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('✗ 服务调用超时')
            return False
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 移动成功: {response.message}')
                return True
            else:
                self.get_logger().error(f'✗ 移动失败 (错误码 {response.error_code}): {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'✗ 服务调用异常: {str(e)}')
            return False
    
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
            self.get_logger().warn('未检测到机械臂运动，可能已在目标位置')
        
        # 等待停止运动
        stable_count = 0
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_robot_status and not self.current_robot_status.in_motion:
                stable_count += 1
                if stable_count >= 5:  # 连续5次确认静止
                    # 额外等待一段时间确保完全稳定
                    time.sleep(0.5)
                    self.get_logger().info('✓ 机械臂已到位并稳定')
                    return True
            else:
                stable_count = 0
        
        self.get_logger().error('✗ 等待机械臂到位超时')
        return False
    
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
        
        return pos_ok and ori_ok


def create_test_pose() -> Pose:
    """创建一个测试位姿"""
    pose = Pose()
    # 这是一个示例位姿，根据实际情况修改
    pose.position.x = -0.074
    pose.position.y = -0.209
    pose.position.z = 0.953
    pose.orientation.w = 0.7026
    pose.orientation.x = -0.0001
    pose.orientation.y = -0.0008
    pose.orientation.z = 0.7116
    return pose


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    comparator = AutoPoseComparator()
    
    try:
        print("\n" + "="*70)
        print("  🤖 自动位姿对比工具")
        print("="*70)
        print("\n此工具将自动：")
        print("  1. 使用 MoveIt2 控制机械臂移动到目标位姿")
        print("  2. 等待机械臂到位")
        print("  3. 从 /demo_robot_status 获取实际到达的位姿")
        print("  4. 对比目标位姿与实际位姿的差异")
        print("\n" + "="*70)
        
        # 等待接收机器人状态
        if not comparator.wait_for_robot_status(timeout=10.0):
            comparator.get_logger().error('无法接收机器人状态，退出')
            return
        
        # 等待服务可用
        if not comparator.wait_for_service(timeout=10.0):
            comparator.get_logger().error('无法连接到 /move_to_pose 服务，退出')
            return
        
        # 获取并显示当前位姿
        rclpy.spin_once(comparator, timeout_sec=0.5)
        current_pose = comparator.get_current_pose_from_status()
        if current_pose:
            comparator.print_pose(current_pose, "当前位姿 (移动前)")
        
        # 创建目标位姿
        print("\n" + "="*70)
        choice = input("\n是否使用默认测试位姿? (y/n): ")
        
        if choice.lower() == 'y':
            target_pose = create_test_pose()
        else:
            # 手动输入目标位姿
            print("\n请输入目标位姿:")
            target_pose = Pose()
            target_pose.position.x = float(input("  位置 x (米): "))
            target_pose.position.y = float(input("  位置 y (米): "))
            target_pose.position.z = float(input("  位置 z (米): "))
            print("\n四元数姿态:")
            target_pose.orientation.w = float(input("  w: "))
            target_pose.orientation.x = float(input("  x: "))
            target_pose.orientation.y = float(input("  y: "))
            target_pose.orientation.z = float(input("  z: "))
        
        # 显示目标位姿
        comparator.print_pose(target_pose, "目标位姿 (期望到达)")
        
        # 询问是否使用关节空间规划
        use_joints_choice = input("\n使用关节空间规划? (y/n, 默认为笛卡尔空间): ")
        use_joints = use_joints_choice.lower() == 'y'
        
        # 确认执行
        input("\n按 Enter 开始移动机械臂...")
        
        # 移动到目标位姿
        if not comparator.move_to_pose(target_pose, use_joints=use_joints):
            comparator.get_logger().error('移动失败，退出')
            return
        
        # 等待机械臂到位
        if not comparator.wait_for_motion_complete(timeout=60.0):
            comparator.get_logger().error('等待机械臂到位超时')
            return
        
        # 获取实际到达的位姿
        time.sleep(0.5)  # 额外稳定时间
        rclpy.spin_once(comparator, timeout_sec=0.5)
        actual_pose = comparator.get_current_pose_from_status()
        
        if actual_pose is None:
            comparator.get_logger().error('无法获取实际位姿')
            return
        
        # 显示实际位姿
        comparator.print_pose(actual_pose, "实际位姿 (从 /demo_robot_status)")
        
        # 计算并显示位姿差异
        diff = comparator.compute_pose_difference(target_pose, actual_pose)
        match = comparator.print_pose_difference(diff)
        
        # 保存结果到文件
        save_choice = input("\n是否保存对比结果到文件? (y/n): ")
        if save_choice.lower() == 'y':
            import json
            from datetime import datetime
            
            result = {
                'timestamp': datetime.now().isoformat(),
                'target_pose': {
                    'position': {'x': target_pose.position.x, 'y': target_pose.position.y, 'z': target_pose.position.z},
                    'orientation': {'w': target_pose.orientation.w, 'x': target_pose.orientation.x, 
                                   'y': target_pose.orientation.y, 'z': target_pose.orientation.z}
                },
                'actual_pose': {
                    'position': {'x': actual_pose.position.x, 'y': actual_pose.position.y, 'z': actual_pose.position.z},
                    'orientation': {'w': actual_pose.orientation.w, 'x': actual_pose.orientation.x,
                                   'y': actual_pose.orientation.y, 'z': actual_pose.orientation.z}
                },
                'difference': {
                    'position_distance_mm': diff['position']['distance'] * 1000,
                    'orientation_angle_deg': math.degrees(diff['quaternion_angle']),
                    'match': match
                }
            }
            
            filename = f"pose_comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(result, f, indent=2)
            comparator.get_logger().info(f'✓ 结果已保存到: {filename}')
        
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
