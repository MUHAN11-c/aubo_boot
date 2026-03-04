#!/usr/bin/env python3
"""
运动学一致性验证脚本
用于检查 MoveIt 和机器人驱动的运动学计算是否一致
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionFK
import numpy as np
import time
import sys
import json
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class KinematicsVerifier(Node):
    def __init__(self):
        super().__init__('kinematics_verifier')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 订阅关节状态
        self.joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # MoveIt FK 服务客户端 (ROS2)
        self.fk_client = self.create_client(
            GetPositionFK, 
            '/compute_fk',
            callback_group=self.callback_group
        )
        
        self.get_logger().info('等待 MoveIt FK 服务...')
        self.fk_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✓ MoveIt FK 服务已连接')
        
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('运动学验证脚本已启动')
        self.get_logger().info('='*80)

    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.joint_state = msg

    def get_current_joints(self, timeout=5.0):
        """获取当前关节角度"""
        self.get_logger().info('等待关节状态数据...')
        start_time = time.time()
        while self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('超时：未接收到关节状态')
                return None
        
        # 提取前6个关节
        joint_names = [
            'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
            'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
        ]
        
        joints = []
        for name in joint_names:
            try:
                idx = self.joint_state.name.index(name)
                joints.append(self.joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f'找不到关节: {name}')
                return None
        
        return joints

    def compute_fk_moveit(self, joint_positions):
        """使用 MoveIt 计算正向运动学"""
        request = GetPositionFK.Request()
        
        # 设置 header
        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()
        
        # 设置要计算的链接
        request.fk_link_names = ['wrist3_Link']
        
        # 设置关节状态
        request.robot_state.joint_state.name = [
            'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
            'wrist1_joint', 'wrist2_joint', 'wrist3_joint'
        ]
        request.robot_state.joint_state.position = joint_positions
        
        # 调用服务（增加超时时间）
        future = self.fk_client.call_async(request)
        
        # 手动等待，增加超时时间到15秒
        timeout = 15.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done():
            try:
                response = future.result()
                if response.error_code.val == 1:  # SUCCESS
                    return response.pose_stamped[0].pose
                else:
                    self.get_logger().error(f'MoveIt FK 失败: error_code={response.error_code.val}')
                    return None
            except Exception as e:
                self.get_logger().error(f'MoveIt FK 服务响应异常: {e}')
                return None
        else:
            self.get_logger().error(f'MoveIt FK 服务调用超时 (>{timeout}秒)')
            return None

    def print_pose(self, pose, prefix=''):
        """打印位姿信息"""
        self.get_logger().info(f'{prefix}位置: ({pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}) m')
        self.get_logger().info(f'{prefix}四元数: (w={pose.orientation.w:.6f}, x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, z={pose.orientation.z:.6f})')
        
        # 转换为 RPY
        import math
        q = pose.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(f'{prefix}欧拉角 (RPY): ({roll:.6f}, {pitch:.6f}, {yaw:.6f}) rad = ({math.degrees(roll):.2f}°, {math.degrees(pitch):.2f}°, {math.degrees(yaw):.2f}°)')

    def compare_poses(self, pose1, pose2, label1='位姿1', label2='位姿2'):
        """对比两个位姿"""
        self.get_logger().info('\n' + '-'*80)
        self.get_logger().info(f'对比 {label1} vs {label2}')
        self.get_logger().info('-'*80)
        
        dx = (pose2.position.x - pose1.position.x) * 1000  # 转换为 mm
        dy = (pose2.position.y - pose1.position.y) * 1000
        dz = (pose2.position.z - pose1.position.z) * 1000
        
        self.get_logger().info(f'位置差异:')
        self.get_logger().info(f'  ΔX = {dx:+.2f} mm')
        self.get_logger().info(f'  ΔY = {dy:+.2f} mm')
        self.get_logger().info(f'  ΔZ = {dz:+.2f} mm')
        self.get_logger().info(f'  3D距离 = {np.sqrt(dx**2 + dy**2 + dz**2):.2f} mm')
        
        # 四元数差异
        dw = pose2.orientation.w - pose1.orientation.w
        dx_q = pose2.orientation.x - pose1.orientation.x
        dy_q = pose2.orientation.y - pose1.orientation.y
        dz_q = pose2.orientation.z - pose1.orientation.z
        
        self.get_logger().info(f'四元数差异:')
        self.get_logger().info(f'  Δw = {dw:+.6f}')
        self.get_logger().info(f'  Δx = {dx_q:+.6f}')
        self.get_logger().info(f'  Δy = {dy_q:+.6f}')
        self.get_logger().info(f'  Δz = {dz_q:+.6f}')

    def verify_from_json(self, json_file):
        """从 JSON 文件读取位姿并验证"""
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info(f'从 JSON 文件验证: {json_file}')
        self.get_logger().info('='*80)
        
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            # 提取关节角度（支持多种格式）
            if 'joint_position_rad' in data:
                joints = data['joint_position_rad']
            elif 'joint_positions' in data:
                joints = data['joint_positions']
            else:
                self.get_logger().error('JSON 文件中未找到关节角度数据')
                return
            
            self.get_logger().info('\nJSON 文件中的关节角度:')
            for i, j in enumerate(joints):
                self.get_logger().info(f'  关节{i}: {j:.6f} rad ({np.degrees(j):.2f}°)')
            
            # 提取位姿（机器人驱动报告的位姿）
            driver_pose = Pose()
            if 'cartesian_position' in data:
                # 新格式
                driver_pose.position.x = data['cartesian_position']['position']['x']
                driver_pose.position.y = data['cartesian_position']['position']['y']
                driver_pose.position.z = data['cartesian_position']['position']['z']
                driver_pose.orientation.w = data['cartesian_position']['orientation']['w']
                driver_pose.orientation.x = data['cartesian_position']['orientation']['x']
                driver_pose.orientation.y = data['cartesian_position']['orientation']['y']
                driver_pose.orientation.z = data['cartesian_position']['orientation']['z']
            else:
                # 旧格式
                driver_pose.position.x = data['position']['x']
                driver_pose.position.y = data['position']['y']
                driver_pose.position.z = data['position']['z']
                driver_pose.orientation.w = data['orientation']['w']
                driver_pose.orientation.x = data['orientation']['x']
                driver_pose.orientation.y = data['orientation']['y']
                driver_pose.orientation.z = data['orientation']['z']
            
            self.get_logger().info('\n【机器人驱动报告的位姿】')
            self.print_pose(driver_pose, '  ')
            
            # 使用相同的关节角度计算 MoveIt FK
            self.get_logger().info('\n计算 MoveIt FK...')
            moveit_pose = self.compute_fk_moveit(joints)
            
            if moveit_pose is None:
                self.get_logger().error('MoveIt FK 计算失败')
                return
            
            self.get_logger().info('\n【MoveIt 计算的位姿】')
            self.print_pose(moveit_pose, '  ')
            
            # 对比
            self.compare_poses(driver_pose, moveit_pose, '机器人驱动', 'MoveIt')
            
            # 判断结论
            dx = abs(moveit_pose.position.x - driver_pose.position.x) * 1000
            dy = abs(moveit_pose.position.y - driver_pose.position.y) * 1000
            dz = abs(moveit_pose.position.z - driver_pose.position.z) * 1000
            
            self.get_logger().info('\n' + '='*80)
            if dx < 1.0 and dy < 1.0 and dz < 1.0:
                self.get_logger().info('✓ 结论: 运动学模型一致 (误差 < 1mm)')
            elif dx < 5.0 and dy < 5.0 and dz < 5.0:
                self.get_logger().warn('⚠ 结论: 运动学模型有小偏差 (误差 < 5mm)')
                self.get_logger().warn('  建议: 可能需要校准 URDF 参数')
            else:
                self.get_logger().error('✗ 结论: 运动学模型存在显著差异 (误差 > 5mm)')
                self.get_logger().error('  原因: URDF DH 参数与实际机器人不匹配')
                self.get_logger().error('  建议: 需要重新校准或从厂家获取准确的 URDF')
            self.get_logger().info('='*80)
            
        except Exception as e:
            self.get_logger().error(f'验证失败: {e}')
            import traceback
            traceback.print_exc()

    def verify_current_state(self):
        """验证当前机器人状态"""
        self.get_logger().info('\n' + '='*80)
        self.get_logger().info('验证当前机器人状态')
        self.get_logger().info('='*80)
        
        # 获取当前关节角度
        joints = self.get_current_joints()
        if joints is None:
            return
        
        self.get_logger().info('\n当前关节角度:')
        for i, j in enumerate(joints):
            self.get_logger().info(f'  关节{i}: {j:.6f} rad ({np.degrees(j):.2f}°)')
        
        # 使用 MoveIt 计算 FK
        self.get_logger().info('\n计算 MoveIt FK...')
        moveit_pose = self.compute_fk_moveit(joints)
        
        if moveit_pose is None:
            self.get_logger().error('MoveIt FK 计算失败')
            return
        
        self.get_logger().info('\n【MoveIt 计算的 wrist3_Link 位姿】')
        self.print_pose(moveit_pose, '  ')
        
        self.get_logger().info('\n提示: 如果要对比机器人驱动的位姿，请提供 camera_pose.json 文件')


def main(args=None):
    rclpy.init(args=args)
    
    verifier = KinematicsVerifier()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(verifier)
    
    # 启动一个线程来处理回调
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # 等待一下，确保收到初始的关节状态
    time.sleep(2.0)
    
    try:
        if len(sys.argv) > 1:
            # 如果提供了 JSON 文件路径，从文件验证
            json_file = sys.argv[1]
            verifier.verify_from_json(json_file)
        else:
            # 否则验证当前状态
            verifier.verify_current_state()
            
            verifier.get_logger().info('\n' + '='*80)
            verifier.get_logger().info('使用方法:')
            verifier.get_logger().info('  验证当前状态: ros2 run demo_driver verify_kinematics.py')
            verifier.get_logger().info('  验证 JSON 文件: ros2 run demo_driver verify_kinematics.py /path/to/camera_pose.json')
            verifier.get_logger().info('='*80)
    
    except KeyboardInterrupt:
        pass
    finally:
        verifier.get_logger().info('\n脚本退出')
        verifier.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
