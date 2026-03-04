#!/usr/bin/env python3
"""
MoveIt2 位姿验证脚本

从 JSON 文件读取位姿，验证：
1. MoveIt2 机械臂运动到位的判断方法
2. MoveIt2 计算位姿与 /demo_robot_status 获取的位姿对比
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from demo_interface.msg import RobotStatus
from demo_interface.srv import MoveToPose
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes

import json
import math
import time
from datetime import datetime
from typing import List, Dict, Optional, Tuple


class MoveItPoseValidator(Node):
    """MoveIt2 位姿验证器"""
    
    def __init__(self):
        super().__init__('moveit_pose_validator')
        
        # 参数
        self.declare_parameter('json_file', '')
        self.declare_parameter('position_tolerance', 0.001)  # 1mm
        self.declare_parameter('orientation_tolerance', 0.01)  # ~0.57°
        self.declare_parameter('velocity_factor', 0.3)
        self.declare_parameter('acceleration_factor', 0.3)
        self.declare_parameter('motion_detection_timeout', 5.0)  # 检测运动开始的超时
        self.declare_parameter('motion_stable_count', 10)  # 静止确认次数
        
        self.json_file = self.get_parameter('json_file').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.velocity_factor = self.get_parameter('velocity_factor').value
        self.acceleration_factor = self.get_parameter('acceleration_factor').value
        self.motion_detection_timeout = self.get_parameter('motion_detection_timeout').value
        self.motion_stable_count = self.get_parameter('motion_stable_count').value
        
        # 订阅机器人状态
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        
        # 订阅 MoveIt2 TCP 位姿（基于TF树计算的FK位姿）
        self.moveit2_tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/moveit2_tcp_pose',
            self.moveit2_tcp_pose_callback,
            10
        )
        
        # 创建服务客户端
        self.move_to_pose_client = self.create_client(MoveToPose, '/move_to_pose')
        
        # 状态变量
        self.current_robot_status: Optional[RobotStatus] = None
        self.current_moveit2_tcp_pose: Optional[Pose] = None
        self.status_received = False
        self.moveit2_pose_received = False
        self.previous_in_motion = False
        self.motion_history = []  # 运动状态历史
        
        # 统计数据
        self.test_results = []
        
        self.get_logger().info('='*70)
        self.get_logger().info('MoveIt2 位姿验证器已启动')
        self.get_logger().info('='*70)
        self.get_logger().info(f'位置容差: {self.position_tolerance*1000:.3f} mm')
        self.get_logger().info(f'姿态容差: {math.degrees(self.orientation_tolerance):.3f}°')
        self.get_logger().info(f'速度因子: {self.velocity_factor}')
        self.get_logger().info(f'运动检测超时: {self.motion_detection_timeout} s')
        self.get_logger().info(f'静止确认次数: {self.motion_stable_count}')
        self.get_logger().info('='*70)
    
    def robot_status_callback(self, msg: RobotStatus):
        """机器人状态回调"""
        self.current_robot_status = msg
        if not self.status_received:
            self.status_received = True
            self.get_logger().info('✓ 已接收到机器人状态数据')
        
        # 记录运动状态变化
        if msg.in_motion != self.previous_in_motion:
            timestamp = time.time()
            if msg.in_motion:
                self.get_logger().info(f'[{timestamp:.3f}] 🏃 机械臂开始运动')
            else:
                self.get_logger().info(f'[{timestamp:.3f}] ⏸️  机械臂停止运动')
            self.previous_in_motion = msg.in_motion
        
        # 记录运动历史（用于分析）
        self.motion_history.append({
            'timestamp': time.time(),
            'in_motion': msg.in_motion,
            'planning_status': msg.planning_status
        })
        # 只保留最近100条记录
        if len(self.motion_history) > 100:
            self.motion_history.pop(0)
    
    def moveit2_tcp_pose_callback(self, msg: PoseStamped):
        """MoveIt2 TCP 位姿回调（基于TF树的FK计算）"""
        self.current_moveit2_tcp_pose = msg.pose
        if not self.moveit2_pose_received:
            self.moveit2_pose_received = True
            self.get_logger().info('✓ 已接收到 MoveIt2 TCP 位姿数据')
    
    def wait_for_robot_status(self, timeout: float = 10.0) -> bool:
        """等待接收机器人状态"""
        self.get_logger().info('等待接收机器人状态...')
        start_time = time.time()
        
        while not self.status_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.status_received
    
    def wait_for_service(self, timeout: float = 10.0) -> bool:
        """等待服务可用"""
        self.get_logger().info('等待 /move_to_pose 服务...')
        if not self.move_to_pose_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('✗ 服务 /move_to_pose 不可用')
            return False
        self.get_logger().info('✓ 服务 /move_to_pose 已就绪')
        return True
    
    def get_current_pose_from_status(self) -> Optional[Pose]:
        """从机器人状态获取当前位姿（机器人控制器报告）"""
        if self.current_robot_status is None:
            return None
        return self.current_robot_status.cartesian_position
    
    def get_moveit2_tcp_pose(self) -> Optional[Pose]:
        """获取 MoveIt2 TCP 位姿（基于TF树的FK计算）"""
        return self.current_moveit2_tcp_pose
    
    def move_to_pose(self, target_pose: Pose, use_joints: bool = False) -> Tuple[bool, str, float]:
        """
        移动到目标位姿
        
        返回：(成功, 消息, 执行时间)
        """
        request = MoveToPose.Request()
        request.target_pose = target_pose
        request.use_joints = use_joints
        request.velocity_factor = self.velocity_factor
        request.acceleration_factor = self.acceleration_factor
        
        start_time = time.time()
        future = self.move_to_pose_client.call_async(request)
        
        # 等待服务响应
        timeout = 60.0
        call_start = time.time()
        while not future.done() and (time.time() - call_start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        execution_time = time.time() - start_time
        
        if not future.done():
            return False, '服务调用超时', execution_time
        
        try:
            response = future.result()
            return response.success, response.message, execution_time
        except Exception as e:
            return False, f'服务调用异常: {str(e)}', execution_time
    
    def wait_for_motion_start(self, timeout: float = None) -> Tuple[bool, float]:
        """
        等待机械臂开始运动
        
        返回：(检测到运动, 等待时间)
        """
        if timeout is None:
            timeout = self.motion_detection_timeout
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.current_robot_status and self.current_robot_status.in_motion:
                wait_time = time.time() - start_time
                self.get_logger().info(f'✓ 检测到运动开始 (耗时: {wait_time:.3f}s)')
                return True, wait_time
        
        wait_time = time.time() - start_time
        self.get_logger().warn(f'⚠️  未检测到运动 (超时: {wait_time:.3f}s)')
        return False, wait_time
    
    def wait_for_motion_complete(self, timeout: float = 60.0) -> Tuple[bool, float, Dict]:
        """
        等待机械臂运动完成
        
        返回：(成功, 等待时间, 详细信息)
        """
        start_time = time.time()
        
        # 阶段1: 等待开始运动
        motion_started, start_wait = self.wait_for_motion_start()
        
        if not motion_started:
            # 可能已经在目标位置，检查是否静止
            stable_count = 0
            check_start = time.time()
            while (time.time() - check_start) < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_robot_status and not self.current_robot_status.in_motion:
                    stable_count += 1
                    if stable_count >= 5:
                        total_time = time.time() - start_time
                        return True, total_time, {
                            'motion_started': False,
                            'already_at_target': True,
                            'start_wait_time': start_wait,
                            'stop_wait_time': 0,
                            'stable_checks': stable_count
                        }
                else:
                    stable_count = 0
        
        # 阶段2: 等待停止运动
        stop_wait_start = time.time()
        stable_count = 0
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            if self.current_robot_status:
                if not self.current_robot_status.in_motion:
                    stable_count += 1
                    if stable_count >= self.motion_stable_count:
                        # 额外稳定延时
                        time.sleep(0.3)
                        stop_wait_time = time.time() - stop_wait_start
                        total_time = time.time() - start_time
                        
                        self.get_logger().info(f'✓ 机械臂已到位并稳定')
                        self.get_logger().info(f'  - 开始运动等待: {start_wait:.3f}s')
                        self.get_logger().info(f'  - 停止运动等待: {stop_wait_time:.3f}s')
                        self.get_logger().info(f'  - 总等待时间: {total_time:.3f}s')
                        self.get_logger().info(f'  - 静止确认次数: {stable_count}')
                        
                        return True, total_time, {
                            'motion_started': motion_started,
                            'already_at_target': False,
                            'start_wait_time': start_wait,
                            'stop_wait_time': stop_wait_time,
                            'stable_checks': stable_count
                        }
                else:
                    stable_count = 0
        
        total_time = time.time() - start_time
        self.get_logger().error(f'✗ 等待机械臂到位超时 (超时: {total_time:.3f}s)')
        return False, total_time, {
            'motion_started': motion_started,
            'timeout': True,
            'start_wait_time': start_wait
        }
    
    def analyze_motion_history(self) -> Dict:
        """分析运动历史记录"""
        if len(self.motion_history) < 2:
            self.get_logger().warn('运动历史记录不足，无法分析')
            return {
                'motion_detected': False,
                'reason': '运动历史记录不足'
            }
        
        # 查找运动开始和结束
        motion_started = False
        motion_ended = False
        start_time = None
        end_time = None
        
        for i in range(1, len(self.motion_history)):
            prev = self.motion_history[i-1]
            curr = self.motion_history[i]
            
            # 检测运动开始（从 false -> true）
            if not prev['in_motion'] and curr['in_motion'] and not motion_started:
                motion_started = True
                start_time = curr['timestamp']
                self.get_logger().info(f'✓ 检测到运动开始 (时间戳: {start_time:.3f})')
            
            # 检测运动结束（从 true -> false）
            if prev['in_motion'] and not curr['in_motion'] and motion_started and not motion_ended:
                motion_ended = True
                end_time = curr['timestamp']
                self.get_logger().info(f'✓ 检测到运动结束 (时间戳: {end_time:.3f})')
        
        if motion_started and motion_ended:
            motion_time = end_time - start_time
            self.get_logger().info(f'✓ 机械臂运动时间: {motion_time:.3f}s')
            return {
                'motion_detected': True,
                'motion_started': True,
                'motion_ended': True,
                'start_time': start_time,
                'end_time': end_time,
                'motion_duration': motion_time
            }
        elif motion_started:
            self.get_logger().warn('⚠️  检测到运动开始但未检测到结束')
            return {
                'motion_detected': True,
                'motion_started': True,
                'motion_ended': False,
                'start_time': start_time
            }
        else:
            self.get_logger().warn('⚠️  未检测到运动（可能已在目标位置）')
            return {
                'motion_detected': False,
                'reason': '未检测到运动状态变化'
            }
    
    def compute_pose_difference(self, pose1: Pose, pose2: Pose) -> Dict:
        """计算两个位姿之间的差异"""
        # 位置差异
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        position_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # 四元数差异
        quat_dot = (pose1.orientation.w * pose2.orientation.w +
                    pose1.orientation.x * pose2.orientation.x +
                    pose1.orientation.y * pose2.orientation.y +
                    pose1.orientation.z * pose2.orientation.z)
        quaternion_angle = 2 * math.acos(min(abs(quat_dot), 1.0))
        
        return {
            'position_distance': position_distance,
            'orientation_angle': quaternion_angle,
            'dx': dx, 'dy': dy, 'dz': dz
        }
    
    def print_pose(self, pose: Pose, title: str):
        """打印位姿"""
        self.get_logger().info(f'\n  {title}:')
        self.get_logger().info(f'    位置: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}')
        self.get_logger().info(f'    姿态: w={pose.orientation.w:.6f}, x={pose.orientation.x:.6f}, '
                              f'y={pose.orientation.y:.6f}, z={pose.orientation.z:.6f}')
    
    def validate_single_pose(self, pose_data: Dict, pose_index: int) -> Dict:
        """
        验证单个位姿
        
        验证内容：
        1. 调用 MoveIt2 移动到目标位姿
        2. 监测运动状态变化（运动开始、运动结束）
        3. 对比 MoveIt2 目标位姿与实际到达位姿
        """
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info(f'  测试位姿 #{pose_index}')
        self.get_logger().info('='*70)
        
        # 构建目标位姿
        target_pose = Pose()
        target_pose.position.x = pose_data['position']['x']
        target_pose.position.y = pose_data['position']['y']
        target_pose.position.z = pose_data['position']['z']
        target_pose.orientation.x = pose_data['orientation']['x']
        target_pose.orientation.y = pose_data['orientation']['y']
        target_pose.orientation.z = pose_data['orientation']['z']
        target_pose.orientation.w = pose_data['orientation']['w']
        
        result = {
            'pose_index': pose_index,
            'target_pose': pose_data,
            'timestamp': datetime.now().isoformat()
        }
        
        # 显示目标位姿
        self.print_pose(target_pose, '目标位姿')
        
        # 记录移动前位姿
        before_pose = self.get_current_pose_from_status()
        if before_pose:
            self.print_pose(before_pose, '移动前位姿')
        
        # 清空运动历史
        self.motion_history = []
        
        # === 验证1: 调用 MoveIt2 移动并同时监测运动状态 ===
        self.get_logger().info('\n📍 验证1: 调用 MoveIt2 移动到目标位姿并监测运动...')
        
        # 注意：move_to_pose 内部会调用 spin_once，已经在监测运动状态
        # 从日志可以看到运动状态变化已经被记录
        move_success, move_message, move_time = self.move_to_pose(target_pose)
        result['moveit_call'] = {
            'success': move_success,
            'message': move_message,
            'execution_time': move_time
        }
        
        if not move_success:
            self.get_logger().error(f'✗ MoveIt2 移动失败: {move_message}')
            result['success'] = False
            result['error'] = f'MoveIt2 移动失败: {move_message}'
            return result
        
        self.get_logger().info(f'✓ MoveIt2 返回成功: {move_message}')
        
        # 分析运动历史记录
        self.get_logger().info('\n🔍 验证1: 分析运动状态记录...')
        
        motion_details = self.analyze_motion_history()
        result['motion_detection'] = motion_details
        
        # === 验证2: 对比 MoveIt2 目标位姿与实际位姿 ===
        self.get_logger().info('\n📊 验证2: 对比目标位姿与实际位姿...')
        
        # 额外等待确保位姿稳定
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # 获取机器人控制器报告的位姿
        actual_pose = self.get_current_pose_from_status()
        if actual_pose is None:
            self.get_logger().error('✗ 无法获取机器人控制器位姿')
            result['success'] = False
            result['error'] = '无法获取机器人控制器位姿'
            return result
        
        self.print_pose(actual_pose, '实际到达位姿 (机器人控制器 /demo_robot_status)')
        
        # 获取 MoveIt2 计算的TCP位姿（基于TF树）
        moveit2_pose = self.get_moveit2_tcp_pose()
        if moveit2_pose is not None:
            self.print_pose(moveit2_pose, 'MoveIt2 计算位姿 (基于TF树 /moveit2_tcp_pose)')
        else:
            self.get_logger().warn('⚠️  无法获取 MoveIt2 TCP 位姿数据')
        
        # 计算位姿差异（目标 vs 机器人控制器）
        diff = self.compute_pose_difference(target_pose, actual_pose)
        
        result['actual_pose'] = {
            'position': {
                'x': actual_pose.position.x,
                'y': actual_pose.position.y,
                'z': actual_pose.position.z
            },
            'orientation': {
                'x': actual_pose.orientation.x,
                'y': actual_pose.orientation.y,
                'z': actual_pose.orientation.z,
                'w': actual_pose.orientation.w
            }
        }
        
        result['pose_difference'] = {
            'position_distance_mm': diff['position_distance'] * 1000,
            'position_distance_m': diff['position_distance'],
            'orientation_angle_deg': math.degrees(diff['orientation_angle']),
            'orientation_angle_rad': diff['orientation_angle'],
            'dx': diff['dx'],
            'dy': diff['dy'],
            'dz': diff['dz']
        }
        
        # 如果有 MoveIt2 位姿数据，计算并记录相关差异
        if moveit2_pose is not None:
            # 计算 MoveIt2 位姿与目标位姿的差异
            diff_moveit2 = self.compute_pose_difference(target_pose, moveit2_pose)
            
            # 计算 MoveIt2 位姿与机器人控制器位姿的差异
            diff_sources = self.compute_pose_difference(moveit2_pose, actual_pose)
            
            result['moveit2_tcp_pose'] = {
                'position': {
                    'x': moveit2_pose.position.x,
                    'y': moveit2_pose.position.y,
                    'z': moveit2_pose.position.z
                },
                'orientation': {
                    'x': moveit2_pose.orientation.x,
                    'y': moveit2_pose.orientation.y,
                    'z': moveit2_pose.orientation.z,
                    'w': moveit2_pose.orientation.w
                }
            }
            
            result['moveit2_vs_target'] = {
                'position_distance_mm': diff_moveit2['position_distance'] * 1000,
                'orientation_angle_deg': math.degrees(diff_moveit2['orientation_angle']),
                'dx': diff_moveit2['dx'],
                'dy': diff_moveit2['dy'],
                'dz': diff_moveit2['dz']
            }
            
            result['moveit2_vs_controller'] = {
                'position_distance_mm': diff_sources['position_distance'] * 1000,
                'orientation_angle_deg': math.degrees(diff_sources['orientation_angle']),
                'dx': diff_sources['dx'],
                'dy': diff_sources['dy'],
                'dz': diff_sources['dz']
            }
        
        # 显示差异
        self.get_logger().info(f'\n  📏 位姿差异分析:')
        self.get_logger().info(f'\n  1️⃣ 目标 vs 机器人控制器:')
        self.get_logger().info(f'    位置差异: {diff["position_distance"]*1000:.3f} mm')
        self.get_logger().info(f'      Δx = {diff["dx"]*1000:.3f} mm')
        self.get_logger().info(f'      Δy = {diff["dy"]*1000:.3f} mm')
        self.get_logger().info(f'      Δz = {diff["dz"]*1000:.3f} mm')
        self.get_logger().info(f'    姿态差异: {math.degrees(diff["orientation_angle"]):.3f}°')
        
        if moveit2_pose is not None:
            self.get_logger().info(f'\n  2️⃣ 目标 vs MoveIt2(TF):')
            self.get_logger().info(f'    位置差异: {diff_moveit2["position_distance"]*1000:.3f} mm')
            self.get_logger().info(f'      Δx = {diff_moveit2["dx"]*1000:.3f} mm')
            self.get_logger().info(f'      Δy = {diff_moveit2["dy"]*1000:.3f} mm')
            self.get_logger().info(f'      Δz = {diff_moveit2["dz"]*1000:.3f} mm')
            self.get_logger().info(f'    姿态差异: {math.degrees(diff_moveit2["orientation_angle"]):.3f}°')
            
            self.get_logger().info(f'\n  3️⃣ MoveIt2(TF) vs 机器人控制器:')
            self.get_logger().info(f'    位置差异: {diff_sources["position_distance"]*1000:.3f} mm')
            self.get_logger().info(f'      Δx = {diff_sources["dx"]*1000:.3f} mm')
            self.get_logger().info(f'      Δy = {diff_sources["dy"]*1000:.3f} mm')
            self.get_logger().info(f'      Δz = {diff_sources["dz"]*1000:.3f} mm')
            self.get_logger().info(f'    姿态差异: {math.degrees(diff_sources["orientation_angle"]):.3f}°')
        
        # 判断是否通过
        pos_ok = diff['position_distance'] <= self.position_tolerance
        ori_ok = diff['orientation_angle'] <= self.orientation_tolerance
        
        result['position_ok'] = pos_ok
        result['orientation_ok'] = ori_ok
        result['success'] = pos_ok and ori_ok
        
        if pos_ok and ori_ok:
            self.get_logger().info(f'\n✅ 验证通过: 位姿在容差范围内')
        else:
            self.get_logger().warn(f'\n⚠️  验证未通过: 位姿超出容差范围')
            if not pos_ok:
                self.get_logger().warn(f'    位置: {diff["position_distance"]*1000:.3f} mm > {self.position_tolerance*1000:.3f} mm')
            if not ori_ok:
                self.get_logger().warn(f'    姿态: {math.degrees(diff["orientation_angle"]):.3f}° > {math.degrees(self.orientation_tolerance):.3f}°')
        
        self.get_logger().info('='*70)
        
        return result
    
    def load_poses_from_json(self, json_file: str) -> List[Dict]:
        """从 JSON 文件加载位姿"""
        self.get_logger().info(f'从文件加载位姿: {json_file}')
        
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
            
            if 'recordedPoses' in data:
                poses = data['recordedPoses']
                self.get_logger().info(f'✓ 成功加载 {len(poses)} 个位姿')
                return poses
            else:
                self.get_logger().error('✗ JSON 文件格式错误: 缺少 recordedPoses 字段')
                return []
        except Exception as e:
            self.get_logger().error(f'✗ 加载 JSON 文件失败: {str(e)}')
            return []
    
    def generate_report(self, results: List[Dict], output_file: str):
        """生成测试报告"""
        total = len(results)
        passed = sum(1 for r in results if r.get('success', False))
        failed = total - passed
        
        # 计算统计数据
        if total > 0:
            valid_results = [r for r in results if 'pose_difference' in r]
            if valid_results:
                avg_pos = sum(r['pose_difference']['position_distance_mm'] for r in valid_results) / len(valid_results)
                avg_ori = sum(r['pose_difference']['orientation_angle_deg'] for r in valid_results) / len(valid_results)
                max_pos = max(r['pose_difference']['position_distance_mm'] for r in valid_results)
                max_ori = max(r['pose_difference']['orientation_angle_deg'] for r in valid_results)
                
                # 计算运动时间统计
                motion_times = []
                for r in valid_results:
                    if 'motion_detection' in r:
                        md = r['motion_detection']
                        if 'motion_duration' in md:
                            motion_times.append(md['motion_duration'])
                        elif 'wait_time' in md:
                            motion_times.append(md['wait_time'])
                avg_motion_time = sum(motion_times) / len(motion_times) if motion_times else 0
                
                # 计算 MoveIt2 位姿统计（如果有数据）
                moveit2_results = [r for r in valid_results if 'moveit2_vs_target' in r]
                if moveit2_results:
                    avg_moveit2_pos = sum(r['moveit2_vs_target']['position_distance_mm'] for r in moveit2_results) / len(moveit2_results)
                    avg_moveit2_ori = sum(r['moveit2_vs_target']['orientation_angle_deg'] for r in moveit2_results) / len(moveit2_results)
                    avg_sources_diff_pos = sum(r['moveit2_vs_controller']['position_distance_mm'] for r in moveit2_results) / len(moveit2_results)
                    avg_sources_diff_ori = sum(r['moveit2_vs_controller']['orientation_angle_deg'] for r in moveit2_results) / len(moveit2_results)
                else:
                    avg_moveit2_pos = avg_moveit2_ori = avg_sources_diff_pos = avg_sources_diff_ori = None
            else:
                avg_pos = avg_ori = max_pos = max_ori = avg_motion_time = 0
                avg_moveit2_pos = avg_moveit2_ori = avg_sources_diff_pos = avg_sources_diff_ori = None
        else:
            avg_pos = avg_ori = max_pos = max_ori = avg_motion_time = 0
            avg_moveit2_pos = avg_moveit2_ori = avg_sources_diff_pos = avg_sources_diff_ori = None
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'summary': {
                'total_tests': total,
                'passed': passed,
                'failed': failed,
                'pass_rate': f'{passed/total*100:.1f}%' if total > 0 else '0%'
            },
            'validation_criteria': {
                'position_tolerance_mm': self.position_tolerance * 1000,
                'orientation_tolerance_deg': math.degrees(self.orientation_tolerance)
            },
            'statistics': {
                'controller_vs_target': {
                    'average_position_error_mm': round(avg_pos, 3),
                    'average_orientation_error_deg': round(avg_ori, 3),
                    'max_position_error_mm': round(max_pos, 3),
                    'max_orientation_error_deg': round(max_ori, 3)
                },
                'average_motion_time_s': round(avg_motion_time, 3)
            },
            'test_results': results
        }
        
        # 如果有 MoveIt2 数据，添加到报告中
        if avg_moveit2_pos is not None:
            report['statistics']['moveit2_vs_target'] = {
                'average_position_error_mm': round(avg_moveit2_pos, 3),
                'average_orientation_error_deg': round(avg_moveit2_ori, 3)
            }
            report['statistics']['moveit2_vs_controller'] = {
                'average_position_error_mm': round(avg_sources_diff_pos, 3),
                'average_orientation_error_deg': round(avg_sources_diff_ori, 3)
            }
        
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        # 打印摘要
        self.get_logger().info('\n' + '='*70)
        self.get_logger().info('  📊 验证测试报告')
        self.get_logger().info('='*70)
        self.get_logger().info(f'\n总测试数: {total}')
        self.get_logger().info(f'通过: {passed} ✅')
        self.get_logger().info(f'未通过: {failed} ⚠️')
        self.get_logger().info(f'通过率: {passed/total*100:.1f}%' if total > 0 else '通过率: 0%')
        
        self.get_logger().info(f'\n📏 位姿误差统计:')
        self.get_logger().info(f'\n  1️⃣ 目标 vs 机器人控制器 (/demo_robot_status):')
        self.get_logger().info(f'    平均位置误差: {avg_pos:.3f} mm')
        self.get_logger().info(f'    平均姿态误差: {avg_ori:.3f}°')
        self.get_logger().info(f'    最大位置误差: {max_pos:.3f} mm')
        self.get_logger().info(f'    最大姿态误差: {max_ori:.3f}°')
        
        if avg_moveit2_pos is not None:
            self.get_logger().info(f'\n  2️⃣ 目标 vs MoveIt2 TF树计算 (/moveit2_tcp_pose):')
            self.get_logger().info(f'    平均位置误差: {avg_moveit2_pos:.3f} mm')
            self.get_logger().info(f'    平均姿态误差: {avg_moveit2_ori:.3f}°')
            
            self.get_logger().info(f'\n  3️⃣ MoveIt2 vs 机器人控制器（两个数据源对比）:')
            self.get_logger().info(f'    平均位置差异: {avg_sources_diff_pos:.3f} mm')
            self.get_logger().info(f'    平均姿态差异: {avg_sources_diff_ori:.3f}°')
            self.get_logger().info(f'\n  💡 建议: 基于TF树的MoveIt2位姿通常更准确，适合手眼标定')
        else:
            self.get_logger().info(f'\n  ⚠️  未检测到 MoveIt2 TCP 位姿数据')
        
        self.get_logger().info(f'\n⏱️  平均运动时间: {avg_motion_time:.3f} s')
        self.get_logger().info(f'\n💾 报告已保存到: {output_file}')
        self.get_logger().info('='*70)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    validator = MoveItPoseValidator()
    
    try:
        print("\n" + "="*70)
        print("  🤖 MoveIt2 位姿验证工具")
        print("="*70)
        print("\n此工具验证：")
        print("  1. MoveIt2 机械臂运动到位的判断方法")
        print("     - 监测运动开始")
        print("     - 监测运动结束")
        print("     - 记录运动时间")
        print("\n  2. MoveIt2 目标位姿与实际位姿的对比")
        print("     - 对比位置差异")
        print("     - 对比姿态差异")
        print("     - 判断是否在容差范围内")
        print("\n" + "="*70)
        
        # 等待初始化
        if not validator.wait_for_robot_status():
            validator.get_logger().error('无法接收机器人状态，退出')
            return
        
        if not validator.wait_for_service():
            validator.get_logger().error('无法连接到服务，退出')
            return
        
        # 加载位姿
        json_file = validator.json_file
        if not json_file:
            # 使用默认文件
            json_file = '/home/mu/IVG/aubo_ros2_ws/src/hand_eye_calibration/config/auto_hand_eye_poses_1767855934821.json'
            validator.get_logger().info(f'使用默认 JSON 文件: {json_file}')
        
        poses = validator.load_poses_from_json(json_file)
        if not poses:
            validator.get_logger().error('没有可用的位姿数据，退出')
            return
        
        # 选择要测试的位姿
        print(f"\n找到 {len(poses)} 个位姿")
        choice = input("要测试的位姿数量 (输入数字，0=全部，默认=5): ")
        
        if choice == '' or choice == '0':
            test_count = min(5, len(poses))
        else:
            try:
                test_count = min(int(choice), len(poses))
            except:
                test_count = min(5, len(poses))
        
        test_poses = poses[:test_count]
        print(f"\n将测试前 {test_count} 个位姿")
        
        input("\n按 Enter 开始验证测试...")
        
        # 执行验证测试
        results = []
        for i, pose_data in enumerate(test_poses, 1):
            result = validator.validate_single_pose(pose_data, i)
            results.append(result)
            
            if i < len(test_poses):
                print(f"\n等待 2 秒后继续下一个测试...")
                time.sleep(2)
        
        # 生成报告
        output_file = f"moveit_validation_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        validator.generate_report(results, output_file)
        
        validator.get_logger().info('\n✓ 验证测试完成')
        
    except KeyboardInterrupt:
        validator.get_logger().info('用户中断')
    except Exception as e:
        validator.get_logger().error(f'发生错误: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
