#!/usr/bin/env python3
"""
批量位姿测试脚本
自动测试多个目标位姿，并生成测试报告
"""

import rclpy
from rclpy.node import Node
from demo_interface.msg import RobotStatus
from demo_interface.srv import MoveToPose
from geometry_msgs.msg import Pose

import math
import time
import json
from datetime import datetime
from typing import List, Dict, Optional


class BatchPoseTester(Node):
    """批量位姿测试器"""
    
    def __init__(self):
        super().__init__('batch_pose_tester')
        
        # 参数
        self.declare_parameter('position_tolerance', 0.001)
        self.declare_parameter('orientation_tolerance', 0.01)
        self.declare_parameter('velocity_factor', 0.3)
        self.declare_parameter('acceleration_factor', 0.3)
        self.declare_parameter('test_count', 5)
        
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.velocity_factor = self.get_parameter('velocity_factor').value
        self.acceleration_factor = self.get_parameter('acceleration_factor').value
        self.test_count = self.get_parameter('test_count').value
        
        # 订阅机器人状态
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/demo_robot_status',
            self.robot_status_callback,
            10
        )
        
        # 创建服务客户端
        self.move_to_pose_client = self.create_client(MoveToPose, '/move_to_pose')
        
        self.current_robot_status: Optional[RobotStatus] = None
        self.status_received = False
        
        self.get_logger().info('批量位姿测试器已启动')
    
    def robot_status_callback(self, msg: RobotStatus):
        """机器人状态回调"""
        self.current_robot_status = msg
        if not self.status_received:
            self.status_received = True
    
    def wait_for_robot_status(self, timeout: float = 5.0) -> bool:
        """等待接收机器人状态"""
        start_time = time.time()
        while not self.status_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.status_received
    
    def wait_for_service(self, timeout: float = 5.0) -> bool:
        """等待服务可用"""
        return self.move_to_pose_client.wait_for_service(timeout_sec=timeout)
    
    def get_current_pose(self) -> Optional[Pose]:
        """获取当前位姿"""
        if self.current_robot_status is None:
            return None
        return self.current_robot_status.cartesian_position
    
    def move_to_pose(self, target_pose: Pose, use_joints: bool = False) -> bool:
        """移动到目标位姿"""
        request = MoveToPose.Request()
        request.target_pose = target_pose
        request.use_joints = use_joints
        request.velocity_factor = self.velocity_factor
        request.acceleration_factor = self.acceleration_factor
        
        future = self.move_to_pose_client.call_async(request)
        
        timeout = 60.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            return False
        
        try:
            response = future.result()
            return response.success
        except:
            return False
    
    def wait_for_motion_complete(self, timeout: float = 30.0) -> bool:
        """等待机械臂运动完成"""
        start_time = time.time()
        
        # 等待开始运动
        motion_started = False
        while not motion_started and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_robot_status and self.current_robot_status.in_motion:
                motion_started = True
                break
        
        # 等待停止运动
        stable_count = 0
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_robot_status and not self.current_robot_status.in_motion:
                stable_count += 1
                if stable_count >= 5:
                    time.sleep(0.5)
                    return True
            else:
                stable_count = 0
        
        return False
    
    def compute_pose_difference(self, pose1: Pose, pose2: Pose) -> Dict:
        """计算位姿差异"""
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
            'dx': dx,
            'dy': dy,
            'dz': dz
        }
    
    def test_single_pose(self, target_pose: Pose, test_id: int) -> Dict:
        """测试单个位姿"""
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'测试 #{test_id}')
        self.get_logger().info(f'{"="*70}')
        
        result = {
            'test_id': test_id,
            'target_pose': {
                'position': {
                    'x': target_pose.position.x,
                    'y': target_pose.position.y,
                    'z': target_pose.position.z
                },
                'orientation': {
                    'w': target_pose.orientation.w,
                    'x': target_pose.orientation.x,
                    'y': target_pose.orientation.y,
                    'z': target_pose.orientation.z
                }
            },
            'success': False,
            'error': None
        }
        
        # 移动到目标位姿
        self.get_logger().info(f'移动到目标位姿...')
        if not self.move_to_pose(target_pose):
            result['error'] = '移动失败'
            self.get_logger().error('✗ 移动失败')
            return result
        
        # 等待到位
        self.get_logger().info('等待机械臂到位...')
        if not self.wait_for_motion_complete():
            result['error'] = '等待到位超时'
            self.get_logger().error('✗ 等待到位超时')
            return result
        
        # 获取实际位姿
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)
        actual_pose = self.get_current_pose()
        
        if actual_pose is None:
            result['error'] = '无法获取实际位姿'
            self.get_logger().error('✗ 无法获取实际位姿')
            return result
        
        result['actual_pose'] = {
            'position': {
                'x': actual_pose.position.x,
                'y': actual_pose.position.y,
                'z': actual_pose.position.z
            },
            'orientation': {
                'w': actual_pose.orientation.w,
                'x': actual_pose.orientation.x,
                'y': actual_pose.orientation.y,
                'z': actual_pose.orientation.z
            }
        }
        
        # 计算差异
        diff = self.compute_pose_difference(target_pose, actual_pose)
        result['difference'] = {
            'position_distance_mm': diff['position_distance'] * 1000,
            'orientation_angle_deg': math.degrees(diff['orientation_angle']),
            'dx': diff['dx'],
            'dy': diff['dy'],
            'dz': diff['dz']
        }
        
        # 判断是否通过
        pos_ok = diff['position_distance'] <= self.position_tolerance
        ori_ok = diff['orientation_angle'] <= self.orientation_tolerance
        result['success'] = pos_ok and ori_ok
        result['position_ok'] = pos_ok
        result['orientation_ok'] = ori_ok
        
        # 输出结果
        self.get_logger().info(f'位置误差: {diff["position_distance"]*1000:.3f} mm')
        self.get_logger().info(f'姿态误差: {math.degrees(diff["orientation_angle"]):.3f}°')
        if result['success']:
            self.get_logger().info('✅ 测试通过')
        else:
            self.get_logger().warn('⚠️  测试未通过')
        
        return result
    
    def generate_report(self, results: List[Dict], output_file: str):
        """生成测试报告"""
        # 统计
        total = len(results)
        passed = sum(1 for r in results if r['success'])
        failed = total - passed
        
        # 计算平均误差
        if total > 0:
            avg_pos_error = sum(r['difference']['position_distance_mm'] for r in results if 'difference' in r) / total
            avg_ori_error = sum(r['difference']['orientation_angle_deg'] for r in results if 'difference' in r) / total
            max_pos_error = max(r['difference']['position_distance_mm'] for r in results if 'difference' in r)
            max_ori_error = max(r['difference']['orientation_angle_deg'] for r in results if 'difference' in r)
        else:
            avg_pos_error = avg_ori_error = max_pos_error = max_ori_error = 0
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'summary': {
                'total_tests': total,
                'passed': passed,
                'failed': failed,
                'pass_rate': f'{passed/total*100:.1f}%' if total > 0 else '0%',
                'position_tolerance_mm': self.position_tolerance * 1000,
                'orientation_tolerance_deg': math.degrees(self.orientation_tolerance)
            },
            'statistics': {
                'average_position_error_mm': round(avg_pos_error, 3),
                'average_orientation_error_deg': round(avg_ori_error, 3),
                'max_position_error_mm': round(max_pos_error, 3),
                'max_orientation_error_deg': round(max_ori_error, 3)
            },
            'test_results': results
        }
        
        # 保存到文件
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        # 打印摘要
        print("\n" + "="*70)
        print("  📊 批量测试报告")
        print("="*70)
        print(f"\n总测试数: {total}")
        print(f"通过: {passed} ✅")
        print(f"未通过: {failed} ⚠️")
        print(f"通过率: {passed/total*100:.1f}%" if total > 0 else "通过率: 0%")
        print(f"\n平均位置误差: {avg_pos_error:.3f} mm")
        print(f"平均姿态误差: {avg_ori_error:.3f}°")
        print(f"最大位置误差: {max_pos_error:.3f} mm")
        print(f"最大姿态误差: {max_ori_error:.3f}°")
        print(f"\n报告已保存到: {output_file}")
        print("="*70 + "\n")


def create_test_poses(count: int) -> List[Pose]:
    """创建测试位姿列表"""
    poses = []
    
    # 基础位姿
    base_pose = Pose()
    base_pose.position.x = -0.074
    base_pose.position.y = -0.209
    base_pose.position.z = 0.953
    base_pose.orientation.w = 0.7026
    base_pose.orientation.x = -0.0001
    base_pose.orientation.y = -0.0008
    base_pose.orientation.z = 0.7116
    
    # 生成变化的测试位姿
    for i in range(count):
        pose = Pose()
        # 在基础位姿周围产生小的变化
        pose.position.x = base_pose.position.x + (i - count//2) * 0.05
        pose.position.y = base_pose.position.y + (i % 3 - 1) * 0.05
        pose.position.z = base_pose.position.z + (i % 2) * 0.02
        pose.orientation = base_pose.orientation
        poses.append(pose)
    
    return poses


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    tester = BatchPoseTester()
    
    try:
        print("\n" + "="*70)
        print("  🤖 批量位姿测试工具")
        print("="*70)
        print("\n此工具将自动测试多个目标位姿并生成测试报告")
        print("="*70 + "\n")
        
        # 等待初始化
        if not tester.wait_for_robot_status(timeout=10.0):
            tester.get_logger().error('无法接收机器人状态，退出')
            return
        
        if not tester.wait_for_service(timeout=10.0):
            tester.get_logger().error('无法连接到服务，退出')
            return
        
        # 生成测试位姿
        test_count = tester.test_count
        test_poses = create_test_poses(test_count)
        
        print(f"将测试 {test_count} 个位姿")
        input("按 Enter 开始批量测试...")
        
        # 执行测试
        results = []
        for i, pose in enumerate(test_poses, 1):
            result = tester.test_single_pose(pose, i)
            results.append(result)
            
            if i < len(test_poses):
                time.sleep(1)  # 测试间隔
        
        # 生成报告
        output_file = f"batch_test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        tester.generate_report(results, output_file)
        
    except KeyboardInterrupt:
        tester.get_logger().info('用户中断')
    except Exception as e:
        tester.get_logger().error(f'发生错误: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
