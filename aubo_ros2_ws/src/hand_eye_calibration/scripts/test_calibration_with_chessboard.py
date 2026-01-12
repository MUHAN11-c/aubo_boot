#!/usr/bin/env python3
"""
使用棋盘格验证手眼标定结果
固定棋盘格，多姿态验证方法
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
from datetime import datetime
import cv2

class CalibrationTester(Node):
    def __init__(self):
        super().__init__('calibration_tester')
        
        self.get_logger().info('='*70)
        self.get_logger().info('🧪 手眼标定测试工具')
        self.get_logger().info('='*70)
        
        # 加载手眼标定结果
        self.load_hand_eye_calibration()
        
        # 存储测试数据
        self.test_data = []
        
    def load_hand_eye_calibration(self):
        """加载手眼标定结果"""
        try:
            # 从最新的标定结果文件加载
            import glob
            import os
            
            calib_dir = '/home/mu/IVG/hand_eye_calibration_validation'
            pattern = os.path.join(calib_dir, 'calibration_data_*.json')
            files = sorted(glob.glob(pattern), reverse=True)
            
            if not files:
                self.get_logger().error('❌ 未找到标定结果文件！')
                return False
                
            latest_file = files[0]
            self.get_logger().info(f'📂 加载标定结果: {os.path.basename(latest_file)}')
            
            with open(latest_file, 'r') as f:
                calib_data = json.load(f)
            
            # 提取T_camera2gripper
            T_cam2gripper = np.array(calib_data['T_camera2gripper'])
            self.T_camera2gripper = T_cam2gripper
            self.T_gripper2camera = np.linalg.inv(T_cam2gripper)
            
            self.get_logger().info('✅ 手眼标定结果加载成功')
            self.get_logger().info(f'   t_camera2gripper = {T_cam2gripper[:3, 3]} mm')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 加载标定结果失败: {str(e)}')
            return False
    
    def run_test(self, num_poses=5):
        """
        运行测试：固定棋盘格，多姿态验证
        
        Args:
            num_poses: 测试的机器人姿态数量
        """
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📋 开始测试：固定棋盘格，多姿态验证')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info(f'目标：采集 {num_poses} 个不同姿态')
        self.get_logger().info('要求：棋盘格固定在工作台，不要移动！')
        self.get_logger().info('')
        
        input('按Enter键开始测试...')
        
        board_positions_in_base = []
        board_orientations_in_base = []
        
        for i in range(num_poses):
            self.get_logger().info('')
            self.get_logger().info(f'━━━ 姿态 #{i+1}/{num_poses} ━━━')
            
            # 1. 移动机器人到新位置
            self.get_logger().info('📍 请将机器人移动到一个新位置（确保能看到棋盘格）')
            input(f'  位置就绪后按Enter继续...')
            
            # 2. 获取机器人当前位姿
            self.get_logger().info('🤖 获取机器人位姿...')
            # TODO: 实际实现中应该从ROS话题获取
            # T_base2gripper = self.get_robot_pose()
            
            self.get_logger().info('⚠️  当前为示例模式，请手动输入或集成ROS话题')
            self.get_logger().info('     在实际使用中，应该从 /demo_robot_status 自动获取')
            
            # 3. 检测棋盘格
            self.get_logger().info('📷 检测棋盘格...')
            # TODO: 实际实现中应该调用相机检测
            # T_camera2board = self.detect_chessboard()
            
            self.get_logger().info('⚠️  当前为示例模式，请使用Web界面检测棋盘格')
            self.get_logger().info('     或集成到现有的检测流程中')
            
            # 4. 计算棋盘格在Base中的位置
            # T_board2base = T_base2gripper @ T_gripper2camera @ T_camera2board
            
            self.get_logger().info('✅ 数据采集完成')
            
        # 5. 分析结果
        self.analyze_results(board_positions_in_base, board_orientations_in_base)
    
    def analyze_results(self, positions, orientations):
        """
        分析测试结果
        
        Args:
            positions: 棋盘格在Base中的位置列表
            orientations: 棋盘格在Base中的姿态列表
        """
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('📊 测试结果分析')
        self.get_logger().info('='*70)
        
        if len(positions) < 2:
            self.get_logger().warning('⚠️ 测试数据不足，至少需要2个姿态')
            return
        
        # 计算位置统计
        positions_array = np.array(positions)
        mean_pos = np.mean(positions_array, axis=0)
        std_pos = np.std(positions_array, axis=0)
        max_deviation = np.max(np.abs(positions_array - mean_pos), axis=0)
        
        self.get_logger().info('')
        self.get_logger().info('📐 位置统计（棋盘格在Base坐标系中）：')
        self.get_logger().info(f'   平均位置: [{mean_pos[0]:.2f}, {mean_pos[1]:.2f}, {mean_pos[2]:.2f}] mm')
        self.get_logger().info(f'   标准差:   [{std_pos[0]:.2f}, {std_pos[1]:.2f}, {std_pos[2]:.2f}] mm')
        self.get_logger().info(f'   最大偏差: [{max_deviation[0]:.2f}, {max_deviation[1]:.2f}, {max_deviation[2]:.2f}] mm')
        
        # 计算总体标准差
        overall_std = np.linalg.norm(std_pos)
        self.get_logger().info(f'   总体标准差: {overall_std:.2f} mm')
        
        # 判断
        self.get_logger().info('')
        if overall_std < 10:
            self.get_logger().info('✅✅✅ 标定结果优秀！ (标准差 < 10mm)')
        elif overall_std < 30:
            self.get_logger().info('✅ 标定结果可用 (标准差 < 30mm)')
        else:
            self.get_logger().error('❌ 标定结果有问题 (标准差 > 30mm)')
            self.get_logger().error('   建议重新标定！')
        
        # 保存结果
        self.save_test_results(positions, orientations, {
            'mean_position': mean_pos.tolist(),
            'std_position': std_pos.tolist(),
            'overall_std': float(overall_std)
        })
    
    def save_test_results(self, positions, orientations, statistics):
        """保存测试结果"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'/home/mu/IVG/hand_eye_calibration_validation/test_results_{timestamp}.json'
        
        data = {
            'timestamp': timestamp,
            'test_type': 'fixed_chessboard_multi_pose',
            'num_poses': len(positions),
            'positions': [p.tolist() if isinstance(p, np.ndarray) else p for p in positions],
            'orientations': [o.tolist() if isinstance(o, np.ndarray) else o for o in orientations],
            'statistics': statistics
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        self.get_logger().info(f'💾 测试结果已保存: {filename}')

def main():
    """主函数"""
    print('')
    print('='*70)
    print('🧪 手眼标定测试工具')
    print('='*70)
    print('')
    print('本工具用于验证手眼标定结果的准确性')
    print('')
    print('测试方法：固定棋盘格，多姿态验证')
    print('  1. 将棋盘格固定在工作台上（不要移动）')
    print('  2. 移动机器人到多个不同位置')
    print('  3. 每个位置检测棋盘格，计算其在Base中的位置')
    print('  4. 分析多次计算结果的一致性')
    print('')
    print('⚠️  当前版本需要配合Web界面使用')
    print('   请使用Web界面中的"测试标定"功能')
    print('')
    print('🚀 完整集成版本即将推出...')
    print('')
    
    # rclpy.init()
    # node = CalibrationTester()
    # node.run_test(num_poses=5)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
