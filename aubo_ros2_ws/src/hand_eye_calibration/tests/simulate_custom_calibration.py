#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Custom模式手眼标定模拟验证脚本
使用与OpenCV模式相同的地面真值和机器人位姿，对比两种模式的表现
"""

import sys
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
from datetime import datetime

# 添加hand_eye_calibration模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'hand_eye_calibration'))
from custom_hand_eye_calibration import CustomHandEyeCalibration


class SimulateCustomCalibration:
    """Custom模式手眼标定模拟验证类"""
    
    def __init__(self):
        """初始化模拟器，设置与OpenCV模式相同的地面真值"""
        # 地面真值：相机到末端执行器的变换（与OpenCV模式完全相同）
        # 单位：毫米
        self.T_cam2gripper_true = np.array([
            [1.0,  0.0,  0.0,  50.0],   # X方向偏移50mm
            [0.0, -1.0,  0.0,   0.0],   # Y方向无偏移，绕X轴旋转180度
            [0.0,  0.0, -1.0, 150.0],   # Z方向偏移150mm，绕X轴旋转180度
            [0.0,  0.0,  0.0,   1.0]
        ])
        
        # 标定板固定位置（基座坐标系，单位：毫米）
        self.board_position_base = np.array([400.0, 0.0, 0.0])
        
        print("="*80)
        print("Custom模式手眼标定模拟验证")
        print("="*80)
        print("\n【地面真值Ground Truth】")
        print("相机到末端执行器的变换矩阵 T_cam2gripper (单位: 毫米):")
        print(self.T_cam2gripper_true)
        print(f"平移向量: {self.T_cam2gripper_true[:3, 3]}")
        print(f"平移范数: {np.linalg.norm(self.T_cam2gripper_true[:3, 3]):.2f} mm")
        print()
    
    def euler_to_rotation_matrix(self, rx, ry, rz):
        """欧拉角转旋转矩阵（ZYX顺序）"""
        r = R.from_euler('xyz', [rx, ry, rz], degrees=False)
        return r.as_matrix()
    
    def generate_robot_poses(self, num_poses=20):
        """
        生成机器人位姿（与OpenCV模式相同的位姿）
        
        参数:
            num_poses: 位姿数量
        
        返回:
            robot_poses: 机器人位姿列表（Base→Gripper变换矩阵，单位：毫米）
        """
        print(f"【步骤1：数据采集】生成 {num_poses} 个模拟机器人位姿")
        robot_poses = []
        
        # 使用与OpenCV模式相同的随机种子
        np.random.seed(42)
        
        for i in range(num_poses):
            # 生成随机位置（与OpenCV模式相同的范围）
            position = np.array([
                np.random.uniform(300, 600),   # X: 300-600mm
                np.random.uniform(-150, 150),  # Y: ±150mm
                np.random.uniform(200, 400)    # Z: 200-400mm
            ])
            
            # 生成随机姿态（与OpenCV模式相同的范围）
            euler_angles = np.array([
                np.random.uniform(-25, 25),   # 绕X轴: ±25度
                np.random.uniform(-25, 25),   # 绕Y轴: ±25度
                np.random.uniform(-45, 45)    # 绕Z轴: ±45度
            ]) * np.pi / 180.0
            
            rotation = self.euler_to_rotation_matrix(*euler_angles)
            
            # 构建4x4变换矩阵
            T_base2gripper = np.eye(4)
            T_base2gripper[:3, :3] = rotation
            T_base2gripper[:3, 3] = position
            
            robot_poses.append(T_base2gripper)
            print(f"  位姿 #{i+1}: 位置=[{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}] mm")
        
        print()
        return robot_poses
    
    def generate_board_poses(self, robot_poses):
        """
        根据地面真值生成标定板在相机坐标系中的位姿
        
        参数:
            robot_poses: 机器人位姿列表（Base→Gripper）
        
        返回:
            board_poses_camera: 标定板位姿列表（Board→Camera）
        """
        print(f"【步骤1：数据采集】根据地面真值生成标定板位姿")
        print(f"  标定板固定位置（基座坐标系）: {self.board_position_base} mm")
        
        board_poses_camera = []
        
        # 标定板在基座坐标系中的位姿（固定）
        T_base2board = np.eye(4)
        T_base2board[:3, 3] = self.board_position_base
        
        for i, T_base2gripper in enumerate(robot_poses):
            # 计算标定板在相机坐标系中的位姿
            # T_board2camera = inv(T_cam2gripper) @ inv(T_base2gripper) @ T_base2board
            T_gripper2cam = np.linalg.inv(self.T_cam2gripper_true)
            T_gripper2base = np.linalg.inv(T_base2gripper)
            T_board2camera = T_gripper2cam @ T_gripper2base @ T_base2board
            
            board_poses_camera.append(T_board2camera)
            board_position_camera = T_board2camera[:3, 3]
            print(f"  位姿 #{i+1}: 标定板在相机坐标系中的位置=[{board_position_camera[0]:.1f}, {board_position_camera[1]:.1f}, {board_position_camera[2]:.1f}] mm")
        
        print()
        return board_poses_camera
    
    def convert_to_custom_format(self, robot_poses, board_poses_camera):
        """
        将数据转换为Custom模式需要的格式
        Custom模式需要：
        - 1个shot_pose（拍照位姿）
        - N个pick_poses（点选位姿）
        - N个camera_points（角点在相机坐标系中的3D坐标）
        
        参数:
            robot_poses: 机器人位姿列表（Base→Gripper）
            board_poses_camera: 标定板位姿列表（Board→Camera）
        
        返回:
            shot_pose: 拍照位姿（4x4矩阵，单位：毫米）
            pick_poses: 点选位姿列表（4x4矩阵，单位：毫米）
            camera_points: 角点在相机坐标系中的3D坐标列表（单位：毫米）
        """
        print(f"【步骤2：数据处理】转换为Custom模式数据格式")
        
        # 使用第一个位姿作为拍照位姿
        shot_pose = robot_poses[0]
        print(f"  拍照位姿(shot_pose): 位置=[{shot_pose[0,3]:.1f}, {shot_pose[1,3]:.1f}, {shot_pose[2,3]:.1f}] mm")
        
        # 其余位姿作为点选位姿
        pick_poses = robot_poses[1:]
        
        # 对于Custom模式，我们模拟标定板中心点在相机坐标系中的3D坐标
        # 在实际应用中，这些是从深度相机获取的角点3D坐标
        camera_points = []
        for i, T_board2camera in enumerate(board_poses_camera[1:], start=1):
            # 标定板中心点在标定板坐标系中是原点
            board_center_in_board = np.array([0.0, 0.0, 0.0, 1.0])
            # 转换到相机坐标系
            board_center_in_camera = T_board2camera @ board_center_in_board
            camera_point = board_center_in_camera[:3]
            camera_points.append(camera_point)
            print(f"  点选位姿 #{i}: 机器人位置=[{pick_poses[i-1][0,3]:.1f}, {pick_poses[i-1][1,3]:.1f}, {pick_poses[i-1][2,3]:.1f}] mm, "
                  f"标定板在相机中=[{camera_point[0]:.1f}, {camera_point[1]:.1f}, {camera_point[2]:.1f}] mm")
        
        print(f"\n  Custom模式数据: shot_pose(1个), pick_poses({len(pick_poses)}个), camera_points({len(camera_points)}个)")
        print()
        return shot_pose, pick_poses, camera_points
    
    def calculate_errors(self, T_estimated):
        """
        计算标定误差
        
        参数:
            T_estimated: 估计的T_cam2gripper
        
        返回:
            errors: 误差字典
        """
        # 平移误差
        t_true = self.T_cam2gripper_true[:3, 3]
        t_estimated = T_estimated[:3, 3]
        translation_error = np.linalg.norm(t_estimated - t_true)
        translation_diff = t_estimated - t_true
        
        # 旋转误差
        R_true = self.T_cam2gripper_true[:3, :3]
        R_estimated = T_estimated[:3, :3]
        R_diff = R_estimated @ R_true.T
        trace = np.trace(R_diff)
        cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        rotation_error = np.arccos(cos_angle) * 180.0 / np.pi
        
        errors = {
            'translation_error_mm': translation_error,
            'rotation_error_deg': rotation_error,
            'translation_diff_mm': translation_diff,
            't_true': t_true,
            't_estimated': t_estimated
        }
        
        return errors
    
    def run_simulation(self, num_poses=20):
        """
        运行完整的模拟验证流程
        
        参数:
            num_poses: 位姿数量
        
        返回:
            results: 验证结果字典
        """
        print("="*80)
        print("开始Custom模式手眼标定模拟验证")
        print("="*80)
        print()
        
        # 步骤1：生成机器人位姿
        robot_poses = self.generate_robot_poses(num_poses)
        
        # 步骤2：生成标定板位姿
        board_poses_camera = self.generate_board_poses(robot_poses)
        
        # 步骤3：转换为Custom模式数据格式
        shot_pose, pick_poses, camera_points = self.convert_to_custom_format(
            robot_poses, board_poses_camera
        )
        
        # 步骤4：调用Custom模式标定算法
        print("【步骤3：算法计算】调用Custom模式手眼标定算法")
        calibrator = CustomHandEyeCalibration()
        
        try:
            # solve_hand_eye返回元组: (T_camera2gripper, fixed_z)
            T_estimated, fixed_z = calibrator.solve_hand_eye(
                shot_pose=shot_pose,
                pick_poses=pick_poses,
                camera_points_shot=camera_points,
                fixed_z=None  # 自动计算Z值
            )
            
            print("\n【步骤3：算法计算】Custom模式标定完成")
            print("估计的相机到末端执行器变换矩阵 T_cam2gripper (单位: 毫米):")
            print(T_estimated)
            print(f"平移向量: {T_estimated[:3, 3]}")
            print(f"平移范数: {np.linalg.norm(T_estimated[:3, 3]):.2f} mm")
            print(f"固定Z值（棋盘格桌面高度）: {fixed_z:.2f} mm")
            print()
            
            # 步骤5：计算误差
            print("【步骤4：误差分析】对比地面真值与估计值")
            errors = self.calculate_errors(T_estimated)
            
            print("\n【误差统计】")
            print(f"  平移误差: {errors['translation_error_mm']:.3f} mm")
            print(f"  旋转误差: {errors['rotation_error_deg']:.3f}°")
            print(f"\n  平移误差分量:")
            print(f"    ΔX = {errors['translation_diff_mm'][0]:+.3f} mm")
            print(f"    ΔY = {errors['translation_diff_mm'][1]:+.3f} mm")
            print(f"    ΔZ = {errors['translation_diff_mm'][2]:+.3f} mm")
            
            # 评估结果
            print("\n" + "="*80)
            print("Custom模式手眼标定模拟验证完成")
            print("="*80)
            print("\n【结果评估】")
            if errors['translation_error_mm'] < 1.0 and errors['rotation_error_deg'] < 0.5:
                print("  ✅ 优秀: 平移误差 < 1mm, 旋转误差 < 0.5°")
            elif errors['translation_error_mm'] < 5.0 and errors['rotation_error_deg'] < 2.0:
                print("  ✅ 良好: 平移误差 < 5mm, 旋转误差 < 2°")
            elif errors['translation_error_mm'] < 20.0 and errors['rotation_error_deg'] < 10.0:
                print("  ⚠️  可接受: 平移误差 < 20mm, 旋转误差 < 10°")
            else:
                print("  ❌ 较大误差: 平移误差 > 20mm 或 旋转误差 > 10°")
            
            # 保存结果
            t_true = self.T_cam2gripper_true[:3, 3]
            results = {
                'ground_truth': {
                    'T_cam2gripper': self.T_cam2gripper_true.tolist(),
                    'translation': t_true.tolist(),
                    'translation_norm': float(np.linalg.norm(t_true))
                },
                'estimated': {
                    'T_cam2gripper': T_estimated.tolist(),
                    'translation': T_estimated[:3, 3].tolist(),
                    'translation_norm': float(np.linalg.norm(T_estimated[:3, 3])),
                    'fixed_z': float(fixed_z)
                },
                'errors': {
                    'translation_error_mm': float(errors['translation_error_mm']),
                    'rotation_error_deg': float(errors['rotation_error_deg']),
                    'translation_diff_mm': errors['translation_diff_mm'].tolist()
                },
                'config': {
                    'num_poses': num_poses,
                    'num_pick_poses': len(pick_poses),
                    'mode': 'Custom (XY+Z Constraint)'
                },
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            # 保存到文件
            output_dir = os.path.join(os.path.dirname(__file__), 'simulation_results')
            os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_file = os.path.join(output_dir, f'custom_simulation_results_{timestamp}.json')
            
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, ensure_ascii=False)
            
            print(f"\n【结果保存】模拟结果已保存到: {output_file}")
            print()
            
            return results
            
        except Exception as e:
            print(f"\n❌ Custom模式标定失败: {str(e)}")
            import traceback
            traceback.print_exc()
            return None


def main():
    """主函数"""
    simulator = SimulateCustomCalibration()
    results = simulator.run_simulation(num_poses=20)
    
    if results:
        print("\n模拟完成，详细结果已保存到: simulation_results/")


if __name__ == '__main__':
    main()

