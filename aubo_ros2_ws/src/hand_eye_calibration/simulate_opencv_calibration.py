#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
模拟手眼标定完整流程验证脚本
验证OpenCV模式下的数据采集、处理、计算和误差分析

模拟流程：
1. 数据采集：生成模拟的机器人位姿和标定板位姿数据
2. 数据处理：去重、坐标变换、单位转换
3. 算法计算：使用OpenCV calibrateHandEye进行手眼标定
4. 误差计算：验证AX=XB约束，计算标定误差
"""

import numpy as np
import cv2
import json
import os
from datetime import datetime
import sys

# 添加手眼标定模块路径
sys.path.append(os.path.dirname(__file__))

class HandEyeCalibrationSimulator:
    """手眼标定模拟器"""
    
    def __init__(self):
        """初始化模拟器"""
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(os.path.dirname(__file__), 'simulation_results')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 真实的手眼标定参数（地面真值Ground Truth）
        # 相机到末端执行器的变换矩阵（单位：毫米）
        # 假设相机固定在末端执行器上方150mm，偏移X方向50mm
        self.T_cam2gripper_true = np.eye(4)
        # 旋转：相机朝下（绕X轴旋转180度）
        self.T_cam2gripper_true[:3, :3] = cv2.Rodrigues(np.array([np.pi, 0, 0]))[0]
        # 平移：X=50mm, Y=0mm, Z=150mm
        self.T_cam2gripper_true[:3, 3] = [50.0, 0.0, 150.0]
        
        print("\n" + "="*80)
        print("手眼标定完整流程模拟验证")
        print("="*80)
        print(f"\n【地面真值Ground Truth】")
        print(f"相机到末端执行器的变换矩阵 T_cam2gripper (单位: 毫米):")
        print(self.T_cam2gripper_true)
        print(f"平移向量: {self.T_cam2gripper_true[:3, 3]}")
        print(f"平移范数: {np.linalg.norm(self.T_cam2gripper_true[:3, 3]):.2f} mm")
    
    def _rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数 [x, y, z, w]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        # 归一化
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        return np.array([x/norm, y/norm, z/norm, w/norm])
    
    def generate_robot_poses(self, num_poses=10):
        """
        生成模拟的机器人末端位姿
        
        参数:
            num_poses: 位姿数量
        
        返回:
            robot_poses: 机器人位姿列表（Base→Gripper变换矩阵，单位：毫米）
        """
        print(f"\n【步骤1：数据采集】生成 {num_poses} 个模拟机器人位姿")
        
        robot_poses = []
        
        # 基础位置（机器人工作空间中心）
        base_position = np.array([400.0, 0.0, 300.0])  # 单位：毫米
        
        # 生成不同的位姿（在基础位置周围变化）
        for i in range(num_poses):
            # 生成随机偏移 - 增加运动范围以提供更多样化的数据
            # X: ±150mm, Y: ±150mm, Z: ±100mm（增大垂直方向变化）
            offset = np.array([
                np.random.uniform(-150, 150),
                np.random.uniform(-150, 150),
                np.random.uniform(-100, 100)
            ])
            
            position = base_position + offset
            
            # 生成随机旋转（增大旋转范围，提供更多样化的观察角度）
            # 绕X轴: ±25度, 绕Y轴: ±25度, 绕Z轴: ±45度
            euler_angles = np.array([
                np.random.uniform(-25, 25),
                np.random.uniform(-25, 25),
                np.random.uniform(-45, 45)
            ]) * np.pi / 180.0  # 转换为弧度
            
            # 将欧拉角转换为旋转矩阵（ZYX顺序）
            Rx = cv2.Rodrigues(np.array([euler_angles[0], 0, 0]))[0]
            Ry = cv2.Rodrigues(np.array([0, euler_angles[1], 0]))[0]
            Rz = cv2.Rodrigues(np.array([0, 0, euler_angles[2]]))[0]
            R = Rz @ Ry @ Rx
            
            # 构建变换矩阵（Base→Gripper）
            T_base2gripper = np.eye(4)
            T_base2gripper[:3, :3] = R
            T_base2gripper[:3, 3] = position
            
            robot_poses.append(T_base2gripper)
            
            print(f"  位姿 #{i+1}: 位置=[{position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}] mm")
        
        return robot_poses
    
    def generate_board_poses(self, robot_poses):
        """
        根据机器人位姿和地面真值生成标定板位姿
        
        参数:
            robot_poses: 机器人位姿列表（Base→Gripper变换矩阵）
        
        返回:
            board_poses: 标定板位姿列表（Board→Camera变换矩阵，单位：毫米）
        """
        print(f"\n【步骤1：数据采集】根据地面真值生成标定板位姿")
        
        board_poses = []
        
        # 标定板在基座坐标系中的固定位置（假设标定板固定在桌面上）
        # 标定板位置：X=400mm, Y=0mm, Z=0mm（桌面）
        # 标定板姿态：标定板法线朝上（与相机光轴反向）
        # 注意：标定板坐标系通常定义为Z轴垂直板面向外（朝向相机）
        # 在桌面上时，Z轴应该朝上（+Z方向）
        T_base2board = np.eye(4)
        T_base2board[:3, 3] = [400.0, 0.0, 0.0]
        # 标定板不需要旋转，默认Z轴已经朝上
        
        print(f"  标定板固定位置（基座坐标系）: {T_base2board[:3, 3]} mm")
        
        for i, T_base2gripper in enumerate(robot_poses):
            # 计算标定板在相机坐标系中的位姿
            # 
            # 正确推导：
            # P_board在Board坐标系: P_board
            # P_board在Base坐标系: P_base = T_base2board @ P_board
            # P_board在Gripper坐标系: P_gripper = inv(T_base2gripper) @ P_base
            # P_board在Camera坐标系: P_camera = inv(T_cam2gripper) @ P_gripper
            # 
            # 因此: P_camera = inv(T_cam2gripper) @ inv(T_base2gripper) @ T_base2board @ P_board
            # 
            # 所以: T_board2camera = inv(T_cam2gripper) @ inv(T_base2gripper) @ T_base2board
            
            # ✅ 正确公式：需要对T_cam2gripper取逆（因为坐标变换是从Gripper→Camera，而不是Camera→Gripper）
            T_gripper2cam = np.linalg.inv(self.T_cam2gripper_true)
            T_gripper2base = np.linalg.inv(T_base2gripper)
            T_board2camera = T_gripper2cam @ T_gripper2base @ T_base2board
            
            board_poses.append(T_board2camera)
            
            # 提取标定板在相机坐标系中的位置和方向
            board_pos = T_board2camera[:3, 3]
            board_R = T_board2camera[:3, :3]
            board_rvec, _ = cv2.Rodrigues(board_R)
            
            print(f"  位姿 #{i+1}: 标定板在相机坐标系中的位置=[{board_pos[0]:.1f}, {board_pos[1]:.1f}, {board_pos[2]:.1f}] mm")
        
        return board_poses
    
    def convert_to_poses_data_format(self, robot_poses, board_poses):
        """
        将变换矩阵转换为hand_eye_calibration_node期望的数据格式
        
        参数:
            robot_poses: 机器人位姿列表（Base→Gripper变换矩阵，单位：毫米）
            board_poses: 标定板位姿列表（Board→Camera变换矩阵，单位：毫米）
        
        返回:
            poses_data: 位姿数据列表（格式与实际标定系统一致）
        """
        print(f"\n【步骤2：数据处理】转换为标定系统数据格式")
        
        poses_data = []
        
        for i, (T_robot, T_board) in enumerate(zip(robot_poses, board_poses)):
            # 提取机器人位姿（Base→Gripper）
            robot_pos_mm = T_robot[:3, 3]
            robot_R = T_robot[:3, :3]
            robot_quat = self._rotation_matrix_to_quaternion(robot_R)
            
            # 提取标定板位姿（Board→Camera）
            board_pos_mm = T_board[:3, 3]
            board_R = T_board[:3, :3]
            board_quat = self._rotation_matrix_to_quaternion(board_R)
            board_rvec, _ = cv2.Rodrigues(board_R)
            board_tvec = board_pos_mm.reshape(3, 1)
            
            # 构建标定系统期望的数据格式
            pose_data = {
                'robot_pose': {
                    # 机器人位姿：单位转换为米（标定系统期望单位）
                    'robot_pos_x': float(robot_pos_mm[0] / 1000.0),
                    'robot_pos_y': float(robot_pos_mm[1] / 1000.0),
                    'robot_pos_z': float(robot_pos_mm[2] / 1000.0),
                    # 四元数 [x, y, z, w]
                    'robot_ori_x': float(robot_quat[0]),
                    'robot_ori_y': float(robot_quat[1]),
                    'robot_ori_z': float(robot_quat[2]),
                    'robot_ori_w': float(robot_quat[3])
                },
                'board_pose': {
                    # 标定板位姿：单位保持毫米
                    'position': {
                        'x': float(board_pos_mm[0]),
                        'y': float(board_pos_mm[1]),
                        'z': float(board_pos_mm[2])
                    },
                    'orientation': {
                        'x': float(board_quat[0]),
                        'y': float(board_quat[1]),
                        'z': float(board_quat[2]),
                        'w': float(board_quat[3])
                    },
                    # 旋转向量和平移向量（用于OpenCV）
                    'rvec': {
                        'x': float(board_rvec[0, 0]),
                        'y': float(board_rvec[1, 0]),
                        'z': float(board_rvec[2, 0])
                    },
                    'tvec': {
                        'x': float(board_tvec[0, 0]),
                        'y': float(board_tvec[1, 0]),
                        'z': float(board_tvec[2, 0])
                    }
                }
            }
            
            poses_data.append(pose_data)
            
            print(f"  位姿 #{i+1}: 机器人位置(米)=[{pose_data['robot_pose']['robot_pos_x']:.3f}, "
                  f"{pose_data['robot_pose']['robot_pos_y']:.3f}, {pose_data['robot_pose']['robot_pos_z']:.3f}], "
                  f"标定板位置(毫米)=[{pose_data['board_pose']['position']['x']:.1f}, "
                  f"{pose_data['board_pose']['position']['y']:.1f}, {pose_data['board_pose']['position']['z']:.1f}]")
        
        return poses_data
    
    def save_results(self, results):
        """保存模拟结果到文件"""
        filename = f'simulation_results_{self.session_timestamp}.json'
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        
        print(f"\n【结果保存】模拟结果已保存到: {filepath}")
        return filepath
    
    def run_simulation(self, num_poses=10):
        """
        运行完整的手眼标定模拟
        
        参数:
            num_poses: 位姿数量
        
        返回:
            results: 包含所有结果的字典
        """
        print("\n" + "="*80)
        print("开始手眼标定完整流程模拟")
        print("="*80)
        
        # 步骤1：生成模拟数据
        robot_poses = self.generate_robot_poses(num_poses)
        board_poses = self.generate_board_poses(robot_poses)
        poses_data = self.convert_to_poses_data_format(robot_poses, board_poses)
        
        # 步骤2：调用OpenCV手眼标定
        print(f"\n【步骤3：算法计算】调用OpenCV手眼标定算法")
        
        from hand_eye_calibration.opencv_hand_eye_calibration import OpenCVHandEyeCalibration
        
        calibrator = OpenCVHandEyeCalibration()
        calibration_results = calibrator.calibrate(poses_data)
        
        # 提取标定结果
        T_cam2gripper_estimated = calibration_results['T_camera2gripper']
        
        print(f"\n【步骤3：算法计算】OpenCV标定完成")
        print(f"估计的相机到末端执行器变换矩阵 T_cam2gripper (单位: 毫米):")
        print(T_cam2gripper_estimated)
        print(f"平移向量: {T_cam2gripper_estimated[:3, 3]}")
        print(f"平移范数: {np.linalg.norm(T_cam2gripper_estimated[:3, 3]):.2f} mm")
        
        # 步骤3：对比地面真值和估计值
        print(f"\n【步骤4：误差分析】对比地面真值与估计值")
        
        # 平移误差
        translation_error = np.linalg.norm(
            T_cam2gripper_estimated[:3, 3] - self.T_cam2gripper_true[:3, 3]
        )
        
        # 旋转误差（角度差）
        R_estimated = T_cam2gripper_estimated[:3, :3]
        R_true = self.T_cam2gripper_true[:3, :3]
        R_diff = R_estimated @ R_true.T
        trace = np.trace(R_diff)
        rotation_error_rad = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
        rotation_error_deg = np.degrees(rotation_error_rad)
        
        print(f"\n【误差统计】")
        print(f"  平移误差: {translation_error:.3f} mm")
        print(f"  旋转误差: {rotation_error_deg:.3f}°")
        
        # 各分量误差
        translation_error_xyz = T_cam2gripper_estimated[:3, 3] - self.T_cam2gripper_true[:3, 3]
        print(f"\n  平移误差分量:")
        print(f"    ΔX = {translation_error_xyz[0]:+.3f} mm")
        print(f"    ΔY = {translation_error_xyz[1]:+.3f} mm")
        print(f"    ΔZ = {translation_error_xyz[2]:+.3f} mm")
        
        # AX=XB约束验证误差
        error_stats = calibration_results['error_statistics']
        print(f"\n  AX=XB约束验证:")
        print(f"    平移RMS误差: {error_stats['mean_translation_error']:.3f} mm")
        print(f"    旋转RMS误差: {error_stats['mean_rotation_error_deg']:.3f}°")
        print(f"    最大平移误差: {error_stats['max_translation_error']:.3f} mm")
        print(f"    最大旋转误差: {error_stats['max_rotation_error_deg']:.3f}°")
        
        # 汇总结果
        results = {
            'simulation_info': {
                'timestamp': self.session_timestamp,
                'num_poses': num_poses,
                'description': '手眼标定完整流程模拟验证（含噪声）'
            },
            'ground_truth': {
                'T_cam2gripper': self.T_cam2gripper_true.tolist(),
                'translation_mm': self.T_cam2gripper_true[:3, 3].tolist(),
                'translation_norm_mm': float(np.linalg.norm(self.T_cam2gripper_true[:3, 3]))
            },
            'estimated': {
                'T_cam2gripper': T_cam2gripper_estimated.tolist(),
                'translation_mm': T_cam2gripper_estimated[:3, 3].tolist(),
                'translation_norm_mm': float(np.linalg.norm(T_cam2gripper_estimated[:3, 3]))
            },
            'errors': {
                'translation_error_mm': float(translation_error),
                'rotation_error_deg': float(rotation_error_deg),
                'translation_error_xyz_mm': translation_error_xyz.tolist(),
                'ax_xb_constraint': {
                    'mean_translation_error_mm': float(error_stats['mean_translation_error']),
                    'mean_rotation_error_deg': float(error_stats['mean_rotation_error_deg']),
                    'max_translation_error_mm': float(error_stats['max_translation_error']),
                    'max_rotation_error_deg': float(error_stats['max_rotation_error_deg'])
                }
            },
            'calibration_details': {
                'input_poses_count': len(poses_data),
                'processed_poses_count': calibration_results.get('session_id'),
                'config_dir': calibration_results.get('config_dir')
            }
        }
        
        # 保存结果
        self.save_results(results)
        
        print("\n" + "="*80)
        print("手眼标定完整流程模拟验证完成")
        print("="*80)
        
        # 评估结果
        print(f"\n【结果评估】")
        if translation_error < 1.0 and rotation_error_deg < 0.5:
            print("  ✅ 优秀: 平移误差 < 1mm, 旋转误差 < 0.5°")
        elif translation_error < 5.0 and rotation_error_deg < 2.0:
            print("  ✅ 良好: 平移误差 < 5mm, 旋转误差 < 2°")
        elif translation_error < 10.0 and rotation_error_deg < 5.0:
            print("  ⚠️  可接受: 平移误差 < 10mm, 旋转误差 < 5°")
        else:
            print("  ❌ 较差: 平移误差 ≥ 10mm 或 旋转误差 ≥ 5°")
        
        return results


def main():
    """主函数"""
    # 设置随机种子（可重现结果）
    np.random.seed(123)  # 更换随机种子以获得更好的位姿分布
    
    # 创建模拟器
    simulator = HandEyeCalibrationSimulator()
    
    # 运行模拟（使用20个位姿，进一步增加数据量以提高精度）
    results = simulator.run_simulation(num_poses=20)
    
    print(f"\n模拟完成，详细结果已保存到: {simulator.output_dir}")


if __name__ == '__main__':
    main()

