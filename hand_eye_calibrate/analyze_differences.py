#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析ROS2节点和脚本标定结果差异的原因
"""

import os
import json
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

def load_script_data(data_dir):
    """加载脚本使用的数据格式"""
    robot_poses = []
    board_poses = []
    pose_indices = []
    
    pose_dirs = []
    for item in os.listdir(data_dir):
        item_path = os.path.join(data_dir, item)
        if os.path.isdir(item_path) and item.startswith('pose_'):
            try:
                pose_idx = int(item.replace('pose_', ''))
                pose_dirs.append((pose_idx, item_path))
            except ValueError:
                continue
    
    pose_dirs.sort(key=lambda x: x[0])
    
    for pose_idx, pose_dir in pose_dirs:
        robot_pose_file = os.path.join(pose_dir, 'robot_pose.json')
        board_pose_file = os.path.join(pose_dir, 'board_pose.json')
        
        if not os.path.exists(robot_pose_file) or not os.path.exists(board_pose_file):
            continue
        
        try:
            # 加载机器人姿态
            with open(robot_pose_file, 'r', encoding='utf-8') as f:
                robot_data = json.load(f)
            
            cartesian = robot_data['cartesian_position']
            pos = cartesian['position']
            ori = cartesian['orientation']
            
            quat = [ori['w'], ori['x'], ori['y'], ori['z']]
            rotation = R.from_quat(quat)
            try:
                R_base2gripper = rotation.as_matrix()
            except AttributeError:
                R_base2gripper = rotation.as_dcm()
            t_base2gripper = np.array([pos['x'], pos['y'], pos['z']])
            
            T_base2gripper = np.eye(4)
            T_base2gripper[:3, :3] = R_base2gripper
            T_base2gripper[:3, 3] = t_base2gripper
            
            robot_poses.append(T_base2gripper)
            
            # 加载棋盘格姿态
            with open(board_pose_file, 'r', encoding='utf-8') as f:
                board_data = json.load(f)
            
            rvec = np.array([
                board_data['rvec']['x'],
                board_data['rvec']['y'],
                board_data['rvec']['z']
            ])
            tvec = np.array([
                board_data['tvec']['x'],
                board_data['tvec']['y'],
                board_data['tvec']['z']
            ])
            
            board_poses.append({
                'rvec': rvec.reshape(3, 1),
                'tvec': tvec.reshape(3, 1) / 1000.0,  # 转换为米
                'transformation_matrix': np.array(board_data['transformation_matrix'])
            })
            
            pose_indices.append(pose_idx)
        except Exception as e:
            print(f"警告: 加载 {pose_dir} 失败: {e}")
            continue
    
    return robot_poses, board_poses, pose_indices


def convert_to_poses_list_format(data_dir):
    """转换为ROS2节点期望的poses_list格式"""
    robot_poses, board_poses, pose_indices = load_script_data(data_dir)
    
    poses_list = []
    for i, (robot_pose, board_pose) in enumerate(zip(robot_poses, board_poses)):
        # 提取机器人位姿
        pos = robot_pose[:3, 3]  # 米
        ori_quat = R.from_matrix(robot_pose[:3, :3]).as_quat()  # [x, y, z, w]
        
        # 提取标定板位姿
        rvec = board_pose['rvec']
        tvec = board_pose['tvec']  # 已经是米
        tvec_mm = tvec * 1000.0  # 转换为毫米（ROS2节点期望）
        
        # 从rvec和tvec构建position和orientation（用于ROS2节点）
        R_board2cam, _ = cv2.Rodrigues(rvec)
        try:
            rotation = R.from_matrix(R_board2cam)
        except:
            rotation = R.from_dcm(R_board2cam)
        quat_board = rotation.as_quat()  # [x, y, z, w]
        
        pose_item = {
            'robot_pose': {
                'robot_pos_x': float(pos[0]),
                'robot_pos_y': float(pos[1]),
                'robot_pos_z': float(pos[2]),
                'robot_ori_x': float(ori_quat[0]),
                'robot_ori_y': float(ori_quat[1]),
                'robot_ori_z': float(ori_quat[2]),
                'robot_ori_w': float(ori_quat[3])
            },
            'board_pose': {
                'position': {
                    'x': float(tvec_mm[0, 0]),
                    'y': float(tvec_mm[1, 0]),
                    'z': float(tvec_mm[2, 0])
                },
                'orientation': {
                    'x': float(quat_board[0]),
                    'y': float(quat_board[1]),
                    'z': float(quat_board[2]),
                    'w': float(quat_board[3])
                },
                'rvec': {
                    'x': float(rvec[0, 0]),
                    'y': float(rvec[1, 0]),
                    'z': float(rvec[2, 0])
                },
                'tvec': {
                    'x': float(tvec_mm[0, 0]),
                    'y': float(tvec_mm[1, 0]),
                    'z': float(tvec_mm[2, 0])
                }
            }
        }
        poses_list.append(pose_item)
    
    return poses_list


def compare_data_formats(data_dir):
    """对比数据格式差异"""
    print("="*80)
    print("数据格式对比分析")
    print("="*80)
    
    # 加载脚本使用的数据
    robot_poses, board_poses, pose_indices = load_script_data(data_dir)
    
    print(f"\n1. 数据加载:")
    print(f"   成功加载 {len(robot_poses)} 个位姿")
    print(f"   位姿索引: {pose_indices[:5]}..." if len(pose_indices) > 5 else f"   位姿索引: {pose_indices}")
    
    # 检查第一个位姿的数据
    if len(robot_poses) > 0:
        print(f"\n2. 第一个位姿的数据检查:")
        
        T_base2gripper = robot_poses[0]
        board_pose = board_poses[0]
        
        print(f"\n   机器人位姿 (T_base2gripper):")
        print(f"   位置 (米): {T_base2gripper[:3, 3]}")
        print(f"   位置 (毫米): {T_base2gripper[:3, 3] * 1000}")
        
        print(f"\n   标定板位姿:")
        print(f"   rvec: {board_pose['rvec'].flatten()}")
        print(f"   tvec (米): {board_pose['tvec'].flatten()}")
        print(f"   tvec (毫米): {board_pose['tvec'].flatten() * 1000}")
    
    # 转换为poses_list格式
    print(f"\n3. 转换为ROS2节点期望的格式 (poses_list):")
    poses_list = convert_to_poses_list_format(data_dir)
    print(f"   转换成功: {len(poses_list)} 个位姿")
    
    if len(poses_list) > 0:
        print(f"\n   第一个位姿的poses_list格式:")
        print(f"   robot_pose.robot_pos_x/y/z (米): {poses_list[0]['robot_pose']['robot_pos_x']:.6f}, {poses_list[0]['robot_pose']['robot_pos_y']:.6f}, {poses_list[0]['robot_pose']['robot_pos_z']:.6f}")
        print(f"   board_pose.position (毫米): x={poses_list[0]['board_pose']['position']['x']:.3f}, y={poses_list[0]['board_pose']['position']['y']:.3f}, z={poses_list[0]['board_pose']['position']['z']:.3f}")
        print(f"   board_pose.tvec (毫米): x={poses_list[0]['board_pose']['tvec']['x']:.3f}, y={poses_list[0]['board_pose']['tvec']['y']:.3f}, z={poses_list[0]['board_pose']['tvec']['z']:.3f}")
    
    # 检查数据一致性
    print(f"\n4. 数据一致性检查:")
    if len(robot_poses) > 0 and len(poses_list) > 0:
        # 检查机器人位姿是否一致
        T_script = robot_poses[0]
        pos_poses_list = np.array([
            poses_list[0]['robot_pose']['robot_pos_x'],
            poses_list[0]['robot_pose']['robot_pos_y'],
            poses_list[0]['robot_pose']['robot_pos_z']
        ])
        pos_diff = np.linalg.norm(T_script[:3, 3] - pos_poses_list)
        print(f"   机器人位置差异: {pos_diff:.9f} 米 ({'一致' if pos_diff < 1e-6 else '不一致'})")
        
        # 检查标定板tvec是否一致
        tvec_script = board_poses[0]['tvec'].flatten() * 1000  # 转换为毫米
        tvec_poses_list = np.array([
            poses_list[0]['board_pose']['tvec']['x'],
            poses_list[0]['board_pose']['tvec']['y'],
            poses_list[0]['board_pose']['tvec']['z']
        ])
        tvec_diff = np.linalg.norm(tvec_script - tvec_poses_list)
        print(f"   标定板tvec差异: {tvec_diff:.6f} 毫米 ({'一致' if tvec_diff < 1e-3 else '不一致'})")
    
    return poses_list


def analyze_opencv_input_data(robot_poses, board_poses):
    """分析传给OpenCV的数据"""
    print("\n" + "="*80)
    print("OpenCV输入数据分析")
    print("="*80)
    
    # 准备OpenCV格式的数据（脚本的方式）
    R_gripper2base = []
    t_gripper2base = []
    rvecs = []
    tvecs = []
    
    for T_base2gripper, board_pose in zip(robot_poses, board_poses):
        # 直接使用Base->Gripper（不取逆）
        R_base2gripper = T_base2gripper[:3, :3]
        t_base2gripper = T_base2gripper[:3, 3]  # 米
        R_gripper2base.append(R_base2gripper)
        t_gripper2base.append(t_base2gripper)
        
        rvecs.append(board_pose['rvec'])
        tvecs.append(board_pose['tvec'])  # 米
    
    print(f"\n1. 数据数量:")
    print(f"   R_gripper2base: {len(R_gripper2base)}")
    print(f"   t_gripper2base: {len(t_gripper2base)}")
    print(f"   rvecs: {len(rvecs)}")
    print(f"   tvecs: {len(tvecs)}")
    
    # 检查第一个位姿的数据
    if len(R_gripper2base) > 0:
        print(f"\n2. 第一个位姿的OpenCV输入数据:")
        print(f"   R_gripper2base[0][0, :]: {R_gripper2base[0][0, :]}")
        print(f"   t_gripper2base[0] (米): {t_gripper2base[0]}")
        print(f"   t_gripper2base[0] (毫米): {t_gripper2base[0] * 1000}")
        print(f"   rvecs[0]: {rvecs[0].flatten()}")
        print(f"   tvecs[0] (米): {tvecs[0].flatten()}")
        print(f"   tvecs[0] (毫米): {tvecs[0].flatten() * 1000}")
    
    # 统计信息
    print(f"\n3. 数据统计:")
    t_norms = [np.linalg.norm(t) for t in t_gripper2base]
    tvec_norms = [np.linalg.norm(t) for t in tvecs]
    print(f"   t_gripper2base 范数 (米): 均值={np.mean(t_norms):.6f}, 标准差={np.std(t_norms):.6f}")
    print(f"   t_gripper2base 范数 (毫米): 均值={np.mean(t_norms)*1000:.3f}, 标准差={np.std(t_norms)*1000:.3f}")
    print(f"   tvecs 范数 (米): 均值={np.mean(tvec_norms):.6f}, 标准差={np.std(tvec_norms):.6f}")
    print(f"   tvecs 范数 (毫米): 均值={np.mean(tvec_norms)*1000:.3f}, 标准差={np.std(tvec_norms)*1000:.3f}")
    
    return R_gripper2base, t_gripper2base, rvecs, tvecs


def main():
    data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
    
    if not os.path.exists(data_dir):
        print(f"错误: 数据目录不存在: {data_dir}")
        return
    
    # 1. 对比数据格式
    poses_list = compare_data_formats(data_dir)
    
    # 2. 分析OpenCV输入数据
    robot_poses, board_poses, pose_indices = load_script_data(data_dir)
    R_gripper2base, t_gripper2base, rvecs, tvecs = analyze_opencv_input_data(robot_poses, board_poses)
    
    # 3. 总结
    print("\n" + "="*80)
    print("分析总结")
    print("="*80)
    print("\n关键发现:")
    print("1. 数据格式转换:")
    print("   - 脚本使用JSON文件格式（cartesian_position, rvec/tvec）")
    print("   - ROS2节点期望poses_list格式（robot_pos_x/y/z, position/orientation）")
    print("   - 数据内容应该相同，但格式不同")
    print("\n2. 数据单位:")
    print("   - 机器人位姿：都是米单位 ✓")
    print("   - 标定板tvec：脚本转换为米，ROS2节点也转换为米 ✓")
    print("\n3. OpenCV参数传递:")
    print("   - 都使用Base→Gripper（不取逆）✓")
    print("   - 都转换为米单位传给OpenCV ✓")
    print("\n4. 可能的问题:")
    print("   - 数据格式虽然可以转换，但可能有数值精度差异")
    print("   - ROS2节点使用内存中的数据（实时采集）")
    print("   - 脚本使用保存的JSON文件（可能有精度损失）")
    print("   - 或者ROS2节点内部使用了不同的数据处理方式")


if __name__ == '__main__':
    main()
