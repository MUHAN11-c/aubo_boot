#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比脚本和ROS2节点使用的数据是否一致
"""

import os
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

def load_script_data(data_dir):
    """加载脚本使用的数据"""
    robot_poses = []
    board_poses = []
    
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
        
        # 加载机器人位姿
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
        
        robot_poses.append({
            'position': t_base2gripper.tolist(),
            'quaternion': quat,
            'rotation_matrix': R_base2gripper.tolist()
        })
        
        # 加载标定板位姿
        with open(board_pose_file, 'r', encoding='utf-8') as f:
            board_data = json.load(f)
        
        rvec = np.array([board_data['rvec']['x'], board_data['rvec']['y'], board_data['rvec']['z']])
        tvec = np.array([board_data['tvec']['x'], board_data['tvec']['y'], board_data['tvec']['z']])
        
        board_poses.append({
            'rvec': rvec.tolist(),
            'tvec': tvec.tolist(),
            'transformation_matrix': board_data['transformation_matrix']
        })
    
    return robot_poses, board_poses

def print_comparison(script_data, ros2_data_name="ROS2节点数据"):
    """打印对比结果"""
    robot_poses_script, board_poses_script = script_data
    
    print("="*80)
    print("数据对比分析")
    print("="*80)
    print(f"\n脚本数据：{len(robot_poses_script)} 个位姿")
    print(f"\n前3个位姿的机器人位置（米）：")
    for i in range(min(3, len(robot_poses_script))):
        pos = robot_poses_script[i]['position']
        print(f"  位姿 {i}: [{pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}]")
    
    print(f"\n前3个位姿的标定板tvec（毫米）：")
    for i in range(min(3, len(board_poses_script))):
        tvec = board_poses_script[i]['tvec']
        print(f"  位姿 {i}: [{tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}]")
    
    print("\n" + "="*80)
    print("建议检查项：")
    print("1. ROS2节点实际使用的数据是否与脚本数据相同")
    print("2. 数据格式是否一致（robot_pose, board_pose结构）")
    print("3. 单位是否一致（机器人位姿：米，标定板tvec：毫米）")
    print("4. 位姿顺序是否一致")
    print("="*80)

if __name__ == '__main__':
    data_dir = '/home/mu/IVG/hand_eye_calibrate/collect_data'
    
    print("加载脚本使用的数据...")
    script_data = load_script_data(data_dir)
    
    print_comparison(script_data)
    
    print("\n要检查ROS2节点使用的数据，请：")
    print("1. 查看ROS2节点的日志，确认使用的数据来源")
    print("2. 或者在前端界面查看发送给后端的数据")
    print("3. 或者查看保存的标定验证数据文件")
