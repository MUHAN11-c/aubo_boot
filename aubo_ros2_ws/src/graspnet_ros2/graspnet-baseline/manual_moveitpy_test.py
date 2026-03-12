#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手动测试 grasp_motion_controller_moveit2 的脚本。

用法示例：
  1) 关节测试（推荐，稳定）:
     python3 manual_moveitpy_test.py --mode joints --joint-index 2 --joint-delta 0.04

  2) 位姿测试（可能因 IK 或约束失败）:
     python3 manual_moveitpy_test.py --mode pose --pose-dz 0.005
"""

from __future__ import annotations

import argparse
import copy
import sys
from typing import List

import rclpy
from sensor_msgs.msg import JointState

from graspnet_ros2.grasp_motion_controller_moveit2 import GraspMotionControllerMoveIt2


DEFAULT_JOINT_NAMES: List[str] = [
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MoveItPy 手动测试脚本")
    parser.add_argument("--mode", choices=["joints", "pose"], default="joints", help="测试模式")
    parser.add_argument("--joint-index", type=int, default=2, help="关节测试时要偏移的关节索引")
    parser.add_argument("--joint-delta", type=float, default=0.04, help="关节测试时偏移量(rad)")
    parser.add_argument("--pose-dz", type=float, default=0.005, help="位姿测试时 Z 方向偏移(m)")
    parser.add_argument("--vel", type=float, default=0.10, help="速度缩放因子")
    parser.add_argument("--acc", type=float, default=0.10, help="加速度缩放因子")
    parser.add_argument("--base-frame", type=str, default="", help="可选覆盖 base_frame，如 world")
    return parser.parse_args()


def wait_joint_states(node: GraspMotionControllerMoveIt2, timeout_sec: float = 5.0) -> JointState | None:
    holder: dict[str, JointState | None] = {"msg": None}

    def cb(msg: JointState) -> None:
        holder["msg"] = msg

    _sub = node.create_subscription(JointState, "/joint_states", cb, 10)

    loop_count = int(timeout_sec / 0.05)
    for _ in range(loop_count):
        if holder["msg"] is not None:
            return holder["msg"]
        rclpy.spin_once(node, timeout_sec=0.05)
    return None


def main() -> int:
    args = parse_args()
    print(f"[TEST] args={args}")

    rclpy.init()
    node = GraspMotionControllerMoveIt2("manual_moveitpy_test")
    node.joint_names = DEFAULT_JOINT_NAMES

    if args.base_frame:
        node.base_frame = args.base_frame
        print(f"[TEST] override base_frame={node.base_frame}")

    try:
        pose = node.get_current_ee_pose()
        print(f"[TEST] get_current_ee_pose => {pose is not None}")
        if pose is not None:
            print(
                "[TEST] current pose xyz="
                f"({pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}) "
                "q="
                f"({pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
                f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f})"
            )

        if args.mode == "joints":
            msg = wait_joint_states(node, timeout_sec=5.0)
            if msg is None:
                print("[TEST] ERROR: 5s 内未收到 /joint_states")
                return 2

            idx_map = {name: i for i, name in enumerate(msg.name)}
            current = [float(msg.position[idx_map[name]]) for name in DEFAULT_JOINT_NAMES]
            target = current.copy()

            if args.joint_index < 0 or args.joint_index >= len(target):
                print(f"[TEST] ERROR: joint-index 超范围: {args.joint_index}")
                return 2

            target[args.joint_index] += args.joint_delta
            print(f"[TEST] current_joints={current}")
            print(f"[TEST] target_joints={target}")

            ok = node.move_to_joints(
                target,
                velocity_factor=args.vel,
                acceleration_factor=args.acc,
            )
            print(f"[TEST] move_to_joints result={ok}")
            return 0 if ok else 1

        # pose 模式
        if pose is None:
            print("[TEST] ERROR: 无法获取当前位姿，无法执行 pose 测试")
            return 2

        target_pose = copy.deepcopy(pose)
        target_pose.position.z += args.pose_dz
        print(
            "[TEST] target_pose xyz="
            f"({target_pose.position.x:.6f}, {target_pose.position.y:.6f}, {target_pose.position.z:.6f})"
        )
        ok = node.move_to_pose(
            target_pose,
            velocity_factor=args.vel,
            acceleration_factor=args.acc,
        )
        print(f"[TEST] move_to_pose result={ok}")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
