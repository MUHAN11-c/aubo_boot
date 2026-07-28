#!/usr/bin/env python3
"""M2/M3 复测客户端：/plan_kinematic_path 规划 + /execute_trajectory 执行。
用法: python3 tools/m2_m3_retest.py [ompl|pilz] [home|camera_pose]
运动范围仅限 home / camera_pose（与 named_pose_controller 同一约束）。
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan

JOINTS = [
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
]
POSES = {
    "home": [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0],
    "camera_pose": [
        -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
        -0.2978658676147461, 1.571584939956665, -0.2750104069709778,
    ],
}


class RetestClient(Node):
    def __init__(self):
        super().__init__("m2_m3_retest")
        self._plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._exec = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def run(self, pipeline, planner_id, target_name):
        target = POSES[target_name]
        if not self._plan.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/plan_kinematic_path not available")
            return 2
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = "manipulator_e5"
        req.motion_plan_request.pipeline_id = pipeline
        req.motion_plan_request.planner_id = planner_id
        req.motion_plan_request.num_planning_attempts = 3
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.max_velocity_scaling_factor = getattr(
            self, "_scaling", 0.3)
        req.motion_plan_request.max_acceleration_scaling_factor = getattr(
            self, "_scaling", 0.3)
        constraints = Constraints()
        for name, position in zip(JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(constraints)

        self.get_logger().info(
            f"planning: pipeline='{pipeline or 'ompl(default)'}' "
            f"planner='{planner_id}' target={target_name}")
        future = self._plan.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        resp = future.result()
        if resp is None:
            self.get_logger().error("planning service call failed")
            return 3
        code = resp.motion_plan_response.error_code.val
        if code != 1:  # SUCCESS
            self.get_logger().error(f"planning failed, error_code={code}")
            return 4
        traj = resp.motion_plan_response.trajectory
        n = len(traj.joint_trajectory.points)
        duration = 0.0
        if n:
            t = traj.joint_trajectory.points[-1].time_from_start
            duration = t.sec + t.nanosec * 1e-9
        self.get_logger().info(f"planned: {n} points, {duration:.2f}s")

        if not self._exec.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/execute_trajectory not available")
            return 5
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info("executing...")
        send_future = self._exec.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("execute goal rejected")
            return 6
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 60.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("execute result timeout")
            return 7
        ecode = result.result.error_code.val
        self.get_logger().info(f"execution error_code={ecode} (1=SUCCESS)")
        return 0 if ecode == 1 else 8


def main():
    pipeline_arg = sys.argv[1] if len(sys.argv) > 1 else "ompl"
    target = sys.argv[2] if len(sys.argv) > 2 else "home"
    scaling = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    if target not in POSES:
        print(f"unknown target {target}; allowed: {list(POSES)}")
        return 64
    if pipeline_arg == "pilz":
        pipeline, planner_id = "pilz_industrial_motion_planner", "PTP"
    else:
        pipeline, planner_id = "ompl", ""
    rclpy.init()
    node = RetestClient()
    node._scaling = scaling
    code = node.run(pipeline, planner_id, target)
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
