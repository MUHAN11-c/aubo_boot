#!/usr/bin/env python3
import math
import threading

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
]
HOME = [0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0]
CAMERA = [
    -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
    -0.2978658676147461, 1.571584939956665, -0.2750104069709778,
]
PATH_TOLERANCE = 0.05  # 与硬件路径门一致（2026-07-25 由 0.02 放宽：
# OMPL 规划路径合法偏离直线 ~0.021 rad；且允许机械臂停在略偏线
# 的位置后仍能被授权服务召回 home）

class NamedPoseController(Node):
    def __init__(self):
        super().__init__("aubo_named_pose_controller")
        self._group = ReentrantCallbackGroup()
        self._positions = {}
        self._busy = False
        self._lock = threading.Lock()
        self._trajectory = ActionClient(
            self, FollowJointTrajectory,
            "/aubo_e5_arm_controller/follow_joint_trajectory",
            callback_group=self._group)
        self._validity = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=self._group)
        self.create_subscription(
            JointState, "/joint_states", self._joint_state,
            qos_profile_sensor_data, callback_group=self._group)
        self.create_service(
            Trigger, "/aubo/move_home",
            lambda request, response: self._request_pose("home", response),
            callback_group=self._group)
        self.create_service(
            Trigger, "/aubo/move_camera_pose",
            lambda request, response: self._request_pose("camera_pose", response),
            callback_group=self._group)
        self.get_logger().info(
            "Authorized services ready: /aubo/move_home and /aubo/move_camera_pose")

    def _joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS and math.isfinite(position):
                self._positions[name] = float(position)

    @staticmethod
    def _path_progress(position):
        # 包围盒约束（与硬件路径门一致，2026-07-25 由直线距离改）：每关节
        # 限在 HOME/CAMERA 区间 ±PATH_TOLERANCE 内 + 线投影进度 [-0.03,1.03]。
        # 放行规划器合法曲率（臂可能停在略偏直线的位置），仍能召回。
        axis = [b - a for a, b in zip(HOME, CAMERA)]
        denominator = sum(value * value for value in axis)
        progress = sum(
            (value - start) * direction
            for value, start, direction in zip(position, HOME, axis)) / denominator
        in_box = all(
            min(h, c) - PATH_TOLERANCE <= v <= max(h, c) + PATH_TOLERANCE
            for v, h, c in zip(position, HOME, CAMERA))
        valid = (-0.03 <= progress <= 1.03 and in_box)
        return valid, min(1.0, max(0.0, progress))

    @staticmethod
    def _wait_future(future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda unused: event.set())
        if not event.wait(timeout):
            raise TimeoutError("ROS operation timed out")
        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()

    def _request_pose(self, name, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "A named-pose motion is already active"
                return response
            if any(joint not in self._positions for joint in JOINTS):
                response.success = False
                response.message = "No complete joint state is available"
                return response
            self._busy = True
        threading.Thread(target=self._execute, args=(name,), daemon=True).start()
        response.success = True
        response.message = "Accepted authorized target: " + name
        return response

    def _collision_free(self, start, target):
        if not self._validity.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("MoveIt /check_state_validity is unavailable")
        for step in range(51):
            ratio = step / 50.0
            point = [a + ratio * (b - a) for a, b in zip(start, target)]
            request = GetStateValidity.Request()
            request.group_name = "manipulator_e5"
            request.robot_state.joint_state.name = list(JOINTS)
            request.robot_state.joint_state.position = point
            result = self._wait_future(self._validity.call_async(request), 2.0)
            if not result.valid:
                raise RuntimeError("Collision or invalid state at sample " + str(step))

    def _execute(self, name):
        try:
            start = [self._positions[joint] for joint in JOINTS]
            valid, unused = self._path_progress(start)
            if not valid:
                raise RuntimeError("Current pose is outside the authorized path")
            target = HOME if name == "home" else CAMERA
            self._collision_free(start, target)
            if not self._trajectory.wait_for_server(timeout_sec=3.0):
                raise RuntimeError("Joint trajectory controller is unavailable")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            initial = JointTrajectoryPoint()
            initial.positions = start
            initial.velocities = [0.0] * 6
            initial.accelerations = [0.0] * 6
            initial.time_from_start = Duration(sec=0, nanosec=100000000)
            # 起始保持段：v2.5.3 时代 SDK 冷连接首个查询/写入会阻塞数秒
            # （2026-07-24 实测 ~3.5s），保持段让 JTC 期望位置停在起点给管道
            # 升温；v1.3.1 冷启动已降至 ~0.3s，保持段 6s→3s（2026-07-25）。
            hold = JointTrajectoryPoint()
            hold.positions = start
            hold.velocities = [0.0] * 6
            hold.accelerations = [0.0] * 6
            hold.time_from_start = Duration(sec=3, nanosec=0)
            final = JointTrajectoryPoint()
            final.positions = list(target)
            final.velocities = [0.0] * 6
            final.accelerations = [0.0] * 6
            # 运动段 12s→8s（2026-07-25，用户反馈速度太慢；~0.13 rad/s
            # 仍为低速安全档；更快可用 pilz/ompl 高缩放或改本值）
            final.time_from_start = Duration(sec=8, nanosec=0)
            goal.trajectory.points = [initial, hold, final]
            handle = self._wait_future(self._trajectory.send_goal_async(goal), 3.0)
            if not handle.accepted:
                raise RuntimeError("Trajectory controller rejected the goal")
            wrapped = self._wait_future(handle.get_result_async(), 25.0)
            if wrapped.result.error_code != 0:
                raise RuntimeError("Trajectory failed with code " + str(wrapped.result.error_code))
            self.get_logger().info("Reached authorized pose: " + name)
        except Exception as error:
            self.get_logger().error("Named-pose motion rejected/failed: " + str(error))
        finally:
            with self._lock:
                self._busy = False

def main(args=None):
    rclpy.init(args=args)
    node = NamedPoseController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
