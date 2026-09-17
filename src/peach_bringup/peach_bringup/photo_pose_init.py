"""Initializes the mock arm to the photo pose on startup.

Mock hardware starts all joints at zero; this node sends a single-point
FollowJointTrajectory to move the arm to the SRDF ``global_photo_pose``
right after the JTC controller becomes active. Only runs in mock mode
(hardware_mode:=mock); real arms start at their physical position.
"""
from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# MUST 关节序（AGENTS 红线：shoulder→wrist3，顺序不得变）
JOINT_ORDER = [
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
]

# SRDF global_photo_pose 值（按 MUST 关节序对齐）
PHOTO_POSE = [0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621]

ACTION_NAME = '/joint_trajectory_controller/follow_joint_trajectory'


class PhotoPoseInitializer(Node):
    """Sends a one-time trajectory to position the mock arm at photo pose."""

    def __init__(self) -> None:
        """Set up action client and send initial trajectory."""
        super().__init__('peach_photo_pose_init')
        self.declare_parameter('timeout_s', 15.0)
        self._client = ActionClient(
            self, FollowJointTrajectory, ACTION_NAME)
        self._timer = self.create_timer(0.5, self._try_send)
        self._sent = False

    def _try_send(self) -> None:
        """Wait for JTC server, then send photo pose trajectory."""
        if self._sent:
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            return
        timeout = float(self.get_parameter('timeout_s').value)
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_ORDER)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in PHOTO_POSE]
        point.time_from_start.sec = 2
        traj.points.append(point)
        goal.trajectory = traj
        goal.path_tolerance = [
            JointTolerance(name=n, position=0.01) for n in JOINT_ORDER]
        self.get_logger().info(
            'Sending initial trajectory to photo pose '
            f'(mock startup, {len(JOINT_ORDER)} joints)')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_accepted)
        self._sent = True
        self._timer.cancel()

    def _on_accepted(self, future) -> None:
        """Log result and shut down."""
        handle = future.result()
        if handle is None:
            self.get_logger().warn('Photo pose init goal rejected')
            rclpy.shutdown()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_done)

    def _on_done(self, future) -> None:
        """Log final result and shut down."""
        try:
            result = future.result()
            self.get_logger().info(
                f'Photo pose init complete: error_code={result.result.error_code}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Photo pose init error: {exc}')
        rclpy.shutdown()


def main(argv=None) -> None:
    """Entry point for the photo pose initializer node."""
    rclpy.init(args=argv)
    node = PhotoPoseInitializer()
    try:
        rclpy.spin(node)
    except Exception:  # noqa: BLE001 rclpy.shutdown() 已调用
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
