"""加载笛卡尔轨迹，Aubo E5 IK → ROS2 JointTrajectory 执行。"""

import os

import numpy as np
import PyKDL
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from ament_index_python.packages import get_package_share_directory

from .robot_model import RobotModel
from .trajectory import CartesianTrajectory


def _cartesian_resource_dir():
    try:
        share = get_package_share_directory("latte_imitation")
        return os.path.join(share, "resource", "cartesian")
    except Exception:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_dir, "resource", "cartesian")


def _resolve_urdf(pkg_name, subpath, param_val):
    if param_val and os.path.exists(param_val):
        return param_val
    try:
        return os.path.join(get_package_share_directory(pkg_name), subpath)
    except Exception:
        return None


class LatteImitationNode(Node):
    """加载笛卡尔轨迹 → Aubo IK → JointTrajectory → Action 执行。"""

    def __init__(self):
        super().__init__("latte_imitation")

        self.declare_parameter("episode_idx", 0)
        self.declare_parameter("arm", "right")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("mode", "debug")
        self.declare_parameter("pos_only", True)
        self.declare_parameter("aubo_urdf", "")

        self._episode_idx = self.get_parameter("episode_idx").value
        self._arm = self.get_parameter("arm").value
        self._speed_scale = self.get_parameter("speed_scale").value
        self._mode = self.get_parameter("mode").value
        self._pos_only = self.get_parameter("pos_only").value

        # 发布者
        self._rm65_pose_pub = self.create_publisher(PoseStamped, "~/ee_pose", 10)
        self._rm65_path_pub = self.create_publisher(Path, "~/ee_path", 10)

        self._aubo = None
        self._aubo_pose_pub = None
        self._aubo_path_pub = None
        self._traj_pub = None
        self._action_client = None

        if self._mode == "action":
            self._init_action_mode()

        self._init_timer = self.create_timer(2.0, self._delayed_start)

    def _init_action_mode(self):
        aubo_urdf = _resolve_urdf("aubo_description", "urdf/aubo_e5_10.urdf",
                                   self.get_parameter("aubo_urdf").value)
        if not aubo_urdf:
            self.get_logger().error("Aubo URDF 未找到，退回到 debug")
            self._mode = "debug"
            return

        self._aubo = RobotModel(aubo_urdf, "base_link", "wrist3_Link")
        self.get_logger().info(f"Aubo 就绪, {self._aubo.num_joints} 关节: {self._aubo.joint_names}")

        self._aubo_pose_pub = self.create_publisher(PoseStamped, "~/aubo_ee_pose", 10)
        self._aubo_path_pub = self.create_publisher(Path, "~/aubo_ee_path", 10)
        self._traj_pub = self.create_publisher(RosJointTrajectory, "~/joint_path_command", 10)

        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def _delayed_start(self):
        self._init_timer.cancel()
        self._run()

    # ── 主流程 ──────────────────────────────────────────────

    def _run(self):
        try:
            self._run_impl()
        except Exception as e:
            self.get_logger().error(f"_run 异常: {e}")

    def _run_impl(self):
        # 1. 加载笛卡尔轨迹
        cart = self._load_cartesian()
        if cart is None:
            return

        dt = cart.dt / self._speed_scale
        self.get_logger().info(
            f"Episode {cart.episode_idx} ({self._arm}): {cart.num_frames} 帧, "
            f"路径长 {cart.path_length():.2f}m"
        )

        # 2. 发布笛卡尔位姿
        self._publish_poses(self._rm65_pose_pub, cart)
        self._rm65_path_pub.publish(cart.to_ros2_path())

        if self._mode != "action" or self._aubo is None:
            self.get_logger().info("Debug 完成")
            return

        # 3. 笛卡尔 → KDL Frame 列表 → Aubo IK
        self.get_logger().info(f"Aubo IK (pos_only={self._pos_only})...")
        target_frames = [_pose_to_kdl_frame(cart.to_pose(i)) for i in range(cart.num_frames)]

        q_traj, failures = self._aubo.ik_trajectory(
            target_frames, pos_only=self._pos_only, n_restarts_first=20,
        )
        n_ok = cart.num_frames - len(failures)
        self.get_logger().info(f"IK: {n_ok}/{cart.num_frames} 收敛")
        if failures:
            self.get_logger().warn(f"失败帧: {failures[:10]}{'...' if len(failures)>10 else ''}")

        # 4. FK 验证（失败帧使用 NaN 保持长度一致）
        aubo_pos = np.full((cart.num_frames, 3), np.nan)
        aubo_quat = np.full((cart.num_frames, 4), np.nan)
        aubo_quat[:, 3] = 1.0
        ok_mask = np.ones(cart.num_frames, dtype=bool)
        ok_mask[failures] = False
        ok_indices = np.where(ok_mask)[0]

        for j, i in enumerate(ok_indices):
            frame = self._aubo.fk(q_traj[j])
            aubo_pos[i] = [frame.p[0], frame.p[1], frame.p[2]]
            M = frame.M
            qw = np.sqrt(max(0, 1.0 + M[0,0] + M[1,1] + M[2,2])) / 2.0
            if qw > 1e-12:
                aubo_quat[i] = [(M[2,1]-M[1,2])/(4.0*qw), (M[0,2]-M[2,0])/(4.0*qw),
                                 (M[1,0]-M[0,1])/(4.0*qw), qw]

        aubo_cart = CartesianTrajectory(
            positions=aubo_pos, orientations=aubo_quat,
            timestamps=cart.timestamps, dt=cart.dt,
        )
        self._publish_poses(self._aubo_pose_pub, aubo_cart)
        self._aubo_path_pub.publish(aubo_cart.to_ros2_path())

        # 5. 构建 JointTrajectory → Action（跳过失败帧）
        traj_msg = RosJointTrajectory()
        traj_msg.joint_names = self._aubo.joint_names
        for j, i in enumerate(ok_indices):
            pt = JointTrajectoryPoint()
            pt.positions = q_traj[j].tolist()
            t = i * dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj_msg.points.append(pt)

        self._traj_pub.publish(traj_msg)
        self.get_logger().info(f"JointTrajectory ({len(traj_msg.points)} 点) 已发布")
        self._send_action_goal(traj_msg)

    # ── 加载 ────────────────────────────────────────────────

    def _load_cartesian(self) -> CartesianTrajectory | None:
        res_dir = _cartesian_resource_dir()
        path = os.path.join(res_dir, self._arm,
                           f"episode_{self._episode_idx:06d}.npz")
        if not os.path.exists(path):
            self.get_logger().error(f"未找到: {path}")
            return None
        return CartesianTrajectory.load(path)

    # ── 发布 & Action ───────────────────────────────────────

    @staticmethod
    def _publish_poses(pub, cart: CartesianTrajectory, step: int = 5):
        now = rclpy.clock.Clock().now().to_msg()
        for i in range(0, cart.num_frames, step):
            pub.publish(cart.to_pose_stamped(i, stamp=now))

    def _send_action_goal(self, traj_msg):
        if self._action_client is None:
            return
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Action 服务器不可用")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj_msg
        self.get_logger().info("发送 Action goal...")
        self._action_client.send_goal_async(
            goal, feedback_callback=lambda _: None
        ).add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Action goal 异常: {e}")
            return
        if not handle.accepted:
            self.get_logger().error("Action 被拒绝")
            return
        self.get_logger().info("Action 已接受")
        handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            code = future.result().result.error_code
            self.get_logger().info(f"Action 完成: error_code={code}")
        except Exception as e:
            self.get_logger().error(f"Action result 异常: {e}")


def _pose_to_kdl_frame(pose):
    """ROS2 Pose → PyKDL.Frame。"""
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ),
        PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z),
    )


def main(args=None):
    rclpy.init(args=args)
    node = LatteImitationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
