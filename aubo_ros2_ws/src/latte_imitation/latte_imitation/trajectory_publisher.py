"""从 latte-pour-demos 提取 RM65 右臂末端笛卡尔轨迹并发布。"""

import os
import numpy as np
import PyKDL
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory

from .robot_model import RobotModel
from .dataset_loader import DatasetLoader


def _default_rm65_urdf():
    try:
        share = get_package_share_directory("latte_imitation")
        return os.path.join(share, "urdf", "rm_65.urdf")
    except Exception:
        return None


def frame_to_pose(frame):
    """PyKDL.Frame -> geometry_msgs/Pose."""
    M = frame.M
    qw = np.sqrt(max(0, 1.0 + M[0, 0] + M[1, 1] + M[2, 2])) / 2.0
    qx = (M[2, 1] - M[1, 2]) / (4.0 * qw) if qw > 1e-12 else 0.0
    qy = (M[0, 2] - M[2, 0]) / (4.0 * qw) if qw > 1e-12 else 0.0
    qz = (M[1, 0] - M[0, 1]) / (4.0 * qw) if qw > 1e-12 else 0.0
    return Pose(
        position=Point(x=float(frame.p[0]), y=float(frame.p[1]), z=float(frame.p[2])),
        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
    )


class LatteImitationNode(Node):
    """加载数据集，FK 计算 RM65 末端位姿，发布为 PoseStamped + Path。"""

    def __init__(self):
        super().__init__("latte_imitation")

        self.declare_parameter("rm65_urdf", _default_rm65_urdf() or "")
        self.declare_parameter("episode_idx", 0)
        self.declare_parameter("local_parquet", "")
        self.declare_parameter("speed_scale", 1.0)

        rm65_urdf = self.get_parameter("rm65_urdf").value
        episode_idx = self.get_parameter("episode_idx").value
        local_parquet = self.get_parameter("local_parquet").value
        speed_scale = self.get_parameter("speed_scale").value

        if not rm65_urdf or not os.path.exists(rm65_urdf):
            raise FileNotFoundError(f"RM65 URDF not found: {rm65_urdf}")

        self._episode_idx = episode_idx
        self._local_parquet = local_parquet
        self._speed_scale = speed_scale
        self._rm65 = RobotModel(rm65_urdf, "base_link", "Link6")
        self._loader = DatasetLoader()

        self._pose_pub = self.create_publisher(PoseStamped, "~/ee_pose", 10)
        self._path_pub = self.create_publisher(Path, "~/ee_path", 10)

        self.get_logger().info(
            f"RM65 运动学就绪, {self._rm65.num_joints} 关节"
        )
        self._init_timer = self.create_timer(2.0, self._delayed_start)

    def _delayed_start(self):
        self._init_timer.cancel()
        self._run()

    def _run(self):
        # 1. 加载数据
        if self._local_parquet and os.path.exists(self._local_parquet):
            self.get_logger().info(f"本地加载: {self._local_parquet}")
            data = self._loader.load_from_local(self._local_parquet, self._episode_idx)
        else:
            data = self._loader.load_episode(self._episode_idx)

        joint_positions = data["joint_positions"]
        T = data["num_frames"]
        dt = data["dt"] / self._speed_scale
        self.get_logger().info(f"已加载 {T} 帧, {T * dt:.1f}s")

        # 2. FK → 笛卡尔末端位姿
        path_msg = Path()
        path_msg.header.frame_id = "base_link"

        for i in range(T):
            frame = self._rm65.fk(joint_positions[i])
            pose = frame_to_pose(frame)

            # 每帧发布 PoseStamped
            ps = PoseStamped(
                header=Header(stamp=self.get_clock().now().to_msg(),
                              frame_id="base_link"),
                pose=pose,
            )
            self._pose_pub.publish(ps)

            # 每隔 5 帧加入 Path（减少消息大小）
            if i % 5 == 0:
                path_msg.poses.append(
                    PoseStamped(header=path_msg.header, pose=pose)
                )

        self._path_pub.publish(path_msg)

        self.get_logger().info(
            f"已发布 {T} 帧 PoseStamped + Path({len(path_msg.poses)} 点)"
        )
        self.get_logger().info(
            f"起点: ({path_msg.poses[0].pose.position.x:.3f}, "
            f"{path_msg.poses[0].pose.position.y:.3f}, "
            f"{path_msg.poses[0].pose.position.z:.3f})"
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
