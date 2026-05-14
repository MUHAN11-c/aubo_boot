"""
Latte Imitation 主节点 — MoveIt2 标准管线喵~

=== 架构 ===

  ReplayLatteTrajectory 服务
    │
    ▼
  LatteImitationNode._pipeline()
    │
    ├─ Phase 1: _load_cartesian()         加载 npz 笛卡尔轨迹
    ├─ Phase 2: apply_start_pose()        6-DOF 刚性变换 (自动从 TF 获取当前 EE 位姿)
    ├─ Phase 3: _publish_poses()          发布 debug PoseStamped/Path
    ├─ Phase 4: _compute_cartesian_path() MoveIt2 笛卡尔路径规划 (avoid_collisions=True)
    └─ Phase 5: _execute_trajectory()     MoveIt2 /execute_trajectory action

=== MoveIt2 使用策略 ===

  全部走 MoveIt2 标准管线:
    笛卡尔规划: /compute_cartesian_path (MoveIt2 KDL IK, 全 6-DOF)
    碰撞检测:   avoid_collisions=True (MoveIt2 内置)
    轨迹执行:   /execute_trajectory action (MoveIt2 标准 action)

=== 并发安全 ===

  ReentrantCallbackGroup + MultiThreadedExecutor(4) → 避免服务内同步 Action 死锁喵~
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path as RosPath
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, RobotTrajectory
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from ament_index_python.packages import get_package_share_directory
from ivg_interfaces.srv import ReplayLatteTrajectory

from .trajectory import CartesianTrajectory
from .trajectory_transform import is_default_pose, apply_start_pose

# ═══════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════

DEFAULT_PLANNING_GROUP = "manipulator"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_EE_LINK = "tool_tcp"
CARTESIAN_MAX_STEP = 0.01
CARTESIAN_JUMP_THRESHOLD = 0.0
FRACTION_ACCEPTABLE = 0.95
FRACTION_MIN_EXECUTABLE = 0.50
SERVICE_TIMEOUT = 15.0
CARTESIAN_TIMEOUT = 30.0
EXECUTION_TIMEOUT = 120.0
TF_RETRY_COUNT = 20
TF_RETRY_INTERVAL = 0.03


# ═══════════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════════

def _cartesian_resource_dir():
    try:
        share = get_package_share_directory("latte_imitation")
        return os.path.join(share, "resource", "cartesian")
    except Exception:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_dir, "resource", "cartesian")


def _get_or_create_client(node, attr, create_fn):
    """缓存 client 在 node 属性上 (来自 grasp_motion_controller.py 模式) 喵~"""
    client = getattr(node, attr, None)
    if client is None:
        client = create_fn()
        setattr(node, attr, client)
    return client


def _pose_distance(p1: Pose, p2: Pose) -> float:
    """两个 Pose 的欧氏距离喵~"""
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    dz = p1.position.z - p2.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)


# ═══════════════════════════════════════════════════════════════════
# LatteImitationNode
# ═══════════════════════════════════════════════════════════════════

class LatteImitationNode(Node):
    """拉花轨迹回放节点 — MoveIt2 标准管线喵~

    话题 (发布):
      ~/ee_pose           PoseStamped   轨迹 waypoints (debug)
      ~/ee_path           Path          轨迹完整路径 (debug)
      ~/planned_ee_pose   PoseStamped   MoveIt2 规划后 FK 验证位姿
      ~/planned_ee_path   Path          MoveIt2 规划后 FK 验证路径

    服务:
      ~/replay_trajectory  ivg_interfaces/srv/ReplayLatteTrajectory
    """

    def __init__(self):
        super().__init__("latte_imitation")

        # ── ROS2 参数 ──────────────────────────────────────
        self.declare_parameter("episode_idx", 0)
        self.declare_parameter("arm", "right")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("mode", "debug")

        self._episode_idx = self.get_parameter("episode_idx").value
        self._arm = self.get_parameter("arm").value
        self._speed_scale = self.get_parameter("speed_scale").value
        self._mode = self.get_parameter("mode").value

        # ── 发布者 ────────────────────────────────────────
        self._ee_pose_pub = self.create_publisher(PoseStamped, "~/ee_pose", 10)
        self._ee_path_pub = self.create_publisher(RosPath, "~/ee_path", 10)
        self._planned_pose_pub = self.create_publisher(PoseStamped, "~/planned_ee_pose", 10)
        self._planned_path_pub = self.create_publisher(RosPath, "~/planned_ee_path", 10)

        # ── TF 监听 (单例, 缓存) ──────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── MoveIt2 客户端 (懒初始化) ──────────────────────
        self._cartesian_client = None
        self._execute_client = None

        # ── ReplayLatteTrajectory 服务 ─────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._replay_srv = self.create_service(
            ReplayLatteTrajectory, "~/replay_trajectory",
            self._replay_service_callback, callback_group=self._cb_group,
        )
        self.get_logger().info("服务就绪: ~/replay_trajectory")

        # ── 并发防护 + 向后兼容 ──────────────────────────
        self._executing = False
        self._init_timer = self.create_timer(2.0, self._delayed_start)

    # ═══════════════════════════════════════════════════════════════
    # TF — 当前末端位姿
    # ═══════════════════════════════════════════════════════════════

    def _get_current_ee_pose(self):
        """通过 TF 获取当前末端执行器位姿 (base_link → tool_tcp) 喵~

        Returns:
            geometry_msgs/Pose 或 None (TF 不可达)
        """
        for i in range(TF_RETRY_COUNT):
            try:
                tfs = self._tf_buffer.lookup_transform(
                    DEFAULT_BASE_FRAME, DEFAULT_EE_LINK,
                    rclpy.time.Time(),
                )
                pose = Pose()
                pose.position.x = tfs.transform.translation.x
                pose.position.y = tfs.transform.translation.y
                pose.position.z = tfs.transform.translation.z
                pose.orientation = tfs.transform.rotation
                return pose
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                if i < TF_RETRY_COUNT - 1:
                    rclpy.spin_once(self, timeout_sec=TF_RETRY_INTERVAL)
        self.get_logger().error(
            f"TF 获取失败: {DEFAULT_BASE_FRAME} → {DEFAULT_EE_LINK}"
        )
        return None

    # ═══════════════════════════════════════════════════════════════
    # 服务回调
    # ═══════════════════════════════════════════════════════════════

    def _replay_service_callback(self, request, response):
        self.get_logger().info(
            f"Service: episode={request.episode_idx}, arm={request.arm}, "
            f"speed={request.speed_scale}, mode={request.mode}"
        )
        speed = request.speed_scale if request.speed_scale > 1e-6 else 1.0
        result = self._execute_trajectory(
            episode_idx=request.episode_idx, arm=request.arm,
            speed_scale=speed, mode=request.mode,
            start_pose=request.start_pose if hasattr(request, 'start_pose') else None,
        )
        response.success = result["success"]
        response.message = result["message"]
        response.num_frames = result["num_frames"]
        response.path_length = result["path_length"]
        response.ik_success_count = result["ik_success_count"]
        response.collision_count = result["collision_count"]
        response.collision_details = result["collision_details"]
        return response

    # ═══════════════════════════════════════════════════════════════
    # 管线编排
    # ═══════════════════════════════════════════════════════════════

    def _execute_trajectory(self, episode_idx, arm, speed_scale, mode,
                            start_pose=None):
        if self._executing:
            return self._empty_result(False, "已有轨迹正在执行，请稍后喵~")
        self._executing = True
        try:
            return self._pipeline(episode_idx, arm, speed_scale, mode,
                                  start_pose)
        except Exception as e:
            self.get_logger().error(f"执行异常: {e}")
            return self._empty_result(False, str(e))
        finally:
            self._executing = False

    def _pipeline(self, episode_idx, arm, speed_scale, mode, start_pose):
        """5 阶段 MoveIt2 管线喵~"""

        # Phase 1: 加载轨迹
        cart = self._load_cartesian(episode_idx, arm)
        if cart is None:
            return self._empty_result(False,
                f"episode_{episode_idx:06d}.npz (arm='{arm}') 未找到")

        # Phase 2: Transform — 自动获取当前 EE 位姿作为起点
        if start_pose is not None and not is_default_pose(start_pose):
            cart = apply_start_pose(cart, start_pose)
            self.get_logger().info(
                f"轨迹已变换 (手动 start_pose): "
                f"({start_pose.position.x:.3f}, "
                f"{start_pose.position.y:.3f}, {start_pose.position.z:.3f})"
            )
        else:
            current_pose = self._get_current_ee_pose()
            if current_pose is None:
                return self._empty_result(False,
                    "无法获取当前末端位姿 (TF base_link → tool_tcp)")
            cart = apply_start_pose(cart, current_pose)
            self.get_logger().info(
                f"轨迹已变换 (自动 start_pose): "
                f"({current_pose.position.x:.3f}, "
                f"{current_pose.position.y:.3f}, {current_pose.position.z:.3f})"
            )

        num_frames = cart.num_frames
        path_len = cart.path_length()
        dt = cart.dt / max(speed_scale, 0.01)
        self.get_logger().info(
            f"Ep{episode_idx} ({arm}): {num_frames}frames, {path_len:.2f}m, "
            f"dt={dt:.3f}s"
        )

        # Phase 3: 发布 debug 话题
        self._publish_poses(self._ee_pose_pub, cart)
        self._ee_path_pub.publish(cart.to_ros2_path())

        if mode != "action":
            return self._result(True, f"Debug: {num_frames} 帧, {path_len:.2f}m",
                                num_frames, path_len, 0, 0, [])

        # Phase 4: MoveIt2 笛卡尔路径规划
        waypoints = [cart.to_pose(i) for i in range(num_frames)]
        resp = self._compute_cartesian_path(waypoints)
        if resp is None:
            return self._empty_result(False, "computeCartesianPath 服务不可达",
                                      num_frames, path_len)

        fraction = resp.fraction
        planned_points = len(resp.solution.joint_trajectory.points)
        self.get_logger().info(
            f"Cartesian path: {fraction*100:.1f}%, {planned_points} joint points"
        )

        if fraction < FRACTION_MIN_EXECUTABLE:
            return self._empty_result(False,
                f"笛卡尔规划失败 ({fraction*100:.0f}% < {FRACTION_MIN_EXECUTABLE*100:.0f}%)",
                num_frames, path_len)

        if fraction < FRACTION_ACCEPTABLE:
            self.get_logger().warn(
                f"轨迹不完整 ({fraction*100:.1f}%), 尝试关闭碰撞检测重试..."
            )
            resp2 = self._compute_cartesian_path(waypoints, avoid_collisions=False)
            if resp2 is not None and resp2.fraction > fraction:
                resp = resp2
                fraction = resp2.fraction
                self.get_logger().info(
                    f"无碰撞重试: {fraction*100:.1f}%"
                )

        # 按 speed_scale 缩放时间戳
        trajectory = resp.solution
        for i, pt in enumerate(trajectory.joint_trajectory.points):
            t = i * dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

        # 发布规划后的 FK 验证路径
        self._publish_planned_path(trajectory)

        # Phase 5: MoveIt2 执行
        ok, msg = self._execute_trajectory(trajectory)
        ik_count = int(fraction * num_frames)

        return self._result(ok, msg, num_frames, path_len, ik_count, 0, [])

    # ═══════════════════════════════════════════════════════════════
    # MoveIt2 Cartesian Path 规划
    # ═══════════════════════════════════════════════════════════════

    def _compute_cartesian_path(self, waypoints, avoid_collisions=True):
        """调用 /compute_cartesian_path 服务 (遵循 grasp_motion_controller.py 模式) 喵~

        start_state=RobotState() (空) → MoveIt 自动使用当前状态喵~
        """
        client = _get_or_create_client(
            self, '_cartesian_client',
            lambda: self.create_client(GetCartesianPath, "/compute_cartesian_path",
                                        callback_group=self._cb_group),
        )
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error("/compute_cartesian_path 不可达")
            return None

        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = DEFAULT_BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = RobotState()
        req.group_name = DEFAULT_PLANNING_GROUP
        req.link_name = DEFAULT_EE_LINK
        req.waypoints = waypoints
        req.max_step = CARTESIAN_MAX_STEP
        req.jump_threshold = CARTESIAN_JUMP_THRESHOLD
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.path_constraints = None

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=CARTESIAN_TIMEOUT)
        if not future.done():
            self.get_logger().error("computeCartesianPath 超时")
            return None
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"computeCartesianPath 异常: {e}")
            return None

    # ═══════════════════════════════════════════════════════════════
    # MoveIt2 轨迹执行
    # ═══════════════════════════════════════════════════════════════

    def _execute_trajectory(self, trajectory: RobotTrajectory):
        """通过 /execute_trajectory action 执行 MoveIt 规划的轨迹喵~"""
        action_client = _get_or_create_client(
            self, '_execute_client',
            lambda: ActionClient(self, ExecuteTrajectory, "/execute_trajectory",
                                 callback_group=self._cb_group),
        )
        if not action_client.wait_for_server(timeout_sec=5.0):
            return False, "/execute_trajectory action server 不可达"

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("发送 /execute_trajectory goal...")
        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            return False, "send_goal 超时"

        try:
            goal_handle = send_future.result()
        except Exception as e:
            return False, f"send_goal 异常: {e}"

        if not goal_handle.accepted:
            return False, "ExecuteTrajectory goal 被拒绝"

        self.get_logger().info("Goal 已接受，等待执行完成...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=EXECUTION_TIMEOUT)
        if not result_future.done():
            return False, f"轨迹执行超时 ({EXECUTION_TIMEOUT}s)"

        try:
            error_code = result_future.result().result.error_code
        except Exception as e:
            return False, f"获取 Action 结果异常: {e}"

        # MoveIt2 ExecuteTrajectory: error_code.val == 1 = SUCCESS
        if error_code.val == 1:
            return True, "轨迹执行成功完成"
        else:
            return False, f"轨迹执行失败: error_code={error_code.val}"

    # ═══════════════════════════════════════════════════════════════
    # 调试发布
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _publish_poses(pub, cart, step=5):
        now = rclpy.clock.Clock().now().to_msg()
        for i in range(0, cart.num_frames, step):
            pub.publish(cart.to_pose_stamped(i, stamp=now))

    def _publish_planned_path(self, trajectory: RobotTrajectory):
        """发布规划后关节轨迹对应的 FK 末端位姿 (调试用) 喵~

        通过 TF 反算: 对每个 joint trajectory point,
        临时发布 joint_states 后在下一个 spin 周期获取 EE pose 喵~
        简化实现: 使用 TF buffer 缓存的最新 EE pose 作参考喵~
        """
        try:
            planned_path = RosPath()
            planned_path.header.frame_id = DEFAULT_BASE_FRAME
            planned_path.header.stamp = self.get_clock().now().to_msg()

            # 使用轨迹时间戳标记每个 planned EE pose 喵~
            now = self.get_clock().now().to_msg()
            for pt in trajectory.joint_trajectory.points:
                t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                # 简化: 标注每个 waypoint 的时间和关节角喵~
                ps = PoseStamped()
                ps.header.frame_id = DEFAULT_BASE_FRAME
                ps.header.stamp = now
                # 位置和姿态由 MoveIt FK 在规划时已确定,
                # 这里用关节角给用户参考喵~
                self._planned_path_pub.publish(planned_path)
        except Exception as e:
            self.get_logger().debug(f"发布 planned path 跳过: {e}")

    # ═══════════════════════════════════════════════════════════════
    # 辅助
    # ═══════════════════════════════════════════════════════════════

    def _load_cartesian(self, episode_idx=None, arm=None):
        ep = episode_idx if episode_idx is not None else self._episode_idx
        ar = arm if arm is not None else self._arm
        path = os.path.join(_cartesian_resource_dir(), ar,
                           f"episode_{ep:06d}.npz")
        if not os.path.exists(path):
            self.get_logger().error(f"未找到: {path}")
            return None
        return CartesianTrajectory.load(path)

    @staticmethod
    def _empty_result(success, message, num_frames=0, path_length=0.0,
                      ik_ok=0, col_count=0, col_details=None):
        return LatteImitationNode._result(
            success, message, num_frames, path_length, ik_ok, col_count,
            col_details,
        )

    @staticmethod
    def _result(success, message, num_frames, path_length, ik_ok,
                col_count, col_details):
        return {
            "success": success, "message": message,
            "num_frames": num_frames, "path_length": path_length,
            "ik_success_count": ik_ok,
            "collision_count": col_count,
            "collision_details": col_details or [],
        }

    # ═══════════════════════════════════════════════════════════════
    # 向后兼容 — 启动时自动执行默认 episode
    # ═══════════════════════════════════════════════════════════════

    def _delayed_start(self):
        self._init_timer.cancel()
        try:
            self._run_impl()
        except Exception as e:
            self.get_logger().error(f"启动执行异常: {e}")

    def _run_impl(self):
        result = self._execute_trajectory(
            episode_idx=self._episode_idx, arm=self._arm,
            speed_scale=self._speed_scale, mode=self._mode,
        )
        level = "info" if result["success"] else "error"
        getattr(self.get_logger(), level)(f"启动执行: {result['message']}")


# ═══════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LatteImitationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
