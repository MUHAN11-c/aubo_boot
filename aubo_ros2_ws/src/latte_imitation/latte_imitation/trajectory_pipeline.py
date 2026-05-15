"""
Latte Imitation 主节点 — MoveIt2 标准管线喵~

=== 完整流程 ===

入口 (3 种触发方式):
  A. ROS 2 Service  /latte_imitation/replay_trajectory  (外部调用)
  B. Launch 参数    启动时自动执行默认 episode           (向后兼容)
  C. test_replay_service.py 交互菜单                    (开发调试)

═══════════════════════════════════════════════════════════════
A/B → _execute_pipeline()                   ← 编排器 + 并发锁
        │  self._executing=True  ←── 防重入互斥
        │
        ▼
      _pipeline()                            ← 5 阶段管线
        │
        ├─ Phase ① _load_cartesian()
        │     npz 文件路径:  resource/cartesian/{arm}/episode_{idx:06d}.npz
        │     返回:  CartesianTrajectory (400 帧 × 7D [xyz+qxyzw])
        │     失败 → (success=false, "episode_xxx.npz 未找到")
        │
        ├─ Phase ② apply_start_pose()
        │     is_default_pose(start_pose)?
        │       YES → _get_current_ee_pose()  TF lookup(base_link→tool_tcp)
        │              重试 20 次 × 30ms, 失败 → "无法获取当前末端位姿"
        │       NO  → 使用手动指定的 start_pose (如相机检测杯子位姿)
        │     变换:  R_rel = R_tgt @ R_orig^T, p_new = R_rel @ (p-p0) + p_target
        │            q_new = q_rel * q_orig (Hamilton 乘积)
        │     特性:  刚性保距 (path_length 不变), 旋转中心 = 第一帧位置
        │
        ├─ Phase ③ _publish_poses()
        │     ~/ee_pose ← 每 5 帧采样 PoseStamped
        │     ~/ee_path ← 完整轨迹 Path (step=5)
        │     mode="debug" → return (success=true, 跳过规划/执行)
        │
        ├─ Phase ④ _compute_cartesian_path()
        │     服务:  /compute_cartesian_path (MoveIt2)
        │     参数:  max_step=0.01, jump_threshold=0.0
        │            start_state=RobotState() 空=当前状态
        │            avoid_collisions=True 内置碰撞检测
        │     fraction 三级处理:
        │       ≥0.95  → 直接进入 Phase ⑤
        │       0.50~  → retry with avoid_collisions=False (取较大 fraction)
        │       <0.50  → fail
        │     成功 → 按 speed_scale 缩放 trajectory timestamps
        │     超时 → CARTESIAN_TIMEOUT=30s
        │
        └─ Phase ⑤ _execute_trajectory()
             Action:  /execute_trajectory (MoveIt2)
             流程:   send_goal_async → wait_for_accept → wait_for_result
             超时:   send_goal 5s, 执行 120s
             成功:   error_code.val == 1 (MoveIt2 SUCCESS)
             返回:   ik_success_count = int(fraction × num_frames)

═══════════════════════════════════════════════════════════════
并发模型:
  Executor:    MultiThreadedExecutor(4)
  Callback:    ReentrantCallbackGroup (服务回调 + MoveIt client 共享)
  防重入:      self._executing 布尔锁 (同一时刻仅一条管线运行)

外部依赖:
  MoveIt2:     /compute_cartesian_path (service, timeout 15s 发现 + 30s 执行)
               /execute_trajectory (action, timeout 5s 发现 + 120s 执行)
  TF:          base_link → tool_tcp (用于自动确定轨迹起点)
  ivg_interfaces: ReplayLatteTrajectory.srv (52 接口之一)

辅助模块:
  trajectory.py             CartesianTrajectory: npz I/O + ROS2 导出
  trajectory_transform.py   apply_start_pose(): 6-DOF 刚性变换 + 四元数工具
  tf_utils.py               共享 TF 查询 (消除重复) 喵~
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path as RosPath
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, RobotState, RobotTrajectory
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
from ivg_interfaces.srv import ReplayLatteTrajectory

from .trajectory import CartesianTrajectory
from .trajectory_transform import apply_start_pose
from .trajectory_transform import is_default_position, is_default_orientation
from .tf_utils import get_current_ee_pose

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


# ═══════════════════════════════════════════════════════════════════
# LatteImitationNode
# ═══════════════════════════════════════════════════════════════════

class LatteImitationNode(Node):
    """拉花轨迹回放节点 — MoveIt2 标准管线喵~

    话题 (发布):
      ~/ee_pose           PoseStamped   轨迹 waypoints (每5帧采样，debug+action 均发布)
      ~/ee_path           Path          轨迹完整路径 (debug+action 均发布)

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

        # 管线参数 (可通过 launch / YAML 覆盖) 喵~
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "tool_tcp")
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("fraction_acceptable", 0.95)
        self.declare_parameter("fraction_min_executable", 0.50)
        self.declare_parameter("waypoint_sample_step", 4)
        self.declare_parameter("service_timeout", 15.0)
        self.declare_parameter("cartesian_timeout", 60.0)
        self.declare_parameter("execution_timeout", 120.0)
        self.declare_parameter("tf_retry_count", 20)
        self.declare_parameter("tf_retry_interval", 0.03)

        self._episode_idx = self.get_parameter("episode_idx").value
        self._arm = self.get_parameter("arm").value
        self._speed_scale = self.get_parameter("speed_scale").value
        self._mode = self.get_parameter("mode").value

        # ── 发布者 ────────────────────────────────────────
        self._ee_pose_pub = self.create_publisher(PoseStamped, "~/ee_pose", 10)
        self._ee_path_pub = self.create_publisher(RosPath, "~/ee_path", 10)

        # ── TF 监听 (单例, 缓存) ──────────────────────────
        from tf2_ros import Buffer, TransformListener
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
        self._init_timer = self.create_timer(4.0, self._delayed_start)

    # ═══════════════════════════════════════════════════════════════
    # TF — 当前末端位姿 (委托给共享 tf_utils 模块)
    # ═══════════════════════════════════════════════════════════════

    def _get_current_ee_pose(self):
        """通过 TF 获取当前末端执行器位姿 (base_link → tool_tcp) 喵~

        Returns:
            geometry_msgs/Pose 或 None (TF 不可达)
        """
        pose = get_current_ee_pose(
            self,
            base_frame=self.get_parameter("base_frame").value,
            ee_link=self.get_parameter("ee_link").value,
            retry_count=self.get_parameter("tf_retry_count").value,
            retry_interval=self.get_parameter("tf_retry_interval").value,
        )
        if pose is None:
            self.get_logger().warn(
                f"TF 暂不可达: {self.get_parameter('base_frame').value} "
                f"→ {self.get_parameter('ee_link').value}"
            )
        return pose

    # ═══════════════════════════════════════════════════════════════
    # 服务回调
    # ═══════════════════════════════════════════════════════════════

    def _replay_service_callback(self, request, response):
        self.get_logger().info(
            f"Service: episode={request.episode_idx}, arm={request.arm}, "
            f"speed={request.speed_scale}, mode={request.mode}"
        )
        speed = request.speed_scale if request.speed_scale > 1e-6 else 1.0
        result = self._execute_pipeline(
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

    def _execute_pipeline(self, episode_idx, arm, speed_scale, mode,
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
        base_frame = self.get_parameter("base_frame").value
        planning_group = self.get_parameter("planning_group").value
        ee_link = self.get_parameter("ee_link").value
        step = self.get_parameter("waypoint_sample_step").value
        fraction_ok = self.get_parameter("fraction_acceptable").value
        fraction_min = self.get_parameter("fraction_min_executable").value

        # Phase 1: 加载轨迹
        cart = self._load_cartesian(episode_idx, arm)
        if cart is None:
            return self._empty_result(False,
                f"episode_{episode_idx:06d}.npz (arm='{arm}') 未找到")

        # Phase 2: Transform — 位置/朝向独立处理喵~
        if start_pose is None:
            start_pose = Pose()

        use_tf_position = is_default_position(start_pose)
        rotate = not is_default_orientation(start_pose)

        if use_tf_position:
            current_pose = self._get_current_ee_pose()
            if current_pose is None:
                return self._empty_result(False,
                    "无法获取当前末端位姿 (TF base_link → tool_tcp)")
            target = current_pose
            pos_src = "TF"
        else:
            target = start_pose
            pos_src = "手动"

        cart = apply_start_pose(cart, target, rotate_orientation=rotate)
        self.get_logger().info(
            f"轨迹已变换 (位置={pos_src}, 旋转={'是' if rotate else '否'}): "
            f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})"
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
        waypoints = [cart.to_pose(i) for i in range(0, num_frames, step)]
        # 确保最后一帧在 waypoints 中 (避免 fraction < 1.0 误判)
        if (num_frames - 1) % step != 0:
            waypoints.append(cart.to_pose(num_frames - 1))
        self.get_logger().info(
            f"Phase ④: {len(waypoints)} waypoints ({num_frames} 帧→{len(waypoints)} 采样点)..."
        )
        resp = self._compute_cartesian_path(waypoints, planning_group, ee_link,
                                            base_frame)
        if resp is None:
            return self._empty_result(False, "computeCartesianPath 服务不可达",
                                      num_frames, path_len)

        fraction = resp.fraction
        planned_points = len(resp.solution.joint_trajectory.points)
        self.get_logger().info(
            f"Cartesian path: {fraction*100:.1f}%, {planned_points} joint points"
        )

        if fraction < fraction_min:
            return self._empty_result(False,
                f"笛卡尔规划失败 ({fraction*100:.0f}% < {fraction_min*100:.0f}%)",
                num_frames, path_len)

        if fraction < fraction_ok:
            self.get_logger().warn(
                f"轨迹不完整 ({fraction*100:.1f}%), 尝试关闭碰撞检测重试..."
            )
            resp2 = self._compute_cartesian_path(waypoints, planning_group,
                                                 ee_link, base_frame,
                                                 avoid_collisions=False)
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

        # Phase 5: MoveIt2 执行
        ok, msg = self._execute_trajectory(trajectory)
        ik_count = int(fraction * num_frames)

        return self._result(ok, msg, num_frames, path_len, ik_count, 0, [])

    # ═══════════════════════════════════════════════════════════════
    # MoveIt2 Cartesian Path 规划
    # ═══════════════════════════════════════════════════════════════

    def _compute_cartesian_path(self, waypoints, planning_group, ee_link,
                                 base_frame, avoid_collisions=True):
        """调用 /compute_cartesian_path 服务喵~"""
        service_timeout = self.get_parameter("service_timeout").value
        cartesian_timeout = self.get_parameter("cartesian_timeout").value
        max_step = self.get_parameter("cartesian_max_step").value
        jump_threshold = self.get_parameter("cartesian_jump_threshold").value

        client = _get_or_create_client(
            self, '_cartesian_client',
            lambda: self.create_client(GetCartesianPath, "/compute_cartesian_path",
                                        callback_group=self._cb_group),
        )
        if not client.wait_for_service(timeout_sec=service_timeout):
            self.get_logger().error("/compute_cartesian_path 不可达")
            return None

        req = GetCartesianPath.Request()
        req.header = Header()
        req.header.frame_id = base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = RobotState()
        req.group_name = planning_group
        req.link_name = ee_link
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.path_constraints = Constraints()

        t0 = time.perf_counter()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=cartesian_timeout)
        elapsed = time.perf_counter() - t0
        if not future.done():
            self.get_logger().error(
                f"computeCartesianPath 超时 ({elapsed:.1f}s > {cartesian_timeout}s)"
            )
            return None
        try:
            result = future.result()
            self.get_logger().info(
                f"computeCartesianPath 完成: {elapsed:.1f}s, "
                f"fraction={result.fraction*100:.1f}%"
            )
            return result
        except Exception as e:
            self.get_logger().error(
                f"computeCartesianPath 异常 ({elapsed:.1f}s): {e}"
            )
            return None

    # ═══════════════════════════════════════════════════════════════
    # MoveIt2 轨迹执行
    # ═══════════════════════════════════════════════════════════════

    def _execute_trajectory(self, trajectory: RobotTrajectory):
        """通过 /execute_trajectory action 执行 MoveIt 规划的轨迹喵~"""
        execution_timeout = self.get_parameter("execution_timeout").value

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
                                         timeout_sec=execution_timeout)
        if not result_future.done():
            return False, f"轨迹执行超时 ({execution_timeout}s)"

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
        for attempt in range(5):
            result = self._run_impl()
            if result["success"]:
                return
            msg = result["message"]
            if "无法获取当前末端" in msg or "不可达" in msg:
                self.get_logger().warn(
                    f"启动执行第 {attempt+1}/5 次: {msg}, 1s 后重试..."
                )
                if attempt < 4:
                    time.sleep(1.0)
            else:
                self.get_logger().warn(f"启动执行失败: {msg}")
                return
        self.get_logger().warn("启动执行: 5 次重试均失败, 放弃 (服务仍可正常调用)")

    def _run_impl(self):
        return self._execute_pipeline(
            episode_idx=self._episode_idx, arm=self._arm,
            speed_scale=self._speed_scale, mode=self._mode,
        )


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
