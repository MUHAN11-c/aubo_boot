"""
Latte Imitation 主节点 — MoveIt2 6 阶段管线 + RViz2 Preview 喵~

=== 完整流程 ===

入口 (3 种触发方式):
  A. ROS 2 Service  ~/replay_trajectory (外部调用)
  B. Launch 参数    启动时自动执行默认 episode (向后兼容)
  C. test_latte_pour.py 交互菜单 (开发调试)

═══════════════════════════════════════════════════════════════
_pipeline() 6 阶段:

  ① Load  — 从 npz 加载轨迹
  ② Retarget  — SE(3) 重定目标 (SPOT + Isaac Teleop)
  ③ Preview  — 发布 RViz2 markers (mode="preview" 在此返回)
  ④ Safety  — 工作空间边界检查
  ⑤ Plan  — MoveIt2 computeCartesianPath
  ⑥ Execute  — MoveIt2 executeTrajectory

═══════════════════════════════════════════════════════════════
Preview 模式 (mode="preview"):

  发布以下话题到 RViz2:
    ~/preview/tcp_path         nav_msgs/Path        绿色 TCP 轨迹
    ~/preview/tcp_waypoints    geometry_msgs/PoseArray 方向箭头 (每5帧)
    ~/preview/spout_path       visualization_msgs/Marker 蓝色 spout 线
    ~/preview/cup_pose         visualization_msgs/Marker 黄色杯子方块
    ~/preview/workspace_bounds visualization_msgs/Marker 红色安全框

═══════════════════════════════════════════════════════════════
并发模型:
  Executor:    MultiThreadedExecutor(4)
  Callback:    ReentrantCallbackGroup (防死锁 — CLAUDE.md 规则 #14)
  防重入:      self._executing 布尔锁

MoveIt2 参数 (CartesianInterpolator 源码审计):
  max_step=0.01          → 覆盖帧间位移 ~3.8mm
  jump_threshold=0.0     → 禁用相对跳变 (L227: factor > 0.0 才启用)
  revolute_jump_threshold=0.0  → 禁用绝对跳变 (L230)
  avoid_collisions=True  → 内置碰撞检测

═══════════════════════════════════════════════════════════════
理论依据:
  SPOT (arXiv:2411.00965): Object-centric SE(3) trajectory
  Isaac Teleop: Se3RelRetargeter delta 语义
  SO(3) Action Repr. (Savva 2025): Hamilton 四元数约定
  SVRC: Object-relative Cartesian → Very High generalization
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Point, PoseArray
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import Marker
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, RobotState, RobotTrajectory
from std_msgs.msg import Header, ColorRGBA
from ament_index_python.packages import get_package_share_directory
from ivg_interfaces.srv import ReplayLatteTrajectory

from .trajectory import CartesianTrajectory
from .trajectory_transform import (
    retarget_trajectory,
    retarget_with_orientation_constraint,
    is_default_position,
    is_default_orientation,
)
from .config_loader import load_tool_offset, load_workspace_safety
from .tf_utils import get_current_ee_pose


# ═══════════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════════

def _load_latte_positions_defaults() -> dict:
    """从 YAML 加载参考位姿默认值, 返回 declare_parameter 用的 {key: default} 字典喵~"""
    import yaml, os
    defaults = {}
    yaml_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "latte_positions.yaml",
    )
    try:
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        for link_name, pose in data.items():
            for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
                key = f"{link_name}.{axis}"
                defaults[key] = float(pose.get(axis, 0.0))
    except Exception:
        # YAML 不可用时用硬编码 fallback
        defaults = {
            "coffee_link.x": -0.645, "coffee_link.y": 0.098, "coffee_link.z": 0.05,
            "coffee_link.roll": 0.0, "coffee_link.pitch": 0.0, "coffee_link.yaw": 0.0,
            "lizhu_link.x": -0.630, "lizhu_link.y": -0.368, "lizhu_link.z": 0.04,
            "lizhu_link.roll": 0.0, "lizhu_link.pitch": 0.0, "lizhu_link.yaw": 0.0,
            "cup0_link.x": -0.528, "cup0_link.y": -0.198, "cup0_link.z": 0.05,
            "cup0_link.roll": 0.0, "cup0_link.pitch": 0.0, "cup0_link.yaw": 0.0,
            "reference_pose.x": -0.419, "reference_pose.y": -0.400, "reference_pose.z": 0.246,
            "reference_pose.roll": -23.5, "reference_pose.pitch": 88.1, "reference_pose.yaw": 76.0,
        }
    return defaults


def _cartesian_resource_dir() -> str:
    try:
        share = get_package_share_directory("latte_imitation")
        return os.path.join(share, "resource", "cartesian")
    except Exception:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_dir, "resource", "cartesian")


def _get_or_create_client(node, attr: str, create_fn):
    client = getattr(node, attr, None)
    if client is None:
        client = create_fn()
        setattr(node, attr, client)
    return client


# ═══════════════════════════════════════════════════════════════════
# LatteImitationNode
# ═══════════════════════════════════════════════════════════════════

class LatteImitationNode(Node):
    """拉花轨迹回放节点 — 6 阶段 MoveIt2 管线 + RViz2 Preview 喵~

    话题 (发布):
      ~/preview/tcp_path        Path       TCP 轨迹 (preview 模式)
      ~/preview/tcp_waypoints   PoseArray  TCP 关键姿态 (preview 模式)
      ~/preview/spout_path      Marker     Spout 轨迹 (preview 模式)
      ~/preview/cup_pose        Marker     杯子位置 (preview 模式)
      ~/preview/workspace_bounds Marker    工作空间安全框 (preview 模式)
      ~/ee_pose                 PoseStamped 末端位姿 (debug 模式)
      ~/ee_path                 Path       轨迹路径 (debug 模式)

    服务:
      ~/replay_trajectory  ivg_interfaces/srv/ReplayLatteTrajectory
    """

    def __init__(self):
        super().__init__("latte_imitation")

        # ── ROS2 参数 ──────────────────────────────────────
        self.declare_parameter("episode_idx", 0)
        self.declare_parameter("arm", "right")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("mode", "preview")

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

        # ── 参考位姿参数 (从 YAML 加载默认值) ──
        _pos = _load_latte_positions_defaults()
        for key, default in _pos.items():
            self.declare_parameter(key, default)
            setattr(self, f"_{key}", self.get_parameter(key).value)

        self._episode_idx = self.get_parameter("episode_idx").value
        self._arm = self.get_parameter("arm").value
        self._speed_scale = self.get_parameter("speed_scale").value
        self._mode = self.get_parameter("mode").value

        # ── Debug 发布者 ──────────────────────────────────
        self._ee_pose_pub = self.create_publisher(PoseStamped, "~/ee_pose", 10)
        self._ee_path_pub = self.create_publisher(RosPath, "~/ee_path", 10)

        # ── Preview 发布者 (RViz2 markers) ──────────────
        self._preview_tcp_path_pub = self.create_publisher(RosPath, "~/preview/tcp_path", 10)
        self._preview_tcp_poses_pub = self.create_publisher(PoseArray, "~/preview/tcp_waypoints", 10)
        self._preview_spout_pub = self.create_publisher(Marker, "~/preview/spout_path", 10)
        self._preview_cup_pub = self.create_publisher(Marker, "~/preview/cup_pose", 10)
        self._preview_workspace_pub = self.create_publisher(Marker, "~/preview/workspace_bounds", 10)

        # ── TF 监听 ────────────────────────────────────────
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── MoveIt2 客户端 (懒初始化) ──────────────────────
        self._cartesian_client = None
        self._execute_client = None

        # ── 服务 ───────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._replay_srv = self.create_service(
            ReplayLatteTrajectory, "~/replay_trajectory",
            self._replay_service_callback, callback_group=self._cb_group,
        )
        self.get_logger().info("服务就绪: ~/replay_trajectory (preview/debug/action)")

        # ── 并发防护 + 向后兼容 ──────────────────────────
        self._exec_lock = threading.Lock()
        self._init_timer = self.create_timer(4.0, self._delayed_start)

    # ═══════════════════════════════════════════════════════════════
    # TF
    # ═══════════════════════════════════════════════════════════════

    def _get_current_ee_pose(self):
        base_frame = self.get_parameter("base_frame").value
        ee_link = self.get_parameter("ee_link").value
        tc = self.get_parameter("tf_retry_count").value
        ti = self.get_parameter("tf_retry_interval").value
        pose = get_current_ee_pose(self, base_frame, ee_link, tc, ti)
        if pose is None:
            self.get_logger().warn(f"TF 不可达: {base_frame} → {ee_link}")
        return pose

    # ═══════════════════════════════════════════════════════════════
    # 服务回调
    # ═══════════════════════════════════════════════════════════════

    def _replay_service_callback(self, request, response):
        pattern = getattr(request, 'pattern_type', '') or ''
        self.get_logger().info(
            f"Service: ep={request.episode_idx}, pattern='{pattern}', arm={request.arm}, "
            f"mode={request.mode}, rpy=({request.roll_deg:.0f},{request.pitch_deg:.0f},{request.yaw_deg:.0f})"
        )
        speed = request.speed_scale if request.speed_scale > 1e-6 else 1.0
        result = self._execute_pipeline(
            episode_idx=request.episode_idx,
            arm=request.arm,
            speed_scale=speed,
            mode=request.mode,
            start_pose=request.start_pose if hasattr(request, 'start_pose') else None,
            rpy_user=(request.roll_deg, request.pitch_deg, request.yaw_deg),
            tool_offset_id=request.tool_offset_id if request.tool_offset_id else "default",
            translation_offset=(
                getattr(request, 'translation_x', 0.0) or 0.0,
                getattr(request, 'translation_y', 0.0) or 0.0,
                getattr(request, 'translation_z', 0.0) or 0.0,
            ),
            waypoint_sample_step=getattr(request, 'waypoint_sample_step', 5) or 5,
            pattern_type=pattern,
            pattern_image_path=getattr(request, 'pattern_image_path', '') or '',
            tulip_layers=getattr(request, 'tulip_layers', 3),
            cup_params=self._extract_cup_params(request),
            pour_params=self._extract_pour_params(request),
        )
        response.success = result["success"]
        response.message = result["message"]
        response.num_frames = result["num_frames"]
        response.path_length = result["path_length"]
        response.ik_success_count = result["ik_success_count"]
        response.collision_count = result["collision_count"]
        response.collision_details = result["collision_details"]
        response.waypoints = result.get("waypoints", []) or []
        return response

    @staticmethod
    def _extract_cup_params(request) -> dict:
        return {
            "center_x": getattr(request, 'cup_center_x', 0.0) or 0.0,
            "center_y": getattr(request, 'cup_center_y', 0.0) or 0.0,
            "surface_z": getattr(request, 'cup_surface_z', 0.15) or 0.15,
            "radius": getattr(request, 'cup_radius', 0.04) or 0.04,
        }

    @staticmethod
    def _extract_pour_params(request) -> dict:
        return {
            "mix_height_offset": getattr(request, 'pour_mix_height_offset', 0.076) or 0.076,
            "draw_height_offset": getattr(request, 'pour_draw_height_offset', 0.006) or 0.006,
            "finish_height_offset": getattr(request, 'pour_finish_height_offset', 0.076) or 0.076,
            "wiggle_amplitude": getattr(request, 'pour_wiggle_amplitude', 0.006) or 0.006,
            "wiggle_frequency": getattr(request, 'pour_wiggle_frequency', 5.0) or 5.0,
            "max_velocity": getattr(request, 'pour_max_velocity', 0.05) or 0.05,
            "max_acceleration": getattr(request, 'pour_max_acceleration', 0.1) or 0.1,
            "max_jerk": getattr(request, 'pour_max_jerk', 0.5) or 0.5,
            "enable_anti_sloshing": getattr(request, 'enable_anti_sloshing', True),
        }

    # ═══════════════════════════════════════════════════════════════
    # 管线编排
    # ═══════════════════════════════════════════════════════════════

    def _execute_pipeline(self, episode_idx, arm, speed_scale, mode,
                           start_pose=None, rpy_user=(0.0, 0.0, 0.0),
                           tool_offset_id="default",
                           pattern_type="", pattern_image_path="",
                           tulip_layers=3, cup_params=None, pour_params=None,
                           translation_offset=(0.0, 0.0, 0.0),
                           waypoint_sample_step=5):
        if not self._exec_lock.acquire(blocking=False):
            return self._empty_result(False, "已有轨迹正在执行，请稍后喵~")
        try:
            return self._pipeline(episode_idx, arm, speed_scale, mode,
                                  start_pose, rpy_user, tool_offset_id,
                                  pattern_type, pattern_image_path,
                                  tulip_layers, cup_params, pour_params,
                                  translation_offset=translation_offset,
                                  waypoint_sample_step=waypoint_sample_step)
        except Exception as e:
            self.get_logger().error(f"执行异常: {e}")
            return self._empty_result(False, str(e))
        finally:
            self._exec_lock.release()

    def _pipeline(self, episode_idx, arm, speed_scale, mode, start_pose,
                   rpy_user, tool_offset_id,
                   pattern_type="", pattern_image_path="",
                   tulip_layers=3, cup_params=None, pour_params=None,
                   translation_offset=(0.0, 0.0, 0.0),
                   waypoint_sample_step=5):
        """6 阶段 MoveIt2 管线 (支持录制回放 + 参数化生成) 喵~"""
        base_frame = self.get_parameter("base_frame").value
        planning_group = self.get_parameter("planning_group").value
        ee_link = self.get_parameter("ee_link").value
        step = waypoint_sample_step
        fraction_ok = self.get_parameter("fraction_acceptable").value
        fraction_min = self.get_parameter("fraction_min_executable").value

        # ═══ Phase ①: Load / Generate ═══
        cart = self._load_or_generate(
            episode_idx, arm, pattern_type, pattern_image_path,
            tulip_layers, cup_params, pour_params)
        if cart is None:
            if pattern_type:
                return self._empty_result(False,
                    f"参数化生成失败: pattern_type='{pattern_type}'")
            return self._empty_result(False,
                f"episode_{episode_idx:06d}.npz (arm='{arm}') 未找到")

        # ═══ Phase ②: OrientProfile (参数化模式 — 动态朝向剖面) ═══
        if pattern_type and cup_params:
            from latte_imitation.latte_art.orientation_profile import compute_pitch_profile
            from latte_imitation.latte_art.bridge import parametric_to_cartesian as _make_cart

            total_frames = cart.num_frames
            # 阶段帧数与 compose_full_trajectory 一致: 融合 50 帧, 收尾 30 帧
            num_mix = 50
            mix_end = num_mix
            draw_end = max(mix_end + 1, total_frames - 30)

            pitch_profile = compute_pitch_profile(total_frames, mix_end, draw_end)
            cart = _make_cart(
                cart.positions,
                roll_deg=0.0, yaw_deg=0.0, dt=cart.dt,
                pitch_profile=pitch_profile,
            )
            self.get_logger().info(
                f"动态朝向剖面: pitch {pitch_profile[0]:.0f}°→"
                f"{pitch_profile[mix_end]:.0f}°→{pitch_profile[-1]:.0f}°"
            )

        # ═══ Phase ③: Retarget ═══
        if start_pose is None:
            start_pose = Pose()

        tf_warning = False
        if pattern_type and cup_params:
            # 参数化模式: retarget 目标 = 杯子位姿
            # XY 以 ROS2 lizhu_link 参数为权威来源, Z 从前端传入
            lizhu_x = float(self.get_parameter("lizhu_link.x").value)
            lizhu_y = float(self.get_parameter("lizhu_link.y").value)
            target = Pose()
            target.position.x = lizhu_x
            target.position.y = lizhu_y
            target.position.z = float(cup_params.get("surface_z", 0.15))
            target.orientation.x = 0.0
            target.orientation.y = 0.0
            target.orientation.z = 0.0
            target.orientation.w = 1.0
            pos_src = "杯子坐标"

            cart = retarget_with_orientation_constraint(
                cart, target,
                rpy_user=rpy_user,
                translation_offset=translation_offset,
            )
        else:
            # 录制回放模式: 保持现有 TF 逻辑
            use_tf_position = is_default_position(start_pose)
            if use_tf_position:
                current_pose = self._get_current_ee_pose()
                if current_pose is None:
                    if mode in ("preview", "debug"):
                        self.get_logger().warn(
                            "TF 不可达, preview/debug 模式使用原点 (0,0,0) 作为起点"
                        )
                        target = Pose()
                        target.position.x = 0.0
                        target.position.y = 0.0
                        target.position.z = 0.0
                        target.orientation.w = 1.0
                        pos_src = "原点(TF不可达)"
                        tf_warning = True
                    else:
                        return self._empty_result(False,
                            "无法获取当前末端位姿 (TF base_link → tool_tcp)")
                else:
                    target = current_pose
                    rotate = not is_default_orientation(start_pose) or any(
                        abs(r) > 1e-9 for r in rpy_user
                    )
                    if rotate:
                        target.orientation = start_pose.orientation
                    pos_src = "TF"
            else:
                target = start_pose
                pos_src = "手动"

            cart = retarget_trajectory(cart, target, rpy_user=rpy_user,
                                       absolute_orientation=False,
                                       translation_offset=translation_offset)

        self.get_logger().info(
            f"轨迹已变换 (位置={pos_src}, rpy={rpy_user}): "
            f"({target.position.x:.3f}, {target.position.y:.3f}, {target.position.z:.3f})"
        )

        num_frames = cart.num_frames
        path_len = cart.path_length()
        dt = cart.dt / max(speed_scale, 0.01)
        self.get_logger().info(
            f"Ep{episode_idx} ({arm}): {num_frames}frames, {path_len:.2f}m, dt={dt:.3f}s"
        )

        # ═══ Phase ③: Preview (RViz2 markers) ═══
        tool_offset = load_tool_offset(tool_offset_id)
        workspace = load_workspace_safety()
        self._publish_preview_markers(cart, target, tool_offset, workspace)
        self._publish_poses(self._ee_pose_pub, cart)

        # 采样 waypoints 用于前端 3D 渲染
        sample_step = max(1, step)
        sampled_poses = []
        for i in range(0, num_frames, sample_step):
            sampled_poses.append(cart.to_pose(i))
        if (num_frames - 1) % sample_step != 0:
            sampled_poses.append(cart.to_pose(num_frames - 1))

        # ═══ Phase ④: Safety Check (所有模式都检查, preview/debug 仅报告) ═══
        safety_violation = ""
        if workspace.safety_policy != "ignore":
            is_safe, safety_msg, _ = cart.check_workspace_bounds(
                x_range=(workspace.x_min, workspace.x_max),
                y_range=(workspace.y_min, workspace.y_max),
                z_range=(workspace.z_min, workspace.z_max),
            )
            if not is_safe:
                safety_violation = safety_msg
                if mode == "action" and workspace.safety_policy == "warn_and_block":
                    return self._empty_result(False, f"Safety: {safety_msg}", num_frames, path_len)
                else:
                    self.get_logger().warn(f"Safety (non-blocking): {safety_msg}")

        if mode in ("preview", "debug"):
            msg = (f"{mode}: {num_frames} 帧, {path_len:.2f}m, "
                   f"rpy=({rpy_user[0]:.0f},{rpy_user[1]:.0f},{rpy_user[2]:.0f})")
            if tf_warning:
                msg += " | 警告: 使用原点(0,0,0)作为起点, TF不可达"
            if safety_violation:
                msg += f" | 安全警告: {safety_violation}"
            return self._result(True, msg, num_frames, path_len, 0, 0, [],
                                waypoints=sampled_poses)

        if mode != "action":
            return self._result(True,
                f"mode='{mode}' (未规划/执行)", num_frames, path_len, 0, 0, [])

        # ═══ Phase ⑤: Cartesian Plan ═══
        waypoints = [cart.to_pose(i) for i in range(0, num_frames, step)]
        if (num_frames - 1) % step != 0:
            waypoints.append(cart.to_pose(num_frames - 1))
        self.get_logger().info(
            f"Phase ⑤: {len(waypoints)} waypoints..."
        )
        resp = self._compute_cartesian_path(waypoints, planning_group, ee_link, base_frame)
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
            return self._empty_result(False,
                f"笛卡尔规划不完整 ({fraction*100:.0f}% < {fraction_ok*100:.0f}%), 不执行",
                num_frames, path_len)

        # 按 speed_scale 缩放时间戳
        trajectory = resp.solution
        for i, pt in enumerate(trajectory.joint_trajectory.points):
            t = i * dt
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

        # ═══ Phase ⑥: Execute ═══
        ok, msg = self._execute_trajectory(trajectory)
        ik_count = int(fraction * num_frames)
        return self._result(ok, msg, num_frames, path_len, ik_count, 0, [])

    # ═══════════════════════════════════════════════════════════════
    # Phase ①: Load
    # ═══════════════════════════════════════════════════════════════

    def _load_or_generate(self, episode_idx, arm, pattern_type, pattern_image_path,
                           tulip_layers, cup_params, pour_params):
        """Phase ①: 录制回放 或 参数化生成 → CartesianTrajectory 喵~"""
        if pattern_type:
            # 模式 B: 参数化生成
            try:
                from latte_imitation.latte_art import (
                    LatteArtTrajectory, CupConfig, PourConfig,
                    compose_full_trajectory, apply_anti_sloshing,
                    parametric_to_cartesian,
                )
            except ImportError as e:
                self.get_logger().error(f"latte_art 模块不可用: {e}")
                return None

            cup = cup_params or {}
            pour = pour_params or {}
            cup_cfg = CupConfig(
                center_x=cup.get("center_x", 0.0),
                center_y=cup.get("center_y", 0.0),
                surface_z=cup.get("surface_z", 0.15),
                radius=cup.get("radius", 0.04),
            )
            pour_cfg = PourConfig(
                mix_height_offset=pour.get("mix_height_offset", 0.076),
                draw_height_offset=pour.get("draw_height_offset", 0.006),
                finish_height_offset=pour.get("finish_height_offset", 0.076),
                wiggle_amplitude=pour.get("wiggle_amplitude", 0.006),
                wiggle_frequency=pour.get("wiggle_frequency", 5.0),
                max_velocity=pour.get("max_velocity", 0.05),
                max_acceleration=pour.get("max_acceleration", 0.1),
                max_jerk=pour.get("max_jerk", 0.5),
            )

            gen = LatteArtTrajectory(cup_cfg, pour_cfg)
            try:
                if pattern_type == "tulip":
                    xyz = gen.tulip(layers=max(1, tulip_layers))
                elif pattern_type == "custom":
                    if not pattern_image_path:
                        self.get_logger().error("custom 模式需要 pattern_image_path")
                        return None
                    xyz = gen.from_image(pattern_image_path)
                else:
                    xyz = getattr(gen, pattern_type)()
            except Exception as e:
                self.get_logger().error(f"轨迹生成失败 ({pattern_type}): {e}")
                return None

            xyz = compose_full_trajectory(xyz, cup_cfg, pour_cfg)
            if pour.get("enable_anti_sloshing", True):
                xyz = apply_anti_sloshing(xyz, pour_cfg)

            cart = parametric_to_cartesian(xyz)
            self.get_logger().info(
                f"参数化生成: pattern={pattern_type} frames={cart.num_frames} "
                f"path={cart.path_length():.3f}m"
            )
            return cart
        else:
            # 模式 A: 录制回放
            return self._load_cartesian(episode_idx, arm)

    def _load_cartesian(self, episode_idx=None, arm=None):
        ep = episode_idx if episode_idx is not None else self._episode_idx
        ar = arm if arm is not None else self._arm
        path = os.path.join(_cartesian_resource_dir(), ar, f"episode_{ep:06d}.npz")
        if not os.path.exists(path):
            self.get_logger().error(f"未找到: {path}")
            return None
        return CartesianTrajectory.load(path)

    # ═══════════════════════════════════════════════════════════════
    # Phase ③: Preview — RViz2 Markers
    # ═══════════════════════════════════════════════════════════════

    def _publish_preview_markers(self, cart, cup_pose, tool_offset, workspace):
        """发布所有 RViz2 preview markers 喵~"""
        now = self.get_clock().now().to_msg()

        # ── TCP Path (绿色) ──
        tcp_path = cart.to_ros2_path(step=2, stamp=now)
        self._preview_tcp_path_pub.publish(tcp_path)

        # ── TCP Waypoints (每 5 帧, 带朝向箭头) ──
        pa = PoseArray()
        pa.header = Header(frame_id=cart.frame_id, stamp=now)
        for i in range(0, cart.num_frames, 5):
            pa.poses.append(cart.to_pose(i))
        self._preview_tcp_poses_pub.publish(pa)

        # ── Spout Path (蓝色虚线) ──
        spout_marker = Marker()
        spout_marker.header = Header(frame_id=cart.frame_id, stamp=now)
        spout_marker.ns = "latte_preview"
        spout_marker.id = 0
        spout_marker.type = Marker.LINE_STRIP
        spout_marker.action = Marker.ADD
        spout_marker.scale.x = 0.003
        spout_marker.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8)
        dx, dy, dz = tool_offset.pos
        for i in range(0, cart.num_frames, 2):
            p = cart.positions[i]
            spout_marker.points.append(Point(x=p[0]+dx, y=p[1]+dy, z=p[2]+dz))
        self._preview_spout_pub.publish(spout_marker)

        # ── Cup Pose (黄色方块) ──
        cup_marker = Marker()
        cup_marker.header = Header(frame_id=cart.frame_id, stamp=now)
        cup_marker.ns = "latte_preview"
        cup_marker.id = 1
        cup_marker.type = Marker.CUBE
        cup_marker.action = Marker.ADD
        cup_marker.pose = cup_pose
        cup_marker.scale.x = 0.06; cup_marker.scale.y = 0.06; cup_marker.scale.z = 0.06
        cup_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)
        self._preview_cup_pub.publish(cup_marker)

        # ── Workspace Bounds (红色线框) ──
        ws_marker = Marker()
        ws_marker.header = Header(frame_id="base_link", stamp=now)
        ws_marker.ns = "latte_preview"
        ws_marker.id = 2
        ws_marker.type = Marker.LINE_LIST
        ws_marker.action = Marker.ADD
        ws_marker.scale.x = 0.003
        ws_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.6)
        x0, x1 = workspace.x_min, workspace.x_max
        y0, y1 = workspace.y_min, workspace.y_max
        z0, z1 = workspace.z_min, workspace.z_max
        corners = [
            (x0,y0,z0),(x1,y0,z0), (x1,y0,z0),(x1,y1,z0),
            (x1,y1,z0),(x0,y1,z0), (x0,y1,z0),(x0,y0,z0),  # 底面
            (x0,y0,z1),(x1,y0,z1), (x1,y0,z1),(x1,y1,z1),
            (x1,y1,z1),(x0,y1,z1), (x0,y1,z1),(x0,y0,z1),  # 顶面
            (x0,y0,z0),(x0,y0,z1), (x1,y0,z0),(x1,y0,z1),
            (x1,y1,z0),(x1,y1,z1), (x0,y1,z0),(x0,y1,z1),  # 竖边
        ]
        for c in corners:
            ws_marker.points.append(Point(x=c[0], y=c[1], z=c[2]))
        self._preview_workspace_pub.publish(ws_marker)

        self.get_logger().info(
            "Preview: TCP path + waypoints + spout + cup + workspace bounds → RViz2"
        )

    # ═══════════════════════════════════════════════════════════════
    # Phase ⑤: MoveIt2 Cartesian Path
    # ═══════════════════════════════════════════════════════════════

    def _compute_cartesian_path(self, waypoints, planning_group, ee_link,
                                 base_frame, avoid_collisions=True):
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
                f"computeCartesianPath 完成: {elapsed:.1f}s, fraction={result.fraction*100:.1f}%"
            )
            return result
        except Exception as e:
            self.get_logger().error(f"computeCartesianPath 异常 ({elapsed:.1f}s): {e}")
            return None

    # ═══════════════════════════════════════════════════════════════
    # Phase ⑥: Execute
    # ═══════════════════════════════════════════════════════════════

    def _execute_trajectory(self, trajectory: RobotTrajectory):
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
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=execution_timeout)
        if not result_future.done():
            return False, f"轨迹执行超时 ({execution_timeout}s)"

        try:
            error_code = result_future.result().result.error_code
        except Exception as e:
            return False, f"获取 Action 结果异常: {e}"

        if error_code.val == 1:
            return True, "轨迹执行成功完成"
        else:
            return False, f"轨迹执行失败: error_code={error_code.val}"

    # ═══════════════════════════════════════════════════════════════
    # 辅助
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _publish_poses(pub, cart, step=5):
        for i in range(0, cart.num_frames, step):
            pub.publish(cart.to_pose_stamped(i))

    @staticmethod
    def _empty_result(success, message, num_frames=0, path_length=0.0,
                      ik_ok=0, col_count=0, col_details=None):
        return LatteImitationNode._result(
            success, message, num_frames, path_length, ik_ok, col_count,
            col_details,
        )

    @staticmethod
    def _result(success, message, num_frames, path_length, ik_ok,
                col_count, col_details, waypoints=None):
        return {
            "success": success, "message": message,
            "num_frames": num_frames, "path_length": path_length,
            "ik_success_count": ik_ok,
            "collision_count": col_count,
            "collision_details": col_details or [],
            "waypoints": waypoints or [],
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
