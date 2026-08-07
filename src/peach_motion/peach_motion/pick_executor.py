"""
拾取执行器 — ~/pick action 全量执行（六阶段）.

APPROACH_PLAN（orbit_planner 生成球面弧线）→ ARC_APPROACH（速度缩放
speed_scaling，帧门控步进：每个 waypoint 执行后等新一帧 candidates 再走）
→ REFINE_WAIT（等 refined_candidates 支撑帧达标或超时，跳变>moved 失败）
→ FINAL_APPROACH（plan_only 可达性验证，失败 unreachable）→ INSERT（可选，
沿套入轴直线 travel）→ RETREAT（退回预接近点）。任意阶段可取消
（停发 + RETREAT → CANCELED）。

可测试性：moveit_factory / tcp_provider 可注入（单测用假实现）。
"""
import json
import math
import time

from geometry_msgs.msg import Point, Pose, PoseStamped
import numpy as np
from peach_motion.orbit_planner import make_orbit_waypoints
from peach_pose_msgs.action import Pick
from peach_pose_msgs.msg import (
    BagGraspCandidateArray,
    TargetRegistry as TargetRegistryMsg,
)
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# 感知话题（peach_pose_ros2 融合决策输出）
CANDIDATES_TOPIC = '/peach_pose_node/grasp_candidates'
REFINED_TOPIC = '/peach_pose_node/refined_candidates'

# 阶段常量（与 Pick.action 的 STAGE_* 一致）
STAGE_NAMES = {
    0: 'APPROACH_PLAN', 1: 'ARC_APPROACH', 2: 'REFINE_WAIT',
    3: 'FINAL_APPROACH', 4: 'INSERT', 5: 'RETREAT',
}

# 参数默认值（与 config/mission.yaml 的 pick_executor 节对齐）
DEFAULTS = {
    'group_name': 'manipulator_e5',  # MoveIt 规划组（AUBO E5）
    'base_frame': 'base_link',        # 基座系
    'tool_frame': 'wrist3_Link',      # TCP 连杆（E5 法兰）
    'speed_scaling': 0.01,            # 速度缩放（真机先压低速）
    'approach_standoff': 0.25,        # 预接近距离 (m)
    'orbit_arc_deg': 60,              # 弧段绕扫角上限 (deg)
    'orbit_points': 7,                # 弧线路径点数
    'frame_gate_timeout_sec': 2.0,    # 帧门控超时 (s)
    'min_support_frames': 20,         # REFINE_WAIT 支撑帧达标数
    'refine_timeout_sec': 8.0,        # REFINE_WAIT 超时 (s)
    'target_jump_dist': 0.10,         # refined 跳变门 (m)：超过判 moved
    'planning_time_sec': 5.0,         # 单次规划时限 (s)
    'planning_attempts': 2,           # 规划尝试次数
    'position_tolerance_m': 0.01,     # 位置球域容差 (m)
    'orientation_tolerance_rad': 0.1,  # 姿态三轴容差 (rad)
}


class FrameGate:
    """帧门控：记录帧计数，wait_next 阻塞到新一帧到达或超时."""

    def __init__(self, timeout_sec: float = 2.0):
        """
        构造门控.

        Args:
            timeout_sec: 等新帧的超时 (s).

        Returns
        -------
            无返回值（None）.

        """
        self.timeout_sec = float(timeout_sec)
        self._count = 0

    def note_frame(self) -> None:
        """帧计数 +1（订阅回调里调用）."""
        self._count += 1

    def wait_next(self, now_fn, sleep_fn) -> bool:
        """
        等到帧计数增加或超时.

        Args:
            now_fn: 取时函数（秒，单调）.
            sleep_fn: 睡眠函数（秒）；测试可注入假时间步进.

        Returns
        -------
            True=等到新帧；False=超时.

        """
        start_count = self._count
        start = now_fn()
        while self._count == start_count:
            if now_fn() - start > self.timeout_sec:
                return False
            sleep_fn(0.02)
        return True


class PickExecutor(Node):
    """~/pick action server：六阶段拾取执行（可注入假 MoveIt/TF 供单测）."""

    def __init__(self, moveit_factory=None, tcp_provider=None,
                 param_overrides=None):
        """
        建节点：参数 → 订阅 → action server → 发布者 → TF.

        Args:
            moveit_factory: 可选，签名为 (node, params: dict) → MoveItClient
                兼容对象；None 用真 MoveItClient（测试注入假实现）.
            tcp_provider: 可选，() → geometry_msgs/Pose（当前 TCP）；
                None 用 TF 查询 base_frame←tool_frame.
            param_overrides: 可选，参数覆盖 dict（测试/调参注入口，
                在读取后合并进 _params）.

        Returns
        -------
            无返回值（None）.

        """
        super().__init__('pick_executor')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        def _get(name):
            return self.get_parameter(name).value

        self._params = {k: _get(k) for k in DEFAULTS}
        if param_overrides:
            self._params.update(param_overrides)
        self._moveit_factory = moveit_factory
        self._tcp_provider = tcp_provider
        self._moveit = None                     # 首个 goal 时懒建

        self._gate = FrameGate(float(_get('frame_gate_timeout_sec')))
        self._latest_candidates = None          # 最新一帧候选（找目标用）
        self._latest_refined = None             # 最新 refined（REFINE_WAIT 用）
        self._latest_registry = None            # 最新注册表（注册表 ID→感知帧桥接）
        self.create_subscription(
            BagGraspCandidateArray, CANDIDATES_TOPIC, self._on_candidates, 10)
        self.create_subscription(
            BagGraspCandidateArray, REFINED_TOPIC, self._on_refined, 10)
        self.create_subscription(
            TargetRegistryMsg, '/mission_manager/registry',
            self._on_registry, 10)

        self.pub_stage = self.create_publisher(String, '~/stage_events', 10)
        self.pub_orbit = self.create_publisher(MarkerArray, '~/planned_orbit', 10)
        self.pub_trace = self.create_publisher(
            PoseStamped, '~/execution_trace', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._server = ActionServer(
            self, Pick, '~/pick', self._on_execute,
            cancel_callback=lambda gh: CancelResponse.ACCEPT)
        self.get_logger().info('pick_executor 就绪（六阶段执行器）')

    # ---- 订阅 ----
    def _on_candidates(self, msg):
        """候选帧：帧门控计数 + 缓存最新帧（找目标用）."""
        self._gate.note_frame()
        self._latest_candidates = msg

    def _on_refined(self, msg):
        """融合帧：缓存最新 refined（REFINE_WAIT 读支撑帧数）."""
        self._latest_refined = msg

    def _on_registry(self, msg):
        """注册表帧：缓存（goal 的注册表 ID → 位置 → 感知帧桥接用）."""
        self._latest_registry = msg

    # ---- 基础设施 ----
    def _moveit_client(self):
        """懒建 MoveItClient（或测试注入的假实现；moveit_msgs 延迟 import）."""
        if self._moveit is None:
            if self._moveit_factory is not None:
                self._moveit = self._moveit_factory(self, self._params)
            else:
                # 延迟 import：moveit_msgs 在源码 overlay，仅运行时（真机/demo）需要
                from peach_motion.moveit_client import MoveItClient
                params = self._params
                self._moveit = MoveItClient(
                    self,
                    group_name=params['group_name'],
                    base_frame=params['base_frame'],
                    tool_frame=params['tool_frame'],
                    velocity_scaling=params['speed_scaling'],
                    acceleration_scaling=params['speed_scaling'],
                    planning_time_sec=params['planning_time_sec'],
                    planning_attempts=params['planning_attempts'],
                    position_tolerance_m=params['position_tolerance_m'],
                    orientation_tolerance_rad=params[
                        'orientation_tolerance_rad'])
        return self._moveit

    def _current_tcp(self):
        """当前 TCP 位姿（注入的 provider 或 TF 查询 base←tool）."""
        if self._tcp_provider is not None:
            return self._tcp_provider()
        try:
            tf = self.tf_buffer.lookup_transform(
                self._params['base_frame'], self._params['tool_frame'], Time())
        except TransformException as ex:
            self.get_logger().warn(f'TCP TF 查询失败: {ex}')
            return None
        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose

    def _stage(self, goal_handle, stage: int, t0: float):
        """发 feedback + ~/stage_events（阶段迁移 JSON）."""
        event = {'stage': STAGE_NAMES[stage], 't': time.monotonic() - t0}
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False)
        self.pub_stage.publish(msg)
        feedback = Pick.Feedback()
        feedback.stage = stage
        feedback.progress = 0.0
        feedback.pose_err_m = -1.0
        feedback.elapsed_sec = float(time.monotonic() - t0)
        goal_handle.publish_feedback(feedback)

    def _feedback_progress(self, goal_handle, stage, progress, pose_err, t0):
        """发阶段内进度 feedback."""
        feedback = Pick.Feedback()
        feedback.stage = stage
        feedback.progress = float(np.clip(progress, 0.0, 1.0))
        feedback.pose_err_m = float(pose_err)
        feedback.elapsed_sec = float(time.monotonic() - t0)
        goal_handle.publish_feedback(feedback)

    def _finish(self, goal_handle, success, message, final_status=2,
                frames_used=0, final_pose=None):
        """统一收尾：组 result + succeed/abort."""
        result = Pick.Result()
        result.success = bool(success)
        result.message = message
        result.final_status = int(final_status)
        result.total_frames_used = int(frames_used)
        if final_pose is not None:
            result.final_pose = final_pose
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self.get_logger().info(f'pick 结束: success={success} {message}')
        return result

    def _find_target(self, target_id: str):
        """
        从最新候选帧找目标；注册表 ID 经位置桥接；空 ID 取最优 ACCEPT.

        匹配顺序：① 感知帧内 target_id 精确匹配；② 注册表档案位置
        （T%03d 注册 ID）→ 感知帧内最近候选（≤0.15 m）；③ 空 ID 取
        最优 ACCEPT。找不到给 None。
        """
        msg = self._latest_candidates
        if msg is None or not msg.candidates:
            return None
        if target_id:
            for c in msg.candidates:
                if c.target_id == target_id:
                    return c
            # 桥接：goal 给的是注册表 ID（T%03d），感知帧里是逐帧 ID
            record = self._registry_position(target_id)
            if record is not None:
                best, best_d = None, 0.15
                for c in msg.candidates:
                    d = float(np.linalg.norm(np.array([
                        c.entry_pose.position.x - record[0],
                        c.entry_pose.position.y - record[1],
                        c.entry_pose.position.z - record[2]])))
                    if d < best_d:
                        best, best_d = c, d
                if best is not None:
                    return best
            return None
        accepted = [c for c in msg.candidates if int(c.status) == 0]
        if accepted:
            return max(accepted, key=lambda c: float(c.confidence))
        return None

    def _registry_position(self, target_id: str):
        """注册表档案位置（T%03d → (3,) 米；无此 ID/无注册表给 None）."""
        msg = self._latest_registry
        if msg is None:
            return None
        for info in msg.targets:
            if info.target_id == target_id:
                return np.array([info.position.x, info.position.y,
                                 info.position.z])
        return None

    def _retreat(self, standoff_pose, cancel_check):
        """RETREAT：规划回预接近点（尽力而为，失败仅告警）."""
        ok, cat = self._moveit_client().plan_and_execute(
            standoff_pose, cancel_check=cancel_check)
        if not ok:
            self.get_logger().warn(f'RETREAT 失败（{cat}），请人工检查机械臂')

    # ---- goal 主流程 ----
    def _on_execute(self, goal_handle):
        """
        执行 goal 主流程：六阶段（见模块 docstring）.

        Args:
            goal_handle: action goal 句柄.

        Returns
        -------
            Pick.Result.

        """
        t0 = time.monotonic()
        goal = goal_handle.request
        p = self._params

        # is_cancel_requested 是 property，包一层供轮询调用
        def cancel_check():
            return goal_handle.is_cancel_requested
        moveit = self._moveit_client()

        # ── APPROACH_PLAN ──
        self._stage(goal_handle, 0, t0)
        target = self._find_target(goal.target_id)
        if target is None:
            return self._finish(goal_handle, False,
                                f'no_target: 最新帧无目标 {goal.target_id!r}')
        tcp = self._current_tcp()
        if tcp is None:
            return self._finish(goal_handle, False, 'no_tcp: TCP TF 不可用')
        tgt = np.array([target.entry_pose.position.x,
                        target.entry_pose.position.y,
                        target.entry_pose.position.z])
        axis = np.array([target.translation_direction.x,
                         target.translation_direction.y,
                         target.translation_direction.z])
        waypoints = make_orbit_waypoints(
            (tcp.position.x, tcp.position.y, tcp.position.z), tgt, axis,
            p['approach_standoff'], p['orbit_arc_deg'], p['orbit_points'])
        if not waypoints:                    # 失败重试一次：绕行角 +15°
            waypoints = make_orbit_waypoints(
                (tcp.position.x, tcp.position.y, tcp.position.z), tgt, axis,
                p['approach_standoff'], p['orbit_arc_deg'] + 15.0,
                p['orbit_points'])
        if not waypoints:
            return self._finish(goal_handle, False,
                                'plan_failed: 轨道路径点生成失败')
        self._publish_orbit(waypoints)

        # ── ARC_APPROACH：帧门控步进（起点是当前位姿，从 1 开始） ──
        self._stage(goal_handle, 1, t0)
        frames_used = 0
        for i, (wp, quat) in enumerate(waypoints[1:], 1):
            if cancel_check():
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                goal_handle.canceled()
                return self._cancel_result()
            ok, cat = moveit.plan_and_execute(
                self._pose_msg((wp, quat)), cancel_check=cancel_check)
            if cat == 'cancelled':
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                goal_handle.canceled()
                return self._cancel_result()
            if not ok:
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                return self._finish(goal_handle, False,
                                    f'plan_failed: 弧线点 {i} 规划失败({cat})')
            self._publish_trace()
            frames_used += 1
            self._gate.wait_next(time.monotonic, time.sleep)   # 帧门控
            err = float(np.linalg.norm(wp - tgt))
            self._feedback_progress(goal_handle, 1, i / (len(waypoints) - 1),
                                    err, t0)

        # ── REFINE_WAIT：等 refined 支撑帧达标或超时 ──
        self._stage(goal_handle, 2, t0)
        refined = self._wait_refined(t0, cancel_check)
        if refined == 'cancelled':
            self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
            goal_handle.canceled()
            return self._cancel_result()
        final_target = target
        if refined is not None:
            jump = float(np.linalg.norm(
                np.array([refined.entry_pose.position.x,
                          refined.entry_pose.position.y,
                          refined.entry_pose.position.z]) - tgt))
            if jump > p['target_jump_dist']:
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                return self._finish(
                    goal_handle, False,
                    f'moved: refined 跳变 {jump:.3f}m > {p["target_jump_dist"]}m')
            final_target = refined

        # ── FINAL_APPROACH：plan_only 可达性验证 ──
        self._stage(goal_handle, 3, t0)
        ok, cat = moveit.plan_and_execute(
            final_target.entry_pose, plan_only=True, cancel_check=cancel_check)
        if cat == 'cancelled':
            self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
            goal_handle.canceled()
            return self._cancel_result()
        if not ok:
            self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
            return self._finish(goal_handle, False,
                                f'unreachable: refined entry_pose 规划失败({cat})')
        # 方向合理性指标：套入轴与竖直方向（重力 −Z）的无向夹角
        cos = abs(float(axis[2]) / max(float(np.linalg.norm(axis)), 1e-9))
        axis_gravity_deg = float(math.degrees(math.acos(min(cos, 1.0))))

        # ── INSERT（可选）：沿套入轴小步进笛卡尔直线 travel ──
        if goal.execute_insert:
            self._stage(goal_handle, 4, t0)
            ok, cat = self._insert_cartesian(moveit, final_target, axis,
                                             cancel_check)
            if cat == 'cancelled':
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                goal_handle.canceled()
                return self._cancel_result()
            if not ok:
                self._retreat(self._pose_msg(waypoints[-1]), lambda: False)
                return self._finish(goal_handle, False,
                                    f'execution_failed: INSERT 失败({cat})')

        # ── RETREAT ──
        self._stage(goal_handle, 5, t0)
        self._retreat(self._pose_msg(waypoints[-1]), cancel_check)
        final = PoseStamped()
        final.header.frame_id = p['base_frame']
        final.pose = final_target.entry_pose
        message = (f'ok: axis_gravity_deg={axis_gravity_deg:.1f} '
                   f'insert={goal.execute_insert}')
        return self._finish(goal_handle, True, message, final_status=0,
                            frames_used=frames_used, final_pose=final)

    @staticmethod
    def _pose_msg(waypoint) -> Pose:
        """(position, quat_xyzw) 路径点 → Pose 消息."""
        p, q = waypoint
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(p[0]), float(p[1]), float(p[2]))
        pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
        pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
        return pose

    @staticmethod
    def _cancel_result() -> Pick.Result:
        """组装 CANCELED 结果（goal_handle.canceled 由调用方先行标记）."""
        result = Pick.Result()
        result.success = False
        result.message = 'cancelled: 用户取消，已 RETREAT'
        result.final_status = 2
        result.total_frames_used = 0
        return result

    def _insert_cartesian(self, moveit, target, axis, cancel_check,
                          step_m: float = 0.005):
        """
        INSERT：沿 translation_direction 小步进位姿序列逐段 plan&execute.

        Args:
            moveit: MoveItClient（或测试假实现）.
            target: 最终目标候选（entry_pose/suggested_travel_m）.
            axis: (3,) 套入轴（单位向量）.
            cancel_check: 取消轮询.
            step_m: 每段步长 (m)，默认 5 mm（姿态保持 entry_pose 姿态）.

        Returns
        -------
            (success, category)：category 语义同 MoveItClient；任一段失败
            即中断返回.

        """
        e = target.entry_pose.position
        travel = float(target.suggested_travel_m)
        steps = max(1, int(math.ceil(travel / step_m)))
        for i in range(1, steps + 1):
            s = travel * i / steps
            pose = Pose()
            pose.position.x = e.x + s * float(axis[0])
            pose.position.y = e.y + s * float(axis[1])
            pose.position.z = e.z + s * float(axis[2])
            pose.orientation = target.entry_pose.orientation
            ok, cat = moveit.plan_and_execute(pose, cancel_check=cancel_check)
            if not ok:
                return False, cat
        return True, ''

    def _wait_refined(self, t0, cancel_check):
        """
        REFINE_WAIT：等 refined 支撑帧达标或超时.

        Returns
        -------
            refined 候选消息；超时给 None；取消给 'cancelled'.

        """
        start = time.monotonic()
        while time.monotonic() - start < self._params['refine_timeout_sec']:
            if cancel_check():
                return 'cancelled'
            msg = self._latest_refined
            if msg is not None and msg.candidates:
                best = max(msg.candidates,
                           key=lambda c: float(c.confidence))
                support = 0
                for flag in best.diagnostic_flags:
                    if flag.startswith('support_frames='):
                        support = int(flag.split('=', 1)[1])
                if support >= self._params['min_support_frames']:
                    return best
            time.sleep(0.05)
        return None

    def _publish_orbit(self, waypoints):
        """发布 ~/planned_orbit：弧线路径 LINE_STRIP 可视化."""
        marker = Marker()
        marker.header.frame_id = self._params['base_frame']
        marker.ns = 'planned_orbit'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.color.r, marker.color.g = 0.2, 0.6
        marker.color.b, marker.color.a = 1.0, 0.9
        for wp, _ in waypoints:
            pt = Point()
            pt.x, pt.y, pt.z = float(wp[0]), float(wp[1]), float(wp[2])
            marker.points.append(pt)
        array = MarkerArray()
        array.markers.append(marker)
        self.pub_orbit.publish(array)

    def _publish_trace(self):
        """发布 ~/execution_trace：当前实际 TCP 位姿（供与计划偏差比对）."""
        tcp = self._current_tcp()
        if tcp is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._params['base_frame']
        msg.pose = tcp
        self.pub_trace.publish(msg)
