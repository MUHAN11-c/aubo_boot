"""
imu_follow 节点：IMU 姿态增量 → TCP 姿态实时跟随（servo / fjt 双后端）.

独立工具包（不进 lifecycle、不改只读 bringup、不订 peach 话题）。
`harvest_system` 仅在 `tool_profile:=adaptive_cylinder_v1` 时 Include
`imu_follow_servo.launch.py`；空心末端不起本节点。`~/enable` 采两组
参考（TF base→tip 当前位姿 + 当前 IMU 四元数），此后每节拍把 IMU 体轴
姿态增量经死区/符号映射/锥限幅/平滑叠加到参考 TCP 姿态上（位置钉死
参考点，只跟姿态；插入/回退时位置沿锁定开口方向移动）。

后端 `motion.backend`：
- `servo`（默认，MoveIt 官方实时方案）：对当前 TF 姿态求体轴误差，P 控制
  成角速度（位置误差同法小增益保持），`TwistStamped` 发 moveit_servo 的
  `delta_twist_cmds`（speed_units、EE 系）；Servo 以 100 Hz 增量 IK 流式
  下发（奇异缩放/碰撞减速/平滑内建），输出 JTC 话题。未 enable 时 pause
  servo，避免与 peach MTC 同时写 JTC。
- `fjt`（真机透传备选）：经 move_group `/compute_ik` 解关节、单步钳制后
  流式 FollowJointTrajectory；透传控制器只有 FJT 动作口时用这条。

`motion.enabled` 默认 false：只发布 `~/target_pose`、`~/command`（fjt）或
`~/command_twist`（servo）供检查、不发运动；peach 不自动开门。真机使用
须另行人工授权。IMU/关节状态断流、连续 IK 失败自动 disable（servo 后端
补发一次零 twist 刹车并 pause；fjt 取消在途 goal）。姿态数学在
follow_core（零 ROS，纯核表驱动测试）。

插入推进（自适应圆柱配套）：`~/insert_start` 在跟随会话内把位置目标从
参考点沿 insert_start 时刻工具开口方向（tip +Z，base 系锁定）按
`insert.speed_m_s` 低速推进、钳 `insert.max_travel_m` 行程；姿态照常跟
IMU。`~/insert_stop` 停推进；`~/insert_retract` 沿锁定开口把行程收回
参考点（跟随保持）。disable / 断流 / 达行程上限亦停推进。peach 仅对
`adaptive_cylinder_v1` FULL 在预抓取→套入→回预抓取窗内调这些服务。
"""

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from imu_follow import follow_core
from imu_follow import params as params_mod
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, ServoCommandType
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_WARN_PERIOD_S = 5.0


class ImuFollowNode(Node):
    """订 /imu/data 与 /joint_states；enable 采参考后按节拍下发速度或轨迹."""

    def __init__(self):
        super().__init__('imu_follow')
        self._listener = params_mod.imu_follow.ParamListener(self)
        self._p = self._listener.get_params()
        self._imu_q = None    # 最新 IMU 四元数（归一）
        self._imu_t = 0.0     # 收到时刻（秒，节点钟）
        self._joints = {}     # 关节名 -> 位置
        self._joints_t = 0.0
        self._enabled = False
        self._ref_pos = (0.0, 0.0, 0.0)             # base→tip 参考位置
        self._ref_tcp_q = (0.0, 0.0, 0.0, 1.0)      # 参考 TCP 姿态
        self._ref_imu_q = (0.0, 0.0, 0.0, 1.0)      # 参考 IMU 姿态
        self._smooth_q = (0.0, 0.0, 0.0, 1.0)       # 平滑后目标姿态
        self._insert_active = False                 # 插入推进中
        self._insert_retracting = False             # 插入回退中（与推进互斥）
        self._insert_dir = (0.0, 0.0, 1.0)          # 推进方向（base 系，锁定）
        self._insert_travel = 0.0                    # 已推进行程（m）
        self._insert_last_t = 0.0                    # 上次积分时刻（节点钟秒）
        self._servo_paused_idle = False             # 未 enable 时已 pause servo
        self._ik_busy = False
        self._ik_failures = 0
        self._goal_handle = None
        self._timer_hz = self._p.rate.update_hz

        self._pub_pose = self.create_publisher(PoseStamped, '~/target_pose', 10)
        self._pub_twist = self.create_publisher(
            TwistStamped, '~/command_twist', 10)
        # servo 输入订阅是 BEST_EFFORT：可靠发布器与其不兼容（收不到）
        self._pub_servo = self.create_publisher(
            TwistStamped, self._p.servo.twist_topic,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       durability=QoSDurabilityPolicy.VOLATILE, depth=10))
        self._pub_cmd = self.create_publisher(JointTrajectory, '~/command', 10)
        self.create_subscription(
            Imu, self._p.topics.imu_topic, self._on_imu, 10)
        self.create_subscription(
            JointState, self._p.topics.joint_states_topic, self._on_joints, 10)
        self.create_service(Trigger, '~/enable', self._on_enable)
        self.create_service(Trigger, '~/disable', self._on_disable)
        self.create_service(Trigger, '~/insert_start', self._on_insert_start)
        self.create_service(Trigger, '~/insert_stop', self._on_insert_stop)
        self.create_service(Trigger, '~/insert_retract', self._on_insert_retract)
        self._ik_cli = self.create_client(
            GetPositionIK, self._p.moveit.compute_ik_service)
        self._servo_type_cli = self.create_client(
            ServoCommandType, self._p.servo.command_type_service)
        self._servo_pause_cli = self.create_client(
            SetBool, self._p.servo.pause_service)
        self._fjt_cli = ActionClient(
            self, FollowJointTrajectory,
            self._p.execution.follow_joint_trajectory_action)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._timer = self.create_timer(1.0 / self._timer_hz, self._tick)
        self.get_logger().info(
            f'imu_follow 就绪（backend={self._p.motion.backend}，'
            f'motion.enabled={self._p.motion.enabled}，默认只算不发）；'
            'enable 前先确认 move_group / moveit_servo 与控制器在线')

    def _now_s(self):
        """节点钟当前秒（新鲜度判定用）."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _signs(self):
        """体轴符号表（follow.invert_* → ±1）."""
        f = self._p.follow
        return (
            -1.0 if f.invert_roll else 1.0,
            -1.0 if f.invert_pitch else 1.0,
            -1.0 if f.invert_yaw else 1.0)

    def _on_imu(self, msg):
        """只存归一姿态与到达时刻；断流由 _tick 判定."""
        o = msg.orientation
        self._imu_q = follow_core.quat_normalize((o.x, o.y, o.z, o.w))
        self._imu_t = self._now_s()

    def _on_joints(self, msg):
        """只留六轴；种子/新鲜度用."""
        for name, pos in zip(msg.name, msg.position):
            if name in self._p.joints.joint_names:
                self._joints[name] = float(pos)
        self._joints_t = self._now_s()

    def _on_enable(self, request, response):
        """前置全就绪才采参考（TF 位姿 + IMU 四元数）."""
        del request
        reason = self._enable_blockers()
        if reason:
            return Trigger.Response(success=False, message=reason)
        try:
            tf = self._tf.lookup_transform(
                self._p.frames.base_frame, self._p.frames.tip_frame, Time())
        except TransformException as exc:
            return Trigger.Response(
                success=False,
                message=f'TF {self._p.frames.base_frame}→'
                        f'{self._p.frames.tip_frame} 查询失败: {exc}')
        t = tf.transform
        self._ref_pos = (t.translation.x, t.translation.y, t.translation.z)
        self._ref_tcp_q = follow_core.quat_normalize(
            (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        self._ref_imu_q = self._imu_q
        self._smooth_q = self._ref_tcp_q
        self._insert_active = False
        self._insert_retracting = False
        self._insert_travel = 0.0
        self._ik_failures = 0
        self._enabled = True
        self._servo_paused_idle = False
        if self._p.motion.backend == 'servo':
            self._activate_servo()
        message = (f'参考已采集（{self._p.frames.base_frame}→'
                   f'{self._p.frames.tip_frame} 位姿 + IMU 四元数）；'
                   f'backend={self._p.motion.backend}，'
                   f'motion.enabled={self._p.motion.enabled}')
        self.get_logger().info(message)
        return Trigger.Response(success=True, message=message)

    def _activate_servo(self):
        """使能 servo 受理 twist：切指令类型 + 确保未暂停（幂等，异步）."""
        if self._servo_type_cli.service_is_ready():
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.TWIST
            self._servo_type_cli.call_async(req)
        else:
            self.get_logger().warning(
                f'{self._p.servo.command_type_service} 不可用，'
                'servo 会拒收 twist（Command type has not been set）',
                throttle_duration_sec=_WARN_PERIOD_S)
        if self._servo_pause_cli.service_is_ready():
            self._set_servo_paused(False)

    def _enable_blockers(self):
        """前置条件清单；全就绪返回 None，否则返回原因（enable 前核对）."""
        if self._imu_q is None:
            return f'尚未收到 IMU（{self._p.topics.imu_topic}）'
        age = self._now_s() - self._imu_t
        if age > self._p.safety.imu_timeout_s:
            return f'IMU 数据超时 {age:.2f}s'
        missing = [n for n in self._p.joints.joint_names
                   if n not in self._joints]
        if missing:
            return f'关节状态缺 {missing}'
        age = self._now_s() - self._joints_t
        if age > self._p.safety.joint_states_timeout_s:
            return f'关节状态超时 {age:.2f}s'
        if not self._tf.can_transform(
                self._p.frames.base_frame, self._p.frames.tip_frame, Time()):
            return (f'TF {self._p.frames.base_frame}→'
                    f'{self._p.frames.tip_frame} 未就绪')
        if self._p.motion.backend == 'servo':
            if self.count_subscribers(self._p.servo.twist_topic) == 0:
                return (f'servo 未订阅 {self._p.servo.twist_topic}'
                        '（用 imu_follow_servo.launch.py 起 moveit_servo？）')
        else:
            if not self._ik_cli.service_is_ready():
                return (f'{self._p.moveit.compute_ik_service} 不可用'
                        '（move_group 未起？）')
            if self._p.motion.enabled and not self._fjt_cli.server_is_ready():
                return (f'{self._p.execution.follow_joint_trajectory_action}'
                        ' 不可用（控制器未起？）')
        return None

    def _on_disable(self, request, response):
        """人工停止：servo 补零速刹车 / fjt 取消在途."""
        del request
        self._disable('服务调用')
        return Trigger.Response(success=True, message='跟随已停')

    def _disable(self, reason):
        """停止跟随并按后端收口（透传/JTC 取消停轨口径）."""
        was = self._enabled
        self._enabled = False
        self._insert_active = False
        self._insert_retracting = False
        handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        if was and self._p.motion.backend == 'servo':
            self._publish_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self._set_servo_paused(True)
            self._servo_paused_idle = True
        if was:
            self.get_logger().warning(f'跟随已停：{reason}')

    def _set_servo_paused(self, paused):
        """异步 pause_servo（True=暂停，避免与 peach MTC 同时写 JTC）."""
        if not self._servo_pause_cli.service_is_ready():
            return
        req = SetBool.Request()
        req.data = paused
        self._servo_pause_cli.call_async(req)

    def _on_insert_start(self, request, response):
        """开始插入推进：锁当前工具开口方向，位置目标沿之前进."""
        del request
        if not self._enabled:
            return Trigger.Response(
                success=False, message='未 enable：先 ~/enable 采参考')
        try:
            tf = self._tf.lookup_transform(
                self._p.frames.base_frame, self._p.frames.tip_frame, Time())
        except TransformException as exc:
            return Trigger.Response(
                success=False,
                message=f'TF {self._p.frames.base_frame}→'
                        f'{self._p.frames.tip_frame} 查询失败: {exc}')
        t = tf.transform
        q_tip = follow_core.quat_normalize(
            (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        # 推进方向 = 工具开口方向（tip +Z）在 base 系的当前指向，锁定不变
        self._insert_dir = follow_core.quat_rotate(q_tip, (0.0, 0.0, 1.0))
        self._insert_travel = 0.0
        self._insert_last_t = self._now_s()
        self._insert_retracting = False
        self._insert_active = True
        message = (
            f'插入推进开始：速度 {self._p.insert.speed_m_s} m/s，'
            f'行程上限 {self._p.insert.max_travel_m} m，'
            f'方向（base 系）= ({self._insert_dir[0]:.3f}, '
            f'{self._insert_dir[1]:.3f}, {self._insert_dir[2]:.3f})')
        self.get_logger().info(message)
        return Trigger.Response(success=True, message=message)

    def _on_insert_stop(self, request, response):
        """停止插入推进（跟随会话保持，位置目标停在当前行程）."""
        del request
        was = self._insert_active
        self._insert_active = False
        message = (
            f'插入推进已停（已推进 {self._insert_travel:.3f} m）' if was
            else '插入推进本就未在进行')
        return Trigger.Response(success=True, message=message)

    def _on_insert_retract(self, request, response):
        """沿锁定开口方向把行程收回参考点（跟随会话保持）."""
        del request
        if not self._enabled:
            return Trigger.Response(
                success=False, message='未 enable：先 ~/enable 采参考')
        self._insert_active = False
        if self._insert_travel <= 1e-6:
            self._insert_travel = 0.0
            self._insert_retracting = False
            return Trigger.Response(
                success=True, message='已在参考点，无需回退')
        self._insert_last_t = self._now_s()
        self._insert_retracting = True
        message = (
            f'插入回退开始：速度 {self._p.insert.speed_m_s} m/s，'
            f'当前行程 {self._insert_travel:.3f} m → 0')
        self.get_logger().info(message)
        return Trigger.Response(success=True, message=message)

    def _tick(self):
        """节拍主体：刷新参数 → 新鲜度门 → 目标姿态 → 按后端下发."""
        if self._listener.is_old(self._p):
            self._p = self._listener.get_params()
            self._refresh_timer()
        if not self._enabled:
            if (self._p.motion.backend == 'servo' and
                    not self._servo_paused_idle and
                    self._servo_pause_cli.service_is_ready()):
                self._set_servo_paused(True)
                self._servo_paused_idle = True
            return
        now = self._now_s()
        if now - self._imu_t > self._p.safety.imu_timeout_s:
            self._disable(f'IMU 断流 {now - self._imu_t:.2f}s')
            return
        if now - self._joints_t > self._p.safety.joint_states_timeout_s:
            self._disable(f'关节状态断流 {now - self._joints_t:.2f}s')
            return
        if self._insert_retracting:
            dt = now - self._insert_last_t
            self._insert_last_t = now
            self._insert_travel = follow_core.retraction_step(
                self._insert_travel, self._p.insert.speed_m_s, dt)
            if self._insert_travel <= 1e-9:
                self._insert_travel = 0.0
                self._insert_retracting = False
                self.get_logger().info(
                    '插入回退到位（行程 0，跟随保持，位置目标=参考点）')
        elif self._insert_active:
            dt = now - self._insert_last_t
            self._insert_last_t = now
            self._insert_travel = follow_core.insertion_step(
                self._insert_travel, self._p.insert.speed_m_s, dt,
                self._p.insert.max_travel_m)
            if self._insert_travel >= self._p.insert.max_travel_m:
                self._insert_active = False
                self.get_logger().warning(
                    f'插入达行程上限 {self._p.insert.max_travel_m:.3f} m，'
                    '自动停止推进（跟随保持，位置目标停在行程终点）')
        pos_target = follow_core.insertion_position(
            self._ref_pos, self._insert_dir, self._insert_travel)
        q_target = follow_core.target_orientation(
            self._ref_tcp_q, self._ref_imu_q, self._imu_q, self._signs(),
            self._p.follow.deadband_rad, self._p.follow.max_delta_rad)
        self._smooth_q = follow_core.slerp_toward(
            self._smooth_q, q_target, self._p.follow.smoothing_alpha)
        pose = PoseStamped()
        pose.header.frame_id = self._p.frames.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pos_target[0]
        pose.pose.position.y = pos_target[1]
        pose.pose.position.z = pos_target[2]
        pose.pose.orientation.x = self._smooth_q[0]
        pose.pose.orientation.y = self._smooth_q[1]
        pose.pose.orientation.z = self._smooth_q[2]
        pose.pose.orientation.w = self._smooth_q[3]
        self._pub_pose.publish(pose)
        if self._p.motion.backend == 'servo':
            self._servo_step(pos_target)
        elif not self._ik_busy:
            self._request_ik(pose)

    def _servo_step(self, pos_target):
        """速度后端：对当前 TF 闭环，姿态/位置误差 P 控制成 twist."""
        try:
            tf = self._tf.lookup_transform(
                self._p.frames.base_frame, self._p.frames.tip_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f'TF 查询失败，跳过节拍: {exc}',
                throttle_duration_sec=_WARN_PERIOD_S, skip_first=True)
            return
        t = tf.transform
        q_cur = follow_core.quat_normalize(
            (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))
        err = follow_core.delta_rotvec(q_cur, self._smooth_q)  # 体轴误差
        omega = follow_core.clamp_rotvec(
            follow_core.scale_vector(err, self._p.servo.orientation_gain),
            self._p.execution.max_omega_rad_s)
        # 位置保持跟随插入推进（推进行程即目标点），横向只剩温和定心
        d_pos = follow_core.apply_deadband(
            (pos_target[0] - t.translation.x,
             pos_target[1] - t.translation.y,
             pos_target[2] - t.translation.z),
            self._p.servo.position_deadband_m)
        v_base = follow_core.clamp_rotvec(
            follow_core.scale_vector(d_pos, self._p.servo.position_gain),
            self._p.execution.max_lin_vel_m_s)
        v_tip = follow_core.quat_rotate(follow_core.quat_conj(q_cur), v_base)
        self._publish_twist(omega, v_tip)

    def _publish_twist(self, omega, v_tip):
        """发一帧 twist：dry 镜像总是发；开门时发 servo 输入话题."""
        msg = TwistStamped()
        msg.header.frame_id = self._p.frames.tip_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = omega
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v_tip
        self._pub_twist.publish(msg)
        if self._p.motion.enabled:
            self._pub_servo.publish(msg)

    def _refresh_timer(self):
        """rate.update_hz 运行期变更时重建节拍定时器."""
        hz = self._p.rate.update_hz
        if hz != self._timer_hz:
            self._timer_hz = hz
            self.destroy_timer(self._timer)
            self._timer = self.create_timer(1.0 / hz, self._tick)

    def _request_ik(self, pose):
        """轨迹后端：以当前关节为种子请求 6D IK（单飞；50 ms）."""
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = self._p.moveit.group
        ik.ik_link_name = self._p.frames.tip_frame
        ik.timeout.sec = 0
        ik.timeout.nanosec = 50_000_000
        ik.avoid_collisions = True
        ik.pose_stamped.header.frame_id = pose.header.frame_id
        ik.pose_stamped.header.stamp = pose.header.stamp
        ik.pose_stamped.pose = pose.pose
        names = list(self._p.joints.joint_names)
        ik.robot_state.joint_state.name = names
        ik.robot_state.joint_state.position = [
            float(self._joints[n]) for n in names]
        self._ik_busy = True
        future = self._ik_cli.call_async(req)
        future.add_done_callback(self._on_ik)

    def _on_ik(self, future):
        """IK 回包：解→单步钳制→发布 ~/command→按门下发 FJT."""
        self._ik_busy = False
        if not self._enabled:
            # disable 后在途回包不补发（停即彻底停，含镜像话题）
            return
        try:
            res = future.result()
        except Exception as exc:  # 服务侧异常不致命，计数防护
            self._count_ik_failure(f'IK 调用异常: {exc}')
            return
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            val = res.error_code.val if res is not None else 'None'
            self._count_ik_failure(f'IK 无解/失败（error_code={val}）')
            return
        self._ik_failures = 0
        names = list(self._p.joints.joint_names)
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        if not all(n in sol for n in names):
            self._count_ik_failure('IK 解缺关节名')
            return
        current = [float(self._joints[n]) for n in names]
        target = [float(sol[n]) for n in names]
        positions = follow_core.clamp_joint_step(
            target, current, self._p.execution.max_joint_step_rad)
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        horizon = self._p.execution.horizon_s
        point.time_from_start = Duration(
            sec=int(horizon), nanosec=int((horizon % 1.0) * 1e9))
        traj.points.append(point)
        self._pub_cmd.publish(traj)
        if self._p.motion.enabled:
            self._send_goal(traj)

    def _count_ik_failure(self, why):
        """连续失败计数；达 safety.max_ik_failures 即 disable."""
        self._ik_failures += 1
        self.get_logger().warning(
            f'{why}（{self._ik_failures}/{self._p.safety.max_ik_failures}）',
            throttle_duration_sec=_WARN_PERIOD_S, skip_first=True)
        if self._ik_failures >= self._p.safety.max_ik_failures:
            self._disable(f'连续 IK 失败 {self._ik_failures} 次：{why}')

    def _send_goal(self, traj):
        """流式下发（新 goal 到达即替换旧 goal，为预期行为）."""
        if not self._fjt_cli.server_is_ready():
            self.get_logger().warning(
                f'{self._p.execution.follow_joint_trajectory_action} 不可用，'
                '跳过下发', throttle_duration_sec=_WARN_PERIOD_S)
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._fjt_cli.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """记录在途句柄（disable 取消用）；被拒只节流告警."""
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'FJT goal 异常: {exc}', throttle_duration_sec=_WARN_PERIOD_S)
            return
        if not handle.accepted:
            self.get_logger().warning(
                'FJT goal 被拒', throttle_duration_sec=_WARN_PERIOD_S)
            return
        if not self._enabled:
            # disable 之后才回包的 goal 不接管，立即取消（不留在途运动）
            handle.cancel_goal_async()
            self.get_logger().info('disable 后到达的 FJT goal 已取消')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """只报真 ABORTED；流式替换的 CANCELED/ABORTED 属预期噪声."""
        try:
            wrapped = future.result()
        except Exception:
            return
        if wrapped.status == GoalStatus.STATUS_ABORTED:
            error_code = getattr(wrapped.result, 'error_code', '?')
            self.get_logger().warning(
                f'FJT ABORTED（error_code={error_code}）',
                throttle_duration_sec=_WARN_PERIOD_S)


def main(argv=None):
    """入口：起节点并 spin；Ctrl+C 退出."""
    rclpy.init(args=argv)
    node = ImuFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
