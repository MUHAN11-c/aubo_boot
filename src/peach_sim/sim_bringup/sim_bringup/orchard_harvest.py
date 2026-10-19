"""
``orchard_harvest`` 自主导航采摘编排（仿真演示，thin 接线层）.

参考口径：导航全走 Nav2 官方 ``nav2_simple_commander``（同 ``orchard_tour``）；
臂运动走 MoveGroup action（同 peach_arm ``MoveGroupInterface`` 的 goal 语义：
组 ``manipulator_e5`` / pilz PTP-LIN / ``base_link`` 参考系 / ``tcp`` 末端）；
感知走 sim_perception 快照的 begin_scene → target_observations →
build_target_model 契约。

状态机（对齐参考仓库 mission_manager 的阶段名，动作是套袋接近）：
``BOOT → LOCALIZE → STOW →`` 每站 ``NAVIGATE → SURVEY → PLAN_HARVEST →
HARVESTING → RETURN``，结束 ``DONE`` / ``FAILED`` / ``STOPPED``。
当前状态锁存在 ``/orchard_harvest/mission_state``。

每站节拍：
1. ``STOW`` / ``NAVIGATE``：收臂行驶，Nav2 到站；
2. ``SURVEY``：对齐拍照位后感知，锁集给出目标和 id；没有 id 不重建；
3. 对锁集里的每个 id：``PLAN_HARVEST`` 重建 → 回到拍照位 → ``HARVESTING``
   抓取（OMPL 预接近，Pilz LIN）→ ``RETURN`` 回拍照位，再下一颗；
4. 本站目标都走完后收纳，去下一站。
   LIN 目标是 1 mm 盒加完整姿态。套入/剪切不在本节点。

仿真专用，不进 harvest_system；权限语义与 peach 真机栈无关。
"""

from __future__ import annotations

import argparse
from enum import Enum
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from peach_interfaces.action import BuildTargetModel
from peach_interfaces.msg import PeachTargetObservationArray
from peach_interfaces.srv import BeginScene
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from sim_worlds.cli import default_share
from sim_worlds.params import check_params, params_from_dict
from sim_worlds.tour_core import stations_by_row
import yaml

ARM_JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
              'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
# SRDF 命名态关节值快照（aubo_e5.srdf；MoveGroup action 不解析
# Constraints.name 命名目标——那是 MoveGroupInterface 客户端能力，
# 这里按官方语义自行展开成关节约束，error=-16 实证）。
SRDF_STATES = {
    'harvest_stow': (0.425083, 0.195177, 1.677740, 1.461739,
                     -0.500161, 0.038621),
    # 仿真单独的拍照位（2026-10-19 IK）：TCP 姿态与 platform_base_link 重合
    # （位置仍是旧拍照位在 base_link 的 TCP）。真机 SRDF 示教值不改。
    'global_photo_pose': (0.468150, 0.465192, 1.850364, 1.385192,
                          0.090365, -0.000013),
}
PILZ = 'pilz_industrial_motion_planner'
# 该拍照位下相机光轴在平台系是 -Y（FK：optical +Z ≈ (0, -1, 0)），
# 相机原点相对平台原点 (0.220, -0.234) m。导航把这条轴转到果袋即可。
PHOTO_AXIS_XY = (0.0, -1.0)
PHOTO_CAM_OFFSET_XY = (0.220, -0.234)


class MissionState(str, Enum):
    """采摘任务状态。阶段对齐参考仓库 mission_manager，动作换成套袋接近."""

    BOOT = 'BOOT'
    LOCALIZE = 'LOCALIZE'
    STOW = 'STOW'
    NAVIGATE = 'NAVIGATE'
    SURVEY = 'SURVEY'
    PLAN_HARVEST = 'PLAN_HARVEST'
    HARVESTING = 'HARVESTING'
    RETURN = 'RETURN'
    DONE = 'DONE'
    STOPPED = 'STOPPED'
    FAILED = 'FAILED'


def _unit(x: float, y: float, z: float) -> tuple[float, float, float]:
    """单位向量；退化时回退到世界 +Z."""
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-9:
        return 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm


def _quat_from_matrix(rotation: tuple[tuple[float, float, float], ...],
                      ) -> tuple[float, float, float, float]:
    """旋转矩阵转单位四元数 (x, y, z, w)，Shepperd 分支."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2][1] - rotation[1][2]) / scale
        y = (rotation[0][2] - rotation[2][0]) / scale
        z = (rotation[1][0] - rotation[0][1]) / scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        w = (rotation[2][1] - rotation[1][2]) / scale
        x = 0.25 * scale
        y = (rotation[0][1] + rotation[1][0]) / scale
        z = (rotation[0][2] + rotation[2][0]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        w = (rotation[0][2] - rotation[2][0]) / scale
        x = (rotation[0][1] + rotation[1][0]) / scale
        y = 0.25 * scale
        z = (rotation[1][2] + rotation[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        w = (rotation[1][0] - rotation[0][1]) / scale
        x = (rotation[0][2] + rotation[2][0]) / scale
        y = (rotation[1][2] + rotation[2][1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def survey_yaw(tree_dx: float, tree_dy: float) -> float:
    """底盘航向，使拍照位光轴指向 (tree_dx, tree_dy)（世界系、相对平台原点）."""
    axis = math.atan2(PHOTO_AXIS_XY[1], PHOTO_AXIS_XY[0])
    target = math.atan2(tree_dy, tree_dx)
    yaw = target - axis
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def park_toward_fruit(fruit_dx: float, fruit_dy: float,
                      reach_m: float) -> tuple[float, float, float, float]:
    """把停车点从站点沿袋心方向前移，使袋心落在臂展内.

    返回 (goal_dx, goal_dy, aim_dx, aim_dy)：目标相对站点的平移，以及
    袋心相对新停车点的向量（用来转相机光轴）。已经在臂展内则不动。
    """
    distance = math.hypot(fruit_dx, fruit_dy)
    if distance <= reach_m or distance < 1e-6:
        return 0.0, 0.0, fruit_dx, fruit_dy
    scale = (distance - reach_m) / distance
    return (fruit_dx * scale, fruit_dy * scale,
            fruit_dx * (1.0 - scale), fruit_dy * (1.0 - scale))


def fruit_survey_yaw(fruit_dx: float, fruit_dy: float) -> float:
    """底盘航向，使相机光轴从相机原点指向果袋中心.

    相机不在平台原点上，航向一变，相机相对果袋的方向也变，迭代到不动点。
    """
    yaw = survey_yaw(fruit_dx, fruit_dy)
    offset_x, offset_y = PHOTO_CAM_OFFSET_XY
    for _ in range(4):
        cosine, sine = math.cos(yaw), math.sin(yaw)
        yaw = survey_yaw(
            fruit_dx - (cosine * offset_x - sine * offset_y),
            fruit_dy - (sine * offset_x + cosine * offset_y))
    return yaw


def _resolve_manifest(stations_file: Path, manifest_field: str | None) -> Path | None:
    """站点 yaml 里的 manifest 路径相对仓库根，安装目录下再向上找."""
    if not manifest_field:
        return None
    raw = Path(manifest_field)
    # 目录从 src/sim 改回 src/peach_sim 后，已安装的站点 yaml 仍写着旧前缀。
    candidates = [raw]
    text = manifest_field.replace('src/sim/', 'src/peach_sim/', 1)
    if text != manifest_field:
        candidates.append(Path(text))
    for item in candidates:
        if item.is_file():
            return item
        for parent in stations_file.parents:
            found = parent / item
            if found.is_file():
                return found
    return None


def bag_centroids(manifest_path: Path) -> dict[int, tuple[float, float]]:
    """每棵树的袋颈 xy 中心（scene_manifest 的 TreeNN/ 目标）."""
    data = json.loads(manifest_path.read_text(encoding='utf-8'))
    groups: dict[int, list[tuple[float, float]]] = {}
    for target in data.get('targets', []):
        name = str(target.get('name', ''))
        if not name.startswith('Tree'):
            continue
        digits = ''
        for char in name[4:]:
            if char.isdigit():
                digits += char
            else:
                break
        if not digits or 'neck' not in target:
            continue
        neck = target['neck']
        groups.setdefault(int(digits), []).append((float(neck[0]), float(neck[1])))
    return {tree: (sum(p[0] for p in pts) / len(pts),
                   sum(p[1] for p in pts) / len(pts))
            for tree, pts in groups.items()}


def tool_orientation(approach: tuple[float, float, float],
                     ) -> tuple[float, float, float, float]:
    """TCP +Z 沿接近方向。基与 peach_arm ``ViewPlanner::toolOrientation`` 相同."""
    zx, zy, zz = _unit(*approach)
    px, py, pz = (0.0, 0.0, 1.0)
    dot = px * zx + py * zy + pz * zz
    xx, xy, xz = px - dot * zx, py - dot * zy, pz - dot * zz
    if xx * xx + xy * xy + xz * xz < 1e-12:
        px, py, pz = (0.0, 1.0, 0.0)
        dot = px * zx + py * zy + pz * zz
        xx, xy, xz = px - dot * zx, py - dot * zy, pz - dot * zz
    xx, xy, xz = _unit(xx, xy, xz)
    yx = zy * xz - zz * xy
    yy = zz * xx - zx * xz
    yz = zx * xy - zy * xx
    rotation = (
        (xx, yx, zx),
        (xy, yy, zy),
        (xz, yz, zz),
    )
    return _quat_from_matrix(rotation)


class HarvestRunner:
    """顺序编排：导航 → 拍照 → 感知锁集 → 建模 → 臂接近 → 下一站."""

    def __init__(self, node: Node, args, side: int = 1):
        """建客户端/订阅与统计口径（side = 作业道在树行的哪一侧，目标余量用）."""
        self.node = node
        self.side = side
        self.args = args
        self.log = node.get_logger()
        self.stats = {
            'stations': 0, 'nav_ok': 0, 'scenes_locked': 0,
            'models_ok': 0, 'approach_ok': 0}
        self.observations: PeachTargetObservationArray | None = None
        self.scene_epoch = 0

        self.navigator = BasicNavigator()
        self.navigator.set_parameters(
            [Parameter('use_sim_time', value=True)])
        self.begin_scene = node.create_client(
            BeginScene, '/peach_scene_perception_node/begin_scene')
        self.build_model = ActionClient(
            node, BuildTargetModel,
            '/peach_target_reconstruction_node/build_target_model')
        self.move_group = ActionClient(node, MoveGroup, '/move_action')
        node.create_subscription(
            PeachTargetObservationArray,
            '/peach/perception/target_observations',
            self._on_observations, 10)
        self._state = MissionState.BOOT
        self._state_pub = node.create_publisher(
            String, '~/mission_state',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._publish_state()
        # VelocityControl 会锁存最后一条速度；μ=0 时导航停发后车会一直滑。
        self._cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def set_state(self, state: MissionState):
        """切换任务状态并锁存到 ~/mission_state."""
        nxt = MissionState(state)
        if self._state == nxt:
            return
        self.log.info(f'Mission state: {self._state.value} → {nxt.value}')
        self._state = nxt
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self._state.value
        self._state_pub.publish(msg)

    # ---------- 感知 ----------

    def _on_observations(self, msg: PeachTargetObservationArray):
        """缓存最新观测（锁集判定与 target_id 收集在锁存消息上做）."""
        self.observations = msg
        if msg.scene_epoch:
            self.scene_epoch = msg.scene_epoch

    def wait_observations_lock(self, timeout_s: float) -> list[str]:
        """
        等当前世代窗口锁集，两层防竞态.

        两层防竞态：按 scene_epoch 过滤旧世代；且必须先见过本世代
        ``collecting``（未锁）消息——begin_scene 会立刻广播一条新 epoch、
        携带旧锁标志的空状态消息，直接采信会秒判空集（实测 1.3s 跳过）。
        """
        self.observations = None
        epoch = self.scene_epoch
        saw_collecting = False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            obs = self.observations
            if obs is None or obs.scene_epoch != epoch:
                continue
            if not obs.target_set_locked:
                saw_collecting = True
                continue
            if saw_collecting:
                return [entry.target_id for entry in obs.observations]
        latest = self.observations
        if latest is not None and latest.scene_epoch == epoch:
            return [entry.target_id for entry in latest.observations]
        return []

    def call_begin_scene(self, request_id: str) -> bool:
        """开一世代采集窗口；成功则记录 scene_epoch."""
        if not self.begin_scene.wait_for_service(timeout_sec=30.0):
            self.log.error('begin_scene 服务 30s 未就绪（perception.launch 起了吗）')
            return False
        req = BeginScene.Request()
        req.request_id = request_id
        req.scene_key = f'sim_harvest/{request_id}'
        future = self.begin_scene.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        resp = future.result()
        ok = bool(resp and resp.accepted)
        if ok:
            self.scene_epoch = resp.scene_epoch
        self.log.info(f'begin_scene accepted={ok} epoch={self.scene_epoch}')
        return ok

    def call_build_model(self, target_id: str) -> object | None:
        """Build_target_model 单目标动作；成功返回 TargetModel."""
        if not self.build_model.wait_for_server(timeout_sec=30.0):
            self.log.error('build_target_model 服务 30s 未就绪')
            return None
        goal = BuildTargetModel.Goal()
        goal.request_id = f'harvest_{target_id}'
        goal.target_id = target_id
        goal.scene_epoch = self.scene_epoch
        future = self.build_model.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log.warning(f'{target_id} build goal 未受理')
            return None
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.args.build_timeout
        while time.monotonic() < deadline:
            # timeout_sec>0 在 use_sim_time 且 /clock 没进本进程时永不返回。
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if result_future.done():
                break
            time.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            self.log.warning(f'{target_id} build 超时（{self.args.build_timeout}s）')
            return None
        result = result_future.result().result
        if not result.success:
            self.log.warning(
                f'{target_id} build 失败：{result.failure_code} {result.message}')
            return None
        self.log.info(
            f'{target_id} 模型成功 quality={result.quality_level} '
            f'views={result.model.view_count}')
        return result.model

    # ---------- 臂运动（MoveGroup action，语义同 peach_arm MGI 用法） ----------

    def _move_goal(self, request: MotionPlanRequest) -> MoveGroup.Goal:
        """Wrap a MoveGroup goal (plan_only=False, no replan)."""
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        return goal

    def _ptp_request(self) -> MotionPlanRequest:
        """Pilz PTP request skeleton (group/pipeline/scaling same as peach_arm)."""
        req = MotionPlanRequest()
        req.group_name = 'manipulator_e5'
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'PTP'
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15
        return req

    def move_to_named(self, state_name: str, timeout_s: float = 60.0) -> bool:
        """Move PTP 到 SRDF 命名态（关节约束自展开，值快照见 SRDF_STATES）."""
        if state_name not in SRDF_STATES:
            raise ValueError(
                f'未知 SRDF 命名态 {state_name!r}（可选：'
                f'{sorted(SRDF_STATES)}）')
        req = self._ptp_request()
        constraints = Constraints()
        for joint, value in zip(ARM_JOINTS, SRDF_STATES[state_name]):
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]
        return self._execute(self._move_goal(req), state_name, timeout_s)

    def move_tcp_pose(self, pose: Pose, frame: str, label: str,
                      pipeline: str, planner: str,
                      timeout_s: float = 60.0) -> bool:
        """TCP 完整位姿目标。LIN 用 1 mm 盒 + 0.01 rad 姿态（Pilz 契约）."""
        lin = pipeline == PILZ and planner == 'LIN'
        req = MotionPlanRequest()
        req.group_name = 'manipulator_e5'
        req.pipeline_id = pipeline
        req.planner_id = planner
        req.allowed_planning_time = 2.0 if lin else 5.0
        req.num_planning_attempts = 1 if lin else 4
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15
        if lin:
            req.cartesian_speed_limited_link = 'tcp'
            req.max_cartesian_speed = 0.1
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = 'tcp'
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        edge = 0.001 if lin else 0.02
        box.dimensions = [edge, edge, edge]
        pc.constraint_region.primitives.append(box)
        region_pose = Pose()
        region_pose.position = pose.position
        region_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = 'tcp'
        oc.orientation = pose.orientation
        tolerance = 0.01 if lin else 0.05
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = tolerance
        oc.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)
        return self._execute(self._move_goal(req), label, timeout_s)

    def _execute(self, goal: MoveGroup.Goal, label: str,
                 timeout_s: float) -> bool:
        """发 goal 并等执行结果（超时取消；error_code 1=成功）."""
        if not self.move_group.wait_for_server(timeout_sec=30.0):
            self.log.error('move_group 30s 未就绪（sim.launch moveit:=true）')
            return False
        future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.log.warning(f'{label} goal 未受理')
            return False
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if result_future.done():
                break
        if not result_future.done():
            handle.cancel_goal()
            self.log.warning(f'{label} 执行超时（{timeout_s}s）')
            return False
        error_code = result_future.result().result.error_code.val
        ok = error_code == 1  # MoveItErrorCodes.SUCCESS
        self.log.info(f'{label} {"执行成功" if ok else f"失败 error={error_code}"}')
        return ok

    # ---------- 主循环 ----------

    def run(self, plan: list) -> int:
        """主循环：逐站导航 + 采摘节拍（细节在 harvest_at_station）."""
        self.stop_base()
        self.set_state(MissionState.LOCALIZE)
        self.log.info('等待 Nav2 激活…')
        self.navigator.waitUntilNav2Active(localizer='amcl')
        self.set_state(MissionState.STOW)
        self.log.info(f'Nav2 就绪；{len(plan)} 站；先收臂到 harvest_stow')
        if not self.move_to_named('harvest_stow'):
            self.log.warning('收纳位失败，带当前臂姿继续')

        for number, station in enumerate(plan, start=1):
            self.stats['stations'] += 1
            tag = f'[{number}/{len(plan)}] {station.tree_id}'
            self.set_state(MissionState.NAVIGATE)
            yaw = station.yaw
            fruit_dx = getattr(station, 'fruit_dx', None)
            park_dx = park_dy = 0.0
            if fruit_dx is not None:
                park_dx, park_dy, aim_dx, aim_dy = park_toward_fruit(
                    fruit_dx, station.fruit_dy, self.args.arm_reach)
                yaw = fruit_survey_yaw(aim_dx, aim_dy)
            elif self.args.stations_file:
                # 清单里没有这棵树的袋时，退回对准树干。
                yaw = survey_yaw(-self.args.tree_side * self.args.row_gap, 0.0)
            goal_x = station.x + park_dx
            goal_y = station.y + park_dy
            self.log.info(
                f'{tag} 导航 → ({goal_x:.2f}, {goal_y:.2f}) yaw={yaw:.2f}'
                + (f' 袋心距停车点 '
                   f'{math.hypot(fruit_dx - park_dx, station.fruit_dy - park_dy):.2f} m'
                   if fruit_dx is not None else ' 无袋心，对准树干'))
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            # 目标向外（走廊侧）让出 goal_clearance：站点离致命带仅 ~0.3 m，
            # AMCL 噪声会把落点推进死区（NavFn "Failed to create plan" 实证）
            goal.pose.position.x = goal_x + self.side * self.args.goal_clearance
            goal.pose.position.y = goal_y
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.navigator.goToPose(goal)
            deadline = time.monotonic() + self.args.nav_timeout
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if time.monotonic() > deadline:
                    self.navigator.cancelTask()
                    break
            self.stop_base()
            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.log.warning(f'{tag} 未到站，跳过本站采摘')
                continue
            self.stats['nav_ok'] += 1
            self.log.info(f'{tag} 到站，开始采摘节拍')
            self.harvest_at_station(tag, station.tree_id)
        self.set_state(
            MissionState.DONE if self.stats['nav_ok'] or not plan
            else MissionState.FAILED)
        return 0

    def stop_base(self):
        """连发零速度，清掉 gz VelocityControl 锁存的上一条 cmd_vel."""
        end = time.monotonic() + 0.4
        while time.monotonic() < end:
            self._cmd_pub.publish(Twist())
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def harvest_at_station(self, tag: str, tree_id: str):
        """到站后：感知出 id → 逐颗重建、拍照、抓取、回程."""
        self.set_state(MissionState.SURVEY)
        if not self.move_to_named('global_photo_pose'):
            self.log.warning(f'{tag} 拍照位失败，跳过感知')
            return
        if not self.call_begin_scene(f'{tree_id}_{int(time.time())}'):
            return
        target_ids = [
            target_id for target_id in
            self.wait_observations_lock(self.args.scene_timeout)
            if target_id]
        if not target_ids:
            self.log.warning(f'{tag} 没有感知目标和 id，不重建')
            return
        self.stats['scenes_locked'] += 1
        self.log.info(f'{tag} 感知 {len(target_ids)} 个 id：{target_ids}')
        for target_id in target_ids[: self.args.max_targets]:
            self._harvest_one_target(tag, target_id)
        self.set_state(MissionState.RETURN)
        if not self.move_to_named('harvest_stow'):
            self.log.warning(f'{tag} 离站收纳失败，带当前臂姿去下一站')

    def _harvest_one_target(self, tag: str, target_id: str):
        """一颗：重建 → 拍照位 → 抓取 → 回拍照位。没有模型就换下一颗."""
        self.set_state(MissionState.PLAN_HARVEST)
        model = self.call_build_model(target_id)
        if model is None:
            self.log.warning(f'{tag} {target_id} 无模型，下一颗')
            return
        self.stats['models_ok'] += 1
        if not self.move_to_named('global_photo_pose'):
            self.log.warning(f'{tag} {target_id} 抓取前拍照位失败，下一颗')
            return
        self.set_state(MissionState.HARVESTING)
        if self.approach(tag, target_id, model):
            self.stats['approach_ok'] += 1
        self.set_state(MissionState.RETURN)
        if not self.move_to_named('global_photo_pose'):
            self.log.warning(f'{tag} {target_id} 回程拍照位失败')

    def approach(self, tag: str, target_id: str, model) -> bool:
        """OMPL 到预接近，Pilz LIN 沿袋轴到 standoff。TCP +Z 指向袋颈."""
        bottom: Point = model.bag_bottom
        axis: Vector3 = model.bag_axis
        ux, uy, uz = _unit(axis.x, axis.y, axis.z)
        if math.sqrt(axis.x ** 2 + axis.y ** 2 + axis.z ** 2) < 1e-6:
            self.log.warning(f'{target_id} 轴退化，用竖直轴')
        qx, qy, qz, qw = tool_orientation((ux, uy, uz))
        standoff = self.args.approach_standoff
        lead = self.args.approach_lead

        def _at(distance: float) -> Pose:
            pose = Pose()
            pose.position.x = bottom.x - ux * distance
            pose.position.y = bottom.y - uy * distance
            pose.position.z = bottom.z - uz * distance
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            return pose

        frame = model.header.frame_id or 'base_link'
        goal = _at(standoff)
        reach = math.sqrt(goal.position.x ** 2 + goal.position.y ** 2
                           + goal.position.z ** 2)
        self.log.info(
            f'{tag} {target_id} LIN 终点 {frame} '
            f'({goal.position.x:.3f}, {goal.position.y:.3f}, '
            f'{goal.position.z:.3f}) 距臂基 {reach:.2f} m '
            f'd95={model.d95_m * 1000:.0f}mm')
        if lead > 0.005:
            if not self.move_tcp_pose(
                    _at(standoff + lead), frame, f'{target_id}-approach',
                    'ompl', 'RRTConnect'):
                return False
        return self.move_tcp_pose(
            goal, frame, f'{target_id}-lin', PILZ, 'LIN')

    def summary(self) -> int:
        """结束统计；返回码按到站率（全败=1）."""
        self.log.info(
            '巡航采摘结束：' + json.dumps(self.stats, ensure_ascii=False))
        return 0 if self.stats['stations'] == 0 else (
            0 if self.stats['nav_ok'] > 0 else 1)


def main(argv: list[str] | None = None) -> int:
    """CLI 入口：解析场景/行/上限参数并跑 HarvestRunner."""
    share = default_share()
    parser = argparse.ArgumentParser(
        prog='orchard_harvest',
        description='自主导航采摘演示：逐站导航 → 感知锁集 → 建模 → 臂接近'
                    '（须先起 harvest.launch 或 sim+perception+nav2）')
    parser.add_argument(
        '--params', type=Path, default=share / 'config' / 'orchard.yaml')
    parser.add_argument('--rows', default='', help='行号逗号分隔（默认全部）')
    parser.add_argument('--max-stations', type=int, default=0,
                        help='最多停靠站数（0=不限制；演示常用 3）')
    parser.add_argument('--max-targets', type=int, default=1,
                        help='每站建模/接近的目标数上限')
    parser.add_argument('--nav-timeout', type=float, default=240.0,
                        help='单站导航墙钟秒数（RTF<1 时仿真时间预算减半，放宽）')
    parser.add_argument('--scene-timeout', type=float, default=90.0,
                        help='等感知窗口锁集的墙钟秒数（低 RTF 放宽）')
    parser.add_argument('--build-timeout', type=float, default=150.0)
    parser.add_argument('--tree-side', type=int, default=1,
                        help='field 站位相对树干的 x 侧（generate_field --side，默认 +1）')
    parser.add_argument('--row-gap', type=float, default=0.9,
                        help='field 停靠位离树干的 x 距离 (m)')
    parser.add_argument('--arm-reach', type=float, default=0.55,
                        help='停车点到袋心的水平距离 (m)，把目标留在 E5 臂展内')
    parser.add_argument('--approach-standoff', type=float, default=0.12,
                        help='Pilz LIN 终点离袋底沿轴距离 (m)')
    parser.add_argument('--approach-lead', type=float, default=0.10,
                        help='OMPL 预接近比 LIN 终点再退的距离 (m)')
    parser.add_argument('--goal-clearance', type=float, default=None,
                        help='导航目标让出致命带的余量 (m)。默认：程序化世界 '
                             '0.12（站点 0.95 离致命带 ~0.3）；field 站点文件 '
                             '1.05 行距已留 0.14，默认 0')
    parser.add_argument('--all-stations', action='store_true',
                        help='巡全部树（默认只访 manifest reachable>0 的有果站'
                             '——行内多数树无可达袋，锁空集是正确感知行为）')
    parser.add_argument(
        '--manifest', type=Path,
        default=share / 'worlds' / 'peach_orchard.manifest.yaml',
        help='目标清单 GT（reachable 过滤用；generate_orchard 产物）')
    parser.add_argument(
        '--stations-file', type=Path, default=None,
        help='停靠位 yaml（generate_field 产物；给出则替代 tour_core 计算，'
             'field 世界模式）。default 须为 None 而非空串：Path("")=='
             'Path(".") 恒真值，空串默认会误入 field 分支并对目录 '
             'read_text 抛 IsADirectoryError（2026-10-15 实测）')
    args = parser.parse_args(argv)

    raw = yaml.safe_load(args.params.read_text(encoding='utf-8'))
    params, problems = params_from_dict(raw if isinstance(raw, dict) else {})
    problems += check_params(params)
    if problems:
        print(f'{args.params}: 场景参数不合法：' + '；'.join(problems))
        return 2
    if args.stations_file:
        data = yaml.safe_load(args.stations_file.read_text(encoding='utf-8'))
        manifest = _resolve_manifest(args.stations_file, data.get('manifest'))
        centroids = bag_centroids(manifest) if manifest is not None else {}
        plan = []
        for station in data['stations']:
            fruit_dx = fruit_dy = None
            tree_id = str(station['tree_id'])
            if tree_id.startswith('field_tree_') and tree_id[11:].isdigit():
                center = centroids.get(int(tree_id[11:]))
                if center is not None:
                    fruit_dx = center[0] - float(station['x'])
                    fruit_dy = center[1] - float(station['y'])
            plan.append(SimpleNamespace(
                tree_id=tree_id, index=station.get('index', 0),
                x=float(station['x']), y=float(station['y']),
                yaw=float(station['yaw']),
                bag_count=int(station.get('bag_count', 0)),
                fruit_dx=fruit_dx, fruit_dy=fruit_dy))
        if not args.all_stations:
            plan = [s for s in plan if s.bag_count > 0]
        print(f'站点文件：{args.stations_file}，{len(plan)} 站'
              f'（有袋 {[s.tree_id for s in plan][:6]}…）')
    else:
        rows = ([int(item) for item in args.rows.split(',') if item.strip()]
                if args.rows else list(range(params.rows.count)))
        plan = stations_by_row(params, rows)
    if not args.stations_file and not args.all_stations:
        manifest = yaml.safe_load(
            args.manifest.read_text(encoding='utf-8'))
        fruitful = {entry['tree'] for entry in manifest['targets']
                    if entry.get('reachable')}
        keep = []
        for item in plan:
            if item.index == -1:
                # 换行航点：仅当两侧行都仍有有果站才保留
                next_ok = any(s.index != -1 and s.row == item.row + 1
                              and s.tree_id in fruitful for s in plan)
                cur_ok = any(s.index != -1 and s.row == item.row
                             and s.tree_id in fruitful for s in plan)
                if next_ok and cur_ok:
                    keep.append(item)
            elif item.tree_id in fruitful:
                keep.append(item)
        print(f'有果站过滤：{len(plan)} → {len(keep)} 站 '
              f'（{sorted(fruitful & {s.tree_id for s in plan})}）')
        plan = keep
    if args.max_stations > 0:
        plan = plan[: args.max_stations]

    # use_sim_time 必须在节点构造时生效。事后 set_parameters 不会让
    # spin_once 的超时跟着 /clock 走，建模型时会一直卡在第一次 spin。
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node(
        'orchard_harvest',
        parameter_overrides=[Parameter(
            'use_sim_time', Parameter.Type.BOOL, True)])
    if args.goal_clearance is None:
        args.goal_clearance = 0.0 if args.stations_file else 0.12
    side = -1 if args.stations_file else params.platform.side
    runner = HarvestRunner(node, args, side=side)
    try:
        rc = runner.run(plan)
        rc = rc or runner.summary()
    except KeyboardInterrupt:
        runner.set_state(MissionState.STOPPED)
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
