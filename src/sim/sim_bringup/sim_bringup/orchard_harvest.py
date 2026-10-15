"""
``orchard_harvest`` 自主导航采摘编排（仿真演示，thin 接线层）.

参考口径：导航全走 Nav2 官方 ``nav2_simple_commander``（同 ``orchard_tour``）；
臂运动走 MoveGroup action（同 peach_arm ``MoveGroupInterface`` 的 goal 语义：
组 ``manipulator_e5`` / pilz PTP-LIN / ``base_link`` 参考系 / ``tcp`` 末端）；
感知走 sim_perception 快照的 begin_scene → target_observations →
build_target_model 契约。

每站节拍（Field Robot Event 一类农田机器人标准节拍）：
1. 行驶臂姿 ``harvest_stow``（SRDF 命名态）→ Nav2 NavigateToPose 到站；
2. PTP ``global_photo_pose`` 拍照位 → ``begin_scene`` → 等窗口锁集
   （target_set_locked，自适应窗 ≤ ~25 s 仿真时间）；
3. 逐目标（默认 1 个/站）``build_target_model`` → 接近动作：tcp 到
   袋底沿轴后退 ``approach_standoff``（0.12 m）的接近点（位置约束，
   姿态自由——完整套袋接触链属 P4+，本节点演示导航-感知-运动闭环）；
4. 回拍照位 → 下一站。站点序列复用 ``sim_worlds.tour_core``（蛇形 +
   headland 换行航点）。

仿真专用，不进 harvest_system；权限语义与 peach 真机栈无关。
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
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
from shape_msgs.msg import SolidPrimitive
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
    'global_photo_pose': (0.425175, 0.194274, 1.679546, 1.462579,
                          -0.279948, 0.038859),
}


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
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if result_future.done():
                break
        if not result_future.done():
            handle.cancel_goal()
            self.log.warning(f'{target_id} build 超时（{self.args.build_timeout}s）')
            return None
        result = result_future.result().result
        if not result.success:
            self.log.warning(
                f'{target_id} build 失败：{result.failure_code}')
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

    def move_tcp_to_point(self, point: Point, frame: str,
                          label: str, timeout_s: float = 60.0) -> bool:
        """TCP 位置目标（球容差 2 cm；姿态自由——接近演示不锁姿态）."""
        req = self._ptp_request()
        req.planner_id = 'LIN'
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = 'tcp'
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]
        pc.constraint_region.primitives.append(sphere)
        region_pose = Pose()
        region_pose.position = point
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = 'tcp'
        oc.absolute_x_axis_tolerance = 3.14
        oc.absolute_y_axis_tolerance = 3.14
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 0.0
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
        self.log.info('等待 Nav2 激活…')
        self.navigator.waitUntilNav2Active(localizer='amcl')
        self.log.info(f'Nav2 就绪；{len(plan)} 站；先收臂到 harvest_stow')
        if not self.move_to_named('harvest_stow'):
            self.log.warning('收纳位失败，带当前臂姿继续')

        for number, station in enumerate(plan, start=1):
            self.stats['stations'] += 1
            tag = f'[{number}/{len(plan)}] {station.tree_id}'
            self.log.info(f'{tag} 导航 → ({station.x:.2f}, {station.y:.2f})')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.navigator.get_clock().now().to_msg()
            # 目标向外（走廊侧）让出 goal_clearance：站点离致命带仅 ~0.3 m，
            # AMCL 噪声会把落点推进死区（NavFn "Failed to create plan" 实证）
            goal.pose.position.x = station.x + self.side * self.args.goal_clearance
            goal.pose.position.y = station.y
            goal.pose.orientation.z = math.sin(station.yaw / 2.0)
            goal.pose.orientation.w = math.cos(station.yaw / 2.0)
            self.navigator.goToPose(goal)
            deadline = time.monotonic() + self.args.nav_timeout
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if time.monotonic() > deadline:
                    self.navigator.cancelTask()
                    break
            if self.navigator.getResult() != TaskResult.SUCCEEDED:
                self.log.warning(f'{tag} 未到站，跳过本站采摘')
                continue
            self.stats['nav_ok'] += 1
            self.log.info(f'{tag} 到站，开始采摘节拍')
            self.harvest_at_station(tag, station.tree_id)
        return 0

    def harvest_at_station(self, tag: str, tree_id: str):
        """Per-station harvest cadence: photo pose, scene, models, approaches."""
        if not self.move_to_named('global_photo_pose'):
            self.log.warning(f'{tag} 拍照位失败，跳过感知')
            return
        if not self.call_begin_scene(f'{tree_id}_{int(time.time())}'):
            return
        target_ids = self.wait_observations_lock(self.args.scene_timeout)
        if not target_ids:
            self.log.warning(f'{tag} 窗口锁集为空（深度窗内无袋），跳过')
            return
        self.stats['scenes_locked'] += 1
        self.log.info(f'{tag} 锁集 {len(target_ids)} 目标：{target_ids}')
        for target_id in target_ids[: self.args.max_targets]:
            model = self.call_build_model(target_id)
            if model is None:
                continue
            self.stats['models_ok'] += 1
            if self.approach(tag, target_id, model):
                self.stats['approach_ok'] += 1
        if not self.move_to_named('global_photo_pose'):
            self.log.warning(f'{tag} 收尾回拍照位失败')
        if not self.move_to_named('harvest_stow'):
            self.log.warning(f'{tag} 收尾收纳位失败，带当前臂姿去下一站')

    def approach(self, tag: str, target_id: str, model) -> bool:
        """接近点 = 袋底沿袋轴（bottom→neck 反向）后退 standoff."""
        bottom: Point = model.bag_bottom
        axis: Vector3 = model.bag_axis
        norm = math.sqrt(axis.x ** 2 + axis.y ** 2 + axis.z ** 2)
        if norm < 1e-6:
            self.log.warning(f'{target_id} 轴退化，用竖直轴')
            axis = Vector3(x=0.0, y=0.0, z=1.0)
            norm = 1.0
        standoff = self.args.approach_standoff
        target = Point(
            x=bottom.x - axis.x / norm * standoff,
            y=bottom.y - axis.y / norm * standoff,
            z=bottom.z - axis.z / norm * standoff)
        frame = model.header.frame_id or 'base_link'
        self.log.info(
            f'{tag} {target_id} 接近点 {frame} '
            f'({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) '
            f'd95={model.d95_m * 1000:.0f}mm')
        return self.move_tcp_to_point(target, frame, f'{target_id}-approach')

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
    parser.add_argument('--approach-standoff', type=float, default=0.12,
                        help='tcp 接近点离袋底沿轴距离 (m)')
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
        plan = [SimpleNamespace(
            tree_id=s['tree_id'], index=s.get('index', 0),
            x=float(s['x']), y=float(s['y']), yaw=float(s['yaw']),
            bag_count=int(s.get('bag_count', 0))) for s in data['stations']]
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

    rclpy.init()
    node = Node('orchard_harvest')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    if args.goal_clearance is None:
        args.goal_clearance = 0.0 if args.stations_file else 0.12
    side = -1 if args.stations_file else params.platform.side
    runner = HarvestRunner(node, args, side=side)
    try:
        rc = runner.run(plan)
        rc = rc or runner.summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
