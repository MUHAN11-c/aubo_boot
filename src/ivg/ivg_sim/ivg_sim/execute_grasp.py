#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ivg_sim 抓取执行闭环：检测位姿 → IK → approach/descend/close/lift → 位移核验.

运行链（系统 python，无 venv/torch 依赖；检测节点在 venv 侧独立进程）：
  1. 订阅 grasp_poses（world 帧）后触发 /graspnet_capture_control
  2. 对 GT manifest 目标对象选最近抓取（偏好接近竖直的 approach）
  3. MoveItPy（OMPL）规划 transit 至抓取上方安全点（场景已注入桌+对象盒）
  4. approach/下降/抬升 = 逐路点 KDL IK（连续种子）+ 直接下发
     FollowJointTrajectory（本版 moveit_py 无 cartesian 接口）
  5. 夹爪经 JointGroupPositionController 话题开合
  6. gz dynamic_pose 流核验对象 Δz（抬离桌面 = 抓取成功）

用法：ros2 run ivg_sim execute_grasp --target potted_meat_can [--keep]
"""

import argparse
import json
import re
import subprocess
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ivg_sim.arm_description import (
    ARM_JOINTS,
    FINGER_GRASP_POSE,
    FINGER_OPEN_POSE,
    SPAWN_XYZ,
    worlds_dir,
)
from ivg_sim.moveit_params import load_manifest

GROUP = 'manipulator_e5'
# 拍照位（stop-and-shoot）：真机示教 global_photo_pose（SRDF 2026-09-17）
PHOTO_POSE = (0.425175, 0.194274, 1.679546, 1.462579, -0.279948, 0.038859)
# transit IK 种子 = SRDF home（肘抬起构型；零位种子会给肘部下沉解，
# 桌上对象盒会拒掉该族目标态）
HOME_SEED = (0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0)  # SRDF home
APPROACH_DIST = 0.05    # 与检测 postprocess.approach_dist 同值
ENVELOP_RISE = 0.01     # 包络抓取：TCP 位于对象中心上方 1cm（四指包住上部）
SAFE_HOVER = 0.25       # transit 终点在抓取点上方的高度（m，world z）
LIFT_H = 0.15           # 抬升高度
DESCEND_STEP = 0.01     # 下降路点步长
MIN_VERT = 0.55         # 目标抓取 approach 竖直度下限（pose z 轴 · world z）
MAX_CENTER_OFF = 0.02   # 目标抓取偏心上限：指内侧可达 ≈±0.056，对象半径 0.05
                        # 时偏心 >0.02 会在下降途中扫飞对象（2026-09-30 实测）
POSES_TIMEOUT_S = 120.0  # 首轮 CUDA 推理 ~45s + 采样
JOINT_VEL = 1.0         # 手工轨迹限速（rad/s，低于 joint_limits.yaml 上限）
# gz 实体名白名单（生成器产物 obj_NN_model；CLI --target 只做子串匹配，
# 出实体名后仍须过此校验才可进入后续字符串使用）
ENTITY_RE = re.compile(r'^[A-Za-z0-9_]+$')


def validate_entity(name: str) -> str:
    if not ENTITY_RE.match(name):
        raise SystemExit('非法实体名')
    return name


def quat_matrix(quat) -> np.ndarray:
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m: np.ndarray):
    tr = 1.0 + m[0, 0] + m[1, 1] + m[2, 2]
    if tr <= 1e-8:
        raise RuntimeError('四元数退化（TF 姿态恒等下理论不可达）')
    s = 0.5 / tr ** 0.5
    return (
        float((m[2, 1] - m[1, 2]) * s),
        float((m[0, 2] - m[2, 0]) * s),
        float((m[1, 0] - m[0, 1]) * s),
        float(0.25 / s),
    )


def apply_tf(tf, pose: Pose) -> Pose:
    """pose 经静态 TF 变换（本栈 TF 姿态恒等，通用实现保留旋转）."""
    out = Pose()
    t = tf.transform.translation
    r = quat_matrix(tf.transform.rotation)
    v = r @ np.array(
        [pose.position.x, pose.position.y, pose.position.z]) \
        + np.array([t.x, t.y, t.z])
    out.position.x, out.position.y, out.position.z = v
    x, y, z, w = matrix_to_quat(r @ quat_matrix(pose.orientation))
    out.orientation.x, out.orientation.y = x, y
    out.orientation.z, out.orientation.w = z, w
    return out


def make_pose(x: float, y: float, z: float, quat_src) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation = quat_src
    return p


def pick_grasp(poses: PoseArray, target_xy, table_top_z: float):
    """选目标对象最近抓取：高度窗过滤 → 竖直度优先 → XY 最近."""
    tx, ty = target_xy
    ranked = []
    for p in poses.poses:
        d = float(np.hypot(p.position.x - tx, p.position.y - ty))
        if not (table_top_z - 0.02 <= p.position.z <= table_top_z + 0.45):
            continue
        vert = float(quat_matrix(p.orientation)[2, 2])
        ranked.append((d, vert, p))
    if not ranked:
        raise SystemExit('抓取位姿全在目标高度窗外')
    good = [r for r in ranked if r[1] >= MIN_VERT and r[0] <= MAX_CENTER_OFF]
    pool = good or ranked
    pool.sort(key=lambda r: r[0])
    d, vert, p = pool[0]
    return p, vert, d


def gz_model_pose(entity: str):
    """gz dynamic_pose 流读实体 (x,y,z)（文本 proto；一次性命令）."""
    out = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/ivg_table/dynamic_pose/info',
         '-n', '1'],
        capture_output=True, text=True, timeout=20)
    blocks = re.split(r'name:\s*"', out.stdout)
    for b in blocks[1:]:
        name = b.split('"', 1)[0]
        if name == entity:
            seg = b.split('position', 1)[-1][:260]
            vals = re.findall(r'[xyz]:\s*([-\d.eE+]+)', seg)
            if len(vals) >= 3:
                return tuple(float(v) for v in vals[:3])
    raise RuntimeError('gz 位姿流中未找到实体 ' + entity)


def interpolate_line(p0: Pose, p1: Pose, step: float) -> list:
    """p0→p1 直线插值路点（姿态保持 p1；步长 step）."""
    a0 = np.array([p0.position.x, p0.position.y, p0.position.z])
    a1 = np.array([p1.position.x, p1.position.y, p1.position.z])
    dist = float(np.linalg.norm(a1 - a0))
    steps = max(1, int(round(dist / step)))
    pts = []
    for i in range(1, steps + 1):
        a = i / steps
        v = a0 * (1 - a) + a1 * a
        pts.append(make_pose(v[0], v[1], v[2], p1.orientation))
    return pts


def ik_via_service(node, ik_cli, pose: Pose, seed) -> list:
    """/compute_ik 单点解（种子=上一解；失败即 SystemExit）."""
    req = GetPositionIK.Request()
    req.ik_request.group_name = GROUP
    req.ik_request.ik_link_name = 'tcp'
    req.ik_request.pose_stamped.header.frame_id = 'base_link'
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.timeout.sec = 0
    req.ik_request.timeout.nanosec = 200_000_000
    req.ik_request.robot_state.joint_state.name = list(ARM_JOINTS)
    req.ik_request.robot_state.joint_state.position = [float(q) for q in seed]
    fut = ik_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(
            f'IK 失败 @ ({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f})')
    js = res.solution.joint_state
    return [js.position[js.name.index(j)] for j in ARM_JOINTS]


def fk_tcp(node, fk_cli, q) -> Pose:
    """/compute_fk 读 q 下 tcp 位姿（base 帧）."""
    req = GetPositionFK.Request()
    req.header.frame_id = 'base_link'
    req.fk_link_names = ['tcp']
    req.robot_state.joint_state.name = \
        list(ARM_JOINTS)
    req.robot_state.joint_state.position = \
        [float(v) for v in q]
    fut = fk_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res.error_code.val != 1:
        raise SystemExit(f'FK 失败 code={res.error_code.val}')
    return res.pose_stamped[0].pose


def ik_refined(node, pose: Pose, seed) -> list:
    """IK + FK 回验的任务空间修正（KDL 单发解 sloppy 可达 10cm@0.24m 工具，
    2026-09-30 实测；6 次迭代收敛到 ≤5mm，未收敛即中止不硬抓）."""
    target = pose
    q = ik_via_service(node, node.ik, target, seed)
    err_norm = 1.0
    for _ in range(6):
        fk = fk_tcp(node, node.fk, q)
        err = np.array([
            target.position.x - fk.position.x,
            target.position.y - fk.position.y,
            target.position.z - fk.position.z])
        err_norm = float(np.linalg.norm(err))
        if err_norm <= 0.005:
            break
        adj = Pose()
        adj.orientation = target.orientation
        adj.position.x = target.position.x + err[0]
        adj.position.y = target.position.y + err[1]
        adj.position.z = target.position.z + err[2]
        target = adj
        q = ik_via_service(node, node.ik, target, q)
    if err_norm > 0.01:
        raise SystemExit(f'IK 精化未收敛（残差 {err_norm*1000:.1f}mm），中止')
    return q


def ik_waypoints(node, ik_cli, start_joints, pose_list) -> list:
    """逐路点 IK（连续种子，保持直线路径的关节连续性）."""
    seed = [float(q) for q in start_joints]
    out = []
    for pose in pose_list:
        seed = ik_via_service(node, ik_cli, pose, seed)
        out.append(seed)
    return out


def ik_nearest_home(node, ik_cli, pose: Pose, seed=None) -> list:
    """多种子 IK，取距 home 最近的解（单发 KDL 常落腕折叠构型撞桌）."""
    seeds = [] if seed is not None else []
    if seed is not None:
        seeds.append(tuple(float(v) for v in seed))
    seeds += [
        HOME_SEED,
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        (0.4, -0.5, 1.2, -0.3, 1.5, 0.0),
        (-0.4, 0.5, 1.2, 0.3, -1.5, 0.0),
        (0.0, -0.8, 1.6, -0.6, 1.2, 0.0),
    ]
    best, best_dist = None, None
    failures = 0
    ref = tuple(float(v) for v in seed) if seed is not None else HOME_SEED
    for s in seeds:
        try:
            sol = ik_via_service(node, ik_cli, pose, s)
        except SystemExit:
            failures += 1
            continue
        d = max(abs(a - b) for a, b in zip(sol, ref))
        if best_dist is None or d < best_dist:
            best, best_dist = sol, d
    if best is None:
        raise SystemExit(f'IK 全种子失败（{failures}/{len(seeds)}）')
    return best


def transit_to(node, hover: Pose, mid: Pose):
    """transit：两段 JTC（高位中转点折臂 → 高位平转到 hover）.

    v1 不走 OMPL move_action（目标采样持续零产出，2026-09-30 实测）；
    零位→hover 单段直发的关节轨迹会横扫桌面放倒整排对象（相机帧实锤）。
    中转点 mid 由 main 构造（基座 −y 侧上方：不经过桌面、避开相机桁架）。
    """
    q_mid = ik_nearest_home(node, node.ik, mid)
    print(f'[execute] transit q_mid={[round(v, 3) for v in q_mid]}')
    goal_q = ik_nearest_home(node, node.ik, hover, seed=q_mid)
    print(f'[execute] transit goal_q={[round(v, 3) for v in goal_q]}')
    cur = node.current_arm_joints()
    send_jtc(node, node.jtc, [q_mid, goal_q], prev_joints=cur)
    return goal_q


def send_jtc(node, jtc_client, points, prev_joints=None):
    """关节序列下发 JTC（时长按关节增量/限速计算）."""
    traj = JointTrajectory()
    traj.joint_names = list(ARM_JOINTS)
    t, prev = 0.0, prev_joints
    for q in points:
        if prev is not None:
            t += max(0.25, float(np.max(np.abs(
                np.array(q) - np.array(prev)))) / JOINT_VEL)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start.sec = int(t)
        pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points.append(pt)
        prev = q
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = jtc_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        raise SystemExit('JTC 拒绝目标')
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=120.0)
    res = res_fut.result().result
    if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        raise SystemExit(f'JTC 执行失败 code={res.error_code}')


class GraspExecutor(Node):

    def __init__(self):
        super().__init__('ivg_execute_grasp', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.poses = None
        self.joint_state = None
        self.create_subscription(
            PoseArray, 'grasp_poses_base', self._on_poses, 10)
        self.create_subscription(
            JointState, 'joint_states', self._on_js,
            rclpy.qos.qos_profile_sensor_data)
        self.trigger = self.create_client(
            SetBool, '/graspnet_capture_control')
        self.jtc = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.scene_pub = self.create_publisher(
            CollisionObject, '/collision_object', 5)
        self.gripper = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 1)

    def _on_poses(self, msg: PoseArray):
        self.poses = msg

    def _on_js(self, msg: JointState):
        self.joint_state = msg

    def current_arm_joints(self):
        js = self.joint_state
        if js is None:
            raise SystemExit('未收到 joint_states')
        return [js.position[js.name.index(j)] for j in ARM_JOINTS]


def _publish_scene(node, tf, manifest):
    """场景注入：桌面板（base 帧）经 /collision_object 话题.

    只注桌板：对象盒曾把 OMPL 目标态饿死（单发 IK 的肘部下沉/腕折叠
    构型逐个撞盒，2026-09-30 实测两轮不同接触对）。对象级避让由构造
    保证——transit 终点在抓取点上方 0.25 m（高于最高对象 0.85 m）、
    descend 为过目标的垂直直线且指距(0.104 m)大于目标盒宽。

    硬闭环：等订阅匹配→发布→/get_planning_scene 验证桌盒确在场景
    （直接发布会输给 discovery 竞态，本轮曾整段静默丢失）。
    """
    from moveit_msgs.msg import CollisionObject as CO
    from moveit_msgs.srv import GetPlanningScene as GetScene
    table_top = float(manifest['table_top_z'])

    # 等订阅匹配（新节点 discovery ~0.2-1s；不匹配时 volatile 发布全丢）
    for _ in range(50):
        if node.scene_pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise RuntimeError('collision_object 订阅者 5s 未匹配（move_group?）')

    # 清历史对象盒（早期脚本版本曾以 obj_NN_<model> 全名注入；幂等）
    for i, o in enumerate(manifest['objects']):
        rm = CO()
        rm.id = validate_entity(f'obj_{i:02d}_{o["model"]}')
        rm.operation = CO.REMOVE
        node.scene_pub.publish(rm)

    co = CO()
    co.header.frame_id = 'base_link'
    co.id = 'ivg_table'
    pr = SolidPrimitive()
    pr.type = SolidPrimitive.BOX
    pr.dimensions = [0.9, 0.7, 0.05]
    co.primitives.append(pr)
    co.primitive_poses.append(apply_tf(
        tf, make_pose(0.0, 0.0, table_top - 0.025, Pose().orientation)))
    node.scene_pub.publish(co)

    # 验证：桌盒确在场景（world_geometries 组件位查询）
    if not hasattr(node, '_scene_query'):
        node._scene_query = node.create_client(GetScene, '/get_planning_scene')
    node._scene_query.wait_for_service(timeout_sec=5.0)
    req = GetScene.Request()
    req.components.components = 8  # WORLD
    for attempt in range(3):
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        fut = node._scene_query.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        world = fut.result().scene.world
        ids = [o.id for o in world.collision_objects]
        if 'ivg_table' in ids:
            return
        node.scene_pub.publish(co)
    raise RuntimeError(f'桌盒注入验证失败（场景 objects={ids}）')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='potted_meat_can',
                        help='目标对象名（manifest model 子串匹配）')
    parser.add_argument('--keep', action='store_true',
                        help='抬升后不张开手指（保持夹持）')
    args = parser.parse_args()

    manifest = load_manifest(worlds_dir())
    matches = [o for o in manifest['objects'] if args.target in o['model']]
    if not matches:
        raise SystemExit('manifest 无匹配对象（--target）')
    obj = matches[0]
    idx = manifest['objects'].index(obj)
    entity = validate_entity(f'obj_{idx:02d}_{obj["model"]}')
    table_top = float(manifest['table_top_z'])
    print(f'[execute] 目标 {entity} xy={obj["xyz"][:2]}')

    rclpy.init()
    node = GraspExecutor()

    if not node.jtc.wait_for_server(timeout_sec=90.0):
        raise SystemExit('JTC action 90s 未就绪（控制器没起？）')
    print('[execute] JTC 就绪')

    # ── stop-and-shoot（v2 腕上相机，真机同构）──
    # 先到拍照位（真机示教 global_photo_pose，2026-09-17 更新），
    # 相机在腕上（标定外参），到位稳定后再触发采集。
    cur = node.current_arm_joints()
    send_jtc(node, node.jtc, [list(PHOTO_POSE)], prev_joints=cur)
    print('[execute] 已到拍照位，稳定 2s…')
    time.sleep(2.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    p_before = gz_model_pose(entity)
    print(f'[execute] {entity} 初始 p=({p_before[0]:.3f}, {p_before[1]:.3f}, '
          f'{p_before[2]:.3f})')

    while not node.trigger.wait_for_service(timeout_sec=5.0):
        pass  # 上限由下方位姿超时兜底
    fut = node.trigger.call_async(SetBool.Request(data=True))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    print('[execute] 已触发采集，等待位姿…')
    t0 = time.time()
    while node.poses is None or not node.poses.poses:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - t0 > POSES_TIMEOUT_S:
            raise SystemExit('等待抓取位姿超时')
    print(f'[execute] 收到 {len(node.poses.poses)} 条位姿')

    grasp_w, vert, dist = pick_grasp(node.poses, obj['xyz'][:2], table_top)
    print(f'[execute] 选定抓取 dist={dist:.3f} vert={vert:.2f}')
    # 执行位姿吸附（验证单元设计：检测选目标+定 yaw，gz 实时真值供位置）：
    # - 位置=对象实沉中心 p_before（manifest 与实测可差 2cm+，按 manifest
    #   落指会整段落空；检测位姿恒带偏心，>0.02 偏心下降中扫飞对象）；
    # - 姿态=竖直工具向（检测 21° 倾斜使指面有效半隙缩到 2mm，同因扫飞）。

    # 竖直工具向（approach=+z，保留检测宽度轴的 yaw）
    r_mat = quat_matrix(grasp_w.orientation)
    x_col = r_mat[:, 0].copy()
    x_col[2] = 0.0
    norm = float(np.linalg.norm(x_col))
    x_col = x_col / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    z_col = np.array([0.0, 0.0, 1.0])
    y_col = np.cross(z_col, x_col)
    vert_quat = Quaternion()
    (vert_quat.x, vert_quat.y,
     vert_quat.z, vert_quat.w) = matrix_to_quat(
        np.column_stack([x_col, y_col, z_col]))

    grasp_w.position.x = p_before[0]
    grasp_w.position.y = p_before[1]
    grasp_w.position.z = p_before[2] + ENVELOP_RISE
    grasp_w.orientation = vert_quat

    tfb = Buffer(node=node)
    TransformListener(tfb, node)
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = tfb.lookup_transform('base_link', 'world', rclpy.time.Time())
            break
        except Exception:
            continue
    else:
        raise SystemExit('world→base_link TF 不可用')

    # 竖直 approach：pre-grasp 在正上方
    grasp_b = apply_tf(tf, grasp_w)
    pre_b = apply_tf(tf, make_pose(
        grasp_w.position.x, grasp_w.position.y,
        grasp_w.position.z + APPROACH_DIST, vert_quat))

    hover_v = apply_tf(tf, make_pose(
        grasp_w.position.x, grasp_w.position.y,
        grasp_w.position.z + SAFE_HOVER, vert_quat))

    # move_group 就绪（管线/执行管理器 8s 延迟启动）
    if not node.move_group.wait_for_server(timeout_sec=60.0):
        raise SystemExit('move_group action 60s 未就绪')
    if not node.ik.wait_for_service(timeout_sec=10.0):
        raise SystemExit('/compute_ik 不可用')
    print('[execute] move_group / compute_ik 就绪')

    try:
        _publish_scene(node, tf, manifest)
        time.sleep(1.0)
        print('[execute] 场景注入完成（桌+对象盒）')
    except Exception as exc:
        print(f'[execute][WARN] 场景注入失败（transit 无障碍退化）：{exc}')

    open_msg = Float64MultiArray()
    open_msg.data = list(FINGER_OPEN_POSE)
    node.gripper.publish(open_msg)

    # 1) transit：两段（高位扫到 hover 正上方 → 纯竖直下行到 hover；
    #    侧向中转点的第二段会有关节弧线下沉扫落对象，2026-09-30 实测）
    transit_mid_b = apply_tf(tf, make_pose(
        grasp_w.position.x, grasp_w.position.y,
        grasp_w.position.z + SAFE_HOVER + 0.25, vert_quat))
    transit_to(node, hover_v, transit_mid_b)
    print('[execute] transit 完成（hover_v 竖直）')

    time.sleep(1.0)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    cur = node.current_arm_joints()

    # 1b) 原位转姿：竖直 → 抓取姿态（nlerp 小步，高处处无碰）
    # （竖直吸附后两者同姿态，此步为恒等保持，留作后续倾斜抓取复用）
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    cur = node.current_arm_joints()

    # 1c) 实时残差闭环：先到 pre 高度，实测 TCP(world, TF) 与对象实位
    # （gz live）的 XY 残差，把抓取线整体平移补掉（一刀补偿 IK 残差+
    # JTC 追踪偏差；开环曾连续把对象拨离 5-9cm，2026-09-30 实测）
    pre_probe = apply_tf(tf, make_pose(
        grasp_w.position.x, grasp_w.position.y,
        grasp_w.position.z + APPROACH_DIST, vert_quat))
    q_probe = ik_refined(node, pre_probe, cur)
    send_jtc(node, node.jtc, [q_probe], prev_joints=cur)
    time.sleep(1.0)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf_tcp = tfb.lookup_transform('world', 'tcp', rclpy.time.Time())
        tcp_w = tf_tcp.transform.translation
        live = gz_model_pose(entity)
        dx = live[0] - tcp_w.x
        dy = live[1] - tcp_w.y
        print(f'[execute] 残差闭环 Δ=({dx:+.3f}, {dy:+.3f})')
        grasp_w.position.x += dx
        grasp_w.position.y += dy
    except Exception as exc:
        print(f'[execute][WARN] 残差闭环失败（沿开环线继续）：{exc}')
    grasp_b = apply_tf(tf, grasp_w)
    pre_b = apply_tf(tf, make_pose(
        grasp_w.position.x, grasp_w.position.y,
        grasp_w.position.z + APPROACH_DIST, vert_quat))
    cur = q_probe

    # 2) descend：pre-grasp → grasp（密集路点压住关节插值外弓——2 点式
    # 会让指端横摆 3cm+ 撞对象；精化 IK + 连续种子）
    q_pre = ik_refined(node, pre_b, cur)
    descend = [q_pre]
    seed = q_pre
    for p in interpolate_line(pre_b, grasp_b, DESCEND_STEP):
        seed = ik_via_service(node, node.ik, p, seed)
        descend.append(seed)
    send_jtc(node, node.jtc, descend, prev_joints=cur)
    q_grasp = descend[-1]
    print('[execute] descend 完成（TCP 至抓取点）')

    # 3) close（Allegro 四指包络：两段插值闭合——先 65% 缓合到位形，
    #    停 2s 让指节逐个接触（自锁 friction 不回弹），再全位形轻夹。
    #    原包 effort 15N/关节限力，接触即停）
    near = [o + (c - o) * 0.65
            for o, c in zip(FINGER_OPEN_POSE, FINGER_GRASP_POSE)]
    near_msg = Float64MultiArray()
    near_msg.data = near
    node.gripper.publish(near_msg)
    time.sleep(2.5)
    close_msg = Float64MultiArray()
    close_msg.data = list(FINGER_GRASP_POSE)
    node.gripper.publish(close_msg)
    time.sleep(2.0)
    print('[execute] 四指包络闭合完成')

    # 4) lift：world +z（base 姿态恒等，z 向同向）
    lift_b = make_pose(
        grasp_b.position.x, grasp_b.position.y,
        grasp_b.position.z + LIFT_H, grasp_b.orientation)
    qs = ik_waypoints(
        node, node.ik, q_grasp, interpolate_line(
            grasp_b, lift_b, DESCEND_STEP))
    send_jtc(node, node.jtc, qs)
    print('[execute] lift 完成')
    time.sleep(1.0)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)

    # 5) 位移核验（全位移：z-only 判定对横扫盲）
    p_after = gz_model_pose(entity)
    dz = p_after[2] - p_before[2]
    dxy = float(np.hypot(p_after[0] - p_before[0], p_after[1] - p_before[1]))
    if dz > 0.05:
        verdict = '成功（对象被抬离桌面）'
    elif dxy > 0.08:
        verdict = '失败（对象被拨离，未夹持）'
    elif dz > 0.01 or dxy > 0.02:
        verdict = '部分（对象移动但未夹起）'
    else:
        verdict = '失败（对象未动）'
    print(f'[execute] {entity} p: ({p_before[0]:.3f},{p_before[1]:.3f},'
          f'{p_before[2]:.3f}) → ({p_after[0]:.3f},{p_after[1]:.3f},'
          f'{p_after[2]:.3f})  (Δz={dz:+.4f}, Δxy={dxy:.4f}) ⇒ {verdict}')

    if not args.keep:
        node.gripper.publish(open_msg)
        time.sleep(1.5)
    node.destroy_node()
    rclpy.shutdown()
    print('JSON: ' + json.dumps({
        'entity': entity, 'dz': round(dz, 4), 'dxy': round(dxy, 4),
        'verdict': verdict,
        'spawn_xyz': list(SPAWN_XYZ), 'grasp_vert': round(vert, 3),
    }, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
