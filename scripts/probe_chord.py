#!/usr/bin/env python3
"""诊断：拍照位→预抓取/staging 直弦笛卡尔到底断在哪、为什么。

对指定用例逐滚转调 /compute_cartesian_path，报 fraction 与断点位置
（= 有效前缀末端）。纯只读诊断，不动臂。不进 colcon test。
"""
import math
import sys

import yaml

CASES_PATH = (
    '/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_arm/config/'
    'field_pregrasp_cases.yaml')
JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
PHOTO_JOINTS = (0.425083, 0.195177, 1.677740, 1.461739, -0.500161, 0.038621)
ROLLS = (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180)


def quat_to_mat(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def mat_to_quat(m):
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s, (m[1][0] - m[0][1]) / s)


def matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3))


def align_z(rot, axis):
    z = (rot[0][2], rot[1][2], rot[2][2])
    n = math.sqrt(sum(v * v for v in axis))
    a = tuple(v / n for v in axis)
    c = max(-1.0, min(1.0, sum(z[i] * a[i] for i in range(3))))
    v = (z[1] * a[2] - z[2] * a[1], z[2] * a[0] - z[0] * a[2],
         z[0] * a[1] - z[1] * a[0])
    vn = math.sqrt(sum(x * x for x in v))
    if vn < 1e-9:
        return rot if c > 0 else matmul(quat_to_mat((1, 0, 0, 0)), rot)
    angle = math.acos(c)
    k = tuple(x / vn for x in v)
    s = math.sin(angle / 2)
    dq = (k[0] * s, k[1] * s, k[2] * s, math.cos(angle / 2))
    return matmul(quat_to_mat(dq), rot)


def roll_z(rot, deg):
    half = math.radians(deg) / 2.0
    rz = quat_to_mat((0, 0, math.sin(half), math.cos(half)))
    return matmul(rot, rz)


def main():
    cases = yaml.safe_load(open(CASES_PATH))['targets_20260909']
    wanted = sys.argv[1:] or ['1437_0', '1503_0']

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('chord_probe')
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    fk_cli.wait_for_service(timeout_sec=10)
    cart_cli.wait_for_service(timeout_sec=10)

    def spin(fut, timeout=20.0):
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    def fk(joints):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tcp']
        req.robot_state.joint_state.name = list(JOINT_ORDER)
        req.robot_state.joint_state.position = list(joints)
        res = spin(fk_cli.call_async(req), 5.0)
        p = res.pose_stamped[0].pose
        return p

    photo_pose = fk(PHOTO_JOINTS)
    photo_xyz = (photo_pose.position.x, photo_pose.position.y,
                 photo_pose.position.z)
    photo_q = (photo_pose.orientation.x, photo_pose.orientation.y,
               photo_pose.orientation.z, photo_pose.orientation.w)
    photo_rot = quat_to_mat(photo_q)
    print(f'photo tcp = ({photo_xyz[0]:.3f},{photo_xyz[1]:.3f},'
          f'{photo_xyz[2]:.3f})')

    def cart_probe(goal_xyz, goal_rot, label):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.start_state.joint_state.name = list(JOINT_ORDER)
        req.start_state.joint_state.position = list(PHOTO_JOINTS)
        req.group_name = 'manipulator_e5'
        req.link_name = 'tcp'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = goal_xyz
        q = mat_to_quat(goal_rot)
        pose.orientation.x, pose.orientation.y = q[0], q[1]
        pose.orientation.z, pose.orientation.w = q[2], q[3]
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        res = spin(cart_cli.call_async(req), 30.0)
        if res is None:
            print(f'  {label}: 无响应')
            return
        frac = res.fraction
        sol = res.solution.joint_trajectory
        stop = ''
        if sol.points:
            last_fk = None
            for pt in (sol.points[-1],):
                js = JointState()
                js.name = list(sol.joint_names)
                js.position = list(pt.positions)
                req2 = GetPositionFK.Request()
                req2.header.frame_id = 'base_link'
                req2.fk_link_names = ['tcp']
                req2.robot_state.joint_state = js
                r2 = spin(fk_cli.call_async(req2), 5.0)
                p = r2.pose_stamped[0].pose.position
                last_fk = (p.x, p.y, p.z)
            stop = (f' 断点=({last_fk[0]:.3f},{last_fk[1]:.3f},'
                    f'{last_fk[2]:.3f})')
        chord = math.dist(photo_xyz, goal_xyz)
        print(f'  {label}: fraction={frac:.3f} '
              f'弦长={chord:.3f}m err={res.error_code.val}{stop}')

    for cid in wanted:
        case = cases[cid]
        entry = case['entry_xyz']
        axis = case['axis']
        n = math.sqrt(sum(v * v for v in axis))
        axis = tuple(v / n for v in axis)
        pregrasp = tuple(entry[i] - 0.03 * axis[i] for i in range(3))
        staging = tuple(entry[i] - 0.16 * axis[i] for i in range(3))
        aligned = align_z(photo_rot, axis)
        print(f'\n== {cid} axis_z={axis[2]:.2f} pregrasp='
              f'({pregrasp[0]:.3f},{pregrasp[1]:.3f},{pregrasp[2]:.3f})')
        print(' 直弦 photo→pregrasp（逐滚转）:')
        for deg in ROLLS:
            cart_probe(pregrasp, roll_z(aligned, deg), f'roll{deg:+04d}')
        print(' 直弦 photo→staging（逐滚转）:')
        for deg in (0, 30, -30, 60, -60, 90, 180):
            cart_probe(staging, roll_z(aligned, deg), f'roll{deg:+04d}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
