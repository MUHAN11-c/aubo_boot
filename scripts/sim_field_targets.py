#!/usr/bin/env python3
"""mock 全链路回放：注入真机过程记录的目标几何，驱动真实 peach_arm 管线。

与 ``replay_field_pregrasp.py``（Python 侧复刻 MoveIt 官方管线）不同，本脚本
走**真实 C++ 节点**：向感知/重建话题注入 runs/ 账本记录的 entry/axis/底/颈几何，
再对 ``/peach_arm/execute_target`` 发周期（默认 ``PREGRASP_ONLY``；
``--mode full`` 为套入干跑：tool 保持关、须显式 ``PROFILE_FULL``，
否则默认 profile=0 会盖成停预抓取），``skip_observation`` 跳过扫描段，
验证阶段执行器的完整接近/接触设计——
与正式接触段同一条 C++ 路径：``goToPhotoPose``（有记录的接近则原路返程，
否则 Pilz PTP，失败才 OMPL）
→ classifyApproach → 果平面折线 LIN（面内斜插 keep-roll + 沿轴垂直进入；失败才 staging PTP）或 LIN-align+LIN → 预抓取停位验证。脚本**不**再直连 JTC 绕零位复位：连开下一颗时臂停在
上一颗 Hold，由周期内 ``goToPhotoPose`` 回拍照位（与真机同一函数）。
``--random N`` 默认在现场典型包络内采感知合法位姿（上半球且
``axis_z≥0.70``、``|entry|≤1.02``、弦长 ≤ 笛卡尔上限）。感知算法允许水平袋；
``--envelope algorithm`` 才把一次现场近水平袋（1021_1）当成上半球均匀先验。

须已起 ``harvest_system.launch.py hardware_mode:=mock camera_enabled:=false``
（RViz2 随栈启动；lifecycle 拉齐全部节点）。mock 无相机与 io 控制器，本脚本
补发：``/aubo_io_controller/robot_status``（仿真就绪态）与
camera_link→camera_depth_optical_frame 静态 TF。仅仿真使用，不碰真机。

不进 colcon test。

用法：
  python3 scripts/sim_field_targets.py --list
  python3 scripts/sim_field_targets.py --case all      # targets_20260909 全部
  python3 scripts/sim_field_targets.py --case 1437_0 1639_1
  python3 scripts/sim_field_targets.py --mode full --case 1757 --velocity 1.0
  python3 scripts/sim_field_targets.py --grid --mode full --velocity 1.0
  python3 scripts/sim_field_targets.py --random 16 --seed 20260910
  python3 scripts/sim_field_targets.py --random 16 --seed 20260910
  python3 scripts/sim_field_targets.py --random 100 --seed 20260910 --velocity 1.0
  python3 scripts/sim_field_targets.py --random 100 --envelope algorithm --seed 20260911 --velocity 1.0
  python3 scripts/sim_field_targets.py --random 30 --envelope algorithm --seed 20260911
"""
from __future__ import annotations

import argparse
import json
import math
import random
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

CASES_PATH = (
    Path(__file__).resolve().parents[1] /
    'src/peach_arm/test/fixtures/field_pregrasp_cases.yaml')
GRID_PATH = (
    Path(__file__).resolve().parents[1] /
    'src/peach_arm/test/fixtures/perception_constraint_grid.yaml')
NODE = 'peach_arm'
RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'
JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
GOAL_TIMEOUT_S = 300.0
# 与 peach_arm.yaml mtc_approach_cartesian_max_distance_m 对齐。
CART_MAX_M = 0.80
# 感知算法：袋底→袋口只许上半球（axis_z≥0，含水平）。现场 09-09 14 袋里
# 13 袋 axis_z≥0.70；仅 1021_1 ≈0.27 近水平。architecture 现场包络
# |entry|≤1.02 ∧ axis_z≥0.70。算法档保留水平与 |entry|≤1.16 作压测（含现场 1639_0）。
ENTRY_NORM_MAX_M = 1.16
TYPICAL_ENTRY_NORM_M = 1.02
TYPICAL_AXIS_Z_MIN = 0.70
ENTRY_JITTER_M = 0.04
AXIS_TILT_DEG = 20.0
AABB_PAD_M = 0.05
# ①层果实胶囊开关/回退，与 peach_arm.yaml mtc_approach_keepout_* 对齐。
KEEP_R_M = 0.12
KEEP_AXIAL_M = 0.12
# 工具体几何默认值；main() 按 --tool-profile 从 aubo_description 档案覆盖
# （单一事实源，两档案现同尺寸 0.200/0.060，未来分叉不再改这里）。
TOOL_BODY_LENGTH_M = 0.200
TOOL_BODY_RADIUS_M = 0.060
FRUIT_INFLATION_M = 0.01
FRUIT_RADIUS_FLOOR_M = 0.025
# sim 注入的感知直径（米）；analytic 在 25–50 mm 采样。
SIM_BAG_DIAMETER_M = 0.06
# VerifyPregrasp 过门后假设的残差档（注入 GraspDecision 预算复算输入）：
# 横向 5mm / 轴角 1.5° 对齐臂侧验证门，颈位 8mm 为 sim 名义值。
SIM_LATERAL95_M = 0.005
SIM_AXIS_ERR_DEG = 1.5
SIM_NECK95_M = 0.008
TOOL_ARCHIVE_DIR = (
    Path(__file__).resolve().parents[1] /
    'src/aubo_description/config')


def load_tool_archive(profile_id: str) -> dict:
    """读 aubo_description/config/<profile_id>.yaml 的几何/误差字段（纯核）."""
    path = TOOL_ARCHIVE_DIR / f'{profile_id}.yaml'
    if not path.is_file():
        raise FileNotFoundError(f'工具档案不存在: {path}')
    raw = yaml.safe_load(path.read_text()) or {}
    geo = raw.get('geometry_m') or {}
    err = raw.get('error_m') or {}
    if str(raw.get('profile_id')) != profile_id:
        raise ValueError(f'{path} profile_id 与文件名不一致')
    return {
        'd_inner': float(geo['D_inner']),
        'd_outer': float(geo['D_outer']),
        'l_insert': float(geo['L_insert']),
        'wall_clearance': float(geo.get('wall_clearance') or 0.002),
        'blade_capture_half_width': float(
            geo.get('blade_capture_half_width') or 0.008),
        'axial_safety_margin': float(geo.get('axial_safety_margin') or 0.004),
        'tool_runout95': float(err.get('tool_runout95') or 0.001),
        'tcp_calibration_error95': float(
            err.get('tcp_calibration_error95') or 0.002),
        'hand_eye_error95': float(err.get('hand_eye_error95') or 0.003),
        'bag_deformation_margin95': float(
            err.get('bag_deformation_margin95') or 0.002),
        'blade_plane_calibration_error95': float(
            err.get('blade_plane_calibration_error95') or 0.002),
        'robot_axial_error95': float(err.get('robot_axial_error95') or 0.002),
        'target_motion95': float(err.get('target_motion95') or 0.003),
    }


def decision_budget(archive: dict, d_bag95: float, length_m: float) -> dict:
    """按档案 D_inner 复算套入/剪切预算（与感知 tool_budget 同一公式）.

    返回 evaluate_sleeve_cut 同构 dict；tool_budget 从已 source 的 overlay
    惰性导入（脚本运行前提是整栈已在 overlay 上）。
    注意：当前档案误差常数下 axial_margin 结构性为负（blade_capture 8mm <
    axial_safety 4mm + 固定误差合计 7mm），真实 refined 链同式同判——sim
    的 allowed 只执行径向门（sleeve_ok）保持臂链可测；axial 口径问题进
    P3 分析（A 级候选）。
    """
    from peach_harvester.vision.common.tool_budget import (  # noqa: PLC0415
        ToolBudgetParams,
        evaluate_sleeve_cut,
    )
    params = ToolBudgetParams(d_inner=archive['d_inner'])
    return evaluate_sleeve_cut(
        d_bag95=d_bag95,
        length_m=length_m,
        center_lateral95=SIM_LATERAL95_M,
        axis_error_deg=SIM_AXIS_ERR_DEG,
        neck_position95=SIM_NECK95_M,
        cut_to_fruit_m=max(0.02, length_m),
        params=params)


def _in_typical_envelope(entry, axis) -> bool:
    n = math.sqrt(sum(v * v for v in entry))
    return n <= TYPICAL_ENTRY_NORM_M and axis[2] >= TYPICAL_AXIS_Z_MIN


def _norm(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _axis_from_quat(xyzw):
    x, y, z, w = xyzw
    return _norm((2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)))


def _optical_from_link_quat():
    # camera_link(REP-103: x前y左z上) → optical(z前x右y下) 的固定旋转四元数。
    # 列 = optical 基在 link 系像：x=(0,-1,0) y=(0,0,-1) z=(1,0,0)。
    m = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
    return (0.25 * s, (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)


def load_casebook() -> dict:
    return yaml.safe_load(CASES_PATH.read_text())


def load_cases() -> dict:
    return load_casebook()['targets_20260909']


def load_grid_cases() -> dict:
    """感知约束网格：带 expect 标签的在达/超臂展/资格门用例."""
    raw = yaml.safe_load(GRID_PATH.read_text()) or {}
    out = {}
    for cid, case in (raw.get('cases') or {}).items():
        entry = list(case['entry_xyz'])
        axis = _norm(case['axis'])
        travel = float(case.get('travel_m') or SIM_BAG_DIAMETER_M)
        length = float(case.get('bag_length_m') or max(travel, 0.06))
        bottom = list(case.get('bag_bottom') or entry)
        neck = list(case.get('bag_neck') or _add(bottom, _scale(axis, length)))
        out[str(cid)] = {
            **case,
            'run': case.get('run', f'grid_{cid}'),
            'target_id': case.get('target_id', 'target_0'),
            'entry_xyz': entry,
            'axis': axis,
            'bag_bottom': bottom,
            'bag_neck': neck,
            'travel_m': travel,
            'bag_diameter_upper_m': float(
                case.get('bag_diameter_upper_m') or SIM_BAG_DIAMETER_M),
            'expect': str(case.get('expect') or 'succeed'),
            'flags': list(case.get('flags') or []),
        }
    return out


def _normalize_legacy_case(cid: str, raw: dict) -> dict:
    """Fill bag_bottom/neck/travel for the 1757/1740 entry-only fixtures."""
    axis = _norm(raw['axis'])
    entry = [float(v) for v in raw['entry_xyz']]
    travel = float(SIM_BAG_DIAMETER_M)
    out = dict(raw)
    out['run'] = raw.get('run') or raw.get('request_id') or cid
    out['target_id'] = raw.get('target_id') or f'target_{cid}'
    out['entry_xyz'] = entry
    out['axis'] = axis
    out['bag_bottom'] = [float(v) for v in raw.get('pregrasp_xyz', entry)]
    out['bag_neck'] = _add(entry, _scale(axis, travel))
    out['travel_m'] = travel
    return out


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _scale(a, s):
    return [x * s for x in a]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def fruit_radius_m(diameter_m, inflation=FRUIT_INFLATION_M, fallback=KEEP_R_M):
    base = diameter_m / 2.0 if diameter_m > 1.0e-6 else fallback
    return max(FRUIT_RADIUS_FLOOR_M, base) + max(0.0, inflation)


def _segment_segment_distance(p1, q1, p2, q2):
    d1 = _sub(q1, p1)
    d2 = _sub(q2, p2)
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    b = _dot(d1, d2)
    c = _dot(d2, d2)
    f = _dot(r, d1)
    g = _dot(r, d2)
    denom = a * c - b * b
    s = 0.0
    t = 0.0
    if a <= 1.0e-12 and c <= 1.0e-12:
        return math.sqrt(_dot(r, r))
    if a <= 1.0e-12:
        t = min(1.0, max(0.0, g / c))
    elif c <= 1.0e-12:
        s = min(1.0, max(0.0, -f / a))
    else:
        s = min(1.0, max(0.0, (b * g - c * f) / denom)) if denom > 1.0e-12 else 0.0
        t = min(1.0, max(0.0, (b * s + g) / c))
        s = min(1.0, max(0.0, (b * t - f) / a))
    closest = _add(r, _sub(_scale(d1, s), _scale(d2, t)))
    return math.sqrt(_dot(closest, closest))


def _axial_of(point, bottom, axis):
    return _dot(_sub(point, bottom), axis)


def tool_body_hits_fruit(tcp, tool_z, bottom, neck, axis, fruit_r):
    """复刻 grasp_geometry.hpp toolCapsuleClearance ≤ 0（有限圆柱，无端球）。"""
    if KEEP_AXIAL_M <= 1.0e-6 or fruit_r <= 1.0e-6:
        return False
    tail = _sub(tcp, _scale(_norm(tool_z), TOOL_BODY_LENGTH_M))
    t0 = _axial_of(tcp, bottom, axis)
    t1 = _axial_of(tail, bottom, axis)
    f0 = _axial_of(bottom, bottom, axis)
    f1 = _axial_of(neck, bottom, axis)
    amin, amax = (t0, t1) if t0 <= t1 else (t1, t0)
    bmin, bmax = (f0, f1) if f0 <= f1 else (f1, f0)
    if amin > bmax + 1.0e-9 or bmin > amax + 1.0e-9:
        return False
    clearance = _segment_segment_distance(tcp, tail, bottom, neck) - \
        TOOL_BODY_RADIUS_M - fruit_r
    return clearance <= 0.0


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0])


def _bag_length(case) -> float:
    return math.dist(case['bag_bottom'], case['bag_neck'])


def _photo_tcp(book) -> list:
    return [float(v) for v in book.get('photo_tcp_xyz', [0.302, -0.232, 0.708])]


def _aabb(points, pad):
    xs, ys, zs = zip(*points)
    return (
        (min(xs) - pad, max(xs) + pad),
        (min(ys) - pad, max(ys) + pad),
        (min(zs) - pad, max(zs) + pad))


def _clamp_upper_hemisphere(bottom, neck, axis):
    """袋底→袋口只许上半球：base 重力 [0,0,-1]，axis_z ≥ 0（含水平）。

    对齐 peach_perception.common.bag_landmarks.clamp_upper_hemisphere
    与 geometry_refiner.orient_axis_bottom_to_neck：只翻转符号，不把斜袋改竖。
    """
    axis = _norm(axis)
    if _dot(_sub(neck, bottom), axis) < 0.0:
        axis = [-v for v in axis]
    if axis[2] < 0.0:
        return list(neck), list(bottom), [-v for v in axis]
    return list(bottom), list(neck), list(axis)


def _tilt_axis(axis, rng, max_deg):
    axis = _norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    x = _norm(_cross(ref, axis))
    y = _norm(_cross(axis, x))
    theta = math.radians(rng.uniform(-max_deg, max_deg))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    side = _add(_scale(x, math.cos(phi)), _scale(y, math.sin(phi)))
    tilted = _add(_scale(axis, math.cos(theta)), _scale(side, math.sin(theta)))
    if tilted[2] < 0.0:
        tilted = [-v for v in tilted]
    return _norm(tilted)


def _sample_upper_hemisphere(rng, min_z):
    """均匀方位 + 极角夹到现场轴 z 下限（感知允许水平，现场最斜约 0.27）。"""
    floor = max(0.0, float(min_z))
    z = rng.uniform(floor, 1.0)
    rho = math.sqrt(max(0.0, 1.0 - z * z))
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return _norm((rho * math.cos(phi), rho * math.sin(phi), z))


def _pose_ok(entry, axis, photo_tcp, box, axis_z_min, entry_norm_max):
    if any(not math.isfinite(v) for v in entry + axis):
        return False
    if axis[2] < axis_z_min:
        return False
    if math.sqrt(sum(v * v for v in entry)) > entry_norm_max:
        return False
    if math.dist(entry, photo_tcp) > CART_MAX_M:
        return False
    for i, (lo, hi) in enumerate(box):
        if entry[i] < lo or entry[i] > hi:
            return False
    return True


def sample_random_cases(
        templates: dict, n: int, seed: int, photo_tcp,
        envelope: str = 'typical') -> dict:
    """在现场坐标包络内采感知合法随机位姿。

    typical：与 architecture 现场包络一致（axis_z≥0.70、|entry|≤1.02）。
    algorithm：感知 clamp 下界（axis_z≥0）+ |entry|≤1.16，含近水平压测。
    半扰动现场点（入口高斯抖动 + 轴小倾角），半在 AABB 内均匀入口 +
    上半球轴。入口=袋底；颈=底+轴×现场袋长。
    """
    if envelope not in ('typical', 'algorithm'):
        raise ValueError(f'未知 envelope={envelope!r}')
    rng = random.Random(seed)
    ids = list(templates)
    entries = [templates[k]['entry_xyz'] for k in ids]
    box = _aabb(entries, AABB_PAD_M)
    if envelope == 'typical':
        axis_z_min = TYPICAL_AXIS_Z_MIN
        entry_norm_max = TYPICAL_ENTRY_NORM_M
    else:
        axis_z_min = 0.0
        entry_norm_max = ENTRY_NORM_MAX_M
    hemisphere_floor = axis_z_min
    if envelope == 'algorithm':
        hemisphere_floor = min(templates[k]['axis'][2] for k in ids)
        hemisphere_floor = max(0.0, float(hemisphere_floor))
    out = {}
    attempts = 0
    while len(out) < n and attempts < n * 80:
        attempts += 1
        kind = 'perturb' if rng.random() < 0.6 else 'aabb'
        if kind == 'perturb':
            src_id = rng.choice(ids)
            src = templates[src_id]
            entry = [
                src['entry_xyz'][i] + rng.gauss(0.0, ENTRY_JITTER_M)
                for i in range(3)]
            axis = _tilt_axis(src['axis'], rng, AXIS_TILT_DEG)
            length = _bag_length(src)
            travel = float(src.get('travel_m') or 0.0)
        else:
            src_id = rng.choice(ids)
            src = templates[src_id]
            entry = [rng.uniform(lo, hi) for (lo, hi) in box]
            axis = _sample_upper_hemisphere(rng, hemisphere_floor)
            length = _bag_length(src)
            travel = float(src.get('travel_m') or 0.0)
        if length < 0.03:
            length = 0.06
        bottom = list(entry)
        neck = _add(bottom, _scale(axis, length))
        bottom, neck, axis = _clamp_upper_hemisphere(bottom, neck, axis)
        entry = list(bottom)
        if not _pose_ok(
                entry, axis, photo_tcp, box, axis_z_min, entry_norm_max):
            continue
        cid = f'rand_{len(out):02d}'
        out[cid] = {
            'run': 'sim_random',
            'target_id': 'target_0',
            'entry_xyz': [round(v, 6) for v in entry],
            'axis': [round(v, 6) for v in axis],
            'bag_bottom': [round(v, 6) for v in bottom],
            'bag_neck': [round(v, 6) for v in neck],
            'travel_m': round(travel, 6),
            'sample_kind': kind,
            'sampled_from': src_id,
            'envelope': envelope,
        }
    if len(out) < n:
        raise RuntimeError(
            f'随机位姿只采到 {len(out)}/{n}（attempts={attempts}）；'
            '检查现场包络与笛卡尔上限')
    return out


def ensure_enabled(node, velocity: float = 0.0) -> None:
    """运行期改参（空闲态全量生效）：开执行与抓取两档，工具保持关。

    velocity>0 时把接触/转移速度与加速度缩放一并设为该值（typed double 走
    rcl_interfaces SetParameters 服务，参数校验上限 1.0）——仅仿真提速，
    真机验证仍按 yaml 0.1 档，不改默认。
    """
    import rclpy
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.srv import SetParameters

    cli = node.create_client(SetParameters, f'/{NODE}/set_parameters')
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'{NODE} set_parameters 服务不可用')

    def call(entries):
        request = SetParameters.Request()
        request.parameters = entries
        future = cli.call_async(request)
        deadline = time.time() + 15.0
        while time.time() < deadline and rclpy.ok() and not future.done():
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError('set_parameters 服务超时')
        failed = [
            (p.name, r.reason) for p, r in
            zip(entries, future.result().results) if not r.successful]
        if failed:
            raise RuntimeError(f'运行期改参被拒: {failed}')

    def bool_param(name):
        param = ParameterMsg()
        param.name = name
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = True
        return param

    def double_param(name, value):
        param = ParameterMsg()
        param.name = name
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(value)
        return param

    call([bool_param('execution.enabled'), bool_param('grasp.enabled')])
    if 0.0 < velocity <= 1.0:
        call([
            double_param(key, velocity) for key in (
                'moveit.velocity_scaling', 'moveit.acceleration_scaling',
                'moveit.transit_velocity_scaling',
                'moveit.transit_acceleration_scaling')])
    cli.destroy()


def wait_active(timeout_s: float = 120.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        out = subprocess.run(
            ['ros2', 'lifecycle', 'get', f'/{NODE}'],
            capture_output=True, text=True, timeout=20)
        if 'active [3]' in out.stdout:
            return
        time.sleep(2.0)
    raise RuntimeError(f'{NODE} 未在 {timeout_s}s 内 Active')


def ensure_profile_match(expected: str) -> None:
    """fail-fast：臂侧 tool.profile_id 必须与 --tool-profile 一致.

    臂侧真实几何来自 launch（xacro + 档案注入），脚本注入的 profile 只是
    标签；错配时 PregraspVerification 会带错档案标签入账，故启动即拒跑。
    """
    out = subprocess.run(
        ['ros2', 'param', 'get', f'/{NODE}', 'tool.profile_id'],
        capture_output=True, text=True, timeout=20)
    # `ros2 param get` 输出形如 "Parameter value: hollow_cylinder_v1"
    actual = out.stdout.strip().rsplit(' ', 1)[-1]
    if out.returncode != 0 or not actual:
        raise RuntimeError(
            f'无法读取 /{NODE} tool.profile_id（栈未起或参数缺失）: {out.stderr.strip()}')
    if actual != expected:
        raise RuntimeError(
            f'工具档案错配: launch 起的是 {actual}，--tool-profile 给了 {expected}；'
            f'请用 tool_profile:={expected} 重启整栈（档案切换须重启生效）')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--case', nargs='+', default=['all'],
        help='targets_20260909 用例 id，或 cases 段 1757/1740；all=09-09 全部'
             '（与 --random 互斥）')
    parser.add_argument(
        '--mode', choices=('pregrasp', 'full'), default='pregrasp',
        help='pregrasp=ExecuteTarget PREGRASP_ONLY（默认）；'
             'full=FULL 套入干跑（须 PROFILE_FULL；tool 保持关、不 SetIO）')
    parser.add_argument(
        '--random', type=int, default=0, metavar='N',
        help='在现场坐标包络内按感知约束采 N 个随机位姿（默认 typical）')
    parser.add_argument(
        '--envelope', choices=('typical', 'algorithm'), default='typical',
        help='typical=axis_z≥0.70 且 |entry|≤1.02（现场多数袋）；'
             'algorithm=上半球含水平、|entry|≤1.16（压测）')
    parser.add_argument(
        '--seed', type=int, default=20260910,
        help='--random 的 RNG 种子')
    parser.add_argument(
        '--velocity', type=float, default=0.0, metavar='V',
        help='把接触/转移速度与加速度缩放设为 V（0<v<=1，仅仿真提速；'
             '0=不动，真机档位仍走 yaml 默认）')
    parser.add_argument(
        '--pick', nargs='+', default=None, metavar='ID',
        help='只跑 --random 采样出的指定 id（如 rand_02 rand_39），'
             '用于失败例专项重测；采样与全量同 seed 确定性一致')
    parser.add_argument('--list', action='store_true')
    parser.add_argument(
        '--grid', action='store_true',
        help='跑 perception_constraint_grid.yaml（多变在达点位 + 倾角/行程/贴边/超臂展）')
    parser.add_argument(
        '--tool-profile', default='adaptive_cylinder_v1',
        choices=('hollow_cylinder_v1', 'adaptive_cylinder_v1'),
        help='写入 ExecuteTarget.tool_profile_id；须与 launch tool_profile 一致')
    args = parser.parse_args()

    book = load_casebook()
    templates = book['targets_20260909']
    legacy = {
        cid: _normalize_legacy_case(cid, raw)
        for cid, raw in (book.get('cases') or {}).items()}
    catalog = {**legacy, **templates}
    photo_tcp = _photo_tcp(book)
    # 工具档案单一事实源：工具体几何与 GraspDecision 预算都按 --tool-profile
    # 从 aubo_description 档案取（--list 也校验，坏档案早失败）。
    tool_archive = load_tool_archive(args.tool_profile)
    global TOOL_BODY_LENGTH_M, TOOL_BODY_RADIUS_M
    TOOL_BODY_LENGTH_M = tool_archive['l_insert']
    TOOL_BODY_RADIUS_M = tool_archive['d_outer'] / 2.0
    if args.grid:
        cases = load_grid_cases()
        selected = list(cases)
    elif args.random > 0:
        cases = sample_random_cases(
            templates, args.random, args.seed, photo_tcp, args.envelope)
        selected = list(cases)
        if args.pick:
            missing = [p for p in args.pick if p not in cases]
            if missing:
                print(f'--pick 不在采样集内: {missing}', file=sys.stderr)
                return 2
            selected = list(args.pick)
    else:
        if args.case == ['all']:
            cases = templates
            selected = list(cases)
        else:
            cases = catalog
            selected = list(args.case)
        unknown = [c for c in selected if c not in cases]
        if unknown:
            print(f'未知用例: {unknown}', file=sys.stderr)
            return 2

    if args.list:
        for cid, case in cases.items():
            e = case['entry_xyz']
            extra = ''
            if case.get('sample_kind'):
                extra = f" {case['sample_kind']}←{case.get('sampled_from')}"
            if case.get('expect'):
                extra += f" expect={case['expect']}"
            print(f"{cid}: entry=({e[0]:.3f},{e[1]:.3f},{e[2]:.3f}) "
                  f"axis_z={case['axis'][2]:.2f} run={case['run']}{extra}")
        return 0

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import (
        Point, Pose, PoseStamped, Quaternion, TransformStamped, Vector3)
    from peach_interfaces.action import ExecuteTarget
    from peach_interfaces.msg import (
        BagFitting, BagFittingArray, BagGrasp2D, BagGraspCandidate,
        BagGraspCandidateArray, GraspDecision, PeachTargetObservation,
        PeachTargetObservationArray, ReconstructionStatus,
    )
    from peach_interfaces.srv import CheckReachability
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import JointState as JointState_
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from builtin_interfaces.msg import Time

    rclpy.init()
    node = Node('sim_field_targets')

    latched = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    volatile = QoSProfile(depth=10)

    group = ReentrantCallbackGroup()
    obs_pub = node.create_publisher(
        PeachTargetObservationArray, '/peach/perception/target_observations', volatile)
    diag_pub = node.create_publisher(
        ReconstructionStatus, '/peach/reconstruction/diagnostics', latched)
    dec_pub = node.create_publisher(
        GraspDecision, '/peach/reconstruction/grasp_decision', latched)
    refined_pub = node.create_publisher(
        BagGraspCandidateArray, '/peach/reconstruction/refined_pose', latched)
    refit_pub = node.create_publisher(
        BagFittingArray, '/peach/reconstruction/refined_diagnostics', latched)
    rs_pub = node.create_publisher(
        _robot_status_type(), '/aubo_io_controller/robot_status', volatile)
    tf_broadcaster = StaticTransformBroadcaster(node)

    action_cli = ActionClient(node, ExecuteTarget, f'/{NODE}/execute_target')
    ack_cli = node.create_client(Trigger, f'/{NODE}/acknowledge_recovery')
    reach_cli = node.create_client(
        CheckReachability, f'/{NODE}/check_reachability')

    state = {'case': None, 'snapshot_id': 1000, 'capturing': False,
             'joint_samples': []}
    lock = threading.Lock()

    def header(frame='base_link'):
        h = Header()
        h.stamp = node.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def point(xyz):
        p = Point()
        p.x, p.y, p.z = (float(v) for v in xyz)
        return p

    def vector(xyz):
        v = Vector3()
        v.x, v.y, v.z = (float(v) for v in xyz)
        return v

    def set_case(case, cid=None):
        with lock:
            state['case'] = case
            if cid is not None:
                state['cid'] = cid
            state['snapshot_id'] += 1

    def make_observation(case):
        axis = case['axis']
        # 观测入口姿态：Z=袋轴（与 CheckReachability/MovePregrasp 同一口径），
        # X 用世界 z 叉乘避免退化。
        z = _norm(axis)
        ref = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
        x = _norm((ref[1] * z[2] - ref[2] * z[1], ref[2] * z[0] - ref[0] * z[2],
                   ref[0] * z[1] - ref[1] * z[0]))
        y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
             z[0] * x[1] - z[1] * x[0])
        m = ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))
        q = _mat_to_quat(m)

        obs = PeachTargetObservation()
        obs.header = header()
        obs.target_id = case['target_id']
        obs.priority = 1
        obs.confirmed = True
        obs.selected = True
        obs.harvest_status = 'SELECTED'
        obs.tracking_status = PeachTargetObservation.OBSERVED
        obs.camera_distance_m = 0.7
        obs.confidence = 0.9
        obs.diagnostic_flags = list(case.get('flags') or [])
        cand = BagGraspCandidate()
        cand.header = header()
        cand.target_id = case['target_id']
        cand.entry_pose.position = point(case['entry_xyz'])
        cand.entry_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        cand.bag_bottom = point(case['bag_bottom'])
        cand.bag_neck = point(case['bag_neck'])
        cand.translation_direction = vector(case['axis'])
        cand.bag_diameter_upper_m = float(
            case.get('bag_diameter_upper_m') or SIM_BAG_DIAMETER_M)
        cand.suggested_travel_m = float(case.get('travel_m') or 0.0)
        cand.confidence = 0.9
        cand.status = BagGraspCandidate.ACCEPT
        cand.diagnostic_flags = list(case.get('flags') or [])
        obs.candidate = cand
        c2d = BagGrasp2D()
        c2d.target_id = case['target_id']
        c2d.bbox_x, c2d.bbox_y, c2d.bbox_w, c2d.bbox_h = 200, 150, 120, 160
        c2d.confidence = 0.9
        c2d.status = BagGrasp2D.ACCEPT
        obs.candidate_2d = c2d
        obs.fitting.foreground_ratio = 0.9
        obs.mask.width = 0  # 节点回退 640x480
        return obs

    def on_obs():
        with lock:
            case = state['case']
        if case is None:
            return
        msg = PeachTargetObservationArray()
        msg.header = header()
        with lock:
            state['snapshot_id'] += 1
            msg.snapshot_id = state['snapshot_id']
        msg.harvest_run_id = f"sim_{case_id(case)}"
        msg.target_set_locked = True
        msg.target_count = 1
        msg.selected_target_id = case['target_id']
        msg.collecting_count = 1
        msg.pending_count = 0
        obs = make_observation(case)
        obs.selected = True
        msg.observations.append(obs)
        obs_pub.publish(msg)

    def on_diag(_=None):
        with lock:
            case = state['case']
        if case is None:
            return
        msg = ReconstructionStatus()
        msg.header = header()
        msg.harvest_run_id = f"sim_{case_id(case)}"
        msg.selected_target_id = case['target_id']
        msg.state = 'READY'
        msg.target_id = case['target_id']
        msg.target_center_base = [
            0.5 * (case['bag_bottom'][i] + case['bag_neck'][i]) for i in range(3)]
        msg.captured_views = 3
        msg.valid_depth_ratio = 0.7
        msg.max_baseline_deg = 20.0
        msg.mean_nearest_baseline_deg = 12.0
        for direction in ((0.3, -0.9, 0.3), (0.5, -0.7, -0.2), (-0.2, -0.9, 0.4)):
            msg.view_directions.append(vector(direction))
        diag_pub.publish(msg)

    def on_refined(_=None):
        with lock:
            case = state['case']
        if case is None:
            return
        refined = BagGraspCandidate()
        refined.header = header()
        refined.target_id = case['target_id']
        refined.entry_pose.position = point(case['entry_xyz'])
        refined.bag_bottom = point(case['bag_bottom'])
        refined.bag_neck = point(case['bag_neck'])
        refined.translation_direction = vector(case['axis'])
        refined.bag_diameter_upper_m = SIM_BAG_DIAMETER_M
        refined.suggested_travel_m = float(case.get('travel_m') or 0.0)
        refined.confidence = 0.9
        refined.status = BagGraspCandidate.ACCEPT
        arr = BagGraspCandidateArray()
        arr.header = header()
        arr.candidates.append(refined)
        refined_pub.publish(arr)

        fit = BagFitting()
        fit.header = header()
        fit.target_id = case['target_id']
        fit.target_kind = 'bag'
        fit.cylinder_rms_m = 0.004
        fit.cylinder_inlier_ratio = 0.9
        fit.valid_depth_ratio = 0.7
        fit.foreground_ratio = 0.9
        fit.n_points = 2000
        fit.status = BagFitting.ACCEPT
        farr = BagFittingArray()
        farr.header = header()
        farr.fittings.append(fit)
        refit_pub.publish(farr)

    def on_decision(_=None):
        with lock:
            case = state['case']
            cid_now = state.get('cid')
        if case is None:
            return
        # 按档案 D_inner × 注入袋径复算预算：不再无条件放行——超内径的
        # 注入（如 hollow×0.10 袋）得到 allowed=False，臂侧 Finalize 拒接触。
        # allowed 只取径向门（见 decision_budget 注释：axial 结构性为负）。
        d95 = float(case.get('bag_diameter_upper_m') or SIM_BAG_DIAMETER_M)
        budget = decision_budget(tool_archive, d95,
                                 float(case.get('travel_m') or 0.06))
        msg = GraspDecision()
        msg.header = header()
        msg.harvest_run_id = f"sim_{case['run']}"
        msg.target_id = case['target_id']
        # 09-18 起非预览执行须带完整身份元组（ExecuteTarget.action 20-23 行）
        msg.model_revision = f"sim-model-{cid_now or case.get('run')}"
        msg.tool_profile_id = args.tool_profile
        msg.calibration_revision = 'sim-calib-v1'
        msg.config_revision = 'sim-config-v1'
        msg.allowed = bool(budget['sleeve_ok'])
        if msg.allowed:
            msg.reason = 'sim_budget_accept'
        elif budget['radial_available_m'] <= 0.0:
            msg.reason = 'sim_budget_deny_tool'
        else:
            msg.reason = 'sim_budget_deny_radial'
        # 臂侧 allowed 从 capability 派生（GraspDecision.msg:21），不看
        # msg.allowed——必须同源填 capability，否则 deny 例臂侧仍放行
        # （2026-09-23 M1 bag_d100_denied 19/20 失配根因）。0=VALID/1=INVALID。
        # cut 按 sim 径向门口径置 VALID（axial 结构性负是 A 级候选，P3
        # 裁定后改真预算）。
        msg.geometry_capability = 0
        msg.pregrasp_capability = 0
        msg.sleeve_capability = 0 if budget['sleeve_ok'] else 1
        msg.cut_capability = 0
        msg.radial_margin_m = float(budget['radial_margin_m'])
        msg.axial_margin_m = float(budget['axial_margin_m'])
        msg.diameter_m = d95
        msg.d95_m = d95
        msg.travel_m = float(case.get('travel_m') or 0.0)
        msg.corridor_clear = bool(msg.allowed)
        msg.valid_until = node.get_clock().now().to_msg()
        msg.valid_until.sec += 30  # 模型有效期窗（过期= model_not_executable）
        dec_pub.publish(msg)

    def on_robot_status(_=None):
        msg = _robot_status_type()()
        msg.mode = 2
        msg.e_stopped = 0
        msg.drives_powered = 1
        msg.motion_possible = 1
        msg.in_motion = 0
        msg.in_error = 0
        rs_pub.publish(msg)

    node.create_timer(0.4, on_obs, callback_group=group)
    node.create_timer(0.5, on_diag, callback_group=group)
    node.create_timer(0.5, on_refined, callback_group=group)
    node.create_timer(0.5, on_decision, callback_group=group)
    node.create_timer(0.1, on_robot_status, callback_group=group)

    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id = 'camera_link'
    tf.child_frame_id = 'camera_depth_optical_frame'
    q = _optical_from_link_quat()
    tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
    tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
    tf_broadcaster.sendTransform(tf)

    # 实测绕行检测数据源：周期期间持续采样 /joint_states（事后 FK 成 TCP 点列）。
    # joint_state_broadcaster 的发布顺序不保证与 URDF 一致（mock 实测为字母序），
    # 必须按 name 映射，不得按位置截取。
    def on_joint_states(message):
        with lock:
            if not state['capturing'] or len(message.position) < 6:
                return
            by_name = dict(zip(message.name, message.position))
            values = [by_name.get(n) for n in JOINT_ORDER]
            if any(v is None for v in values):
                return
            state['joint_samples'].append(
                (time.monotonic(), [float(v) for v in values]))

    node.create_subscription(
        JointState_, '/joint_states', on_joint_states, 10)

    def _tcp_metrics(points):
        if len(points) < 2:
            return None
        start, goal = points[0], points[-1]
        chord = math.dist(start, goal)
        head = points[3] if len(points) >= 4 else None

        def seg_dist(p, a, b):
            ab = [b[i] - a[i] for i in range(3)]
            len2 = sum(v * v for v in ab)
            if len2 < 1e-16:
                return math.dist(p, a)
            t = max(0.0, min(1.0, sum(
                (p[i] - a[i]) * ab[i] for i in range(3)) / len2))
            return math.dist(p, [a[i] + t * ab[i] for i in range(3)])

        path = 0.0
        max_dev = 0.0
        max_recede = 0.0
        for i in range(1, len(points)):
            path += math.dist(points[i - 1], points[i])
            max_dev = max(max_dev, seg_dist(points[i], start, goal))
            max_recede = max(
                max_recede, max(0.0, math.dist(points[i], goal) - chord))
        ratio = path / chord if chord >= 0.02 else 0.0
        max_z = max(p[2] for p in points)
        return {
            'points': len(points), 'path_m': round(path, 3),
            'chord_m': round(chord, 3), 'ratio': round(ratio, 2),
            'max_dev_m': round(max_dev, 3),
            'max_recede_m': round(max_recede, 3),
            'max_z_m': round(max_z, 3),
            'start_xyz': [round(v, 3) for v in start],
            'head_xyz': [round(v, 3) for v in head] if head else None,
            'end_xyz': [round(v, 3) for v in goal],
            'xyz': [[round(v, 4) for v in p] for p in points],
        }

    def measure_tcp_segments(samples):
        """周期内 TCP 按静止 0.6s 切段（与 watchdog 同口径）。

        正式 ExecuteTarget：先 goToPhotoPose（接近原路返程，否则 PTP），停稳后再接近。
        返回有运动的段列表。调用方按是否靠近拍照位挑选接近段/回拍照段，
        不要默认末段=接近（FULL 末段常是回 stow/拍照）。
        """
        from moveit_msgs.srv import GetPositionFK
        fk_cli = node.create_client(GetPositionFK, '/compute_fk')
        if not fk_cli.wait_for_service(timeout_sec=5.0):
            return []
        last_t = -1.0
        timed = []

        def fk(positions):
            req = GetPositionFK.Request()
            req.header.frame_id = 'base_link'
            req.fk_link_names = ['tcp']
            req.robot_state.joint_state = JointState_()
            req.robot_state.joint_state.name = list(JOINT_ORDER)
            req.robot_state.joint_state.position = list(positions)
            fut = fk_cli.call_async(req)
            res = spin_until(fut, 5.0)
            if res is None or not res.pose_stamped:
                return None
            p = res.pose_stamped[0].pose.position
            return (float(p.x), float(p.y), float(p.z))

        for stamp, positions in samples:
            if stamp - last_t < 0.1 and last_t >= 0:
                continue
            xyz = fk(positions)
            last_t = stamp
            if xyz is not None:
                timed.append((stamp, xyz))
        if len(timed) < 2:
            return []
        still_s = 0.6
        chunks = []
        start = 0
        still_since = None
        for i in range(1, len(timed)):
            step = math.dist(timed[i - 1][1], timed[i][1])
            t = timed[i][0]
            if step > 5e-4:
                still_since = None
                continue
            if still_since is None:
                still_since = t
            elif t - still_since > still_s:
                metrics = _tcp_metrics([p for _, p in timed[start:i]])
                if metrics and metrics['path_m'] >= 0.01:
                    chunks.append(metrics)
                start = i
                still_since = None
        tail = _tcp_metrics([p for _, p in timed[start:]])
        if tail and tail['path_m'] >= 0.01:
            chunks.append(tail)
        return chunks

    def _keepout_hit(tcp, entry, axis):
        # 与 grasp_task.cpp 逐段审查同口径：工具有限圆柱×果实胶囊；
        # 反爬只对「从袋底出发的段」（起点 s≤0）生效。无姿态时假定
        # 工具 Z 已对轴（接近段目标姿态）。
        points = (tcp or {}).get('xyz') or []
        if not points or not entry or not axis:
            return False, ''
        if KEEP_AXIAL_M <= 1.0e-6:
            return False, ''
        ax = _norm(axis)
        fruit_r = fruit_radius_m(SIM_BAG_DIAMETER_M)
        bottom = list(entry)
        neck = _add(bottom, _scale(ax, 0.07))

        def sr(p):
            return _axial_of(p, bottom, ax), math.sqrt(sum(
                (p[i] - bottom[i] - _axial_of(p, bottom, ax) * ax[i]) ** 2
                for i in range(3)))

        start_s, _ = sr(points[0])
        s_max = max(start_s, 0.0) + 0.02
        for p in points:
            axial, radial = sr(p)
            if tool_body_hits_fruit(p, ax, bottom, neck, ax, fruit_r):
                return True, (
                    f'工具筒体触果 s={axial:.3f}m r={radial:.3f}m '
                    f'R={fruit_r:.3f}m')
            if start_s <= 0.0 and axial > s_max:
                return True, (
                    f'袋底段上方绕行 s={axial:.3f}m > 上限 {s_max:.3f}m')
        return False, ''

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    def safe_spin():
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as error:
                print(f'注入回调异常: {error}', file=sys.stderr)

    spin = threading.Thread(target=safe_spin, daemon=True)
    spin.start()

    def case_id(case):
        return f"{case['run']}:{case['target_id']}"

    def spin_until(fut, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if fut.done():
                return fut.result()
            time.sleep(0.05)
        return None

    def run_case(cid, case):
        print(f'\n=== case {cid} entry=' +
              str([round(v, 3) for v in case['entry_xyz']]) +
              f" axis_z={case['axis'][2]:.2f}"
              + (f" {case['sample_kind']}←{case['sampled_from']}"
                 if case.get('sample_kind') else '') +
              ' ===', flush=True)
        set_case(case, cid)
        time.sleep(1.5)  # 锁定集/精化锁存就位
        with lock:
            state['joint_samples'] = []
            state['capturing'] = True
        if not action_cli.wait_for_server(timeout_sec=10.0):
            return {'case': cid, 'error': 'execute_target action 不可用'}
        expect = str(case.get('expect') or '')
        if expect == 'skip_select':
            return {
                'case': cid, 'expect': expect, 'matched': True,
                'outcome': 'skipped_select',
                'reason': ','.join(case.get('flags') or []),
            }
        if expect == 'deny_decision':
            # 决策层拒绝校验（不发 ExecuteTarget）：unrefined 接触链
            # （skip_reconstruction，b121972）在臂侧不查 GraspDecision
            # （target_cache.cpp markUnrefinedHold 无条件自授权），预算门
            # 只存在于决策发布层——此处断言注入决策确为拒绝。
            d95 = float(case.get('bag_diameter_upper_m') or SIM_BAG_DIAMETER_M)
            budget = decision_budget(
                tool_archive, d95, float(case.get('travel_m') or 0.06))
            denied = (not budget['sleeve_ok']) and budget['radial_margin_m'] <= 0.0
            return {
                'case': cid, 'expect': expect, 'matched': denied,
                'outcome': 'decision_denied' if denied else 'decision_leak',
                'reason': budget['reason'],
                'radial_margin_m': budget['radial_margin_m'],
            }

        def probe_reach():
            if not reach_cli.wait_for_service(timeout_sec=5.0):
                return None, 'service_unavailable'
            req = CheckReachability.Request()
            req.require_sleeve = args.mode == 'full'
            axis = _norm(case['axis'])
            z = axis
            ref = (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0)
            x = _norm((ref[1] * z[2] - ref[2] * z[1],
                       ref[2] * z[0] - ref[0] * z[2],
                       ref[0] * z[1] - ref[1] * z[0]))
            y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
                 z[0] * x[1] - z[1] * x[0])
            q = _mat_to_quat(((x[0], y[0], z[0]), (x[1], y[1], z[1]),
                              (x[2], y[2], z[2])))
            stamped = PoseStamped()
            stamped.header = header()
            stamped.pose.position.x = float(case['entry_xyz'][0])
            stamped.pose.position.y = float(case['entry_xyz'][1])
            stamped.pose.position.z = float(case['entry_xyz'][2])
            stamped.pose.orientation.x = q[0]
            stamped.pose.orientation.y = q[1]
            stamped.pose.orientation.z = q[2]
            stamped.pose.orientation.w = q[3]
            req.tcp_poses.append(stamped)
            req.suggested_travel_m.append(float(case.get('travel_m') or 0.0))
            fut = reach_cli.call_async(req)
            resp = spin_until(fut, 8.0)
            if resp is None:
                return None, 'service_timeout'
            ok = bool(resp.reachable and resp.reachable[0])
            code = str(resp.error_codes[0] if resp.error_codes else '')
            return ok, code

        if expect in ('skip_ik', 'skip_cartesian') or args.grid:
            ok, code = probe_reach()
            row = {
                'case': cid, 'expect': expect, 'reachable': ok,
                'error_code': code,
            }
            if expect == 'skip_ik':
                row['matched'] = (ok is False and code == 'no_ik')
                return row
            if expect == 'skip_cartesian':
                row['matched'] = (ok is False and code == 'sleeve_no_cartesian')
                return row
            if ok is False:
                row['matched'] = False
                row['error'] = f'reachability 拒: {code}'
                return row
        goal = ExecuteTarget.Goal()
        goal.request_id = f'sim_{cid}'
        goal.run_id = f'sim_{case["run"]}'
        goal.cycle_id = f'sim_{cid}:{case["target_id"]}'
        goal.target_id = case['target_id']
        # mode 是接触深度权威：FULL 不被默认 profile=0 盖成 HOLD。
        # 仍显式 PROFILE_FULL，避免旧客户端/旧注释歧义。
        if args.mode == 'full':
            goal.mode = ExecuteTarget.Goal.FULL
            goal.profile = ExecuteTarget.Goal.PROFILE_FULL
        else:
            goal.mode = ExecuteTarget.Goal.PREGRASP_ONLY
            goal.profile = ExecuteTarget.Goal.PROFILE_PREGRASP_HOLD
        # 09-18 起身份元组不完整直接拒；与 GraspDecision 注入同串（每案唯一）
        goal.model_revision = f'sim-model-{cid}'
        goal.tool_profile_id = args.tool_profile
        goal.calibration_revision = 'sim-calib-v1'
        goal.config_revision = 'sim-config-v1'
        goal.skip_observation = True
        t0 = time.time()
        send = action_cli.send_goal_async(goal)
        gh = spin_until(send, 15.0)
        if gh is None:
            return {'case': cid, 'error': 'goal 响应超时'}
        if not gh.accepted:
            return {'case': cid, 'error': 'goal 被拒'}
        result = spin_until(gh.get_result_async(), GOAL_TIMEOUT_S)
        elapsed = time.time() - t0
        with lock:
            state['capturing'] = False
            samples = list(state['joint_samples'])
        if result is None:
            # 超时必须取消服务端 goal：否则动作一直 running，臂拒绝后续
            # 所有 goal（2026-09-23 M1 tilt_1639_1 超时后级联 goal 被拒）。
            try:
                spin_until(gh.cancel_goal_async(), 10.0)
                # 等臂侧真到 CANCELED 终态再进下一例，否则周期收尾窗口
                # 内下一例仍会被拒（goal 被拒级联残余）
                spin_until(gh.get_result_async(), 30.0)
                time.sleep(1.5)
            except Exception:  # noqa: BLE001
                pass
            return {'case': cid, 'error': 'goal 超时（已取消）',
                    'elapsed_s': elapsed}
        res = result.result
        segments = measure_tcp_segments(samples) if samples else []

        def _near_photo(xyz):
            return bool(xyz) and len(xyz) == 3 and math.dist(xyz, photo_tcp) < 0.08

        def _split_photo_roundtrip(seg):
            """FULL 周期常把「拍照→预抓取→回拍照」合成一段（弦≈0、路径长）。

            静止切段在袋口停不稳时切不开，picker 会把 photo→photo 当接近。
            按离拍照位最远的点拆成出程/返程。
            """
            xyz = seg.get('xyz') or []
            if len(xyz) < 4:
                return None, None
            if not (
                _near_photo(seg.get('start_xyz')) and
                _near_photo(seg.get('end_xyz'))):
                return None, None
            if float(seg.get('path_m') or 0.0) < 0.20:
                return None, None
            far_i = max(
                range(len(xyz)), key=lambda i: math.dist(xyz[i], photo_tcp))
            if math.dist(xyz[far_i], photo_tcp) < 0.15:
                return None, None
            outbound = _tcp_metrics(xyz[:far_i + 1])
            inbound = _tcp_metrics(xyz[far_i:])
            return outbound, inbound

        expanded = []
        for seg in segments:
            outbound, inbound = _split_photo_roundtrip(seg)
            if outbound and inbound:
                expanded.extend([outbound, inbound])
            else:
                expanded.append(seg)
        segments = expanded

        tcp_approach = None
        tcp_return = None
        for seg in segments:
            if float(seg.get('chord_m') or 0.0) < 0.08:
                continue
            if tcp_approach is None and _near_photo(seg.get('start_xyz')):
                tcp_approach = seg
            if _near_photo(seg.get('end_xyz')):
                tcp_return = seg
        tcp_photo = tcp_return
        tcp = tcp_approach or (segments[-1] if segments else None)
        from_photo = bool(tcp and _near_photo(tcp.get('start_xyz')))
        detour, detour_detail = _keepout_hit(
            tcp, case['entry_xyz'], case['axis'])
        photo_detour, photo_detour_detail = False, ''
        out = {
            'case': cid,
            'run': case['run'],
            'target_id': case['target_id'],
            'entry_xyz': list(case['entry_xyz']),
            'axis': list(case['axis']),
            'from_photo': from_photo,
            'status': int(result.status),
            'outcome': int(res.outcome),
            'completion_level': int(res.completion_level),
            'failure_code': int(res.failure_code),
            'reason': res.reason,
            'recovery_required': bool(res.recovery_required),
            'pregrasp_passed': bool(res.pregrasp.passed),
            'pregrasp_axis_deg': float(res.pregrasp.axis_angle_deg),
            'pregrasp_lateral_m': float(res.pregrasp.lateral_error_m),
            'stage_names': list(res.stage_names),
            'stage_durations_s': [d.sec + d.nanosec / 1e9 for d in res.stage_durations],
            'elapsed_s': elapsed,
            'tcp_photo': tcp_photo,
            'tcp_detour': tcp,
            'detour_flag': detour,
            'detour_detail': detour_detail,
            'photo_detour_flag': photo_detour,
            'photo_detour_detail': photo_detour_detail,
            'sample_kind': case.get('sample_kind'),
            'sampled_from': case.get('sampled_from'),
            'envelope': case.get('envelope'),
            'mode': args.mode,
            'grasped': bool(res.harvest.grasped),
            'harvest_commanded': bool(res.harvest.commanded),
            'expect': expect,
        }
        if expect == 'succeed':
            out['matched'] = (
                int(res.outcome) == 0
                and int(res.completion_level) >= 3
                and not bool(res.harvest.grasped))
        elif expect == 'deny_decision':
            # 预算拒绝（如 hollow×0.10 袋）：臂侧在接触授权处拒，不得进套入段
            # （LEVEL_SLEEVE_COMPLETED=3）；不锁具体 failure_code 整数。
            out['matched'] = (
                int(res.outcome) != 0
                and int(res.completion_level) < 3
                and not bool(res.harvest.grasped))
        elif expect:
            out['matched'] = False
        if res.recovery_required:
            ack_cli.wait_for_service(timeout_sec=5.0)
            ack = ack_cli.call_async(Trigger.Request())
            spin_until(ack, 10.0)
        return out

    wait_active()
    ensure_profile_match(args.tool_profile)
    ensure_enabled(node, args.velocity)
    print(f'{NODE} Active；execution/grasp 已开（tool 保持关）'
          f'；mode={args.mode}'
          + (f'；速度/加速度缩放={args.velocity:g}（仅仿真）'
             if args.velocity > 0.0 else ''))
    print('回拍照位走 ExecuteTarget 内 goToPhotoPose（接近原路返程，否则 PTP），'
          '不经脚本 JTC', flush=True)
    if args.random > 0:
        print(f'随机位姿 {len(selected)} 个 seed={args.seed} '
              f'envelope={args.envelope}', flush=True)

    outcomes = []
    out_path = RESULTS_DIR / f'sim_field_targets_{time.strftime("%Y%m%d_%H%M%S")}.jsonl'
    with open(out_path, 'a', encoding='utf-8') as sink:
        for cid in selected:
            record = run_case(cid, cases[cid])
            outcomes.append(record)
            sink.write(json.dumps(record, ensure_ascii=False) + '\n')
            sink.flush()
            reason = str(record.get('reason') or record.get('error') or '')
            print(f"  -> outcome={record.get('outcome')} "
                  f"completion={record.get('completion_level')} "
                  f"matched={record.get('matched')} "
                  f"code={record.get('error_code', '')} "
                  f"reason={reason[:120]}",
                  flush=True)
            if record.get('tcp_photo'):
                tcp = record['tcp_photo']
                print(f"  [回拍照] 弦={tcp['chord_m']}m 路径={tcp['path_m']}m "
                      f"比={tcp['ratio']} 偏离={tcp['max_dev_m']}m "
                      f"回退={tcp['max_recede_m']}m 起={tcp['start_xyz']} "
                      f"终={tcp['end_xyz']}",
                      flush=True)
            if record.get('tcp_detour'):
                tcp = record['tcp_detour']
                origin = '从拍照位' if record.get('from_photo') else '未从拍照位'
                keep = (f"  ⚠ 果实胶囊: {record['detour_detail']}"
                        if record['detour_flag'] else '  ✓ 果实胶囊外')
                print(f"  [接近] {origin} 点数={tcp['points']} 弦={tcp['chord_m']}m "
                      f"路径={tcp['path_m']}m 比={tcp['ratio']} "
                      f"偏离={tcp['max_dev_m']}m 回退={tcp['max_recede_m']}m "
                      f"max_z={tcp.get('max_z_m')} "
                      f"起={tcp['start_xyz']} 早段={tcp['head_xyz']} "
                      f"终={tcp['end_xyz']}" + keep,
                      flush=True)
            time.sleep(0.05 if args.velocity > 0.0 else 0.5)

    print(f'\n结果已写入 {out_path}')
    if args.grid:
        matched = sum(1 for r in outcomes if r.get('matched'))
        print(f'网格期望命中 {matched}/{len(outcomes)}')
        if rclpy.ok():
            rclpy.shutdown()
        return 0 if matched == len(outcomes) else 1
    if args.mode == 'full':
        passed = sum(
            1 for r in outcomes
            if r.get('outcome') == ExecuteTarget.Result.SUCCEEDED
            and r.get('completion_level', 0) >=
            ExecuteTarget.Result.LEVEL_SLEEVE_COMPLETED
            and not r.get('grasped'))
        print(
            f'FULL 干跑成功 {passed}/{len(outcomes)}'
            f'（套入到位、grasped=false、tool 关）')
    else:
        passed = sum(
            1 for r in outcomes
            if r.get('outcome') == ExecuteTarget.Result.SUCCEEDED)
        keepout_hits = sum(1 for r in outcomes if r.get('detour_flag'))
        photo_keepout = sum(
            1 for r in outcomes
            if r.get('from_photo') and r.get('detour_flag'))
        print(
            f'成功 {passed}/{len(outcomes)}；从拍照位果实胶囊后检 {photo_keepout}'
            f'；含未从拍照位返程旗标 {keepout_hits}')
    env_rows = [
        r for r in outcomes
        if r.get('entry_xyz') and r.get('axis') and
        _in_typical_envelope(r['entry_xyz'], r['axis'])]
    env_ok = sum(
        1 for r in env_rows
        if r.get('outcome') == ExecuteTarget.Result.SUCCEEDED)
    print(
        f'现场典型包络 (|entry|≤{TYPICAL_ENTRY_NORM_M:g} ∧ '
        f'axis_z≥{TYPICAL_AXIS_Z_MIN:g}) {env_ok}/{len(env_rows)}')
    if rclpy.ok():
        rclpy.shutdown()
    return 0


def _robot_status_type():
    from aubo_msgs.msg import RobotStatus
    return RobotStatus


def _mat_to_quat(m):
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


def _constraints_type():
    from moveit_msgs.msg import Constraints
    return Constraints


def _joint_constraint_type():
    from moveit_msgs.msg import JointConstraint
    return JointConstraint


if __name__ == '__main__':
    sys.exit(main())
