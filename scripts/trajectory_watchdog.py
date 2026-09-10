#!/usr/bin/env python3
"""实时轨迹 watchdog：执行期间持续 FK 监测 TCP，记录段指标。

按段评估（机械臂静止 >0.6s 即重置基线，段=拍照位 PTP / 接近各阶段）：
每段起止弦、路径长、绕行比、相对弦偏离、回退。默认不按绕行比停轨
（合法袋底 G 比直弦长；口侧/上方由技能节点袋囊 keepout 拒发）。
``--ratio/--dev/--recede`` >0 才停轨（经
``/peach_manipulation_node/cancel_cycle``）。FK 走 ``/compute_fk``
（15 Hz 截流）。``/joint_states`` 按名字映射（mock 广播器是字母序）。

须已起 mock harvest_system。不进 colcon test；仿真验证用，不碰真机。

用法：
  python3 scripts/trajectory_watchdog.py                 # 只记录，不停轨
  python3 scripts/trajectory_watchdog.py --ratio 2.5 --dev 0.40 --recede 0.15
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)
RESULTS_DIR = Path(__file__).resolve().parents[1] / 'runs'
STATIONARY_RAD_S = 0.05     # 全轴 |Δq|/Δt 低于此视为静止
STATIONARY_WINDOW_S = 0.6   # 静止持续超过此时长 → 重置段基线
FK_PERIOD_S = 1.0 / 15.0


def _dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _seg_dist(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    len2 = sum(v * v for v in ab)
    if len2 < 1e-16:
        return _dist(p, a)
    t = max(0.0, min(1.0, sum((p[i] - a[i]) * ab[i] for i in range(3)) / len2))
    return _dist(p, [a[i] + t * ab[i] for i in range(3)])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ratio', type=float, default=0.0,
                        help='路径/弦上限；0=不按绕行比停轨')
    parser.add_argument('--dev', type=float, default=0.0,
                        help='弦偏离上限（米）；0=不查')
    parser.add_argument('--recede', type=float, default=0.0,
                        help='回退上限（米）；0=不查')
    parser.add_argument('--chord-min', type=float, default=0.05,
                        help='弦长小于此值不评绕行比（近似原地）')
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionFK
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node('trajectory_watchdog')

    lock = threading.Lock()
    latest = {'names': None, 'positions': None, 't': 0.0, 'running': False}
    stopped = threading.Event()

    def on_joint_state(message):
        with lock:
            latest['names'] = list(message.name)
            latest['positions'] = [float(v) for v in message.position]
            latest['t'] = time.monotonic()

    def on_status(message):
        try:
            import json as _json
            running = bool(_json.loads(message.data).get('running', False))
        except (ValueError, AttributeError):
            return
        with lock:
            latest['running'] = running

    node.create_subscription(JointState, '/joint_states', on_joint_state, 10)
    latched = QoSProfile(
        depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(
        String, '/peach_manipulation_node/status', on_status, latched)
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    cancel_cli = node.create_client(Trigger, '/peach_manipulation_node/cancel_cycle')

    out_path = RESULTS_DIR / (
        f'trajectory_watchdog_{time.strftime("%Y%m%d_%H%M%S")}.jsonl')
    sink = open(out_path, 'a', encoding='utf-8')

    # 段状态：start=段起点 TCP；points=段内点列；moving=是否在动。
    seg = {'start': None, 'prev': None, 'path': 0.0, 'max_dev': 0.0,
           'max_recede': 0.0, 'still_since': None, 'violations': 0}

    def fk(positions):
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['tcp']
        req.robot_state.joint_state.name = list(JOINT_ORDER)
        req.robot_state.joint_state.position = positions
        fut = fk_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
        res = fut.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return (float(p.x), float(p.y), float(p.z))

    def record(event):
        event['recorded_at'] = time.strftime('%H:%M:%S')
        sink.write(json.dumps(event, ensure_ascii=False) + '\n')
        sink.flush()

    print(f'watchdog 就绪：门 ratio={args.ratio} dev={args.dev} '
          f'recede={args.recede}；停轨经 ~/cancel_cycle；记录 {out_path}')

    next_fk = 0.0
    while rclpy.ok() and not stopped.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if now < next_fk:
            continue
        next_fk = now + FK_PERIOD_S
        with lock:
            names, positions, stamp, running = (
                latest['names'], latest['positions'], latest['t'],
                latest['running'])
        if not names or positions is None:
            continue
        by_name = dict(zip(names, positions))
        ordered = [by_name.get(n) for n in JOINT_ORDER]
        if any(v is None for v in ordered):
            continue
        tip = fk(ordered)
        if tip is None:
            continue

        # 只在技能周期 running 时累计：仿真复位/手动挪臂不评绕行。
        if not running:
            seg.update({'start': None, 'prev': None, 'path': 0.0,
                        'max_dev': 0.0, 'max_recede': 0.0, 'max_radius': 0.0,
                        'still_since': None})
            seg['prev'] = tip
            continue

        if seg['start'] is None:
            seg.update({'start': tip, 'path': 0.0, 'max_dev': 0.0,
                        'max_recede': 0.0, 'max_radius': 0.0})
            seg['prev'] = tip
            continue

        # 段内静止检测：全静止 >0.6s 视为一段结束，重置基线（分段=拍照位
        # PTP / 接近各阶段 / LIN 各段）。
        step = _dist(seg['prev'], tip)
        moving = step > 5e-4
        if moving:
            seg['still_since'] = None
        elif seg['still_since'] is None:
            seg['still_since'] = now
        elif now - seg['still_since'] > STATIONARY_WINDOW_S and step < 5e-4:
            chord = _dist(seg['start'], tip)
            if seg['path'] > 0.01:
                ratio = seg['path'] / chord if chord >= args.chord_min else 0.0
                record({
                    'kind': 'segment',
                    'start': list(seg['start']), 'end': list(tip),
                    'path_m': round(seg['path'], 3),
                    'chord_m': round(chord, 3),
                    'ratio': round(ratio, 2),
                    'max_dev_m': round(seg['max_dev'], 3),
                    'max_recede_m': round(seg['max_recede'], 3),
                })
            seg.update({'start': tip, 'path': 0.0, 'max_dev': 0.0,
                        'max_recede': 0.0, 'max_radius': 0.0})
            seg['prev'] = tip
            continue

        seg['path'] += step
        chord = _dist(seg['start'], tip)
        seg['max_dev'] = max(seg['max_dev'], _seg_dist(tip, seg['start'], tip))
        # 回退（段内近似）：途中距起点峰值 − 当前弦——臂先远离再折回时 >0。
        seg['max_radius'] = max(seg['max_radius'], chord)
        seg['max_recede'] = max(seg['max_recede'],
                                max(0.0, seg['max_radius'] - chord))
        seg['prev'] = tip

        over = []
        if args.ratio > 0.0 and chord >= args.chord_min and (
                seg['path'] / chord > args.ratio):
            over.append(f"绕行比 {seg['path'] / chord:.2f}>{args.ratio}")
        if args.dev > 0.0 and seg['max_dev'] > args.dev:
            over.append(f"弦偏离 {seg['max_dev']:.3f}>{args.dev}")
        if args.recede > 0.0 and seg['max_recede'] > args.recede:
            over.append(f"回退 {seg['max_recede']:.3f}>{args.recede}")
        if over:
            seg['violations'] += 1
            event = {
                'kind': 'VIOLATION', 'detail': '；'.join(over),
                'start': list(seg['start']), 'now': list(tip),
                'path_m': round(seg['path'], 3), 'chord_m': round(chord, 3),
                'ratio': round(seg['path'] / chord if chord > 1e-9 else 0.0, 2),
                'max_dev_m': round(seg['max_dev'], 3),
                'max_recede_m': round(seg['max_recede'], 3),
            }
            record(event)
            print(f'⚠ 绕行越门：{event["detail"]} @ now={event["now"]} '
                  f'→ 停轨（cancel_cycle）', flush=True)
            if cancel_cli.wait_for_service(timeout_sec=2.0):
                cancel_cli.call_async(Trigger.Request())
            stopped.set()
            break
        print(f'[watch] path={seg["path"]:.3f} chord={chord:.3f} '
              f'ratio={seg["path"] / chord if chord > 1e-9 else 0.0:.2f} '
              f'dev={seg["max_dev"]:.3f} recede={seg["max_recede"]:.3f}',
              end='\r', flush=True)

    print(f'\nwatchdog 退出；记录 {out_path}')
    sink.close()
    rclpy.shutdown()
    return 0 if not seg['violations'] else 3


if __name__ == '__main__':
    sys.exit(main())
