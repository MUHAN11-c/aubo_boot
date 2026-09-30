#!/usr/bin/env python3
"""smoothness_analyze.py — 关节轨迹平滑度量化分析（读取 rosbag2，numpy only）.

2026-09-30 自 aubo_boot aubo_ros2_jazzy_ws/tools 移植，原样保真。

录制:
  ros2 bag record -o <bag_dir> /joint_states /joint_trajectory_controller/state
分析:
  python3 scripts/smoothness_analyze.py <bag_dir> [--topic /joint_states]

指标:
  - 采样频率与间隙（丢帧/发布抖动）
  - 微分速度 vs 反馈速度一致性
  - 速度塌陷事件（运动中 |v| 跌破运动中位数 20%，持续 >30ms）= 欠喂停顿
  - 加速度分位数/跳变、jerk 峰值
"""
import argparse

import numpy as np


def load_bag(bag_dir, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f'话题 {topic} 不在 bag 中（现有: {list(types)}）')
    msg_cls = get_message(types[topic])
    ts, names, pos, vel = [], None, [], []
    while reader.has_next():
        name, data, t = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_cls)
        if names is None:
            names = list(msg.name)
        ts.append(t * 1e-9)
        pos.append(msg.position)
        vel.append(msg.velocity if len(msg.velocity) else [np.nan] * len(msg.position))
    if not ts:
        raise SystemExit(f'bag 中 {topic} 无消息')
    order = np.argsort(ts)
    return (np.array(ts)[order], np.array(pos)[order], np.array(vel)[order], names)


def pct(x, p):
    return float(np.percentile(x, p)) if len(x) else float('nan')


def analyze(t, pos, vel_fb, names):
    dt = np.diff(t)
    print(f'样本 {len(t)}  时长 {t[-1]-t[0]:.1f}s  关节 {names}')
    print(f'采样: 中位 {1/np.median(dt):.1f}Hz  最大间隙 {dt.max()*1000:.0f}ms  '
          f'>20ms 间隙 {int((dt > 0.02).sum())} 次')

    vel_diff = np.diff(pos, axis=0) / dt[:, None]
    speed = np.linalg.norm(vel_diff, axis=1)
    moving = speed > max(np.percentile(speed, 50) * 0.2, 0.005)
    print(f'运动区间占比 {moving.mean()*100:.0f}%  合成速度 p50={np.median(speed):.4f} '
          f'p99={pct(speed,99):.4f} rad/s')

    thr = max(np.median(speed[moving]) * 0.2, 1e-4) if moving.any() else 1e-4
    collapse = moving & (speed < thr)
    events, run = [], 0
    for i, c in enumerate(collapse):
        run = run + 1 if c else 0
        if c and run * np.median(dt) > 0.03 and (i + 1 == len(collapse) or not collapse[i + 1]):
            events.append(run * np.median(dt))
    print(f'速度塌陷事件（>30ms）: {len(events)} 次', end='')
    if events:
        print(f'  总时长 {sum(events)*1000:.0f}ms  最长 {max(events)*1000:.0f}ms')
    else:
        print()

    print(f'\n{"关节":<16}{"v_p99":>8}{"a_p99":>9}{"a跳变p99":>10}{"jerk_p99":>10}{"jerk_max":>10}')
    acc = np.diff(vel_diff, axis=0) / dt[1:, None]
    for j, nm in enumerate(names):
        a = acc[:, j]
        a_jump = np.abs(np.diff(a))
        jerk = np.diff(a) / dt[2:]
        v = vel_diff[:, j]
        print(f'{nm:<16}{pct(np.abs(v),99):>8.3f}{pct(np.abs(a),99):>9.2f}'
              f'{pct(a_jump,99):>10.2f}{pct(np.abs(jerk),99):>10.0f}{np.abs(jerk).max():>10.0f}')

    if not np.isnan(vel_fb).all():
        err = np.abs(vel_fb[1:] - vel_diff)
        print(f'\n反馈/微分速度偏差 p50={np.nanmedian(err):.4f} '
              f'p99={np.nanpercentile(err, 99):.4f} rad/s')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/joint_states')
    args = ap.parse_args()
    analyze(*load_bag(args.bag, args.topic))


if __name__ == '__main__':
    main()
