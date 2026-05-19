#!/usr/bin/env python3
"""
轨迹阶段分割器
===============
基于关节速度将完整轨迹分割为:
  靠近阶段 (Approach):  从起始位置移动到杯子上方
  倒奶阶段 (Pouring):   实际倒咖啡 (融合+成形+收尾)
  离开阶段 (Departure): 从杯子上方移开

靠近和离开的特征: 速度从0开始增大/减小到0, 平缓
倒奶的特征: 持续的中等速度活动, 可能含细微摆动

用法:
    python segment_trajectory.py                  # 分析所有 episodes
    python segment_trajectory.py --episode 20     # 分析单个
    python segment_trajectory.py --extract-all    # 提取所有episode的纯倒奶段
"""

import numpy as np
import pandas as pd
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
import json
import argparse
from scipy.ndimage import uniform_filter1d
from scipy.signal import find_peaks, savgol_filter

from analyze_dataset import (load_episode, compute_derivatives, DT,
                              RIGHT_ARM, LEFT_ARM, DATA_PATH, OUTPUT_DIR)


@dataclass
class TrajectorySegment:
    """轨迹片段"""
    name: str                       # "approach", "pouring", "departure"
    start_frame: int
    end_frame: int
    states: np.ndarray              # (N, 14)
    mean_speed: float
    max_speed: float
    duration_s: float


def compute_activity_signal(df: pd.DataFrame, smooth_window: int = 15) -> np.ndarray:
    """
    计算双臂总活动量信号 (用于阶段分割)
    活动量 = 所有关节速度绝对值的和 (平滑处理)

    这个信号能清楚区分:
    - 静止/慢速移动 (靠近/离开)
    - 持续倒奶活动 (倒奶)
    """
    states = np.vstack(df['observation.state'].values)
    vel = np.diff(states, axis=0) / DT

    # 总活动量: 所有14个关节速度绝对值的和
    activity = np.sum(np.abs(vel), axis=1)

    # 平滑处理去除高频噪声
    smooth_activity = savgol_filter(activity, smooth_window, 3)

    # 补第一帧 (用0)
    full_activity = np.concatenate([[0], smooth_activity])

    return full_activity


def segment_full_trajectory(df: pd.DataFrame) -> List[TrajectorySegment]:
    """
    将完整轨迹分割为: 靠近 → 倒奶 → 离开

    策略:
    1. 计算活动量信号
    2. 用阈值检测"活跃段"(倒奶) vs "非活跃段"(靠近/离开)
    3. 根据时间顺序: 第一段=靠近, 中间=倒奶, 最后=离开
    """
    n = len(df)
    activity = compute_activity_signal(df)

    # 自适应阈值: 活动量中位数 + 0.3*标准差
    threshold = np.median(activity) + 0.3 * np.std(activity)

    # 标记活跃帧
    active = activity > threshold

    # 找活跃段的连续区间
    # 用形态学: 先闭运算合并小间隙, 再开运算去除噪声
    from scipy.ndimage import binary_closing, binary_opening

    # 闭运算合并间隙 (< 0.5s = 10帧 内的间隙合并)
    active_closed = binary_closing(active, structure=np.ones(10))
    # 开运算去除短噪声 (< 0.25s = 5帧)
    active_clean = binary_opening(active_closed, structure=np.ones(5))

    # 找连续活跃段
    changes = np.diff(active_clean.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1

    # 处理边界情况
    if active_clean[0]:
        starts = np.concatenate([[0], starts])
    if active_clean[-1]:
        ends = np.concatenate([ends, [n]])

    segments = []

    if len(starts) == 0:
        # 没有检测到活跃段 → 整个轨迹可能都是低速移动
        print("  警告: 未检测到明显活跃段, 使用启发式分割")
        # 启发式: 前20% = 靠近, 中间60% = 倒奶, 后20% = 离开
        seg_defs = [
            ("approach", 0, int(n * 0.2)),
            ("pouring", int(n * 0.2), int(n * 0.8)),
            ("departure", int(n * 0.8), n),
        ]
        for name, start, end in seg_defs:
            states_arr = np.vstack(df['observation.state'].values[start:end])
            seg = TrajectorySegment(
                name=name,
                start_frame=start,
                end_frame=end,
                states=states_arr,
                mean_speed=float(np.mean(activity[start:end])),
                max_speed=float(np.max(activity[start:end])),
                duration_s=(end - start) * DT,
            )
            segments.append(seg)
        return segments

    # 找最长的活跃段 = 倒奶阶段
    longest_idx = np.argmax(ends - starts)
    pouring_start = starts[longest_idx]
    pouring_end = ends[longest_idx]

    # 靠近阶段 = 倒奶之前的非活跃 + 过渡段
    # (包含从停止状态加速到倒奶速度的过程)
    approach_end = pouring_start

    # 离开阶段 = 倒奶之后的非活跃段
    departure_start = pouring_end

    # --- 靠近阶段 ---
    states_arr = np.vstack(df['observation.state'].values[:approach_end])
    if approach_end > 0:
        segments.append(TrajectorySegment(
            name="approach",
            start_frame=0,
            end_frame=approach_end,
            states=states_arr,
            mean_speed=float(np.mean(activity[:approach_end])),
            max_speed=float(np.max(activity[:approach_end])),
            duration_s=approach_end * DT,
        ))

    # --- 倒奶阶段 ---
    states_arr = np.vstack(df['observation.state'].values[pouring_start:pouring_end])
    segments.append(TrajectorySegment(
        name="pouring",
        start_frame=pouring_start,
        end_frame=pouring_end,
        states=states_arr,
        mean_speed=float(np.mean(activity[pouring_start:pouring_end])),
        max_speed=float(np.max(activity[pouring_start:pouring_end])),
        duration_s=(pouring_end - pouring_start) * DT,
    ))

    # --- 离开阶段 ---
    if departure_start < n:
        states_arr = np.vstack(df['observation.state'].values[departure_start:])
        segments.append(TrajectorySegment(
            name="departure",
            start_frame=departure_start,
            end_frame=n,
            states=states_arr,
            mean_speed=float(np.mean(activity[departure_start:])),
            max_speed=float(np.max(activity[departure_start:])),
            duration_s=(n - departure_start) * DT,
        ))

    return segments


def extract_pouring_phases(pouring_states: np.ndarray) -> Dict[str, slice]:
    """
    在倒奶段内部检测三个子阶段: 融合 → 成形 → 收尾

    基于倒奶段内部的速度变化模式:
    - 融合: 开始倒奶, 速度逐渐上升
    - 成形: 速度高峰持续区
    - 收尾: 速度下降
    """
    n = len(pouring_states)
    vel = np.diff(pouring_states, axis=0) / DT
    speed = np.sum(np.abs(vel), axis=1)
    speed_smooth = savgol_filter(speed, min(21, n//10*2+1), 3)

    # 找速度峰值位置
    peak_idx = np.argmax(speed_smooth)

    # 融合 = 开始到峰值
    mix_end = min(peak_idx, int(n * 0.35))

    # 收尾 = 最后20%
    finish_start = max(int(n * 0.75), peak_idx + int(n * 0.2))

    return {
        'mixing': slice(0, mix_end),
        'drawing': slice(mix_end, finish_start),
        'finishing': slice(finish_start, n),
    }


def analyze_pouring_segment(segment: TrajectorySegment) -> Dict:
    """对倒奶段进行详细分析"""
    states = segment.states
    vel = np.diff(states, axis=0) / DT
    n = len(states)

    # 左右臂速度
    right_speed = np.linalg.norm(vel[:, RIGHT_ARM], axis=1)
    left_speed = np.linalg.norm(vel[:, LEFT_ARM], axis=1)

    # 右臂关节的活动分布
    right_joint_activity = np.mean(np.abs(vel[:, RIGHT_ARM]), axis=0)

    # 子阶段分割
    sub_phases = extract_pouring_phases(states)

    # 摆动检测 (在成形阶段)
    drawing_states = states[sub_phases['drawing']]
    wiggle_results = []
    if len(drawing_states) > 20:
        for j in range(14):
            # FFT
            detrended = drawing_states[:, j] - np.polyval(
                np.polyfit(np.arange(len(drawing_states)), drawing_states[:, j], 1),
                np.arange(len(drawing_states))
            )
            fft = np.fft.rfft(detrended)
            freqs = np.fft.rfftfreq(len(detrended), d=DT)
            mag = np.abs(fft)
            peak_idx = np.argmax(mag[1:]) + 1
            wiggle_results.append({
                'joint': j,
                'dominant_freq_hz': float(freqs[peak_idx]),
                'magnitude': float(mag[peak_idx]),
                'amplitude_std': float(np.std(detrended)),
            })

    return {
        'n_frames': n,
        'duration_s': segment.duration_s,
        'right_arm_speed_mean': float(np.mean(right_speed)),
        'right_arm_speed_max': float(np.max(right_speed)),
        'left_arm_speed_mean': float(np.mean(left_speed)),
        'left_arm_speed_max': float(np.max(left_speed)),
        'activity_ratio_right_left': float(np.sum(right_speed) / (np.sum(left_speed) + 1e-10)),
        'right_joint_activity': right_joint_activity.tolist(),
        'sub_phases': {
            name: {
                'start_frame': int(sl.start),
                'end_frame': int(sl.stop),
                'duration_s': float((sl.stop - sl.start) * DT),
                'pct': float((sl.stop - sl.start) / n * 100),
            }
            for name, sl in sub_phases.items()
        },
        'wiggle_in_drawing': wiggle_results,
        # 速度剖面
        'speed_profile': {
            'right': right_speed.tolist(),
            'left': left_speed.tolist(),
        },
    }


def analyze_all_with_segmentation() -> Dict:
    """分析所有 episode, 三阶段分割 + 倒奶段详细分析"""
    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))
    results = []

    print(f"分析 {len(episode_files)} 个 episodes (靠近→倒奶→离开)...\n")

    for ep_path in episode_files:
        ep_idx = int(ep_path.stem.split('_')[1])
        df = pd.read_parquet(ep_path)
        print(f"Episode {ep_idx}: ", end='')

        segments = segment_full_trajectory(df)

        ep_result = {
            'episode': ep_idx,
            'n_frames': len(df),
            'duration_s': len(df) * DT,
            'segments': [],
        }

        for seg in segments:
            seg_info = {
                'name': seg.name,
                'start_frame': seg.start_frame,
                'end_frame': seg.end_frame,
                'duration_s': seg.duration_s,
                'pct': seg.duration_s / (len(df) * DT) * 100,
                'mean_speed': seg.mean_speed,
                'max_speed': seg.max_speed,
            }
            print(f"{seg.name}={seg.duration_s:.1f}s({seg_info['pct']:.0f}%) ", end='')

            if seg.name == 'pouring':
                seg_info['pouring_analysis'] = analyze_pouring_segment(seg)

            ep_result['segments'].append(seg_info)

        print()
        results.append(ep_result)

    # 汇总统计
    all_pouring_durations = []
    all_approach_durations = []
    all_departure_durations = []
    all_pouring_wiggles = []

    for r in results:
        for seg in r['segments']:
            if seg['name'] == 'approach':
                all_approach_durations.append(seg['duration_s'])
            elif seg['name'] == 'pouring':
                all_pouring_durations.append(seg['duration_s'])
                if 'pouring_analysis' in seg:
                    for w in seg['pouring_analysis']['wiggle_in_drawing']:
                        if w['dominant_freq_hz'] > 0.5:
                            all_pouring_wiggles.append({
                                'episode': r['episode'],
                                'joint': w['joint'],
                                'freq': w['dominant_freq_hz'],
                            })
            elif seg['name'] == 'departure':
                all_departure_durations.append(seg['duration_s'])

    summary = {
        'n_episodes': len(results),
        'approach': {
            'mean_duration_s': float(np.mean(all_approach_durations)),
            'std_duration_s': float(np.std(all_approach_durations)),
        },
        'pouring': {
            'mean_duration_s': float(np.mean(all_pouring_durations)),
            'std_duration_s': float(np.std(all_pouring_durations)),
            'n_wiggle_detected': len(all_pouring_wiggles),
            'max_wiggle_hz': float(max([w['freq'] for w in all_pouring_wiggles])) if all_pouring_wiggles else 0,
        },
        'departure': {
            'mean_duration_s': float(np.mean(all_departure_durations)),
            'std_duration_s': float(np.std(all_departure_durations)),
        },
    }

    return {
        'summary': summary,
        'per_episode': results,
    }


def extract_pouring_trajectories(output_file: str = None):
    """
    提取所有 episode 的纯倒奶段轨迹，保存为独立的 numpy 文件
    供下游任务使用 (模仿学习、轨迹分析等)
    """
    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))
    all_pouring = {}

    for ep_path in episode_files:
        ep_idx = int(ep_path.stem.split('_')[1])
        df = pd.read_parquet(ep_path)
        segments = segment_full_trajectory(df)

        for seg in segments:
            if seg.name == 'pouring':
                all_pouring[f'ep_{ep_idx:04d}'] = {
                    'states': seg.states,
                    'start_frame': seg.start_frame,
                    'end_frame': seg.end_frame,
                    'duration_s': seg.duration_s,
                }

    if output_file:
        np.savez_compressed(output_file, **{
            k: v['states'] for k, v in all_pouring.items()
        })
        # 同时保存元数据
        meta = {k: {kk: (int(vv) if isinstance(vv, (np.integer,)) else float(vv) if isinstance(vv, (np.floating,)) else vv) for kk, vv in v.items() if kk != 'states'}
                for k, v in all_pouring.items()}
        meta_path = Path(output_file).with_suffix('.json')
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        print(f"已保存 {len(all_pouring)} 个纯倒奶轨迹到 {output_file}")

    return all_pouring


# ═══════════════════════════════════════════════════════════════
# 可视化
# ═══════════════════════════════════════════════════════════════

def plot_segmentation(episode_idx: int, save_path: str = None):
    """绘制带三阶段标注的轨迹图"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    df = load_episode(episode_idx)
    segments = segment_full_trajectory(df)
    activity = compute_activity_signal(df)
    states = np.vstack(df['observation.state'].values)
    t = np.arange(len(df)) * DT

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f'Episode {episode_idx} — Full Trajectory Segmentation\n'
                 f'(Approach → Pouring → Departure)', fontsize=13, fontweight='bold')

    # 颜色
    phase_colors = {'approach': 'blue', 'pouring': 'red', 'departure': 'green'}

    # ── 子图1: 活动量信号 + 阈值 ──
    ax = axes[0]
    threshold = np.median(activity) + 0.3 * np.std(activity)
    ax.plot(t, activity, 'k-', lw=0.8, alpha=0.7)
    ax.axhline(y=threshold, color='orange', ls='--', lw=1, label=f'Threshold ({threshold:.3f})')
    for seg in segments:
        ax.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                   alpha=0.15, color=phase_colors.get(seg.name, 'gray'))
        mid_t = (seg.start_frame + seg.end_frame) / 2 * DT
        ax.text(mid_t, activity.max() * 0.95, seg.name.capitalize(),
                ha='center', fontsize=10, fontweight='bold',
                color=phase_colors.get(seg.name, 'black'))
    ax.set_ylabel('Activity (sum|vel|)')
    ax.legend(fontsize=8)

    # ── 子图2: 右臂关节位置 ──
    ax = axes[1]
    for j in range(7):
        ax.plot(t, states[:, j], lw=0.5, alpha=0.7, label=f'J{j}')
    for seg in segments:
        ax.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                   alpha=0.1, color=phase_colors.get(seg.name, 'gray'))
    ax.set_ylabel('Right Arm Joint (rad)')
    ax.legend(loc='upper right', ncol=4, fontsize=7)

    # ── 子图3: 左臂关节位置 ──
    ax = axes[2]
    for j in range(7):
        ax.plot(t, states[:, j + 7], lw=0.5, alpha=0.7, label=f'J{j+7}')
    for seg in segments:
        ax.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                   alpha=0.1, color=phase_colors.get(seg.name, 'gray'))
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Left Arm Joint (rad)')
    ax.legend(loc='upper right', ncol=4, fontsize=7)

    plt.tight_layout()
    sp = save_path or str(OUTPUT_DIR / f'ep_{episode_idx:04d}_segmentation.png')
    plt.savefig(sp, bbox_inches='tight', dpi=150)
    print(f"图已保存: {sp}")
    plt.close()


# ═══════════════════════════════════════════════════════════════
# 主函数
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='轨迹阶段分割器')
    parser.add_argument('--episode', type=int, default=None)
    parser.add_argument('--extract-all', action='store_true',
                        help='提取所有episode的纯倒奶段到文件')
    parser.add_argument('--plot', action='store_true', help='绘制分割图')
    parser.add_argument('--output', type=str, default=None)
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)

    if args.extract_all:
        output = args.output or str(OUTPUT_DIR / 'pouring_trajectories.npz')
        extract_pouring_trajectories(output)
        return

    if args.episode is not None:
        df = load_episode(args.episode)
        segments = segment_full_trajectory(df)

        print(f"\nEpisode {args.episode} ({len(df)} 帧, {len(df)*DT:.1f}s):")
        print(f"{'阶段':<12} {'帧范围':<18} {'时长':<10} {'占比':<8} {'平均速度':<12} {'最高速度'}")
        print("-" * 75)
        for seg in segments:
            print(f"{seg.name:<12} {seg.start_frame:>4}-{seg.end_frame:<4}      "
                  f"{seg.duration_s:<8.1f}s {seg.duration_s/(len(df)*DT)*100:<6.1f}% "
                  f"{seg.mean_speed:<10.4f}  {seg.max_speed:.4f}")

            if seg.name == 'pouring':
                analysis = analyze_pouring_segment(seg)
                print(f"\n  倒奶段子阶段:")
                for name, info in analysis['sub_phases'].items():
                    print(f"    {name}: {info['duration_s']:.1f}s ({info['pct']:.0f}%)")
                print(f"  右/左臂活动比: {analysis['activity_ratio_right_left']:.2f}")
                print(f"  右臂关节活动分布: {[f'{v:.4f}' for v in analysis['right_joint_activity']]}")
                # 摆动
                wiggles = [w for w in analysis['wiggle_in_drawing'] if w['dominant_freq_hz'] > 0.5]
                if wiggles:
                    best = max(wiggles, key=lambda w: w['dominant_freq_hz'])
                    print(f"  成形阶段最强摆动: 关节{best['joint']}, {best['dominant_freq_hz']:.2f}Hz")

        if args.plot:
            plot_segmentation(args.episode)

    else:
        # 批量分析
        results = analyze_all_with_segmentation()
        summary = results['summary']

        print(f"\n{'='*60}")
        print(f"批量分析汇总 ({summary['n_episodes']} episodes)")
        print(f"{'='*60}")
        print(f"靠近阶段: {summary['approach']['mean_duration_s']:.1f}s ± "
              f"{summary['approach']['std_duration_s']:.1f}s")
        print(f"倒奶阶段: {summary['pouring']['mean_duration_s']:.1f}s ± "
              f"{summary['pouring']['std_duration_s']:.1f}s")
        print(f"离开阶段: {summary['departure']['mean_duration_s']:.1f}s ± "
              f"{summary['departure']['std_duration_s']:.1f}s")

        # 检查成形阶段摆动
        if summary['pouring']['n_wiggle_detected'] > 0:
            print(f"\n成形阶段检测到 {summary['pouring']['n_wiggle_detected']} 次摆动")
            print(f"最高摆动频率: {summary['pouring']['max_wiggle_hz']:.2f}Hz")
        else:
            print(f"\n成形阶段未检测到 >0.5Hz 的周期性摆动")
            print(f"→ 这是一致性结论: 此数据集为'倒咖啡'任务, 不含拉花图案摆动")

        # 保存
        output_path = args.output or str(OUTPUT_DIR / 'segmentation_result.json')
        with open(output_path, 'w') as f:
            json.dump(results['summary'], f, indent=2, default=str)
        print(f"\n结果已保存: {output_path}")

        # 绘制几个示例
        if args.plot:
            for ep in [20, 25, 30]:
                if DATA_PATH / f'episode_{ep:06d}.parquet':
                    plot_segmentation(ep)


if __name__ == '__main__':
    main()
