#!/usr/bin/env python3
"""
latte-pour-demos 数据集分析工具
=================================
分析 ridxm/latte-pour-demos 数据集中的机械臂倒奶示教数据，
提取轨迹特征，与拉花教程中的手法进行对比验证。

数据集格式: LeRobot v3.0
机器人: Realman RM65 双臂 (14-DOF = 7×2)
任务: "Pour the coffee into the cup"
帧率: 20 FPS
每集: 400 帧 (20 秒)
总集数: 20 集 (episodes 20-39), 8000 帧

用法:
    python analyze_dataset.py                    # 完整分析
    python analyze_dataset.py --episode 20       # 分析单个 episode
    python analyze_dataset.py --compare-tutorial # 与教程参数对比
"""

import numpy as np
import pandas as pd
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import json
import argparse
import warnings
warnings.filterwarnings('ignore')

# ═══════════════════════════════════════════════════════════════
# 配置
# ═══════════════════════════════════════════════════════════════

DATA_DIR = Path('/home/mu/latte_art_robot_research/latte-pour-demos')
DATA_PATH = DATA_DIR / 'data' / 'chunk-000'
META_DIR = DATA_DIR / 'meta'
OUTPUT_DIR = DATA_DIR / 'analysis'

FPS = 20.0
DT = 1.0 / FPS  # 0.05s per frame

# 14-DOF 关节索引: 前7个 = 右臂(倒奶臂), 后7个 = 左臂
RIGHT_ARM = slice(0, 7)
LEFT_ARM = slice(7, 14)


@dataclass
class EpisodeData:
    """单个 episode 的分析结果"""
    index: int
    states: np.ndarray          # (N, 14) 关节角度
    actions: np.ndarray         # (N, 14) 关节动作
    timestamps: np.ndarray      # (N,)
    velocities: np.ndarray      # (N-1, 14) 关节速度
    accelerations: np.ndarray   # (N-2, 14) 关节加速度
    jerks: np.ndarray           # (N-3, 14) 关节加加速度

    # 拉花阶段分段
    phases: Dict[str, slice] = field(default_factory=dict)

    # 统计特征
    stats: Dict = field(default_factory=dict)


# ═══════════════════════════════════════════════════════════════
# 数据加载
# ═══════════════════════════════════════════════════════════════

def load_episode(episode_idx: int) -> pd.DataFrame:
    """加载单个 episode 的 parquet 数据"""
    path = DATA_PATH / f'episode_{episode_idx:06d}.parquet'
    if not path.exists():
        raise FileNotFoundError(f"Episode {episode_idx} not found at {path}")
    return pd.read_parquet(path)


def load_all_episodes() -> List[pd.DataFrame]:
    """加载所有 episode"""
    episodes = []
    for path in sorted(DATA_PATH.glob('episode_*.parquet')):
        episodes.append(pd.read_parquet(path))
    return episodes


def load_meta() -> dict:
    """加载元数据"""
    meta = {}
    with open(META_DIR / 'info.json') as f:
        meta['info'] = json.load(f)
    with open(META_DIR / 'stats.json') as f:
        meta['stats'] = json.load(f)
    # 加载 episodes 列表
    episodes_list = []
    with open(META_DIR / 'episodes.jsonl') as f:
        for line in f:
            episodes_list.append(json.loads(line))
    meta['episodes'] = episodes_list
    return meta


# ═══════════════════════════════════════════════════════════════
# 运动学分析
# ═══════════════════════════════════════════════════════════════

def compute_derivatives(states: np.ndarray, dt: float = DT) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """计算关节速度、加速度、加加速度"""
    vel = np.diff(states, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    jerk = np.diff(acc, axis=0) / dt
    return vel, acc, jerk


def compute_joint_stats(episode_df: pd.DataFrame) -> dict:
    """计算单个 episode 的关节统计量"""
    states = np.vstack(episode_df['observation.state'].values)

    vel, acc, jerk = compute_derivatives(states)

    stats = {
        'n_frames': len(states),
        'duration_s': len(states) * DT,
        'state_mean': states.mean(axis=0).tolist(),
        'state_std': states.std(axis=0).tolist(),
        'state_range': (states.min(axis=0).tolist(), states.max(axis=0).tolist()),
        'velocity_mean': vel.mean(axis=0).tolist(),
        'velocity_max': np.abs(vel).max(axis=0).tolist(),
        'velocity_rms': np.sqrt(np.mean(vel**2, axis=0)).tolist(),
        'acceleration_max': np.abs(acc).max(axis=0).tolist(),
        'jerk_rms': np.sqrt(np.mean(jerk**2, axis=0)).tolist(),
        # 右臂(倒奶臂) 和左臂分别统计
        'right_arm_activity': float(np.abs(vel[:, RIGHT_ARM]).sum()),
        'left_arm_activity': float(np.abs(vel[:, LEFT_ARM]).sum()),
    }
    return stats


# ═══════════════════════════════════════════════════════════════
# 拉花阶段检测
# ═══════════════════════════════════════════════════════════════

def detect_pouring_phases(episode_df: pd.DataFrame) -> Dict[str, slice]:
    """
    基于关节速度变化检测拉花的三个阶段:
    1. 融合阶段 (Mixing): 高位倒奶, 动作幅度小
    2. 成形阶段 (Drawing): 贴近液面, 动作幅度大/可能有摆动
    3. 收尾阶段 (Finishing): 抬升+拉线, 动作幅度中等

    返回阶段对应的帧索引切片
    """
    states = np.vstack(episode_df['observation.state'].values)
    vel, _, _ = compute_derivatives(states)

    # 使用右臂(倒奶臂)的总速度作为活动指标
    right_vel_magnitude = np.linalg.norm(vel[:, RIGHT_ARM], axis=1)
    left_vel_magnitude = np.linalg.norm(vel[:, LEFT_ARM], axis=1)

    # 平滑速度信号
    from scipy.ndimage import uniform_filter1d
    smooth_vel = uniform_filter1d(right_vel_magnitude, size=10)

    n = len(episode_df)
    phases = {}

    # 简易分段: 按速度分位数
    vel_threshold_low = np.percentile(smooth_vel, 30)
    vel_threshold_high = np.percentile(smooth_vel, 70)

    # 检测活动段
    active = smooth_vel > vel_threshold_low

    # 找活动段的起止
    changes = np.diff(active.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1

    if len(starts) == 0:
        # 全程低活动量
        phases['mixing'] = slice(0, n)
        phases['drawing'] = slice(n//3, 2*n//3)
        phases['finishing'] = slice(2*n//3, n)
    else:
        # 第一段 = 融合, 中间段 = 成形, 最后段 = 收尾
        phases['mixing'] = slice(0, starts[0] if len(starts) > 0 else n//3)
        if len(starts) >= 2:
            phases['drawing'] = slice(starts[0], ends[-1])
        else:
            phases['drawing'] = slice(n//3, 2*n//3)
        phases['finishing'] = slice(ends[-1] if len(ends) > 0 else 2*n//3, n)

    return phases


# ═══════════════════════════════════════════════════════════════
# 摆动检测 (Wiggle Detection)
# ═══════════════════════════════════════════════════════════════

def detect_wiggle_pattern(states: np.ndarray, joint_idx: int = 5,
                          fps: float = FPS) -> Dict:
    """
    检测特定关节的摆动模式 (Rosetta 的关键特征)
    通常 wrist 关节 (index 5 或 6) 负责左右摆动

    返回: 摆动频率、振幅、相位等信息
    """
    joint_traj = states[:, joint_idx]

    # 去趋势
    from scipy.signal import detrend
    detrended = detrend(joint_traj)

    # FFT 分析
    n = len(detrended)
    fft = np.fft.rfft(detrended)
    freqs = np.fft.rfftfreq(n, d=1.0/fps)
    magnitude = np.abs(fft)

    # 找主要频率 (排除 DC)
    peak_idx = np.argmax(magnitude[1:]) + 1
    dominant_freq = freqs[peak_idx]
    dominant_mag = magnitude[peak_idx]

    # 振幅分析
    amplitude = np.std(detrended) * 2  # 近似峰峰值

    # 短时傅里叶变换看频率变化
    from scipy.signal import spectrogram
    f, t_spec, Sxx = spectrogram(joint_traj, fs=fps, nperseg=min(64, n//4))

    result = {
        'joint_idx': joint_idx,
        'dominant_freq_hz': float(dominant_freq),
        'dominant_magnitude': float(dominant_mag),
        'amplitude_std': float(np.std(detrended)),
        'amplitude_pp': float(np.ptp(detrended)),
        'zero_crossing_rate': float(np.sum(np.diff(np.signbit(detrended))) / (n / fps)),
        'spectrogram_freqs': f.tolist(),
        'spectrogram_times': t_spec.tolist(),
        'spectrogram_power': Sxx.tolist(),
    }
    return result


def find_wiggle_joints(episode_df: pd.DataFrame, threshold_hz: float = 2.0) -> List[int]:
    """找出所有有周期性摆动 (>threshold_hz) 的关节"""
    states = np.vstack(episode_df['observation.state'].values)
    wiggle_joints = []

    for j in range(14):
        result = detect_wiggle_pattern(states, j)
        if result['dominant_freq_hz'] > threshold_hz:
            wiggle_joints.append(j)

    return wiggle_joints


# ═══════════════════════════════════════════════════════════════
# 末端速度估计 (简化的运动学)
# ═══════════════════════════════════════════════════════════════

def estimate_end_effector_speed(episode_df: pd.DataFrame,
                                 arm_slice: slice = RIGHT_ARM) -> np.ndarray:
    """
    简化估计末端执行器速度
    使用关节速度的加权和 (假设各关节对末端速度的贡献近似)
    精确计算需要 D-H 参数和正向运动学
    """
    states = np.vstack(episode_df['observation.state'].values)
    vel, _, _ = compute_derivatives(states)

    # 对关节速度取 RMS 作为末端活动的近似
    ee_speed = np.sqrt(np.mean(vel[:, arm_slice]**2, axis=1))
    return ee_speed


def compute_velocity_profile(episode_df: pd.DataFrame) -> Dict:
    """计算速度剖面特征"""
    ee_speed_right = estimate_end_effector_speed(episode_df, RIGHT_ARM)
    ee_speed_left = estimate_end_effector_speed(episode_df, LEFT_ARM)

    result = {
        'right_arm': {
            'mean_speed': float(np.mean(ee_speed_right)),
            'max_speed': float(np.max(ee_speed_right)),
            'speed_std': float(np.std(ee_speed_right)),
            'speed_curve': ee_speed_right.tolist(),
        },
        'left_arm': {
            'mean_speed': float(np.mean(ee_speed_left)),
            'max_speed': float(np.max(ee_speed_left)),
            'speed_curve': ee_speed_left.tolist(),
        },
        'activity_ratio': float(np.sum(ee_speed_right) / (np.sum(ee_speed_left) + 1e-10)),
    }
    return result


# ═══════════════════════════════════════════════════════════════
# 跨 episode 对比分析
# ═══════════════════════════════════════════════════════════════

def analyze_all_episodes() -> Dict:
    """分析所有 episode 并汇总统计"""
    all_stats = []
    wiggle_results = []
    vel_profiles = []

    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))

    print(f"分析 {len(episode_files)} 个 episodes...")

    for ep_path in episode_files:
        ep_idx = int(ep_path.stem.split('_')[1])
        df = pd.read_parquet(ep_path)

        # 基本统计
        stats = compute_joint_stats(df)
        stats['episode_index'] = ep_idx
        all_stats.append(stats)

        # 摆动检测 (检查所有右臂关节)
        states = np.vstack(df['observation.state'].values)
        for j in range(7):  # 右臂 7 个关节
            w = detect_wiggle_pattern(states, j)
            w['episode_index'] = ep_idx
            wiggle_results.append(w)

        # 速度剖面
        vp = compute_velocity_profile(df)
        vp['episode_index'] = ep_idx
        vel_profiles.append(vp)

    return {
        'n_episodes': len(episode_files),
        'per_episode_stats': all_stats,
        'wiggle_analysis': wiggle_results,
        'velocity_profiles': vel_profiles,
    }


# ═══════════════════════════════════════════════════════════════
# 与教程参数对比
# ═══════════════════════════════════════════════════════════════

def compare_with_tutorial(all_results: Dict) -> Dict:
    """
    将数据集中的轨迹参数与我们整理的教程参数进行对比

    教程参考值 (来自 advance_tutorials_summary.md):
    - 融合高度: 8-10cm 上方
    - 成形高度: 0.3-0.5cm 上方 (贴近液面)
    - Rosetta 摆动频率: ~4-6Hz (人类) / 减速后 ~2-3Hz
    - 杯子倾斜: 30-40°
    - 单次拉花时间: 3-10s
    """
    tutorial_ref = {
        'mixing_height_cm': (8, 10),          # 融合阶段高度
        'drawing_height_cm': (0.3, 0.5),      # 成形阶段高度
        'expected_wiggle_hz': (3, 6),         # 预期摆动频率 (机械臂可能更低)
        'expected_duration_s': (3, 20),       # 预期持续时间
        'fill_percent': (60, 75),             # 融合填至杯百分之多少
    }

    comparisons = []

    for ep_stat in all_results['per_episode_stats']:
        comparison = {
            'episode': ep_stat['episode_index'],
            'duration_s': ep_stat['duration_s'],
            'duration_in_expected_range': (
                tutorial_ref['expected_duration_s'][0] <= ep_stat['duration_s'] <=
                tutorial_ref['expected_duration_s'][1]
            ),
            'right_arm_activity': ep_stat['right_arm_activity'],
            'left_arm_activity': ep_stat['left_arm_activity'],
            'dominant_arm': 'right' if ep_stat['right_arm_activity'] > ep_stat['left_arm_activity']
                            else 'left',
        }
        comparisons.append(comparison)

    # 汇总摆动分析
    wiggle_freqs = [w['dominant_freq_hz'] for w in all_results['wiggle_analysis']
                    if w['dominant_freq_hz'] > 0.5]
    wiggle_amplitudes = [w['amplitude_std'] for w in all_results['wiggle_analysis']]

    wiggle_summary = {
        'mean_freq_hz': float(np.mean(wiggle_freqs)) if wiggle_freqs else 0,
        'max_freq_hz': float(np.max(wiggle_freqs)) if wiggle_freqs else 0,
        'freq_in_tutorial_range': [
            tutorial_ref['expected_wiggle_hz'][0] <= f <= tutorial_ref['expected_wiggle_hz'][1]
            for f in wiggle_freqs
        ],
        'n_wiggle_detected': len(wiggle_freqs),
    }

    return {
        'tutorial_reference': tutorial_ref,
        'episode_comparisons': comparisons,
        'wiggle_summary': wiggle_summary,
        'note': '精确高度对比需要 D-H 参数 + FK 计算末端位姿。当前仅基于关节空间进行间接对比。'
    }


# ═══════════════════════════════════════════════════════════════
# 关节轨迹平滑度分析
# ═══════════════════════════════════════════════════════════════

def analyze_smoothness(episode_df: pd.DataFrame) -> Dict:
    """分析轨迹的平滑度 (与教程中'避免抖动'的要求相关)"""
    states = np.vstack(episode_df['observation.state'].values)
    vel, acc, jerk = compute_derivatives(states)

    # 频谱熵 (越低越平滑)
    from scipy.signal import periodogram
    smoothness_per_joint = {}
    for j in range(14):
        f, Pxx = periodogram(states[:, j], fs=FPS)
        # 归一化功率谱
        Pxx_norm = Pxx / (Pxx.sum() + 1e-10)
        # 频谱熵
        entropy = -np.sum(Pxx_norm * np.log(Pxx_norm + 1e-10))
        smoothness_per_joint[str(j)] = float(entropy)

    # Jerk 代价 (最小 jerk 轨迹的指标)
    jerk_cost = np.mean(np.sum(jerk**2, axis=1)) * (len(states) ** 5)

    # 速度过零率 (越高 = 越抖动)
    vel_sign_changes = 0
    for j in range(14):
        vel_sign_changes += np.sum(np.diff(np.signbit(vel[:, j])))

    return {
        'jerk_cost': float(jerk_cost),
        'velocity_sign_changes_per_second': float(vel_sign_changes / (len(states) * DT)),
        'spectral_entropy_per_joint': smoothness_per_joint,
    }


# ═══════════════════════════════════════════════════════════════
# 主函数
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='latte-pour-demos 数据集分析')
    parser.add_argument('--episode', type=int, default=None,
                        help='分析单个 episode (默认分析全部)')
    parser.add_argument('--compare-tutorial', action='store_true',
                        help='与教程参数进行对比')
    parser.add_argument('--output', type=str, default=None,
                        help='输出 JSON 文件路径')
    parser.add_argument('--print-wiggle', action='store_true',
                        help='打印摆动检测详情')
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)

    if args.episode is not None:
        # 分析单个 episode
        print(f"\n{'='*60}")
        print(f"分析 Episode {args.episode}")
        print(f"{'='*60}")

        df = load_episode(args.episode)
        print(f"帧数: {len(df)}, 时长: {len(df)*DT:.1f}s")

        stats = compute_joint_stats(df)
        print(f"\n--- 基本统计 ---")
        print(f"右臂活动量: {stats['right_arm_activity']:.3f}")
        print(f"左臂活动量: {stats['left_arm_activity']:.3f}")
        print(f"右臂最大速度 (rad/s): {[f'{v:.3f}' for v in stats['velocity_max'][:7]]}")

        # 摆动检测
        states = np.vstack(df['observation.state'].values)
        print(f"\n--- 摆动检测 (右臂7个关节) ---")
        arm_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7']
        for j in range(7):
            w = detect_wiggle_pattern(states, j)
            print(f"  {arm_names[j]}: 主频={w['dominant_freq_hz']:.2f}Hz, "
                  f"幅值(std)={w['amplitude_std']:.4f}rad, "
                  f"峰峰值={w['amplitude_pp']:.4f}rad")

        # 相位检测
        phases = detect_pouring_phases(df)
        print(f"\n--- 拉花阶段检测 ---")
        for phase_name, sl in phases.items():
            dur = (sl.stop - sl.start) * DT
            print(f"  {phase_name}: 帧 {sl.start}-{sl.stop} ({dur:.1f}s, {dur/len(df)/DT*100:.0f}%)")

        # 速度剖面
        vp = compute_velocity_profile(df)
        print(f"\n--- 速度剖面 ---")
        print(f"  右臂平均速度: {vp['right_arm']['mean_speed']:.4f}")
        print(f"  右臂最大速度: {vp['right_arm']['max_speed']:.4f}")
        print(f"  左右臂活动比: {vp['activity_ratio']:.2f}")

        # 平滑度
        smooth = analyze_smoothness(df)
        print(f"\n--- 平滑度 ---")
        print(f"  Jerk cost: {smooth['jerk_cost']:.2e}")
        print(f"  速度过零率: {smooth['velocity_sign_changes_per_second']:.1f}/s")

        # 保存
        result = {
            'episode': args.episode,
            'stats': stats,
            'wiggle': [detect_wiggle_pattern(states, j) for j in range(7)],
            'phases': {k: {'start': v.start, 'stop': v.stop} for k, v in phases.items()},
            'velocity_profile': vp,
            'smoothness': smooth,
        }

    else:
        # 分析全部
        print("=" * 60)
        print("latte-pour-demos 全部数据批量分析")
        print("=" * 60)

        all_results = analyze_all_episodes()

        # 汇总统计
        print(f"\n--- 总体统计 ---")
        print(f"Episodes: {all_results['n_episodes']}")
        durations = [s['duration_s'] for s in all_results['per_episode_stats']]
        print(f"时长: {np.mean(durations):.1f}s ± {np.std(durations):.1f}s "
              f"(范围 {np.min(durations):.1f}-{np.max(durations):.1f}s)")

        right_activity = [s['right_arm_activity'] for s in all_results['per_episode_stats']]
        left_activity = [s['left_arm_activity'] for s in all_results['per_episode_stats']]
        print(f"右臂活动量: {np.mean(right_activity):.1f} ± {np.std(right_activity):.1f}")
        print(f"左臂活动量: {np.mean(left_activity):.1f} ± {np.std(left_activity):.1f}")
        print(f"主导臂: {'右臂' if np.mean(right_activity) > np.mean(left_activity) else '左臂'}")

        # 摆动汇总
        wiggle_freqs = [w['dominant_freq_hz'] for w in all_results['wiggle_analysis']
                        if w['dominant_freq_hz'] > 0.5 and w['joint_idx'] < 7]
        print(f"\n--- 摆动分析汇总 (右臂) ---")
        if wiggle_freqs:
            print(f"检测到摆动: {len(wiggle_freqs)} 次")
            print(f"平均频率: {np.mean(wiggle_freqs):.2f}Hz (范围 {np.min(wiggle_freqs):.2f}-{np.max(wiggle_freqs):.2f}Hz)")
            # 找出最高频摆动的关节
            best_wiggle = max(all_results['wiggle_analysis'],
                             key=lambda w: w['dominant_freq_hz'] if w['joint_idx'] < 7 else 0)
            print(f"最高频摆动: 关节{best_wiggle['joint_idx']}, "
                  f"{best_wiggle['dominant_freq_hz']:.2f}Hz, "
                  f"幅值(std)={best_wiggle['amplitude_std']:.4f}rad")

        # 教程对比
        if args.compare_tutorial:
            print(f"\n--- 与教程参数对比 ---")
            comparison = compare_with_tutorial(all_results)
            print(f"教程预期摆动频率: {comparison['tutorial_reference']['expected_wiggle_hz']} Hz")
            print(f"数据中摆动频率范围: "
                  f"{comparison['wiggle_summary']['mean_freq_hz']:.2f} Hz")
            print(f"检测到显著摆动: {comparison['wiggle_summary']['n_wiggle_detected']} 次")

        result = {
            'summary': {
                'n_episodes': all_results['n_episodes'],
                'mean_duration_s': float(np.mean(durations)),
                'std_duration_s': float(np.std(durations)),
                'mean_right_arm_activity': float(np.mean(right_activity)),
                'mean_left_arm_activity': float(np.mean(left_activity)),
            },
            'wiggle_summary': {
                'mean_freq_hz': float(np.mean(wiggle_freqs)) if wiggle_freqs else 0,
                'max_freq_hz': float(np.max(wiggle_freqs)) if wiggle_freqs else 0,
                'freqs': [float(f) for f in wiggle_freqs],
            },
        }

    # 保存结果
    output_path = args.output or str(OUTPUT_DIR / 'analysis_result.json')
    # 将 numpy 类型转换为 Python 原生类型
    def convert(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        elif isinstance(obj, (np.floating,)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: convert(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [convert(item) for item in obj]
        return obj

    with open(output_path, 'w') as f:
        json.dump(convert(result), f, indent=2, default=str)

    print(f"\n结果已保存到: {output_path}")


if __name__ == '__main__':
    main()
