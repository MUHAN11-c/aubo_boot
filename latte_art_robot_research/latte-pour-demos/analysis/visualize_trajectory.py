#!/usr/bin/env python3
"""
拉花轨迹可视化工具
=====================
将 latte-pour-demos 数据中的关节轨迹可视化，
并与理论拉花模型进行对比。

用法:
    python visualize_trajectory.py --episode 20          # 可视化单个episode
    python visualize_trajectory.py --all --save          # 批量生成全部图表
    python visualize_trajectory.py --compare-theory       # 与理论模型对比
"""

import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import json
from analyze_dataset import (load_episode, compute_derivatives,
                              detect_wiggle_pattern, detect_pouring_phases,
                              compute_velocity_profile, FPS, DT,
                              RIGHT_ARM, LEFT_ARM, DATA_PATH, OUTPUT_DIR)

plt.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 12,
    'axes.labelsize': 10,
    'figure.dpi': 150,
})

# 中文字体支持
try:
    plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
except Exception:
    pass

ARM_JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
COLORS = plt.cm.tab10(np.linspace(0, 1, 14))


# ═══════════════════════════════════════════════════════════════
# 单 Episode 可视化
# ═══════════════════════════════════════════════════════════════

def plot_episode_overview(episode_idx: int, save_path: str = None):
    """绘制单个 episode 的完整概览图"""
    df = load_episode(episode_idx)
    states = np.vstack(df['observation.state'].values)
    vel, acc, jerk = compute_derivatives(states)
    timestamps = np.arange(len(states)) * DT
    phases = detect_pouring_phases(df)

    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(4, 2, figure=fig, hspace=0.35, wspace=0.3)

    # ── 子图1: 右臂(倒奶臂) 关节位置 ──
    ax1 = fig.add_subplot(gs[0, 0])
    for j in range(7):
        ax1.plot(timestamps, states[:, j], color=COLORS[j], label=ARM_JOINT_NAMES[j], alpha=0.8, lw=0.6)
    ax1.set_title('Right Arm (Pouring) Joint Positions')
    ax1.set_ylabel('Joint Angle (rad)')
    ax1.legend(loc='upper right', ncol=4, fontsize=7)
    # 标记阶段
    for phase_name, sl in phases.items():
        if sl.start < sl.stop:
            ax1.axvspan(sl.start * DT, (sl.stop - 1) * DT, alpha=0.1,
                       label=phase_name, color={'mixing': 'blue', 'drawing': 'red', 'finishing': 'green'}[phase_name])

    # ── 子图2: 左臂关节位置 ──
    ax2 = fig.add_subplot(gs[0, 1])
    for j in range(7):
        ax2.plot(timestamps, states[:, j + 7], color=COLORS[j + 7], label=ARM_JOINT_NAMES[j], alpha=0.8, lw=0.6)
    ax2.set_title('Left Arm Joint Positions')
    ax2.set_ylabel('Joint Angle (rad)')

    # ── 子图3: 右臂关节速度 ──
    ax3 = fig.add_subplot(gs[1, 0])
    for j in range(7):
        ax3.plot(timestamps[1:], vel[:, j], color=COLORS[j], label=ARM_JOINT_NAMES[j], alpha=0.7, lw=0.5)
    ax3.set_title('Right Arm Joint Velocities')
    ax3.set_ylabel('Velocity (rad/s)')
    ax3.axhline(y=0, color='black', lw=0.5, ls='--')

    # ── 子图4: 左臂关节速度 ──
    ax4 = fig.add_subplot(gs[1, 1])
    for j in range(7):
        ax4.plot(timestamps[1:], vel[:, j + 7], color=COLORS[j + 7], alpha=0.7, lw=0.5)
    ax4.set_title('Left Arm Joint Velocities')
    ax4.set_ylabel('Velocity (rad/s)')
    ax4.axhline(y=0, color='black', lw=0.5, ls='--')

    # ── 子图5: 末端执行器估计速度 ──
    ax5 = fig.add_subplot(gs[2, :])
    vp = compute_velocity_profile(df)
    t_vel = timestamps[1:]
    ax5.plot(t_vel, vp['right_arm']['speed_curve'], 'b-', label='Right Arm (Pouring)', alpha=0.8, lw=1)
    ax5.plot(t_vel, vp['left_arm']['speed_curve'], 'orange', label='Left Arm', alpha=0.6, lw=1)
    ax5.set_title('Estimated End-Effector Speed')
    ax5.set_ylabel('Speed (rad/s RMS)')
    ax5.set_xlabel('Time (s)')
    ax5.legend()
    for phase_name, sl in phases.items():
        if sl.start < sl.stop:
            ax5.axvspan(max(0, sl.start - 1) * DT, (sl.stop - 1) * DT, alpha=0.1,
                       color={'mixing': 'blue', 'drawing': 'red', 'finishing': 'green'}[phase_name])

    # ── 子图6: 摆动关节的频谱 ──
    ax6 = fig.add_subplot(gs[3, 0])
    # 找摆动最显著的关节
    best_wiggle = None
    best_freq = 0
    for j in range(14):
        w = detect_wiggle_pattern(states, j)
        if w['dominant_freq_hz'] > best_freq and w['dominant_freq_hz'] < FPS/2 - 1:
            best_freq = w['dominant_freq_hz']
            best_wiggle = w

    if best_wiggle:
        n = len(states)
        fft = np.fft.rfft(states[:, best_wiggle['joint_idx']] - np.mean(states[:, best_wiggle['joint_idx']]))
        freqs = np.fft.rfftfreq(n, d=1.0/FPS)
        mag = np.abs(fft)
        ax6.plot(freqs[1:], mag[1:], 'b-', lw=0.8)
        ax6.axvline(x=best_freq, color='r', ls='--', label=f'Peak: {best_freq:.2f}Hz')
        ax6.set_title(f'Joint {best_wiggle["joint_idx"]} FFT (Best Wiggle)')
        ax6.set_xlabel('Frequency (Hz)')
        ax6.set_ylabel('Magnitude')
        ax6.set_xlim(0, FPS/2)
        ax6.legend()

    # ── 子图7: 加速度剖面 (右臂) ──
    ax7 = fig.add_subplot(gs[3, 1])
    t_acc = timestamps[2:]
    acc_mag = np.linalg.norm(acc[:, RIGHT_ARM], axis=1)
    ax7.plot(t_acc, acc_mag, 'r-', lw=0.8)
    ax7.set_title('Right Arm Acceleration Magnitude')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Acceleration (rad/s²)')

    fig.suptitle(f'Episode {episode_idx} — Latte Pour Demo Analysis\n'
                 f'(Realman RM65 Dual Arm, {len(states)} frames, {len(states)*DT:.1f}s)',
                 fontsize=13, fontweight='bold')

    if save_path:
        plt.savefig(save_path, bbox_inches='tight', dpi=150)
        print(f"图已保存: {save_path}")
    else:
        plt.savefig(str(OUTPUT_DIR / f'episode_{episode_idx:04d}_overview.png'),
                    bbox_inches='tight', dpi=150)
        print(f"图已保存: {OUTPUT_DIR / f'episode_{episode_idx:04d}_overview.png'}")
    plt.close()


# ═══════════════════════════════════════════════════════════════
# 批量对比可视化
# ═══════════════════════════════════════════════════════════════

def plot_all_episodes_comparison(save_path: str = None):
    """绘制所有 episode 的对比图"""
    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))
    n_eps = len(episode_files)

    print(f"加载 {n_eps} 个 episodes...")

    # 收集所有数据
    all_speed_profiles = []
    all_wiggle_freqs = []
    all_durations = []
    all_right_activity = []

    for ep_path in episode_files:
        ep_idx = int(ep_path.stem.split('_')[1])
        df = pd.read_parquet(ep_path)
        states = np.vstack(df['observation.state'].values)

        vp = compute_velocity_profile(df)
        all_speed_profiles.append(vp['right_arm']['speed_curve'])
        all_durations.append(len(states) * DT)
        all_right_activity.append(np.sum(np.abs(np.diff(states[:, RIGHT_ARM], axis=0))))

        for j in range(7):
            w = detect_wiggle_pattern(states, j)
            if w['dominant_freq_hz'] > 0.5:
                all_wiggle_freqs.append({
                    'episode': ep_idx,
                    'joint': j,
                    'freq': w['dominant_freq_hz'],
                    'amplitude': w['amplitude_std'],
                })

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Latte-Pour-Demos: All Episodes Cross-Comparison', fontsize=14, fontweight='bold')

    # 子图1: 各 episode 速度剖面叠加
    ax = axes[0, 0]
    for i, sp in enumerate(all_speed_profiles):
        t = np.arange(len(sp)) * DT
        ax.plot(t, sp, alpha=0.3, lw=0.5)
    ax.set_title('All Episodes Speed Profiles')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('EE Speed (rad/s RMS)')

    # 子图2: 各 episode 时长分布
    ax = axes[0, 1]
    ax.hist(all_durations, bins=15, edgecolor='black', alpha=0.7)
    ax.axvline(x=np.mean(all_durations), color='r', ls='--',
               label=f'Mean: {np.mean(all_durations):.1f}s')
    ax.set_title('Episode Duration Distribution')
    ax.set_xlabel('Duration (s)')
    ax.set_ylabel('Count')
    ax.legend()

    # 子图3: 摆动频率分布
    ax = axes[0, 2]
    if all_wiggle_freqs:
        freqs = [w['freq'] for w in all_wiggle_freqs]
        ax.hist(freqs, bins=20, edgecolor='black', alpha=0.7)
        ax.axvline(x=np.mean(freqs), color='r', ls='--',
                   label=f'Mean: {np.mean(freqs):.2f}Hz')
        ax.axvspan(3, 6, alpha=0.1, color='green', label='Tutorial range (3-6Hz)')
        ax.set_title('Wiggle Frequency Distribution')
        ax.set_xlabel('Frequency (Hz)')
        ax.legend()
    else:
        ax.text(0.5, 0.5, 'No significant wiggle detected\n(>0.5Hz)', ha='center',
                transform=ax.transAxes, fontsize=12)

    # 子图4: 各关节的平均速度 (全部 episode)
    ax = axes[1, 0]
    all_vel_mean = []
    for ep_path in episode_files:
        df = pd.read_parquet(ep_path)
        states = np.vstack(df['observation.state'].values)
        vel = np.diff(states, axis=0) / DT
        all_vel_mean.append(np.mean(np.abs(vel), axis=0))
    all_vel_mean = np.array(all_vel_mean)
    ax.boxplot([all_vel_mean[:, j] for j in range(14)])
    ax.set_xticklabels([f'{j}' for j in range(14)])
    ax.set_title('Joint Velocity Distribution (14 joints)')
    ax.set_xlabel('Joint Index (0-6: Right, 7-13: Left)')
    ax.set_ylabel('Mean Absolute Velocity (rad/s)')
    ax.axvline(x=6.5, color='red', ls='--', alpha=0.5, label='Right/Left split')
    ax.legend(fontsize=8)

    # 子图5: 右臂活动量 vs 左臂活动量
    ax = axes[1, 1]
    right_activity = []
    left_activity = []
    for ep_path in episode_files:
        df = pd.read_parquet(ep_path)
        states = np.vstack(df['observation.state'].values)
        vel = np.diff(states, axis=0) / DT
        right_activity.append(np.sum(np.abs(vel[:, RIGHT_ARM])))
        left_activity.append(np.sum(np.abs(vel[:, LEFT_ARM])))
    ax.scatter(right_activity, left_activity, alpha=0.7)
    max_val = max(max(right_activity), max(left_activity))
    ax.plot([0, max_val], [0, max_val], 'r--', alpha=0.5, label='Equal activity')
    ax.set_title('Right vs Left Arm Activity')
    ax.set_xlabel('Right Arm Total Activity')
    ax.set_ylabel('Left Arm Total Activity')
    ax.legend()

    # 子图6: 速度剖面模板 (平均)
    ax = axes[1, 2]
    # 将所有速度剖面插值到相同长度
    from scipy.interpolate import interp1d
    target_len = 400
    interpolated = []
    for sp in all_speed_profiles:
        if len(sp) > 2:
            f = interp1d(np.linspace(0, 1, len(sp)), sp, kind='linear',
                         fill_value='extrapolate')
            interpolated.append(f(np.linspace(0, 1, target_len)))
    if interpolated:
        mean_profile = np.mean(interpolated, axis=0)
        std_profile = np.std(interpolated, axis=0)
        t_norm = np.linspace(0, 1, target_len)
        ax.fill_between(t_norm, mean_profile - std_profile, mean_profile + std_profile,
                        alpha=0.3, color='blue')
        ax.plot(t_norm, mean_profile, 'b-', lw=1.5, label='Mean ± 1σ')
        ax.set_title('Averaged Speed Profile Template')
        ax.set_xlabel('Normalized Time')
        ax.set_ylabel('EE Speed (a.u.)')
        ax.legend()

    plt.tight_layout()
    sp = save_path or str(OUTPUT_DIR / 'all_episodes_comparison.png')
    plt.savefig(sp, bbox_inches='tight', dpi=150)
    print(f"图已保存: {sp}")
    plt.close()


# ═══════════════════════════════════════════════════════════════
# 与理论模型对比
# ═══════════════════════════════════════════════════════════════

def plot_theory_comparison(save_path: str = None):
    """
    将数据集中提取的轨迹模式与拉花教程中的理论模型进行可视化对比
    """
    from scipy.interpolate import interp1d

    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    fig.suptitle('Theory vs. Data: Latte Art Trajectory Comparison',
                 fontsize=14, fontweight='bold')

    # ── 1. 理论心形轨迹 vs 实际关节轨迹 ──
    ax = axes[0, 0]
    # 理论心形 (来自 latte_art_trajectory.py 的 heart() 方法)
    t_theory = np.linspace(0, 1, 200)
    cup_radius = 0.04
    # 混合期
    mix_end = 50
    # 摇摆期 (XY 小幅振荡)
    sway = 0.003 * np.sin(2 * np.pi * 4 * t_theory[50:140])
    # Z轴 (从高到低到高)
    z_theory = np.ones(200) * 0.076
    z_theory[50:140] = 0.006
    z_theory[140:] = 0.076
    ax.plot(t_theory, z_theory, 'b-', lw=2, label='Theory Z (height)')
    ax.fill_between(t_theory[50:140], 0.02, -0.02, alpha=0.1, color='red', label='Drawing phase')
    ax.set_title('Theory: Heart Pattern Height Profile')
    ax.set_xlabel('Normalized Time')
    ax.set_ylabel('Z Height (m)')
    ax.legend(fontsize=8)

    # ── 2. 实际关节摆动模式 (选一个典型的) ──
    ax = axes[0, 1]
    # 加载几个 episode, 找有摆动的关节
    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))
    best_joint = None
    best_osc = 0

    for ep_path in episode_files[:5]:
        df = pd.read_parquet(ep_path)
        states = np.vstack(df['observation.state'].values)
        for j in range(7):
            w = detect_wiggle_pattern(states, j)
            if w['dominant_freq_hz'] > best_osc and w['dominant_freq_hz'] < FPS/2 - 1:
                best_osc = w['dominant_freq_hz']
                best_joint = (ep_path, j, states)

    if best_joint:
        ep_path, joint_idx, states = best_joint
        ep_idx = int(ep_path.stem.split('_')[1])
        t_actual = np.arange(len(states)) * DT
        # 去趋势后显示摆动
        from scipy.signal import detrend
        joint_traj = detrend(states[:, joint_idx])
        ax.plot(t_actual, joint_traj, 'b-', lw=0.8, label=f'Ep{ep_idx} J{joint_idx}')
        ax.set_title(f'Actual Joint Oscillation (Ep{ep_idx}, Joint{joint_idx})\n'
                     f'Dominant Freq: {best_osc:.2f}Hz')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Detrended Joint Angle (rad)')
        ax.legend(fontsize=8)

    # ── 3. 理论 Rosetta 摆动+后移 vs 实际 ──
    ax = axes[0, 2]
    # 理论: 振幅递减的正弦波
    t_r = np.linspace(0, 1, 300)
    A_decay = 0.01 * (1 - t_r)
    x_theory = A_decay * np.sin(2 * np.pi * 5 * t_r * 1.5)
    ax.plot(t_r, x_theory, 'b-', lw=1.5, label='Theory: Decaying amplitude wiggle')
    ax.set_title('Theory: Rosetta Wiggle (Amp Decay)')
    ax.set_xlabel('Normalized Time')
    ax.set_ylabel('X Displacement (m)')
    ax.legend(fontsize=8)

    # ── 4. 速度剖面对比 ──
    ax = axes[1, 0]
    # 理论速度剖面
    t_v = np.linspace(0, 1, 400)
    v_theory = np.ones_like(t_v) * 0.5
    v_theory[:60] = np.linspace(0.1, 0.5, 60)   # 加速
    v_theory[-60:] = np.linspace(0.5, 0.1, 60)  # 减速
    # 成形阶段加速
    v_theory[120:280] = np.linspace(0.5, 0.9, 160)
    ax.plot(t_v, v_theory, 'b-', lw=2, label='Theory (variable flow rate)')

    # 实际速度剖面 (归一化)
    for ep_path in episode_files[:5]:
        df = pd.read_parquet(ep_path)
        vp = compute_velocity_profile(df)
        sp = vp['right_arm']['speed_curve']
        if len(sp) > 2:
            f = interp1d(np.linspace(0, 1, len(sp)), sp, fill_value='extrapolate')
            ax.plot(np.linspace(0, 1, 400), f(np.linspace(0, 1, 400)),
                    alpha=0.3, lw=0.5, color='gray')
    ax.set_title('Speed Profile: Theory vs Data')
    ax.set_xlabel('Normalized Time')
    ax.set_ylabel('Normalized Speed')
    ax.legend(fontsize=8)

    # ── 5. 关节活动热力图 (所有episode, 右臂) ──
    ax = axes[1, 1]
    activity_matrix = []
    for ep_path in episode_files:
        df = pd.read_parquet(ep_path)
        states = np.vstack(df['observation.state'].values)
        vel = np.diff(states, axis=0) / DT
        activity_matrix.append(np.mean(np.abs(vel[:, RIGHT_ARM]), axis=0))
    activity_matrix = np.array(activity_matrix)
    im = ax.imshow(activity_matrix.T, aspect='auto', cmap='YlOrRd')
    ax.set_title('Right Arm Joint Activity Heatmap')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Joint Index')
    ax.set_yticks(range(7))
    ax.set_yticklabels(ARM_JOINT_NAMES)
    plt.colorbar(im, ax=ax, label='Mean |Velocity| (rad/s)')

    # ── 6. 与教程参数对照表 ──
    ax = axes[1, 2]
    ax.axis('off')
    tutorial_params = [
        ('Parameter', 'Tutorial', 'Dataset'),
        ('Mixing Height', '8-10cm', 'N/A (joint space)'),
        ('Drawing Height', '0.3-0.5cm', 'N/A (joint space)'),
        ('Expected Wiggle', '3-6 Hz (human)', 'See FFT analysis'),
        ('Duration', '3-10s (human)', f'{np.mean([len(pd.read_parquet(p)) for p in episode_files]) * DT:.1f}s'),
        ('Flow Rate', 'Increasing', 'From speed profile'),
        ('Cup Tilt', '30-45°', 'N/A'),
    ]
    table = ax.table(cellText=tutorial_params, cellLoc='left',
                     loc='center', colWidths=[0.25, 0.25, 0.3])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    ax.set_title('Tutorial vs Dataset Comparison', fontweight='bold')

    plt.tight_layout()
    sp = save_path or str(OUTPUT_DIR / 'theory_vs_data_comparison.png')
    plt.savefig(sp, bbox_inches='tight', dpi=150)
    print(f"图已保存: {sp}")
    plt.close()


# ═══════════════════════════════════════════════════════════════
# 主函数
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='拉花轨迹可视化')
    parser.add_argument('--episode', type=int, default=None, help='可视化单个episode')
    parser.add_argument('--all', action='store_true', help='批量生成全部对比图')
    parser.add_argument('--save', action='store_true', help='保存图片')
    parser.add_argument('--compare-theory', action='store_true', help='与理论模型对比')
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)

    if args.episode is not None:
        save_path = str(OUTPUT_DIR / f'episode_{args.episode:04d}_overview.png') if args.save else None
        plot_episode_overview(args.episode, save_path)

    if args.all:
        plot_all_episodes_comparison(
            str(OUTPUT_DIR / 'all_episodes_comparison.png') if args.save else None
        )

    if args.compare_theory:
        plot_theory_comparison(
            str(OUTPUT_DIR / 'theory_vs_data_comparison.png') if args.save else None
        )

    # 默认: 无参数时运行所有
    if not any([args.episode, args.all, args.compare_theory]):
        print("运行默认可视化...")
        plot_episode_overview(20, str(OUTPUT_DIR / 'episode_0020_overview.png'))
        plot_episode_overview(25, str(OUTPUT_DIR / 'episode_0025_overview.png'))
        plot_episode_overview(30, str(OUTPUT_DIR / 'episode_0030_overview.png'))
        plot_all_episodes_comparison(str(OUTPUT_DIR / 'all_episodes_comparison.png'))
        plot_theory_comparison(str(OUTPUT_DIR / 'theory_vs_data_comparison.png'))
        print("\n全部可视化完成!")


if __name__ == '__main__':
    main()
