#!/usr/bin/env python3
"""
视频 + 关节数据联合分析
==========================
将数据集视频与关节轨迹数据同步分析：
1. 在分段边界提取关键帧
2. 生成侧-by-side对比图 (视频帧 + 关节轨迹)
3. 按拉花心形四步法 (打圈→摆动→抬起→划穿) 分析视频内容

用法:
    python video_joint_analysis.py --episode 20           # 单集分析
    python video_joint_analysis.py --episode 20 --video   # 生成视频帧+轨迹对比图
    python video_joint_analysis.py --all --sample         # 采样多个episode的关键帧
"""

import numpy as np
import pandas as pd
from pathlib import Path
import json
import argparse
import subprocess
import sys
import os

DATA_DIR = Path('/home/mu/latte_art_robot_research/latte-pour-demos')
DATA_PATH = DATA_DIR / 'data' / 'chunk-000'
VIDEO_DIR = DATA_DIR / 'videos' / 'chunk-000'
OUTPUT_DIR = DATA_DIR / 'analysis'
FRAMES_OUT = OUTPUT_DIR / 'key_frames'
DT = 0.05  # 20 FPS

# 三个摄像头视角
CAMERAS = ['observation.images.top_camera', 'observation.images.left_wrist', 'observation.images.right_wrist']

from analyze_dataset import load_episode, compute_derivatives, RIGHT_ARM, LEFT_ARM, DT as _DT
from segment_trajectory import (segment_full_trajectory, compute_activity_signal,
                                 analyze_pouring_segment, extract_pouring_phases)


def extract_frames(video_path: str, frame_indices: list, output_dir: str, prefix: str):
    """用 ffmpeg 提取指定帧"""
    os.makedirs(output_dir, exist_ok=True)
    for idx in frame_indices:
        out_path = os.path.join(output_dir, f'{prefix}_frame_{idx:04d}.jpg')
        # ffmpeg select 帧号
        cmd = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-i', video_path,
            '-vf', f'select=eq(n\\,{idx})',
            '-vframes', '1',
            out_path
        ]
        subprocess.run(cmd)
    return [os.path.join(output_dir, f'{prefix}_frame_{idx:04d}.jpg')
            for idx in frame_indices]


def extract_keyframe_grid(video_path: str, frame_indices: list,
                          output_path: str, cols: int = 6):
    """提取关键帧并拼接为网格图"""
    import cv2
    cap = cv2.VideoCapture(video_path)
    frames = []
    labels = []

    for idx in frame_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (320, 240))
            frames.append(frame)
            labels.append(f'F{idx}')

    cap.release()

    if not frames:
        print("未能提取任何帧")
        return

    # 计算网格尺寸
    n = len(frames)
    rows = (n + cols - 1) // cols

    # 拼接
    h, w = 240, 320
    grid = np.ones((rows * (h + 30), cols * w, 3), dtype=np.uint8) * 240

    for i, (frame, label) in enumerate(zip(frames, labels)):
        r = i // cols
        c = i % cols
        y0 = r * (h + 30)
        x0 = c * w
        grid[y0:y0 + h, x0:x0 + w] = frame
        cv2.putText(grid, label, (x0 + 5, y0 + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.imwrite(output_path, grid)
    print(f"关键帧网格已保存: {output_path}")


def analyze_episode_with_video(episode_idx: int):
    """
    对单个 episode 进行完整的视频+关节联合分析
    按拉花心形五步法标注关键帧
    """
    print(f"\n{'='*70}")
    print(f"Episode {episode_idx} — 视频+关节联合分析")
    print(f"{'='*70}")

    # 加载关节数据
    df = load_episode(episode_idx)
    states = np.vstack(df['observation.state'].values)
    activity = compute_activity_signal(df)
    segments = segment_full_trajectory(df)
    n = len(df)

    # 找倒奶段
    pouring_seg = None
    for seg in segments:
        if seg.name == 'pouring':
            pouring_seg = seg
            break

    if pouring_seg is None:
        print("未找到倒奶段!")
        return

    # 倒奶段内的子阶段
    sub_phases = extract_pouring_phases(pouring_seg.states)
    pouring_start = pouring_seg.start_frame
    pouring_end = pouring_seg.end_frame

    # 关键帧 (相对于整个episode)
    key_frames = {
        '00_start': 0,
        '01_approach_mid': max(0, pouring_start // 2),
        '02_pouring_start': pouring_start,
        '03_mixing_mid': pouring_start + sub_phases['mixing'].start + (sub_phases['mixing'].stop - sub_phases['mixing'].start) // 2,
        '04_drawing_start': pouring_start + sub_phases['drawing'].start,
        '05_drawing_mid': pouring_start + sub_phases['drawing'].start + (sub_phases['drawing'].stop - sub_phases['drawing'].start) // 2,
        '06_drawing_end': pouring_start + sub_phases['drawing'].stop,
        '07_finishing': pouring_start + sub_phases['finishing'].start + (sub_phases['finishing'].stop - sub_phases['finishing'].start) // 2,
        '08_pouring_end': pouring_end,
        '09_departure_mid': min(n - 1, pouring_end + (n - pouring_end) // 2),
        '10_end': n - 1,
    }

    # 打印关键帧对应的心形拉花步骤
    print(f"\n--- 拉花心形五步法 vs 数据集帧 ---")
    print(f"总帧数: {n} (20s)")
    print(f"倒奶段: 帧 {pouring_start}-{pouring_end} ({pouring_seg.duration_s:.1f}s)")

    step_mapping = [
        ('Step 2: 打圈方式轻倒 (融合)', f'帧 {pouring_start}-{pouring_start+sub_phases["mixing"].stop} '
         f'({(sub_phases["mixing"].stop - sub_phases["mixing"].start)*DT:.1f}s)'),
        ('Step 3: 固定中心点左右摆动 (成形)', f'帧 {pouring_start+sub_phases["drawing"].start}-{pouring_start+sub_phases["drawing"].stop} '
         f'({(sub_phases["drawing"].stop - sub_phases["drawing"].start)*DT:.1f}s)'),
        ('Step 4: 竖直抬起 (收尾开始)', f'帧 {pouring_start+sub_phases["finishing"].start}'),
        ('Step 5: 划穿收尾', f'帧 {pouring_start+sub_phases["finishing"].start}-{pouring_end}'),
    ]
    for step, frame_info in step_mapping:
        print(f"  {step}: {frame_info}")

    # 关节活动分析
    print(f"\n--- 各阶段关节活动 ---")
    for seg in segments:
        print(f"  {seg.name}: 时长={seg.duration_s:.1f}s, 均速={seg.mean_speed:.4f}, 最高速={seg.max_speed:.4f}")

    # 提取关键帧
    FRAMES_OUT.mkdir(exist_ok=True)
    video_paths = {}
    for cam in CAMERAS:
        cam_name = cam.split('.')[-1]
        vp = VIDEO_DIR / cam / f'episode_{episode_idx:06d}.mp4'
        if vp.exists():
            video_paths[cam_name] = str(vp)

    print(f"\n可用视频: {list(video_paths.keys())}")

    # 为每个摄像头提取关键帧网格
    for cam_name, vp in video_paths.items():
        # 挑12帧均匀采样
        indices = [key_frames[k] for k in sorted(key_frames.keys())]
        # 确保索引在有效范围内
        indices = [i for i in indices if 0 <= i < n]
        extract_keyframe_grid(
            vp, indices,
            str(FRAMES_OUT / f'ep{episode_idx:04d}_{cam_name}_keyframes.jpg'),
            cols=6
        )

    # 生成倒奶段帧动画帧 (密集采样)
    if pouring_seg.duration_s > 0:
        # 在倒奶段每0.2s取一帧
        n_pour_frames = min(30, pouring_end - pouring_start)
        pour_frames = np.linspace(pouring_start, pouring_end-1, n_pour_frames, dtype=int).tolist()

        for cam_name, vp in video_paths.items():
            extract_keyframe_grid(
                vp, pour_frames,
                str(FRAMES_OUT / f'ep{episode_idx:04d}_{cam_name}_pouring_sequence.jpg'),
                cols=10
            )

    return {
        'episode': episode_idx,
        'key_frames': {k: int(v) for k, v in key_frames.items()},
        'segments': [{'name': s.name, 'start': s.start_frame, 'end': s.end_frame,
                       'duration_s': s.duration_s} for s in segments],
        'sub_phases': {
            name: {'start': int(pouring_start + sl.start), 'end': int(pouring_start + sl.stop)}
            for name, sl in sub_phases.items()
        },
        'video_paths': video_paths,
    }


def create_comparison_figure(episode_idx: int):
    """
    创建视频帧 + 关节轨迹对齐的可视化图
    上半部分: 视频关键帧时间线
    下半部分: 关节轨迹 + 阶段标注
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    from scipy.ndimage import uniform_filter1d

    df = load_episode(episode_idx)
    states = np.vstack(df['observation.state'].values)
    activity = compute_activity_signal(df)
    segments = segment_full_trajectory(df)
    n = len(df)
    t = np.arange(n) * DT

    # 找倒奶段
    pouring_seg = next((s for s in segments if s.name == 'pouring'), None)
    if pouring_seg is None:
        print("未找到倒奶段")
        return

    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(4, 1, height_ratios=[2, 2, 2, 1.5], hspace=0.4)

    phase_colors = {'approach': 'blue', 'pouring': 'red', 'departure': 'green'}

    # ── Row 1: 右臂关节位置 + 阶段标注 ──
    ax1 = fig.add_subplot(gs[0])
    for j in range(7):
        ax1.plot(t, states[:, j], lw=0.6, alpha=0.7, label=f'J{j}')
    for seg in segments:
        ax1.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                     alpha=0.1, color=phase_colors.get(seg.name, 'gray'))
        mid = (seg.start_frame + seg.end_frame) / 2 * DT
        ax1.text(mid, ax1.get_ylim()[1] * 0.95, seg.name,
                 ha='center', fontweight='bold', fontsize=9,
                 color=phase_colors.get(seg.name, 'black'))
    ax1.set_ylabel('Right Arm Joint (rad)')
    ax1.set_title(f'Episode {episode_idx}: Right Arm (Pouring Arm) Joint Trajectories', fontweight='bold')
    ax1.legend(loc='right', ncol=4, fontsize=7)

    # ── Row 2: 左臂关节位置 ──
    ax2 = fig.add_subplot(gs[1])
    for j in range(7):
        ax2.plot(t, states[:, j + 7], lw=0.6, alpha=0.7, label=f'J{j+7}')
    for seg in segments:
        ax2.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                     alpha=0.1, color=phase_colors.get(seg.name, 'gray'))
    ax2.set_ylabel('Left Arm Joint (rad)')
    ax2.set_title('Left Arm (Cup Holding Arm) Joint Trajectories', fontweight='bold')
    ax2.legend(loc='right', ncol=4, fontsize=7)

    # ── Row 3: 活动量信号 + 心形步骤标注 ──
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, activity, 'k-', lw=0.8, alpha=0.7)
    threshold = np.median(activity) + 0.3 * np.std(activity)
    ax3.axhline(y=threshold, color='orange', ls='--', lw=1, alpha=0.5, label='Threshold')
    for seg in segments:
        ax3.axvspan(seg.start_frame * DT, seg.end_frame * DT,
                     alpha=0.1, color=phase_colors.get(seg.name, 'gray'))

    # 标注心形拉花步骤
    if pouring_seg:
        sub_phases = extract_pouring_phases(pouring_seg.states)
        ps = pouring_seg.start_frame
        step_annotations = [
            (ps * DT, 'Step2\n打圈'),  # But wait, this should be aligned with sub_phases
        ]
        # Step 2: 打圈融合
        s2_start = ps * DT
        s2_end = (ps + sub_phases['mixing'].stop) * DT
        s2_mid = (s2_start + s2_end) / 2
        # Step 3: 固定点摆动
        s3_start = (ps + sub_phases['drawing'].start) * DT
        s3_end = (ps + sub_phases['drawing'].stop) * DT
        s3_mid = (s3_start + s3_end) / 2
        # Step 4: 竖直抬起
        s4_start = (ps + sub_phases['finishing'].start) * DT
        s4_end = (ps + sub_phases['finishing'].stop) * DT

        for label, x in [('融合', s2_mid), ('摆动', s3_mid), ('抬起', s4_start), ('划穿', (s4_start + s4_end) / 2)]:
            ax3.axvline(x=x, color='red', ls=':', lw=0.8, alpha=0.6)
            ax3.text(x, ax3.get_ylim()[1] * 0.85, label, ha='center', fontsize=8, color='red')

    ax3.set_ylabel('Activity')
    ax3.set_title('Activity Signal + Heart Pattern Steps', fontweight='bold')

    # ── Row 4: 倒奶段放大 + 子阶段 ──
    ax4 = fig.add_subplot(gs[3])
    if pouring_seg:
        ps = pouring_seg.start_frame
        pe = pouring_seg.end_frame
        t_pour = t[ps:pe]
        right_vel = np.diff(states[ps:pe, :7], axis=0) / DT
        right_speed = np.linalg.norm(right_vel, axis=1)
        left_vel = np.diff(states[ps:pe, 7:], axis=0) / DT
        left_speed = np.linalg.norm(left_vel, axis=1)

        ax4.plot(t_pour[1:], right_speed, 'b-', lw=1, label='Right Arm (pour)', alpha=0.8)
        ax4.plot(t_pour[1:], left_speed, 'orange', lw=1, label='Left Arm (cup)', alpha=0.8)

        # 标注子阶段
        for name, sl in sub_phases.items():
            s0 = (ps + sl.start) * DT
            s1 = (ps + sl.stop) * DT
            ax4.axvspan(s0, s1, alpha=0.1,
                        color={'mixing': 'blue', 'drawing': 'red', 'finishing': 'green'}[name])
            mid = (s0 + s1) / 2
            ax4.text(mid, ax4.get_ylim()[1] * 0.9, name, ha='center', fontsize=9)

        ax4.set_xlim(t_pour[0], t_pour[-1])
        ax4.set_ylabel('EE Speed (rad/s)')
        ax4.set_title('Pouring Phase Detail: Speed Profile + Sub-phases', fontweight='bold')
        ax4.legend(fontsize=8)

    plt.tight_layout()
    save_path = str(FRAMES_OUT / f'ep{episode_idx:04d}_trajectory_steps.png')
    plt.savefig(save_path, bbox_inches='tight', dpi=150)
    print(f"轨迹步骤对比图: {save_path}")
    plt.close()


def sample_all_episodes(n_sample: int = 10):
    """从所有episode中采样分析"""
    episode_files = sorted(DATA_PATH.glob('episode_*.parquet'))
    indices = np.linspace(0, len(episode_files) - 1, n_sample, dtype=int)

    results = {}
    for idx in indices:
        ep_idx = int(episode_files[idx].stem.split('_')[1])
        result = analyze_episode_with_video(ep_idx)
        results[ep_idx] = result
        create_comparison_figure(ep_idx)

    # 汇总统计
    pour_durations = []
    for r in results.values():
        for seg in r['segments']:
            if seg['name'] == 'pouring':
                pour_durations.append(seg['duration_s'])

    print(f"\n采样 {n_sample} 个episodes 汇总:")
    print(f"  倒奶段平均时长: {np.mean(pour_durations):.2f}s ± {np.std(pour_durations):.2f}s")

    return results


def main():
    parser = argparse.ArgumentParser(description='视频+关节联合分析')
    parser.add_argument('--episode', type=int, default=None)
    parser.add_argument('--video', action='store_true', help='生成视频帧+轨迹对比图')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--sample', type=int, default=10, help='采样数量')
    args = parser.parse_args()

    FRAMES_OUT.mkdir(exist_ok=True)

    if args.episode is not None:
        analyze_episode_with_video(args.episode)
        if args.video:
            create_comparison_figure(args.episode)

    if args.all:
        sample_all_episodes(n_sample=args.sample)

    if not any([args.episode, args.all]):
        # 默认: 分析几个典型
        for ep in [20, 25, 30]:
            analyze_episode_with_video(ep)
            create_comparison_figure(ep)
        print("\n完成! 查看 analysis/key_frames/ 目录中的关键帧图片")


if __name__ == '__main__':
    main()
