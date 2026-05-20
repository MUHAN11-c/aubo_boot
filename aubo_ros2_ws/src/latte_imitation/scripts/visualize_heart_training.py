#!/usr/bin/env python3
"""
心形轨迹训练全程可视化 — 对比原始/ProMP/DMP/Ensemble 喵~

展示:
  1. 原始完整轨迹 + 提取的成形段
  2. ProMP 学习: 原始 vs 重构 (3D + XY/XZ/YZ 投影)
  3. DMP vs ProMP vs Ensemble 对比
  4. RMSE 条形图

使用:
  python3 scripts/visualize_heart_training.py
  python3 scripts/visualize_heart_training.py --episode 32
  python3 scripts/visualize_heart_training.py --episodes top5 --save /tmp/heart_viz.png
"""

import os, sys, argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无 GUI 后端
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

from latte_imitation.promp_learner import ProMP3D
from latte_imitation.dmp_learner import DMP3D
from latte_imitation.trajectory import CartesianTrajectory

HEART_DIR = os.path.join(PKG_DIR, "resource", "heart")
CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
DT = 0.05


def extract_forming_segment(positions):
    """提取成形段 (Z 低位最长连续段) 喵~"""
    z = positions[:, 2]
    win = min(21, len(z) // 10 * 2 + 1)
    if win < 3:
        return positions
    z_smooth = savgol_filter(z, win, 3)
    z_median = np.median(z_smooth)
    z_low = z_smooth < z_median
    changes = np.diff(z_low.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1
    if z_low[0]: starts = np.concatenate([[0], starts])
    if z_low[-1]: ends = np.concatenate([ends, [len(z_low)]])
    if len(starts) == 0:
        return positions, 0, len(positions)
    longest = np.argmax(ends - starts)
    fs, fe = starts[longest], ends[longest]
    if fe - fs < 40:
        fs, fe = len(positions)//4, 3*len(positions)//4
    return positions[fs:fe], fs, fe


def visualize(episodes, save_path=None):
    """生成全程可视化图喵~"""
    episodes_list = episodes if isinstance(episodes, list) else [episodes]

    n_eps = len(episodes_list)
    # 布局: 每 episode 2 行 (3D对比 + 误差分析)
    fig = plt.figure(figsize=(6 * n_eps + 2, 14))
    gs = GridSpec(4, max(2, n_eps), figure=fig,
                  hspace=0.35, wspace=0.3)

    for col, ep in enumerate(episodes_list):
        # ── 加载原始数据 ──
        ep_path = os.path.join(CARTESIAN_DIR, f"episode_{ep:06d}.npz")
        if not os.path.exists(ep_path):
            print(f"  [跳过] episode {ep} 不存在")
            continue

        cart = CartesianTrajectory.load(ep_path)
        pos_full = cart.positions
        T_full = len(pos_full)

        # 提取成形段
        form_raw, fs, fe = extract_forming_segment(pos_full)
        T_form = len(form_raw)

        # ── 训练 ProMP ──
        promp = ProMP3D(n_basis=30, sigma=0.03)
        promp.learn(form_raw)
        recon_promp = promp.generate(T=T_form)
        rmse_promp = np.sqrt(np.mean(np.sum((form_raw - recon_promp)**2, axis=1))) * 1000

        # ── 训练 DMP ──
        dmp = DMP3D(n_basis=25, dt=0.01)
        dmp.learn(form_raw, tau=1.0)
        recon_dmp = dmp.generate(tau=1.0, T=T_form)
        rmse_dmp = np.sqrt(np.mean(np.sum((form_raw - recon_dmp)**2, axis=1))) * 1000

        # ── Row 1: 完整轨迹 + 成形段标注 ──
        ax_full = fig.add_subplot(gs[0, col % 2] if n_eps > 1 else gs[0, :])
        t_full = np.arange(T_full) * DT
        ax_full.plot(t_full, pos_full[:, 2], 'gray', lw=0.8, alpha=0.5, label='完整 Z')
        ax_full.plot(t_full, pos_full[:, 0], 'blue', lw=0.8, alpha=0.3, label='完整 X')
        ax_full.axvspan(fs * DT, fe * DT, alpha=0.15, color='red', label=f'成形段 [{fs},{fe}]')
        ax_full.set_title(f'Episode {ep} — 完整轨迹 (20s)\n成形段={T_form}帧 ({T_form*DT:.1f}s)')
        ax_full.set_xlabel('时间 (s)')
        ax_full.set_ylabel('位置 (m)')
        ax_full.legend(fontsize=7)
        ax_full.grid(True, alpha=0.3)

        # ── Row 2: 3D 对比 — 原始 vs ProMP vs DMP ──
        ax_3d = fig.add_subplot(gs[1, col % 2] if n_eps > 1 else gs[1, :], projection='3d')
        ax_3d.plot(form_raw[:, 0], form_raw[:, 1], form_raw[:, 2],
                   'k-', lw=1.5, label=f'原始 (路径={np.sum(np.linalg.norm(np.diff(form_raw,axis=0),axis=1))*1000:.0f}mm)')
        ax_3d.plot(recon_promp[:, 0], recon_promp[:, 1], recon_promp[:, 2],
                   'g-', lw=1.2, label=f'ProMP (RMSE={rmse_promp:.1f}mm)')
        ax_3d.plot(recon_dmp[:, 0], recon_dmp[:, 1], recon_dmp[:, 2],
                   'r--', lw=1.0, alpha=0.7, label=f'DMP (RMSE={rmse_dmp:.1f}mm)')
        ax_3d.set_title(f'心形成形段 3D 对比')
        ax_3d.set_xlabel('X (m)'); ax_3d.set_ylabel('Y (m)'); ax_3d.set_zlabel('Z (m)')
        ax_3d.legend(fontsize=6, loc='upper left')

        # ── Row 3: XY/XZ/YZ 投影对比 ──
        for i, (dim1, dim2, label1, label2) in enumerate([(0, 1, 'X', 'Y'), (0, 2, 'X', 'Z'), (1, 2, 'Y', 'Z')]):
            ax_proj = fig.add_subplot(gs[2, i] if n_eps == 1 else gs[2, col])
            ax_proj.plot(form_raw[:, dim1], form_raw[:, dim2], 'k-', lw=1.0, alpha=0.7, label='原始')
            ax_proj.plot(recon_promp[:, dim1], recon_promp[:, dim2], 'g-', lw=1.0, alpha=0.8, label='ProMP')
            ax_proj.plot(recon_dmp[:, dim1], recon_dmp[:, dim2], 'r--', lw=0.8, alpha=0.5, label='DMP')
            ax_proj.set_xlabel(f'{label1} (m)'); ax_proj.set_ylabel(f'{label2} (m)')
            ax_proj.set_title(f'{label1}{label2} 投影 — ep{ep}')
            ax_proj.legend(fontsize=6)
            ax_proj.grid(True, alpha=0.3)
            ax_proj.set_aspect('equal')

        # Print stats
        print(f'\nep{ep}:')
        print(f'  ProMP RMSE: {rmse_promp:.1f}mm  {"✓ 完美" if rmse_promp < 5 else ""}')
        print(f'  DMP   RMSE: {rmse_dmp:.1f}mm')
        print(f'  成形段: {T_form}帧, 路径长={np.sum(np.linalg.norm(np.diff(form_raw,axis=0),axis=1))*1000:.0f}mm')

    # ── Row 4: RMSE 对比条形图 ──
    ax_bar = fig.add_subplot(gs[3, :])
    eps_labels = [f'ep{ep}' for ep in episodes_list]
    promp_rmses = []
    dmp_rmses = []

    for ep in episodes_list:
        ep_path = os.path.join(CARTESIAN_DIR, f"episode_{ep:06d}.npz")
        cart = CartesianTrajectory.load(ep_path)
        form_raw, _, _ = extract_forming_segment(cart.positions)

        promp = ProMP3D(n_basis=30, sigma=0.03)
        promp.learn(form_raw)
        recon_p = promp.generate(T=len(form_raw))
        promp_rmses.append(np.sqrt(np.mean(np.sum((form_raw-recon_p)**2, axis=1)))*1000)

        dmp = DMP3D(n_basis=25, dt=0.01)
        dmp.learn(form_raw, tau=1.0)
        recon_d = dmp.generate(tau=1.0, T=len(form_raw))
        dmp_rmses.append(np.sqrt(np.mean(np.sum((form_raw-recon_d)**2, axis=1)))*1000)

    x_pos = np.arange(len(eps_labels))
    w = 0.35
    bars1 = ax_bar.bar(x_pos - w/2, promp_rmses, w, label='ProMP', color='green', alpha=0.7)
    bars2 = ax_bar.bar(x_pos + w/2, dmp_rmses, w, label='DMP', color='red', alpha=0.7)

    # 标注数值
    for bar, val in zip(bars1, promp_rmses):
        ax_bar.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 1,
                    f'{val:.1f}', ha='center', va='bottom', fontsize=8, color='green')
    for bar, val in zip(bars2, dmp_rmses):
        ax_bar.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 1,
                    f'{val:.1f}', ha='center', va='bottom', fontsize=8, color='red')

    ax_bar.set_xlabel('Episode')
    ax_bar.set_ylabel('RMSE (mm)')
    ax_bar.set_title('ProMP vs DMP — 心形成形段重构误差')
    ax_bar.set_xticks(x_pos)
    ax_bar.set_xticklabels(eps_labels)
    ax_bar.legend()
    ax_bar.grid(True, alpha=0.3, axis='y')
    ax_bar.axhline(y=5, color='green', ls='--', alpha=0.5, label='完美阈值 (5mm)')
    ax_bar.axhline(y=30, color='orange', ls='--', alpha=0.5, label='优秀阈值 (30mm)')

    fig.suptitle('心形拉花轨迹训练 — ProMP vs DMP 全程对比', fontsize=14, fontweight='bold', y=0.99)

    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'\n可视化已保存: {save_path}')
    else:
        out_path = os.path.join(HEART_DIR, "training_visualization.png")
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        print(f'\n可视化已保存: {out_path}')

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="心形训练全程可视化")
    parser.add_argument("--episodes", type=str, default="32,25,23",
                        help="Episode 列表 (逗号分隔, 默认: 32,25,23)")
    parser.add_argument("--save", type=str, default=None,
                        help="保存路径 (默认: resource/heart/training_visualization.png)")
    args = parser.parse_args()

    episodes = [int(x.strip()) for x in args.episodes.split(",")]
    print(f"可视化 episodes: {episodes}")
    visualize(episodes, args.save)


if __name__ == "__main__":
    main()
