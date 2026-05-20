#!/usr/bin/env python3
"""
心形 DMP 训练 — 从笛卡尔示教轨迹学习可泛化心形基元喵~

支持:
  - 单条示教: --episode 32
  - 多条 ensemble (中位数聚合): --episodes 32,25,23 --ensemble
  - 全部 40 条聚合: --episodes all --ensemble

输出:
  - resource/heart/heart_dmp_model.npz  (DMP 权重)
  - 终端验证报告 (训练 RMSE / 泛化测试)

使用:
  python3 scripts/train_heart_dmp.py                          # 默认 ep32, 单条
  python3 scripts/train_heart_dmp.py --episode 32             # 指定单条
  python3 scripts/train_heart_dmp.py --episodes top5 --ensemble  # TOP5 聚合
  python3 scripts/train_heart_dmp.py --episodes all --ensemble   # 全 40 条聚合
"""

import os, sys, argparse
import numpy as np
from scipy.signal import savgol_filter

# 确保能找到 latte_imitation 模块
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PKG_DIR)

from latte_imitation.dmp_learner import DMP3D
from latte_imitation.promp_learner import ProMP3D
from latte_imitation.trajectory import CartesianTrajectory

CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
HEART_DIR = os.path.join(PKG_DIR, "resource", "heart")
MODEL_OUTPUT = os.path.join(HEART_DIR, "heart_dmp_model.npz")

TOP5_EPISODES = [32, 25, 23, 6, 8]


def load_trajectories(episode_spec, source="cartesian"):
    """加载轨迹 — 单条或多条喵~"""
    if source == "heart":
        data_dir = HEART_DIR
        prefix, suffix = "heart_raw_ep", ".npz"
    else:
        data_dir = CARTESIAN_DIR
        prefix, suffix = "episode_", ".npz"

    episodes = _parse_episode_spec(episode_spec, data_dir, prefix, suffix)

    trajs = {}
    for ep in episodes:
        if source == "heart":
            fname = f"{prefix}{ep:02d}{suffix}"
        else:
            fname = f"{prefix}{ep:06d}{suffix}"
        path = os.path.join(data_dir, fname)
        if not os.path.exists(path):
            print(f"  [跳过] {fname} 不存在")
            continue
        data = np.load(path)
        positions = data["positions"]  # (T, 3)
        trajs[ep] = positions

    return trajs


def _parse_episode_spec(spec, data_dir, prefix, suffix):
    """解析 episode 选择表达式喵~"""
    if spec == "all":
        import glob
        episodes = []
        pattern = os.path.join(data_dir, f"{prefix}*{suffix}")
        for f in sorted(glob.glob(pattern)):
            basename = os.path.basename(f)
            try:
                ep_str = basename.replace(prefix, "").replace(suffix, "")
                if ep_str.startswith("0") and len(ep_str) <= 2:
                    episodes.append(int(ep_str))
                else:
                    episodes.append(int(ep_str))
            except ValueError:
                continue
        return sorted(episodes)
    elif spec == "top5":
        return sorted(TOP5_EPISODES)
    elif spec.startswith("top"):
        try:
            n = int(spec[3:])
        except ValueError:
            n = 5
        return sorted(TOP5_EPISODES[:n])
    elif "," in spec:
        return sorted([int(x.strip()) for x in spec.split(",")])
    else:
        try:
            return [int(spec)]
        except ValueError:
            return sorted(set(TOP5_EPISODES + [int(spec)])) if spec else TOP5_EPISODES


def ensemble_median(trajs, target_frames=200):
    """多条轨迹时间对齐 → 中位数聚合 → SG 平滑喵~"""
    from scipy.interpolate import interp1d

    if len(trajs) == 1:
        ep_idx, pos = next(iter(trajs.items()))
        return pos, {"method": "single", "source_episode": ep_idx}

    resampled = []
    for ep_idx, pos in trajs.items():
        T = len(pos)
        t_orig = np.linspace(0, 1, T)
        t_new = np.linspace(0, 1, target_frames)
        pos_new = np.column_stack([
            interp1d(t_orig, pos[:, d], kind='linear')(t_new)
            for d in range(3)
        ])
        resampled.append(pos_new)

    stack = np.stack(resampled, axis=0)
    aggregated = np.median(stack, axis=0)

    # SG 平滑
    win = min(11, target_frames // 10 * 2 + 1)
    if win >= 3:
        aggregated = savgol_filter(aggregated, win, 3, axis=0)

    meta = {
        "method": "median_ensemble",
        "n_episodes": len(trajs),
        "source_episodes": sorted(trajs.keys()),
    }
    return aggregated, meta


def evaluate_rmse(original, reconstructed):
    """计算原始轨迹与重构轨迹之间的 RMSE (mm) 喵~"""
    T_orig = len(original)
    T_recon = len(reconstructed)

    # 时间对齐
    t_orig = np.linspace(0, 1, T_orig)
    t_recon = np.linspace(0, 1, T_recon)
    from scipy.interpolate import interp1d
    recon_interp = np.column_stack([
        interp1d(t_recon, reconstructed[:, d], kind='linear')(t_orig)
        for d in range(3)
    ])

    diff = original - recon_interp
    rmse_per_dim = np.sqrt(np.mean(diff ** 2, axis=0)) * 1000  # mm
    rmse_total = np.sqrt(np.mean(np.sum(diff ** 2, axis=1))) * 1000  # mm
    return rmse_total, rmse_per_dim


def extract_forming_segment(positions, dt=0.05, detrend=True):
    """从完整轨迹中提取纯成形段, 可选线性去趋势喵~

    DMP 学习去趋势后的纯心形形状 (无缓慢Y推拉), 生成时再加回趋势。
    成形段 = Z 在低位最长连续段 + 线性去趋势。
    """
    z = positions[:, 2]
    win = min(21, len(z) // 10 * 2 + 1)
    if win < 3:
        return positions, np.zeros(3), np.zeros(3)

    z_smooth = savgol_filter(z, win, 3)
    z_median = np.median(z_smooth)
    z_low = z_smooth < z_median

    changes = np.diff(z_low.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1
    if z_low[0]: starts = np.concatenate([[0], starts])
    if z_low[-1]: ends = np.concatenate([ends, [len(z_low)]])

    if len(starts) == 0:
        fs, fe = len(positions)//4, 3*len(positions)//4
    else:
        longest = np.argmax(ends - starts)
        fs, fe = starts[longest], ends[longest]
        if fe - fs < 40:
            fs = max(0, len(positions) // 4)
            fe = min(len(positions), 3 * len(positions) // 4)

    form = positions[fs:fe].copy()

    if detrend:
        # 线性去趋势, 保留趋势用于生成时加回
        from scipy.signal import detrend
        trend_start = form[0].copy()
        trend_end = form[-1].copy()
        form_detrend = detrend(form, axis=0, type='linear')
        return form_detrend, trend_start, trend_end
    else:
        return form, form[0].copy(), form[-1].copy()


def main():
    parser = argparse.ArgumentParser(
        description="心形 DMP 训练 — 从成形段学习可泛化心形基元",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                         # 默认 ep32 单条
  %(prog)s --episodes top5 --ensemble              # TOP5 聚合
  %(prog)s --episodes all --ensemble               # 40 条聚合
  %(prog)s --episodes 32 --test-generalize         # 泛化测试
        """,
    )
    parser.add_argument("--episodes", type=str, default="32",
                        help="Episode 选择: all | top5 | 32,25,23 | 32 (默认: 32)")
    parser.add_argument("--source", type=str, default="cartesian",
                        choices=["cartesian", "heart"],
                        help="数据源 (默认: cartesian)")
    parser.add_argument("--ensemble", action="store_true",
                        help="多条中位数聚合后训练")
    parser.add_argument("--method", type=str, default="promp",
                        choices=["promp", "dmp"],
                        help="学习方法: promp (时序基函数, 推荐) | dmp (动态系统)")
    parser.add_argument("--n-basis", type=int, default=30,
                        help="基函数数量 (promp默认30, dmp默认25)")
    parser.add_argument("--target-frames", type=int, default=120,
                        help="Ensemble 聚合目标帧数 (默认: 120)")
    parser.add_argument("--tau", type=float, default=1.0,
                        help="时间缩放因子 (默认: 1.0)")
    parser.add_argument("--test-generalize", action="store_true",
                        help="测试泛化: 改变终点位置, 验证形状保持")
    parser.add_argument("--output", type=str, default=MODEL_OUTPUT,
                        help=f"模型输出路径 (默认: {MODEL_OUTPUT})")
    args = parser.parse_args()

    # ── 加载轨迹 ──
    trajs = load_trajectories(args.episodes, args.source)
    if not trajs:
        print("[错误] 未找到任何轨迹数据喵~")
        sys.exit(1)

    print(f"加载了 {len(trajs)} 条轨迹: {sorted(trajs.keys())}")

    # ── 提取成形段 (去趋势) ──
    forming_segments = {}
    forming_trends = {}  # 保存趋势用于生成时加回
    for ep_idx, pos in trajs.items():
        seg, trend_start, trend_end = extract_forming_segment(pos, detrend=True)
        forming_segments[ep_idx] = seg
        forming_trends[ep_idx] = (trend_start, trend_end)
        print(f"  ep{ep_idx}: {len(pos)}→{len(seg)} 帧成形段(去趋势) "
              f"({len(seg)*0.05:.1f}s), X振幅={seg[:,0].std()*1000:.1f}mm")

    # ── Ensemble 聚合 (可选) ──
    if args.ensemble and len(forming_segments) > 1:
        print(f"\n中位数聚合 {len(forming_segments)} 条成形段 → {args.target_frames} 帧...")
        trajectory, ensemble_meta = ensemble_median(forming_segments, args.target_frames)
        # 聚合趋势: 用中位数
        all_starts = np.array([t[0] for t in forming_trends.values()])
        all_ends = np.array([t[1] for t in forming_trends.values()])
        trend_start = np.median(all_starts, axis=0)
        trend_end = np.median(all_ends, axis=0)
        print(f"  源 episodes: {ensemble_meta['source_episodes']}")
    else:
        ep_idx, trajectory = next(iter(forming_segments.items()))
        ensemble_meta = {"method": "single", "source_episode": ep_idx}
        trend_start, trend_end = forming_trends[ep_idx]

    path_len = float(np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1)))
    print(f"  成形段(去趋势): {len(trajectory)} 帧, 路径长 {path_len*1000:.1f}mm")
    print(f"  趋势: start={trend_start}, end={trend_end}")

    # ── 训练 (ProMP 或 DMP) ──
    if args.method == "promp":
        print(f"\n训练 ProMP (n_basis={args.n_basis}, 时序基函数)...")
        model = ProMP3D(n_basis=args.n_basis, sigma=0.03)
        if args.ensemble and len(forming_segments) > 1:
            # 多条 → learn_multiple
            all_segs = []
            for ep_idx, seg in forming_segments.items():
                all_segs.append(seg)
            model.learn_multiple(all_segs)
        else:
            model.learn(trajectory)

        T_check = len(trajectory) if not args.ensemble else args.target_frames
        recon = model.generate(T=T_check)
        rmse_total, rmse_per_dim = evaluate_rmse(trajectory, recon)

        print(f"\n{'='*60}")
        print(f"  ProMP 训练结果")
        print(f"{'='*60}")
        print(f"  方法: {'learn_multiple' if args.ensemble and len(forming_segments) > 1 else 'learn'}")

    else:  # dmp
        print(f"\n训练 DMP (n_basis={args.n_basis}, tau={args.tau})...")
        model = DMP3D(n_basis=args.n_basis, dt=0.01)
        model.learn(trajectory, tau=args.tau)

        recon = model.generate(tau=args.tau, T=len(trajectory))
        rmse_total, rmse_per_dim = evaluate_rmse(trajectory, recon)

        print(f"\n{'='*60}")
        print(f"  DMP 训练结果")
        print(f"{'='*60}")

    print(f"  成形段: {len(trajectory)} 帧, 路径长 {path_len:.3f}m")
    print(f"  起点=({trajectory[0,0]:.3f},{trajectory[0,1]:.3f},{trajectory[0,2]:.3f})")
    print(f"  终点=({trajectory[-1,0]:.3f},{trajectory[-1,1]:.3f},{trajectory[-1,2]:.3f})")
    print(f"  重构 RMSE: {rmse_total:.2f} mm")
    print(f"     X: {rmse_per_dim[0]:.2f} mm")
    print(f"     Y: {rmse_per_dim[1]:.2f} mm")
    print(f"     Z: {rmse_per_dim[2]:.2f} mm")

    if rmse_total < 5:
        status = "✓ 完美"
    elif rmse_total < 30:
        status = "✓ 优秀"
    elif rmse_total < 50:
        status = "✓ 良好"
    else:
        status = "⚠ 一般"
    print(f"  评估: {status} (<5mm=完美, <30mm=优秀, <50mm=良好) 喵~")

    # ── 泛化测试 ──
    if args.test_generalize and args.method == "dmp":
        print(f"\n{'='*60}")
        print(f"  泛化测试 — 改变终点")
        print(f"{'='*60}")
        for offset in [(0.02, 0, 0), (0, 0.02, 0), (0, 0, 0.01)]:
            new_goal = model.goal + np.array(offset)
            gen = model.generate(goal=new_goal, tau=args.tau, T=len(trajectory))
            gen_path = float(np.sum(np.linalg.norm(np.diff(gen, axis=0), axis=1)))
            print(f"  goal_offset=({offset[0]:+.2f},{offset[1]:+.2f},{offset[2]:+.2f}): "
                  f"path={gen_path:.3f}m")

    if args.test_generalize and args.method == "promp":
        print(f"\n{'='*60}")
        print(f"  ProMP 泛化测试 — 改变起点/终点")
        print(f"{'='*60}")
        new_goal = trajectory[-1] + np.array([0.02, 0.02, 0.0])
        gen = model.generate(T=len(trajectory), goal=new_goal)
        gen_path = float(np.sum(np.linalg.norm(np.diff(gen, axis=0), axis=1)))
        print(f"  goal_offset=(+0.02,+0.02,0): path={gen_path:.3f}m")

    # ── 保存模型 ──
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    model.save(args.output)
    # 追加趋势信息和方法标记
    existing = dict(np.load(args.output, allow_pickle=True))
    existing["trend_start"] = trend_start
    existing["trend_end"] = trend_end
    existing["method"] = args.method
    # np.savez_compressed 自动加 .npz 后缀, 所以去掉原来的
    tmp_base = args.output.replace(".npz", "_tmp")
    np.savez_compressed(tmp_base, **existing)
    os.replace(tmp_base + ".npz", args.output)

    print(f"\n模型已保存: {args.output} (方法={args.method})")
    trend_delta = trend_end - trend_start
    print(f"  趋势 delta (mm): X={trend_delta[0]*1000:.1f}, Y={trend_delta[1]*1000:.1f}, Z={trend_delta[2]*1000:.1f}")


if __name__ == "__main__":
    main()
