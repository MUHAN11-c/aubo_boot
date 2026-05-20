#!/usr/bin/env python3
"""
心形拉花参数标定 — 从笛卡尔轨迹示教数据提取统计参数喵~

数据源:
  - resource/cartesian/right/episode_*.npz  (40 条, 全部录制)
  - resource/heart/heart_raw_ep*.npz        (5 条精选心形)

提取:
  1. 成形段微摆频率分布 (速度域 FFT)
  2. 成形段微摆振幅 (1-8Hz 带通滤波)
  3. Z 高度三阶段分割 (高位融合→低位成形→高位收尾)
  4. 阶段时间比例 (mix / sway / lift / draw-through)
  5. Y 轴推拉幅度

使用:
  python3 scripts/calibrate_heart_params.py                           # 全部 40 条
  python3 scripts/calibrate_heart_params.py --episodes 32             # 单条 ep32
  python3 scripts/calibrate_heart_params.py --episodes 32,25,23       # 多条指定
  python3 scripts/calibrate_heart_params.py --episodes top5           # TOP5 精选
  python3 scripts/calibrate_heart_params.py --source heart            # 用心形精选源
  python3 scripts/calibrate_heart_params.py --output /tmp/out.yaml    # 指定输出
"""

import os, sys, argparse, json
import numpy as np
from scipy.fft import fft, fftfreq
from scipy.signal import butter, filtfilt, savgol_filter, find_peaks
from collections import OrderedDict


# ═══════════════════════════════════════════════════════════════════
# 配置
# ═══════════════════════════════════════════════════════════════════

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
CARTESIAN_DIR = os.path.join(PKG_DIR, "resource", "cartesian", "right")
HEART_DIR = os.path.join(PKG_DIR, "resource", "heart")
CONFIG_DIR = os.path.join(PKG_DIR, "config")

DT = 0.05  # 20fps
F_NYQUIST = 0.5 / DT  # 10 Hz

# generator.py heart() 的默认值 (PourConfig + generator 缩放因子)
GEN_DEFAULTS = {
    "wiggle_amplitude": 0.006,
    "wiggle_frequency": 5.0,
    "wiggle_scale_factor": 0.3,      # heart() L50: amplitude * 0.3 → 有效 1.8mm
    "mix_height_offset": 0.076,       # 7.6cm
    "draw_height_offset": 0.006,      # 0.6cm
    "finish_height_offset": 0.076,    # 7.6cm
    "mix_ratio": 0.25,
    "sway_ratio": 0.45,               # 0.70 - 0.25
    "lift_ratio": 0.15,               # 0.85 - 0.70
}

# TOP5 心形 episode 编号 (按质量排名, 来自 resource/heart/README.md)
TOP5_EPISODES = [32, 25, 23, 6, 8]


# ═══════════════════════════════════════════════════════════════════
# 数据加载
# ═══════════════════════════════════════════════════════════════════

def load_episodes(episode_spec, source="cartesian"):
    """加载指定 episodes 的轨迹数据喵~

    Args:
        episode_spec: "all" | "top5" | "topN" | "32,25,23" | "32" (单条)
        source: "cartesian" (全部 40 条) | "heart" (精选心形)

    Returns:
        OrderedDict[int, np.ndarray]: {episode_idx: positions_array (T,3)}
    """
    if source == "heart":
        data_dir = HEART_DIR
        prefix = "heart_raw_ep"
        suffix = ".npz"
    else:
        data_dir = CARTESIAN_DIR
        prefix = "episode_"
        suffix = ".npz"

    # 解析 episode 编号列表
    episodes = _parse_episode_spec(episode_spec, data_dir, prefix, suffix)

    result = OrderedDict()
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
        positions = data["positions"]  # (T, 3) float32
        result[ep] = positions

    return result


def _parse_episode_spec(spec, data_dir, prefix, suffix):
    """解析 episode 选择表达式喵~"""
    if spec == "all":
        episodes = []
        import glob
        pattern = os.path.join(data_dir, f"{prefix}*{suffix}")
        for f in sorted(glob.glob(pattern)):
            basename = os.path.basename(f)
            try:
                ep_str = basename.replace(prefix, "").replace(suffix, "")
                # heart 格式: "ep06" → 6, cartesian 格式: "episode_000006" → 6
                # 兼容两种格式
                if ep_str.startswith("0") and len(ep_str) <= 2:
                    episodes.append(int(ep_str))
                else:
                    # cartesian 格式: "000000", "000006"
                    episodes.append(int(ep_str))
            except ValueError:
                continue
        return sorted(episodes)

    elif spec == "top5":
        return sorted(TOP5_EPISODES)

    elif spec.startswith("top") or spec.startswith("TOP"):
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
            print(f"  无法解析 --episodes={spec}, 回退到 all")
            return _parse_episode_spec("all", data_dir, prefix, suffix)


# ═══════════════════════════════════════════════════════════════════
# 信号分析工具
# ═══════════════════════════════════════════════════════════════════

def find_forming_phase(z, dt=0.05):
    """通过 Z 高度三段分析找成形段 (Z 低位最长的连续段) 喵~"""
    win = min(21, len(z) // 10 * 2 + 1)
    if win < 3:
        return len(z) // 4, 3 * len(z) // 4
    z_smooth = savgol_filter(z, win, 3)

    z_median = np.median(z_smooth)
    z_low = (z_smooth > z_smooth.min() + 0.02) & (z_smooth < z_median + 0.03)

    changes = np.diff(z_low.astype(int))
    starts = np.where(changes == 1)[0] + 1
    ends = np.where(changes == -1)[0] + 1
    if z_low[0]:
        starts = np.concatenate([[0], starts])
    if z_low[-1]:
        ends = np.concatenate([ends, [len(z_low)]])

    if len(starts) == 0:
        return len(z) // 4, 3 * len(z) // 4

    longest = np.argmax(ends - starts)
    fs, fe = starts[longest], ends[longest]

    if fe - fs < 30:
        fs = max(30, len(z) // 4)
        fe = min(len(z) - 30, 3 * len(z) // 4)

    return fs, fe


def compute_wiggle_frequencies(x_vel, dt=0.05, min_freq=0.5, max_freq=9.0):
    """从 X 速度信号提取微摆主频 (高通滤波 + FFT) 喵~"""
    n = len(x_vel)
    if n < 30:
        return []

    try:
        b, a = butter(4, 0.8 / F_NYQUIST, btype='high')
        x_vel_hp = filtfilt(b, a, x_vel)
    except ValueError:
        x_vel_hp = x_vel - x_vel.mean()

    yf = fft(x_vel_hp)
    xf = fftfreq(n, dt)
    pos_mask = (xf > min_freq) & (xf < max_freq)

    if pos_mask.sum() < 3:
        return []

    powers = np.abs(yf[pos_mask])
    freqs = xf[pos_mask]
    threshold = powers.mean() + 1.5 * powers.std()
    peaks, _ = find_peaks(powers, height=threshold)

    if len(peaks) == 0:
        top_idx = np.argsort(powers)[-3:]
        return sorted(freqs[top_idx].tolist())

    return sorted(freqs[peaks].tolist())


def compute_wiggle_amplitude(x_vel, dt=0.05, low_freq=0.8, high_freq=8.0):
    """从 X 速度信号提取微摆振幅 (带通滤波后积分) 喵~"""
    n = len(x_vel)
    if n < 30:
        return 0.0, 0.0

    try:
        b, a = butter(4, [low_freq / F_NYQUIST, high_freq / F_NYQUIST], btype='band')
        x_vel_bp = filtfilt(b, a, x_vel)
    except ValueError:
        x_vel_bp = x_vel

    x_disp = np.cumsum(x_vel_bp) * dt
    x_disp -= x_disp.mean()

    return float(np.abs(x_disp).std()), float(x_disp.max() - x_disp.min())


def segment_z_phases(z, dt=0.05):
    """通过 Z 速度变化点分割三段: 高位融合→低位成形→高位收尾 喵~"""
    win_z = min(21, len(z) // 10 * 2 + 1)
    if win_z < 3:
        n = len(z)
        return {"descend_start": n//4, "z_min_idx": n//2, "rise_start": 3*n//4,
                "mix_z": float(z[:n//4].max()), "draw_z": float(z[n//4:3*n//4].mean()),
                "finish_z": float(z[3*n//4:].max())}

    z_smooth = savgol_filter(z, win_z, 3)
    z_vel = np.diff(z_smooth) / dt
    win_v = min(15, len(z_vel) // 10 * 2 + 1)
    if win_v < 3:
        win_v = 3
    z_vel_smooth = savgol_filter(z_vel, win_v, 3)

    n = len(z)

    # Z 最低点 (排除首尾 20%)
    z_min_idx = np.argmin(z_smooth[n//5:-n//5]) + n//5

    # 找下降开始: 从最低点往前找 Z 速度接近零的点
    descend_start = n // 5
    for i in range(z_min_idx, max(n//5, z_min_idx - 100), -1):
        if z_vel_smooth[i] > -0.001:
            descend_start = i
            break

    # 找上升开始: 从最低点往后找 Z 速度转正且高度明显上升的点
    rise_start = z_min_idx
    for i in range(z_min_idx, min(n - 20, z_min_idx + 100)):
        if z_vel_smooth[i] > 0.002 and z_smooth[i] > z_smooth[z_min_idx] + 0.01:
            rise_start = i
            break

    draw_slice = z_smooth[descend_start:rise_start]
    draw_z_val = float(draw_slice.mean()) if len(draw_slice) > 0 else float(z_smooth.mean())

    return {
        "descend_start": descend_start,
        "z_min_idx": z_min_idx,
        "rise_start": rise_start,
        "mix_z": float(z_smooth[:descend_start].max()) if descend_start > 0 else float(z_smooth.max()),
        "draw_z": draw_z_val,
        "finish_z": float(z_smooth[rise_start:].max()),
    }


# ═══════════════════════════════════════════════════════════════════
# 单条 episode 分析
# ═══════════════════════════════════════════════════════════════════

def analyze_one(ep_idx, positions):
    """分析单条轨迹, 返回指标 dict 喵~"""
    T = len(positions)
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]

    vel = np.diff(positions, axis=0) / DT
    x_vel, y_vel, z_vel = vel[:, 0], vel[:, 1], vel[:, 2]

    # 成形段
    fs, fe = find_forming_phase(z)
    if fe <= fs + 10:
        fs, fe = T // 4, 3 * T // 4

    # 微摆频率与振幅
    x_vel_form = x_vel[fs:fe]
    freqs = compute_wiggle_frequencies(x_vel_form)
    amp_std, amp_p2p = compute_wiggle_amplitude(x_vel_form)

    # Z 三段
    z_phases = segment_z_phases(z)

    # Y 推拉
    y_range = float(y[fs:fe].max() - y[fs:fe].min())

    # 路径长度
    path_len = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))

    return {
        "episode": ep_idx,
        "frames": T,
        "path_length": path_len,
        "forming_segment": (fs, fe),
        "forming_duration_s": (fe - fs) * DT,
        "z_mean": float(z.mean()),
        "z_min": float(z.min()),
        "z_max": float(z.max()),
        "z_phases": z_phases,
        "wiggle_frequencies": freqs,
        "wiggle_amplitude_std_mm": round(amp_std * 1000, 3),
        "wiggle_amplitude_p2p_mm": round(amp_p2p * 1000, 3),
        "x_position_std_mm": round(x[fs:fe].std() * 1000, 1),
        "y_range_mm": round(y_range * 1000, 1),
        "draw_z_abs": float(z[fs:fe].mean()),
        "mix_z_abs": float(z[:fs].max()) if fs > 0 else float(z.max()),
        "finish_z_abs": float(z[fe:].max()) if fe < T else float(z.max()),
        # 阶段时间比例
        "_mix_pct": z_phases["descend_start"] / T * 100,
        "_sway_pct": (z_phases["rise_start"] - z_phases["descend_start"]) / T * 100,
        "_lift_pct": (T - z_phases["rise_start"]) / T * 100,
    }


# ═══════════════════════════════════════════════════════════════════
# 多条 ensemble
# ═══════════════════════════════════════════════════════════════════

def ensemble_trajectory(episodes_dict, target_frames=400):
    """多条轨迹时间对齐 → 中位数聚合 → SG 平滑, 返回聚合轨迹 (T,3) 喵~"""
    from scipy.interpolate import interp1d

    n_eps = len(episodes_dict)
    if n_eps == 1:
        ep_idx, pos = next(iter(episodes_dict.items()))
        return pos, {"method": "single", "source_episode": ep_idx}

    # 时间归一化 + 插值到统一帧数
    resampled = []
    for ep_idx, pos in episodes_dict.items():
        T = len(pos)
        t_orig = np.linspace(0, 1, T)
        t_new = np.linspace(0, 1, target_frames)
        f_x = interp1d(t_orig, pos[:, 0], kind='linear')
        f_y = interp1d(t_orig, pos[:, 1], kind='linear')
        f_z = interp1d(t_orig, pos[:, 2], kind='linear')
        pos_new = np.column_stack([f_x(t_new), f_y(t_new), f_z(t_new)])
        resampled.append(pos_new)

    stack = np.stack(resampled, axis=0)  # (n_eps, T, 3)

    # 中位数聚合 (鲁棒, 抗 outlier)
    aggregated = np.median(stack, axis=0)  # (T, 3)

    # SG 平滑
    win = min(11, target_frames // 10 * 2 + 1)
    if win >= 3:
        aggregated = savgol_filter(aggregated, win, 3, axis=0)

    meta = {
        "method": "median_ensemble",
        "n_episodes": n_eps,
        "source_episodes": sorted(episodes_dict.keys()),
        "target_frames": target_frames,
        "sg_window": win,
    }
    return aggregated, meta


# ═══════════════════════════════════════════════════════════════════
# 汇总报告
# ═══════════════════════════════════════════════════════════════════

def summarize(results, ensemble_meta=None):
    """汇总统计 → 终端报告 + stats dict 喵~"""
    n = len(results)
    header = "全部 40 条轨迹" if n >= 30 else f"TOP{n}" if results[0]["episode"] in TOP5_EPISODES else f"{n} 条轨迹"
    print(f"\n{'='*80}")
    print(f"  心形参数标定报告 — {header} (n={n})")
    if ensemble_meta:
        print(f"  Ensemble: {ensemble_meta.get('method', 'N/A')}, "
              f"源 episodes: {ensemble_meta.get('source_episodes', [])}")
    print(f"{'='*80}\n")

    # ── ① 微摆频率 ──
    all_freqs = []
    for r in results:
        all_freqs.extend(r["wiggle_frequencies"])
    all_freqs = np.array(all_freqs) if all_freqs else np.array([5.0])

    print("── ① 微摆频率 (速度域 FFT, 高通 0.8Hz) ──")
    print(f"  检测到 {len(all_freqs)} 个峰值频率")
    print(f"  频率范围: {all_freqs.min():.1f} - {all_freqs.max():.1f} Hz")
    print(f"  中位数: {np.median(all_freqs):.2f} Hz")
    print(f"  均值: {all_freqs.mean():.2f} ± {all_freqs.std():.2f} Hz")
    p25, p75 = np.percentile(all_freqs, [25, 75])
    print(f"  IQR: [{p25:.1f}, {p75:.1f}] Hz")
    print(f"  生成器默认: {GEN_DEFAULTS['wiggle_frequency']} Hz (固定)")
    if abs(np.median(all_freqs) - GEN_DEFAULTS['wiggle_frequency']) < 1.0:
        print(f"  → ✓ 频率 {GEN_DEFAULTS['wiggle_frequency']}Hz 在中位数 {np.median(all_freqs):.1f}Hz 附近, 合理喵~")
    else:
        print(f"  → ⚠ 建议默认频率改为 {np.median(all_freqs):.1f} Hz 喵~")

    # ── ② 微摆振幅 ──
    amp_stds = np.array([r["wiggle_amplitude_std_mm"] for r in results])
    amp_p2ps = np.array([r["wiggle_amplitude_p2p_mm"] for r in results])

    print(f"\n── ② 微摆振幅 (1-8Hz 带通) ──")
    print(f"  std:  {amp_stds.mean():.2f} ± {amp_stds.std():.2f} mm  [{amp_stds.min():.1f}, {amp_stds.max():.1f}]")
    print(f"  P2P:  {amp_p2ps.mean():.2f} ± {amp_p2ps.std():.2f} mm  [{amp_p2ps.min():.1f}, {amp_p2ps.max():.1f}]")
    gen_eff_amp = GEN_DEFAULTS["wiggle_amplitude"] * GEN_DEFAULTS["wiggle_scale_factor"] * 1000
    print(f"  生成器有效振幅: {gen_eff_amp:.1f} mm "
          f"({GEN_DEFAULTS['wiggle_amplitude']*1000:.0f}mm × {GEN_DEFAULTS['wiggle_scale_factor']})")
    if abs(gen_eff_amp - amp_stds.mean()) < 2.0:
        print(f"  → ✓ 振幅缩放因子 {GEN_DEFAULTS['wiggle_scale_factor']} 验证通过喵~")
    else:
        new_scale = amp_stds.mean() / (GEN_DEFAULTS["wiggle_amplitude"] * 1000)
        print(f"  → ⚠ 建议修改 wiggle_scale_factor 为 {new_scale:.2f} 喵~")

    # ── ③ Z 高度 ──
    mix_z = np.array([r["mix_z_abs"] for r in results])
    draw_z = np.array([r["draw_z_abs"] for r in results])
    finish_z = np.array([r["finish_z_abs"] for r in results])

    print(f"\n── ③ Z 高度三段 (RM65 base_link 坐标系) ──")
    print(f"  融合段 Z: {mix_z.mean():.3f} ± {mix_z.std():.3f} m")
    print(f"  成形段 Z: {draw_z.mean():.3f} ± {draw_z.std():.3f} m")
    print(f"  收尾段 Z: {finish_z.mean():.3f} ± {finish_z.std():.3f} m")
    mix_offset = (mix_z - draw_z).mean()
    finish_offset = (finish_z - draw_z).mean()
    print(f"  融合高度偏移 (相对成形段): {mix_offset*1000:.0f} mm ({mix_offset*100:.0f} cm)")
    print(f"  收尾高度偏移 (相对成形段): {finish_offset*1000:.0f} mm ({finish_offset*100:.0f} cm)")
    print(f"  生成器 mix_offset={GEN_DEFAULTS['mix_height_offset']*1000:.0f}mm "
          f"finish_offset={GEN_DEFAULTS['finish_height_offset']*1000:.0f}mm")
    print(f"  → RM65 坐标系的偏移值, SE(3) retarget 后自动适配 AUBO E5 喵~")

    # ── ④ 阶段时间比例 ──
    mix_pcts = np.array([r["_mix_pct"] for r in results])
    sway_pcts = np.array([r["_sway_pct"] for r in results])
    lift_pcts = np.array([r["_lift_pct"] for r in results])

    print(f"\n── ④ 阶段时间比例 ──")
    print(f"  融合: {mix_pcts.mean():.0f}% ± {mix_pcts.std():.0f}%  [{mix_pcts.min():.0f}-{mix_pcts.max():.0f}]")
    print(f"  成形: {sway_pcts.mean():.0f}% ± {sway_pcts.std():.0f}%  [{sway_pcts.min():.0f}-{sway_pcts.max():.0f}]")
    print(f"  收尾: {lift_pcts.mean():.0f}% ± {lift_pcts.std():.0f}%  [{lift_pcts.min():.0f}-{lift_pcts.max():.0f}]")
    gen_mix = GEN_DEFAULTS["mix_ratio"] * 100
    gen_sway = GEN_DEFAULTS["sway_ratio"] * 100
    gen_lift = (1 - GEN_DEFAULTS["mix_ratio"] - GEN_DEFAULTS["sway_ratio"]) * 100
    print(f"  生成器: mix={gen_mix:.0f}% sway={gen_sway:.0f}% lift={gen_lift:.0f}%")
    if abs(mix_pcts.mean() - gen_mix) > 10:
        print(f"  → ⚠ 融合段占比差异较大 (数据 {mix_pcts.mean():.0f}% vs 生成器 {gen_mix:.0f}%)")
        print(f"     原因: 录制数据含完整接近运动, 生成器仅模拟融合段喵~")

    # ── ⑤ Y 轴 ──
    y_ranges = np.array([r["y_range_mm"] for r in results])
    print(f"\n── ⑤ Y 轴推拉幅度 ──")
    print(f"  Y 范围: {y_ranges.mean():.0f} ± {y_ranges.std():.0f} mm  [{y_ranges.min():.0f}, {y_ranges.max():.0f}]")

    # ── ⑥ 路径长度 ──
    path_lens = np.array([r["path_length"] for r in results])
    print(f"\n── ⑥ 路径长度 ──")
    print(f"  {path_lens.mean():.3f} ± {path_lens.std():.3f} m  [{path_lens.min():.3f}, {path_lens.max():.3f}]")

    return {
        "n_episodes": n,
        "wiggle_freq_median": float(np.median(all_freqs)),
        "wiggle_freq_mean": float(all_freqs.mean()),
        "wiggle_freq_std": float(all_freqs.std()),
        "wiggle_freq_range": [float(all_freqs.min()), float(all_freqs.max())],
        "wiggle_amplitude_std_mm_mean": float(amp_stds.mean()),
        "wiggle_amplitude_std_mm_std": float(amp_stds.std()),
        "wiggle_amplitude_p2p_mm_mean": float(amp_p2ps.mean()),
        "draw_z_mean": float(draw_z.mean()),
        "mix_height_offset_cm": float(mix_offset * 100),
        "finish_height_offset_cm": float(finish_offset * 100),
        "forming_duration_s_mean": float(np.array([r["forming_duration_s"] for r in results]).mean()),
        "mix_ratio_mean": float(mix_pcts.mean() / 100),
        "sway_ratio_mean": float(sway_pcts.mean() / 100),
        "y_range_mm_mean": float(y_ranges.mean()),
        "path_length_mean": float(path_lens.mean()),
        "episode_indices": sorted([r["episode"] for r in results if isinstance(r["episode"], int)]),
    }


# ═══════════════════════════════════════════════════════════════════
# YAML 输出
# ═══════════════════════════════════════════════════════════════════

def save_yaml(stats, ensemble_meta, output_path):
    """保存标定结果喵~"""
    import yaml

    # 用纯 dict 避免 OrderedDict 导致的 YAML 冗余标记
    def _plain(d):
        if isinstance(d, OrderedDict):
            return {k: _plain(v) for k, v in d.items()}
        if isinstance(d, dict):
            return {k: _plain(v) for k, v in d.items()}
        if isinstance(d, list):
            return [_plain(x) for x in d]
        return d

    calibrated = {
        ("meta", OrderedDict([
            ("description", "从笛卡尔轨迹示教数据标定的心形 PourConfig 参数"),
            ("source", "resource/cartesian/right/" if stats["n_episodes"] >= 30
                      else "resource/heart/"),
            ("n_episodes", stats["n_episodes"]),
            ("episode_indices", stats["episode_indices"]),
            ("generated_by", "scripts/calibrate_heart_params.py"),
        ])),
        ("pour_config", OrderedDict([
            ("wiggle_amplitude_m", GEN_DEFAULTS["wiggle_amplitude"]),
            ("wiggle_amplitude_effective_mm", round(stats["wiggle_amplitude_std_mm_mean"], 2)),
            ("wiggle_scale_factor", GEN_DEFAULTS["wiggle_scale_factor"]),
            ("wiggle_frequency_hz", round(stats["wiggle_freq_median"], 1)),
            ("wiggle_frequency_mean_hz", round(stats["wiggle_freq_mean"], 1)),
            ("wiggle_frequency_range_hz", [round(f, 1) for f in stats["wiggle_freq_range"]]),
            ("mix_height_offset_cm", round(stats["mix_height_offset_cm"], 1)),
            ("draw_height_offset_cm", 0.6),
            ("finish_height_offset_cm", round(stats["finish_height_offset_cm"], 1)),
            ("max_velocity_ms", 0.05),
            ("max_acceleration_ms2", 0.1),
            ("max_jerk_ms3", 0.5),
        ])),
        ("heart_phases", OrderedDict([
            ("mix_ratio", round(stats["mix_ratio_mean"], 2)),
            ("sway_ratio", round(stats["sway_ratio_mean"], 2)),
            ("lift_ratio", round(1.0 - stats["mix_ratio_mean"] - stats["sway_ratio_mean"], 2)),
        ])),
        ("ensemble", ensemble_meta if ensemble_meta else {"method": "none"}),
        ("notes", [
            f"wiggle_amplitude 保持 {GEN_DEFAULTS['wiggle_amplitude']}m, *0.3 缩放不变 (数据验证有效振幅 ~{stats['wiggle_amplitude_std_mm_mean']:.1f}mm)",
            f"wiggle_frequency 实测中位数 {stats['wiggle_freq_median']:.1f}Hz, 范围 {stats['wiggle_freq_range']}",
            "Z 高度偏移通过 SE(3) retarget 自动适配 AUBO E5 工作空间",
            "阶段比例各 episode 差异显著, heart() 固定比例仅为简化模型",
        ]),
    ])

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(calibrated, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"\n标定结果已保存到: {output_path}")


# ═══════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="心形拉花参数标定 — 从笛卡尔示教数据提取统计参数",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                              # 全部 40 条
  %(prog)s --episodes 32                # 单条 ep32
  %(prog)s --episodes 32,25,23          # 多条组合
  %(prog)s --episodes top5              # TOP5 精选
  %(prog)s --episodes all --ensemble    # 40 条中位数聚合
  %(prog)s --source heart --episodes top5  # 精选心形源
        """,
    )
    parser.add_argument("--episodes", type=str, default="all",
                        help="Episode 选择: all | top5 | top3 | 32,25,23 | 32 (默认: all)")
    parser.add_argument("--source", type=str, default="cartesian",
                        choices=["cartesian", "heart"],
                        help="数据源: cartesian (全部40条) | heart (精选心形) (默认: cartesian)")
    parser.add_argument("--ensemble", action="store_true",
                        help="用中位数聚合多条轨迹生成 ensemble, 并分析聚合轨迹的特征")
    parser.add_argument("--output", type=str,
                        default=os.path.join(CONFIG_DIR, "heart_params_calibrated.yaml"),
                        help="输出 YAML 路径")
    parser.add_argument("--json", type=str, default=None,
                        help="同时输出 JSON 到指定路径")
    args = parser.parse_args()

    # ── 加载数据 ──
    data_dir = CARTESIAN_DIR if args.source == "cartesian" else HEART_DIR
    if not os.path.isdir(data_dir):
        print(f"[错误] 数据目录不存在: {data_dir}")
        sys.exit(1)

    print(f"数据源: {args.source} ({data_dir})")
    print(f"Episode 选择: {args.episodes}")

    episodes_dict = load_episodes(args.episodes, args.source)

    if not episodes_dict:
        print("[错误] 未找到任何数据文件喵~")
        sys.exit(1)

    print(f"加载了 {len(episodes_dict)} 条轨迹: {sorted(episodes_dict.keys())}")

    # ── 逐条分析 ──
    results = []
    for ep_idx, positions in episodes_dict.items():
        r = analyze_one(ep_idx, positions)
        results.append(r)

    # ── Ensemble (可选) ──
    ensemble_meta = None
    if args.ensemble and len(episodes_dict) > 1:
        print(f"\n生成 Ensemble 轨迹 ({len(episodes_dict)} 条 → 中位数聚合)...")
        agg_pos, ensemble_meta = ensemble_trajectory(episodes_dict)
        # 分析聚合轨迹
        agg_result = analyze_one(-1, agg_pos)
        agg_result["episode"] = "ensemble"
        results.append(agg_result)
        print(f"  Ensemble: {agg_result['path_length']:.3f}m, "
              f"成形段={agg_result['forming_duration_s']:.1f}s, "
              f"微摆振幅={agg_result['wiggle_amplitude_std_mm']}mm")

    # ── 汇总 ──
    stats = summarize(results, ensemble_meta)
    save_yaml(stats, ensemble_meta, args.output)

    if args.json:
        json_stats = {k: (list(v) if isinstance(v, np.ndarray) else v)
                      for k, v in stats.items()}
        json_stats["per_episode"] = []
        for r in results:
            d = {k: v for k, v in r.items() if not k.startswith("_")}
            d["wiggle_frequencies"] = [round(f, 1) for f in d.get("wiggle_frequencies", [])]
            d["_mix_pct"] = round(r["_mix_pct"], 1)
            d["_sway_pct"] = round(r["_sway_pct"], 1)
            d["_lift_pct"] = round(r["_lift_pct"], 1)
            json_stats["per_episode"].append(d)
        with open(args.json, "w") as f:
            json.dump(json_stats, f, indent=2, ensure_ascii=False)
        print(f"JSON 已保存到: {args.json}")

    # ── 每 episode 详情 (仅当 ≤20 条时打印) ──
    if len(results) <= 20:
        print(f"\n{'─'*80}")
        print(f"  逐条详情")
        print(f"{'─'*80}")
        for r in results:
            ep = r["episode"]
            if ep == "ensemble":
                print(f"\n  [ensemble]: {r['path_length']:.3f}m, 成形段={r['forming_duration_s']:.1f}s")
            else:
                print(f"\n  ep{ep}: {r['path_length']:.3f}m, 成形段={r['forming_duration_s']:.1f}s")
            print(f"    微摆频率: {[round(f,1) for f in r['wiggle_frequencies']]}")
            print(f"    微摆振幅: std={r['wiggle_amplitude_std_mm']}mm, P2P={r['wiggle_amplitude_p2p_mm']}mm")
            print(f"    Z: mix={r['mix_z_abs']:.3f}m, draw={r['draw_z_abs']:.3f}m, finish={r['finish_z_abs']:.3f}m")
            print(f"    阶段%: mix={r['_mix_pct']:.0f}%, sway={r['_sway_pct']:.0f}%, lift={r['_lift_pct']:.0f}%")


if __name__ == "__main__":
    main()
