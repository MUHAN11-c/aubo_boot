"""
心形轨迹质量评分 — 纯几何评估, 不依赖摄像头 喵~

评分维度 (总分 100):
  1. 对称性 (30分) — X轴偏离中线的RMS越小越好
  2. 成形段 (25分) — Z高度三段的占比是否合理
  3. 微摆质量 (25分) — 摆动频率/振幅是否在合理范围
  4. 工作空间 (20分) — 是否在 AUBO E5 安全边界内

用法:
  from latte_imitation.heart_scorer import score_heart_trajectory
  score, details = score_heart_trajectory(trajectory_xyz)
"""

import numpy as np
from scipy.signal import savgol_filter
from scipy.fft import fft, fftfreq

# AUBO E5 工作空间 (来自 workspace_safety.yaml)
WS_X = (-0.87, 0.87)
WS_Y = (-0.87, 0.87)
WS_Z = (-0.85, 1.10)

# 合理参数范围 (来自 40 条数据标定)
FREQ_OK = (0.8, 8.0)      # 合理摆动频率 Hz
AMP_OK = (0.5, 15.0)      # 合理微摆振幅 mm (std)
FORMING_RATIO_OK = (0.05, 0.60)  # 成形段占比


def _extract_forming_indices(z, dt=0.05):
    """提取成形段索引 (Z低位最长连续段)"""
    w = min(21, len(z) // 10 * 2 + 1)
    if w < 3: return 0, len(z)
    zs = savgol_filter(z, w, 3)
    lo = zs < np.median(zs)
    ch = np.diff(lo.astype(int))
    st = np.where(ch == 1)[0] + 1; en = np.where(ch == -1)[0] + 1
    if lo[0]: st = np.concatenate([[0], st])
    if lo[-1]: en = np.concatenate([en, [len(lo)]])
    if len(st) == 0: return len(z)//4, 3*len(z)//4
    b = np.argmax(en - st); fs, fe = st[b], en[b]
    if fe - fs < 10: fs, fe = len(z)//4, 3*len(z)//4
    return fs, fe


def score_heart_trajectory(trajectory, dt=0.05, verbose=False):
    """心形轨迹质量评分 0-100 喵~

    Args:
        trajectory: (T, 3) XYZ 轨迹 (米)
        dt: 时间步长
        verbose: 打印详细扣分项

    Returns:
        (total_score, details_dict)
    """
    T = len(trajectory)
    if T < 20:
        return 0.0, {"error": "轨迹太短"}

    x, y, z = trajectory[:, 0], trajectory[:, 1], trajectory[:, 2]
    scores = {}
    details = {}

    # ═══ 1. 对称性 (30分) ═══
    # 心形应左右对称, X 偏离中线的 RMS 越小越好
    fs, fe = _extract_forming_indices(z, dt)
    x_form = x[fs:fe]
    x_mid = (x_form.max() + x_form.min()) / 2
    x_asym_rms = np.sqrt(np.mean((x_form - x_mid) ** 2))  # 不对称度
    # 对称性评分: RMS < 2mm → 满分, > 30mm → 0分
    sym_score = max(0, 30 * (1 - x_asym_rms / 0.030))
    scores["symmetry"] = sym_score
    details["x_asym_rms_mm"] = round(x_asym_rms * 1000, 1)

    # ═══ 2. 成形段占比 (25分) ═══
    form_ratio = (fe - fs) / T
    # 占比在 [5%, 60%] 区间 → 满分; 超出线性扣分
    lo, hi = FORMING_RATIO_OK
    if lo <= form_ratio <= hi:
        form_score = 25.0
    elif form_ratio < lo:
        form_score = 25 * form_ratio / lo
    else:
        form_score = 25 * (1 - form_ratio) / (1 - hi)
    scores["forming_ratio"] = max(0, form_score)
    details["forming_ratio"] = round(form_ratio, 3)
    details["forming_frames"] = fe - fs

    # ═══ 3. 微摆质量 (25分) ═══
    # 3a. 频率 (15分): 主频应在 0.8-8Hz 内
    x_vel = np.diff(x_form) / dt
    nv = len(x_vel)
    if nv > 10:
        yf = np.abs(fft(x_vel - x_vel.mean()))
        xf = fftfreq(nv, dt)
        pm = (xf > 0.3) & (xf < 12)
        if pm.sum() > 0:
            peak_idx = np.argmax(yf[pm])
            peak_freq = xf[pm][peak_idx]
        else:
            peak_freq = 0.0
    else:
        peak_freq = 0.0

    flo, fhi = FREQ_OK
    if flo <= peak_freq <= fhi:
        freq_score = 15.0
    elif peak_freq < flo:
        freq_score = 15 * peak_freq / flo
    else:
        freq_score = 15 * max(0, 1 - (peak_freq - fhi) / 10)
    scores["wiggle_freq"] = max(0, freq_score)
    details["peak_freq_hz"] = round(peak_freq, 2)

    # 3b. 振幅 (10分): 微摆振幅应在 0.5-15mm 范围内
    x_amp_std = np.abs(x_form - x_form.mean()).std() * 1000  # mm
    alo, ahi = AMP_OK
    if alo <= x_amp_std <= ahi:
        amp_score = 10.0
    elif x_amp_std < alo:
        amp_score = 10 * x_amp_std / alo  # 太小(无摆动) → 扣分
    else:
        amp_score = 10 * max(0, 1 - (x_amp_std - ahi) / 30)  # 太大 → 扣分
    scores["wiggle_amp"] = max(0, amp_score)
    details["x_amp_std_mm"] = round(x_amp_std, 2)

    # ═══ 4. 工作空间安全 (20分) ═══
    violations = 0
    for i in range(T):
        if not (WS_X[0] <= x[i] <= WS_X[1]): violations += 1
        if not (WS_Y[0] <= y[i] <= WS_Y[1]): violations += 1
        if not (WS_Z[0] <= z[i] <= WS_Z[1]): violations += 1
    violation_ratio = violations / (T * 3)  # 三个维度
    ws_score = max(0, 20 * (1 - violation_ratio))
    scores["workspace"] = ws_score
    details["ws_violations"] = violations
    details["ws_violation_ratio"] = round(violation_ratio, 4)

    # ═══ 汇总 ═══
    total = sum(scores.values())
    details["scores_breakdown"] = {k: round(v, 2) for k, v in scores.items()}

    if verbose:
        print(f"心形评分: {total:.1f}/100")
        for k, v in scores.items():
            bar = "█" * int(v / 2) + "░" * (15 - int(v / 2))
            print(f"  {k:15s} [{bar}] {v:.1f}")
        print(f"  details: {details}")

    return round(total, 2), details


def score_against_reference(trajectory, reference, dt=0.05):
    """额外: 与参考轨迹的 RMSE 评分 (0-100, 越小越接近) 喵~

    用于模拟模式下替代真实摄像头评分。
    """
    from scipy.interpolate import interp1d

    T_ref = len(reference)
    T_traj = len(trajectory)

    # 时间对齐
    t_ref = np.linspace(0, 1, T_ref)
    t_traj = np.linspace(0, 1, T_traj)
    ref_interp = np.column_stack([
        interp1d(t_ref, reference[:, d], kind='linear')(t_traj)
        for d in range(3)
    ])

    rmse = np.sqrt(np.mean(np.sum((trajectory - ref_interp) ** 2, axis=1))) * 1000
    # 转换为 0-100 评分: RMSE < 5mm → 100, > 200mm → 0
    score = max(0, min(100, 100 * (1 - (rmse - 5) / 195)))
    return round(score, 2), {"rmse_mm": round(rmse, 2)}
