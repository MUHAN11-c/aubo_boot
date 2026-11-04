#!/usr/bin/env python3
"""2026-10-23 汇报数据整理：真机抓取 + 五点力 + CoP->剪切点。

只读 runs/ 与工具档案，输出 summary.json / summary.csv 与图表到 assets/。
"""
from __future__ import annotations

import csv
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle, Wedge

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
ASSETS = OUT / "assets"
ASSETS.mkdir(parents=True, exist_ok=True)

INK = "#1E2430"
MUTED = "#6B7280"
GRID = "#E6E1D8"
PEACH = "#E76F51"
CORAL = "#F4A261"
TEAL = "#2A9D8F"
SAND = "#E9C46A"
BLUE = "#457B9D"
PAPER = "#FFFFFF"

plt.rcParams.update({
    "font.family": "Noto Sans CJK JP",
    "axes.unicode_minus": False,
    "axes.edgecolor": GRID,
    "axes.labelcolor": MUTED,
    "axes.titlecolor": INK,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "text.color": INK,
    "figure.facecolor": "none",
    "axes.facecolor": "none",
    "savefig.facecolor": "none",
    "legend.frameon": False,
    "font.size": 9.5,
})

PAD_MM = {
    "1": (15.75, 4.30),
    "2": (9.45, -13.30),
    "3": (-9.65, -13.15),
    "5": (0.00, 16.45),
    "7": (-15.70, 4.55),
}
PAD_ORDER = ["1", "2", "3", "5", "7"]
PAD_XY = np.array([PAD_MM[k] for k in PAD_ORDER], dtype=float)
CH_COLORS = {"1": TEAL, "2": BLUE, "3": SAND, "5": CORAL, "7": PEACH}

D_PAD_MM = 72.29
D_BLADE_MM = 37.0
RATIO = D_BLADE_MM / D_PAD_MM
CAP_DEG = 5.0


def read_force_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open()))
    t = np.array([float(r["t_s"]) for r in rows])
    head = rows[0].keys()
    cols = ["f1", "f2", "f3", "f5", "f7"] if "f1" in head else ["f1_n", "f2_n", "f3_n", "f5_n", "f7_n"]
    f = np.array([[float(r[c]) for c in cols] for r in rows])
    return t, f, np.column_stack([f, f.sum(1)])


def cop_series(f: np.ndarray) -> np.ndarray:
    total = f.sum(1)
    out = np.full((len(f), 2), np.nan)
    ok = total > 1e-9
    out[ok] = (f[ok] @ PAD_XY) / total[ok, None]
    return out


def summarize_harvest() -> dict:
    cycles = sorted(ROOT.glob("runs/session_2026102*/cycles/*.json"))
    rows = []
    for p in cycles:
        d = json.loads(p.read_text())
        d["_session"] = p.parts[-3]
        rows.append(d)
    block = [d for d in rows if d["_session"] == "session_20261022_163539"]
    level6 = lambda d: (d.get("completion_level") or 0) >= 6
    ok = [d for d in block if level6(d)]
    by_target = {}
    for t in ("target_0", "target_1", "target_2", "target_3"):
        a = [d for d in block if d["target_id"] == t]
        s = [d for d in a if level6(d)]
        by_target[t] = {"n": len(a), "ok": len(s), "rate": round(100 * len(s) / len(a), 1)}

    def stage(d, name):
        if name not in d["stage_names"]:
            return None
        return d["stage_durations"][d["stage_names"].index(name)]

    approaches = [stage(d, "approach_insert") for d in ok]
    retreats = [stage(d, "retreat") for d in ok]
    elapsed = [d["elapsed_s"] for d in ok]
    fails = [
        {
            "target": d["target_id"],
            "level": d.get("completion_level"),
            "failure_code": d.get("failure_code"),
            "reason": d.get("reason"),
            "elapsed_s": d.get("elapsed_s"),
        }
        for d in block
        if not level6(d)
    ]
    day_level6 = sum(1 for d in rows if level6(d))
    day_strict = sum(1 for d in rows if level6(d) and d.get("outcome") == 0)
    return {
        "block": {
            "session": "session_20261022_163539",
            "cycles": len(block),
            "success": len(ok),
            "rate": round(100 * len(ok) / len(block), 1),
            "quality_fail": sum(1 for d in block if d.get("failure_code") == "skipped_quality"),
            "unreachable_fail": sum(1 for d in block if d.get("failure_code") == "skipped_unreachable"),
            "wall_first": min(d["wall_start"] for d in block),
            "wall_last": max(d["wall_end"] for d in block),
            "per_target": by_target,
            "approach_median_s": round(float(np.median(approaches)), 2),
            "approach_p10_s": round(float(np.percentile(approaches, 10)), 2),
            "approach_p90_s": round(float(np.percentile(approaches, 90)), 2),
            "retreat_median_s": round(float(np.median(retreats)), 2),
            "elapsed_median_s": round(float(np.median(elapsed)), 2),
            "elapsed_min_s": round(float(np.min(elapsed)), 2),
            "elapsed_max_s": round(float(np.max(elapsed)), 2),
            "fails": fails,
        },
        "day": {
            "cycles": len(rows),
            "level6": day_level6,
            "level6_rate": round(100 * day_level6 / len(rows), 1),
            "strict_success": day_strict,
            "strict_rate": round(100 * day_strict / len(rows), 1),
            "level7": sum(1 for d in rows if (d.get("completion_level") or 0) >= 7),
            "sessions": sorted({d["_session"] for d in rows}),
        },
    }


def summarize_force() -> dict:
    tests = {}
    for name, path in {
        "10s": ROOT / "runs/force_10s_20261021_095244/force.csv",
        "20s": ROOT / "runs/force_20s_20261021_102812/force.csv",
        "manual": ROOT / "runs/force_20s_manual/force.csv",
    }.items():
        t, f, _ = read_force_csv(path)
        dt = np.diff(t)
        tests[name] = {
            "frames": len(t),
            "duration_s": round(float(t[-1] - t[0]), 2),
            "rate_hz": round(float((len(t) - 1) / (t[-1] - t[0])), 1),
            "peaks_n": [round(float(np.nanmax(f[:, i])), 3) for i in range(5)],
            "all5_10g": int(((f > 0.0980665).all(1)).sum()),
            "max_simul_10g": int((f > 0.0980665).sum(1).max()),
            "max_simul_50g": int((f > 0.4903325).sum(1).max()),
        }

    # 10 s 单通道力-行程线性
    t, f, _ = read_force_csv(ROOT / "runs/force_10s_20261021_095244/force.csv")
    tr = np.array([float(r["tr5_mm"]) for r in csv.DictReader((ROOT / "runs/force_10s_20261021_095244/force.csv").open())])
    m = (f[:, 3] > 0) & (tr > 0)
    slope, _ = np.polyfit(tr[m], f[m, 3], 1)
    r = float(np.corrcoef(tr[m], f[m, 3])[0, 1])
    tests["10s"]["slope_n_per_mm"] = round(float(slope), 4)
    tests["10s"]["slope_g_per_mm"] = round(float(slope * 1000 / 9.80665), 2)
    tests["10s"]["slope_r"] = round(r, 5)

    # 20 s 手动满行程 AD（峰值）
    rows = list(csv.DictReader((ROOT / "runs/force_20s_manual/force.csv").open()))
    ad = np.array([[float(r[c]) for c in ("ad1", "ad2", "ad3", "ad5", "ad7")] for r in rows])
    tests["manual"]["full_stroke_ad"] = [int(x) for x in ad.max(0)]
    tests["manual"]["ad_range"] = int(ad.max(0).max() - ad.max(0).min())

    # 真机五点 episode
    ep = json.loads((ROOT / "runs/session_20261022_150712/force/episode_001.json").read_text())
    t, f, _ = read_force_csv(ROOT / "runs/session_20261022_150712/force/episode_001.csv")
    total = f.sum(1)
    cop = cop_series(f)
    mag = np.linalg.norm(cop, axis=1)
    valid = ((f > 0.1).sum(1) >= 3) & (total > 1.0)
    peak_i = int(np.argmax(total))
    peak_f = f[peak_i]
    peak_cop = cop[peak_i]
    peak_mag = float(np.linalg.norm(peak_cop))
    left = float(f[valid][:, [2, 4]].sum(1).mean())
    right = float(f[valid][:, [0, 1]].sum(1).mean())
    top = float(f[valid][:, 3].mean())
    contact_start = float(t[total > 0.3][0])
    contact_end = float(t[total > 0.3][-1])
    # cycle 阶段边界（session_150712 target_0）
    cycle = json.loads((ROOT / "runs/session_20261022_150712/cycles/20261022_150946_target_0.json").read_text())
    approach_end_cycle = float(sum(cycle["stage_durations"][: cycle["stage_names"].index("approach_insert") + 1]))
    from datetime import datetime
    cycle_start = datetime.fromisoformat(cycle["wall_start"])
    ep_start = datetime.fromisoformat(ep["wall_start"])
    offset = (ep_start - cycle_start).total_seconds() - (cycle["start_unix"] % 1)
    approach_end = approach_end_cycle - offset

    def geom(p: float, cap: float = CAP_DEG) -> dict:
        theta = math.degrees(math.atan(p / D_PAD_MM))
        th = min(theta, cap)
        pad_shift = D_PAD_MM * math.tan(math.radians(th))
        blade_shift = D_BLADE_MM * math.tan(math.radians(th))
        axial = D_BLADE_MM * (1 - math.cos(math.radians(th)))
        return {
            "cop_mm": round(p, 2),
            "theta_req_deg": round(theta, 2),
            "theta_applied_deg": round(th, 2),
            "pad_shift_mm": round(pad_shift, 2),
            "residual_mm": round(max(0.0, p - pad_shift), 2),
            "blade_shift_mm": round(blade_shift, 2),
            "blade_axial_mm": round(axial, 2),
        }

    p50 = float(np.median(mag[valid]))
    p90 = float(np.percentile(mag[valid], 90))
    pmax = float(np.max(mag[valid]))
    to2 = {k: geom(v, cap=90) for k, v in {"p50": p50, "p90": p90, "peak": peak_mag}.items()}
    for k, v in to2.items():
        theta2 = math.degrees(math.atan(max(0.0, v["cop_mm"] - 2.0) / D_PAD_MM))
        to2[k] = {
            "cop_mm": v["cop_mm"],
            "theta_to_2mm_deg": round(theta2, 2),
            "blade_shift_to_2mm_mm": round(D_BLADE_MM * math.tan(math.radians(theta2)), 2),
        }

    real5 = {
        "episode": "runs/session_20261022_150712/force/episode_001.csv",
        "target": "target_0",
        "frames": len(t),
        "duration_s": round(float(ep["duration_s"]), 2),
        "contact_s": round(float(ep["contact_s"]), 2),
        "peaks_n": [round(float(x), 3) for x in f.max(0)],
        "total_peak_n": round(float(total.max()), 3),
        "peak_t_s": round(float(t[peak_i]), 3),
        "peak_forces_n": [round(float(x), 3) for x in peak_f],
        "peak_cop_mm": [round(float(x), 2) for x in peak_cop],
        "peak_cop_mag_mm": round(peak_mag, 2),
        "peak_spread_pct": round(float((peak_f.max() - peak_f.min()) / peak_f.mean() * 100), 1),
        "all5_10g_frames": int(((f > 0.0980665).all(1)).sum()),
        "valid_cop_frames": int(valid.sum()),
        "cop_mean_mm": [round(float(x), 2) for x in np.nanmean(cop[valid], axis=0)],
        "cop_p50_mm": round(p50, 2),
        "cop_p90_mm": round(p90, 2),
        "cop_max_mm": round(pmax, 2),
        "left_mean_n": round(left, 2),
        "right_mean_n": round(right, 2),
        "top_mean_n": round(top, 2),
        "left_right_ratio": round(left / max(right, 1e-6), 1),
        "contact_start_s": round(contact_start, 2),
        "contact_end_s": round(contact_end, 2),
        "approach_end_s": round(approach_end, 2),
        "peak_after_approach_end_s": round(float(t[peak_i]) - approach_end, 2),
        "geom_peak": geom(peak_mag),
        "geom_p50": geom(p50),
        "geom_p90": geom(p90),
        "geom_max": geom(pmax),
        "geom_cap": geom(D_PAD_MM * math.tan(math.radians(CAP_DEG)), cap=CAP_DEG),
        "to2": to2,
        "ratio_blade_pad": round(RATIO, 4),
    }

    ep1 = json.loads((ROOT / "runs/session_20261022_144719/force/episode_001.json").read_text())
    t1, f1, _ = read_force_csv(ROOT / "runs/session_20261022_144719/force/episode_001.csv")
    real1 = {
        "frames": len(t1),
        "duration_s": round(float(ep1["duration_s"]), 2),
        "peaks_n": [round(float(x), 3) for x in f1.max(0)],
        "max_simul_10g": int((f1 > 0.0980665).sum(1).max()),
    }
    return {"bench": tests, "real5": real5, "real1": real1, "geometry": {
        "d_pad_mm": D_PAD_MM, "d_blade_mm": D_BLADE_MM, "ratio": round(RATIO, 4),
        "cap_deg": CAP_DEG, "pad_at_cap_mm": round(D_PAD_MM * math.tan(math.radians(CAP_DEG)), 2),
        "blade_at_cap_mm": round(D_BLADE_MM * math.tan(math.radians(CAP_DEG)), 2),
    }}


# ---------------------------------------------------------------- charts
def downsample_max(t, f, bins=160):
    edges = np.linspace(t[0], t[-1], bins + 1)
    idx = np.digitize(t, edges) - 1
    out_t, out_f = [], []
    for b in range(bins):
        m = idx == b
        if not m.any():
            continue
        j = np.argmax(f[m].sum(1))
        out_t.append(t[m][j])
        out_f.append(f[m][j])
    return np.array(out_t), np.array(out_f)


def chart_manual():
    t, f, _ = read_force_csv(ROOT / "runs/force_20s_manual/force.csv")
    tt, ff = downsample_max(t, f, 220)
    fig, ax = plt.subplots(figsize=(6.3, 2.85), dpi=220)
    fig.subplots_adjust(left=0.09, right=0.895, top=0.84, bottom=0.17)
    for i, ch in enumerate(PAD_ORDER):
        ax.plot(tt, ff[:, i], color=CH_COLORS[ch], lw=1.3, label=f"ch{ch}")
    ax.axhline(3.628, color=MUTED, lw=0.9, ls=(0, (3, 3)), alpha=0.7)
    ax.text(19.8, 3.70, "370 g 饱和", ha="right", va="bottom", fontsize=8, color=MUTED)
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 4.05)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("单点力 / N")
    ax.grid(axis="y", color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(ncol=5, loc="upper left", bbox_to_anchor=(0, 1.13), fontsize=8.6,
              handlelength=1.2, columnspacing=1.4, handletextpad=0.4)
    ax.text(0.25, 2.62, "逐点满行程按压\n五点同时 >10 g：0 帧",
            fontsize=8.6, color=INK, va="top",
            bbox=dict(boxstyle="round,pad=0.38", fc="white", ec=GRID, lw=0.8, alpha=0.94))
    fig.savefig(ASSETS / "p2_manual_20s.png", transparent=True)
    plt.close(fig)


def chart_real5():
    t, f, _ = read_force_csv(ROOT / "runs/session_20261022_150712/force/episode_001.csv")
    total = f.sum(1)
    tt, ff = downsample_max(t, f, 180)
    tot_ds = np.interp(tt, t, total)
    fig, ax = plt.subplots(figsize=(6.3, 2.85), dpi=220)
    fig.subplots_adjust(left=0.09, right=0.895, top=0.84, bottom=0.17)
    ax2 = ax.twinx()
    ax2.fill_between(tt, tot_ds, color=PEACH, alpha=0.16, lw=0, zorder=1)
    ax2.set_ylim(0, 6.6)
    ax2.set_yticks([0, 2, 4, 6])
    ax2.tick_params(colors=MUTED, labelsize=7.8, length=2)
    ax2.set_ylabel("合力 ΣF / N", color=PEACH, fontsize=8.2, labelpad=2)
    ax2.spines["top"].set_visible(False)
    for s in ("left", "bottom"):
        ax2.spines[s].set_visible(False)
    for i, ch in enumerate(PAD_ORDER):
        ax.plot(tt, ff[:, i], color=CH_COLORS[ch], lw=1.4, label=f"ch{ch}", zorder=4)
    ax.axvline(2.24, color=MUTED, lw=1.0, ls=(0, (4, 3)), zorder=2)
    ax.text(2.30, 2.72, "插入结束 / 撤退开始", fontsize=7.3, color=MUTED, va="top")
    ax.scatter([2.405], [2.167], s=28, color=PEACH, edgecolor="white", lw=1.2, zorder=6)
    ax.annotate("ΣF 5.24 N @ 2.405 s", xy=(2.405, 2.167), xytext=(2.78, 2.30),
                fontsize=8.0, color=INK,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8, shrinkB=3))
    ax.text(0.10, 2.70, "172 帧五点同时 >0.1 N", fontsize=8.2, color=INK, va="top",
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=GRID, lw=0.8, alpha=0.94))
    ax.set_xlim(0, 4.26)
    ax.set_ylim(0, 2.85)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("单点力 / N")
    ax.grid(axis="y", color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    handles = [plt.Line2D([], [], color=CH_COLORS[c], lw=1.6) for c in PAD_ORDER]
    handles.append(plt.Rectangle((0, 0), 1, 1, fc=PEACH, alpha=0.25, ec="none"))
    ax.legend(handles, [f"ch{c}" for c in PAD_ORDER] + ["合力"], ncol=6,
              loc="upper left", bbox_to_anchor=(0, 1.14), fontsize=8.2,
              handlelength=1.1, columnspacing=0.9, handletextpad=0.35)
    fig.savefig(ASSETS / "p2_real5_episode.png", transparent=True)
    plt.close(fig)


def chart_padplane(real5: dict):
    t, f, _ = read_force_csv(ROOT / "runs/session_20261022_150712/force/episode_001.csv")
    cop = cop_series(f)
    total = f.sum(1)
    valid = ((f > 0.1).sum(1) >= 3) & (total > 1.0)
    pts = cop[valid]
    mean = pts.mean(0)
    fig, ax = plt.subplots(figsize=(4.35, 4.35), dpi=240)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.97, bottom=0.10)
    ax.set_aspect("equal")
    ax.set_xlim(-33, 33)
    ax.set_ylim(-33, 33)
    ax.add_patch(Circle((0, 0), 27.5, fc="#FBF8F3", ec=GRID, lw=1.0, zorder=0))
    for r, color, ls, label in [
        (2.0, TEAL, (0, (2, 2)), "2 mm 容差"),
        (D_PAD_MM * math.tan(math.radians(CAP_DEG)), SAND, (0, (4, 3)), "5° 上限 6.32 mm"),
    ]:
        ax.add_patch(Circle((0, 0), r, fill=False, ec=color, lw=1.3, ls=ls, zorder=3))
    ax.scatter(pts[:, 0], pts[:, 1], s=18, color=PEACH, alpha=0.34, lw=0, zorder=2)
    means = f[valid].mean(0)
    for i, ch in enumerate(PAD_ORDER):
        x, y = PAD_MM[ch]
        ax.add_patch(Circle((x, y), 3.7, fc="white", ec=INK, lw=1.0, zorder=4))
        ax.text(x, y + 1.15, ch, ha="center", va="center", fontsize=9.2, fontweight="bold",
                color=INK, zorder=5)
        ax.text(x, y - 1.65, f"{means[i]:.2f}", ha="center", va="center", fontsize=6.3,
                color=MUTED, zorder=5)
    # 平均 CoP 箭头
    ax.add_patch(FancyArrowPatch((0, 0), tuple(mean), arrowstyle="-|>", mutation_scale=13,
                                 color=INK, lw=1.6, zorder=6))
    ax.scatter([mean[0]], [mean[1]], s=42, color=INK, edgecolor="white", lw=1.1, zorder=7)
    ax.text(2.8, 3.0, "2 mm", fontsize=6.8, color=TEAL, ha="left", va="bottom")
    ax.text(6.6, -7.0, "5° 上限 6.32", fontsize=6.8, color=SAND, ha="left", va="center")
    ax.text(-31.5, -30.8, "CoP 均值 (−9.2, +0.6) · 315 帧", fontsize=7.6, color=INK, ha="left", va="bottom")
    ax.text(-31.5, 30.8, "CoP 云：始终在 −X 侧", fontsize=8.6, color=PEACH, fontweight="bold", va="top")
    ax.set_xticks([-30, -20, -10, 0, 10, 20, 30])
    ax.set_yticks([-30, -20, -10, 0, 10, 20, 30])
    ax.tick_params(labelsize=7.5, length=0)
    ax.grid(color=GRID, lw=0.6, alpha=0.6)
    for s in ("top", "right", "left", "bottom"):
        ax.spines[s].set_visible(False)
    fig.savefig(ASSETS / "p3_padplane.png", transparent=True)
    plt.close(fig)


def chart_side():
    fig, ax = plt.subplots(figsize=(3.25, 4.55), dpi=240)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.04)
    ax.set_aspect("equal")
    ax.set_xlim(-36, 46)
    ax.set_ylim(-88, 10)
    ax.axis("off")

    # 果实侧方向
    ax.annotate("", xy=(0, 6.6), xytext=(0, 1.2),
                arrowprops=dict(arrowstyle="-|>", color=PEACH, lw=1.2))
    ax.text(2.2, 4.6, "果实侧 +Z", fontsize=8.2, color=PEACH, va="center")

    # 喉道
    for x in (-26, 26):
        ax.plot([x, x], [0, -80], color=GRID, lw=2.2, solid_capstyle="round", zorder=1)
    ax.plot([-26, 26], [0, 0], color=INK, lw=1.3, zorder=3)
    ax.text(-27, 1.8, "TCP / 开口", fontsize=8.0, color=INK, ha="left")

    # 原轴
    ax.plot([0, 0], [0, -80], color=MUTED, lw=1.0, ls=(0, (3, 3)), zorder=2)

    # 原刀口 / 原膜片
    ax.add_patch(Polygon([[-11, -35.6], [11, -35.6], [8.5, -38.4], [-8.5, -38.4]],
                         closed=True, fc="#D9D4CB", ec=MUTED, lw=0.8, zorder=3))
    ax.plot([-26, 26], [-D_PAD_MM, -D_PAD_MM], color="#CFC8BC", lw=4.2, solid_capstyle="butt", zorder=2)
    for x in (-16.35, 0, 16.35):
        ax.add_patch(Rectangle((x - 2.6, -D_PAD_MM - 0.9), 5.2, 1.8, fc=TEAL, ec="none", zorder=4))

    # 5° 旋转后
    th = math.radians(CAP_DEG)
    bx, by = -D_BLADE_MM * math.sin(th), -D_BLADE_MM * math.cos(th)
    px, py = -D_PAD_MM * math.sin(th), -D_PAD_MM * math.cos(th)
    ax.plot([0, px * 1.12], [0, py * 1.12], color=PEACH, lw=2.0, zorder=4)
    # 旋转后的膜片线（垂直旋转轴）
    ux, uy = math.cos(th), -math.sin(th)
    ax.plot([px - 25 * ux, px + 25 * ux], [py - 25 * uy, py + 25 * uy],
            color=PEACH, lw=2.0, alpha=0.95, zorder=4)
    ax.plot([bx - 10 * ux, bx + 10 * ux], [by - 10 * uy, by + 10 * uy],
            color=PEACH, lw=2.6, alpha=0.95, zorder=5)
    ax.scatter([px], [py], s=18, color=PEACH, zorder=6)
    ax.scatter([bx], [by], s=18, color=PEACH, zorder=6)

    # 角度弧
    ax.add_patch(Wedge((0, 0), 18, 270 - CAP_DEG, 270, width=0.7, fc=PEACH, ec="none", alpha=0.55, zorder=3))
    ax.text(-7.8, -15.5, "5°", fontsize=9.2, color=PEACH, fontweight="bold")

    # 位移标注
    ax.annotate("", xy=(px, -78.5), xytext=(0, -78.5),
                arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=1.0, shrinkA=0, shrinkB=0))
    ax.text(px / 2, -82.4, "Δ膜片 6.32", ha="center", va="top", fontsize=8.2, color=INK)
    ax.annotate("", xy=(bx, -42.5), xytext=(0, -42.5),
                arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=1.0, shrinkA=0, shrinkB=0))
    ax.text(bx / 2, -46.2, "Δ刀口 3.24", ha="center", va="top", fontsize=8.2, color=INK)

    # 右侧深度标注
    ax.annotate("", xy=(29.5, -D_BLADE_MM), xytext=(29.5, 0),
                arrowprops=dict(arrowstyle="<|-|>", color=MUTED, lw=0.8, shrinkA=0, shrinkB=0))
    ax.text(31.2, -18.5, "37", fontsize=8.4, color=MUTED, va="center")
    ax.annotate("", xy=(33.5, -D_PAD_MM), xytext=(33.5, 0),
                arrowprops=dict(arrowstyle="<|-|>", color=MUTED, lw=0.8, shrinkA=0, shrinkB=0))
    ax.text(35.2, -36.0, "72.29", fontsize=8.4, color=MUTED, va="center")
    ax.text(-35.2, -34.8, "刀口", fontsize=8.2, color=MUTED, ha="left", va="center")
    ax.text(-35.2, -D_PAD_MM, "膜片", fontsize=8.2, color=MUTED, ha="left", va="center")

    fig.savefig(ASSETS / "p3_sideview.png", transparent=True, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def write_summary(summary: dict):
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    rows = []
    h = summary["harvest"]["block"]
    rows.append(["harvest", "block", f"{h['success']}/{h['cycles']}", f"{h['rate']}%"])
    for t, v in h["per_target"].items():
        rows.append(["harvest", t, f"{v['ok']}/{v['n']}", f"{v['rate']}%"])
    r5 = summary["force"]["real5"]
    rows += [
        ["force", "real5 total peak N", r5["total_peak_n"], ""],
        ["force", "real5 CoP p50 mm", r5["cop_p50_mm"], ""],
        ["force", "real5 CoP p90 mm", r5["cop_p90_mm"], ""],
        ["force", "blade shift @5deg mm", r5["geom_peak"]["blade_shift_mm"], ""],
        ["force", "theta req p50 deg", r5["geom_p50"]["theta_req_deg"], ""],
        ["force", "residual p50 mm", r5["geom_p50"]["residual_mm"], ""],
    ]
    with (OUT / "summary.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["group", "metric", "value", "rate"])
        w.writerows(rows)


def main():
    summary = {
        "meta": {
            "generated_at": "2026-10-23",
            "sources": [
                "runs/session_20261022_163539/cycles/*.json",
                "runs/session_20261022_150712/force/episode_001.csv",
                "runs/session_20261022_144719/force/episode_001.csv",
                "runs/force_10s_20261021_095244/force.csv",
                "runs/force_20s_20261021_102812/force.csv",
                "runs/force_20s_manual/force.csv",
                "src/aubo_description/config/bite_shear_v1.yaml",
                "src/serial_force/doc/geometry.md",
            ],
        },
        "harvest": summarize_harvest(),
        "force": summarize_force(),
    }
    write_summary(summary)
    chart_manual()
    chart_real5()
    chart_padplane(summary["force"]["real5"])
    chart_side()
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
