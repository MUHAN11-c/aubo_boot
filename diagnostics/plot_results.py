#!/usr/bin/env python3
"""diagnostics 探针数据可视化。

读取 results/ 下各探针最新 CSV（每次运行覆盖），生成对应的 PNG 曲线
（同名 *_latest.png，覆盖写），并打印文字摘要。

用法:
    plot_results.py [--results-dir DIR] [--show]

使用项目 Python 环境: aubo_py3.12/bin/python
图表标签用英文，避免服务器环境缺中文字体出现方块字。
"""

import argparse
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")  # 无显示环境也能出图；--show 时也不弹窗，只落盘
import matplotlib.pyplot as plt
import numpy as np

BLOCK_THRESHOLD_MS = 100.0  # 与探针端 kBlockThresholdMs 一致
HOLE_THRESHOLD_MS = 100.0   # 推送空洞阈值，与探针端一致


def read_csv(path):
    """读取探针 CSV，返回 {列名: np.array}；空字段为 nan。文件缺失返回 None。"""
    if not os.path.isfile(path):
        return None
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    columns = {}
    for name in rows[0].keys():
        values = []
        for row in rows:
            cell = (row[name] or "").strip()
            if name == "phase":
                values.append(cell)
                continue
            try:
                values.append(float(cell))
            except ValueError:
                values.append(np.nan)
        columns[name] = (
            np.array(values, dtype=object)
            if name == "phase"
            else np.array(values, dtype=float)
        )
    return columns


def summarize(name, values):
    values = values[~np.isnan(values)]
    if values.size == 0:
        return f"{name}: no data"
    return (
        f"{name}: n={values.size} min={values.min():.3f} "
        f"avg={values.mean():.3f} p99={np.percentile(values, 99):.3f} "
        f"max={values.max():.3f}"
    )


def plot_runtime(data, csv_path, out_png):
    t, latency, rc = data["t_sec"], data["latency_ms"], data["rc"]
    blocks = latency > BLOCK_THRESHOLD_MS
    failures = rc != 0
    span = t[-1] - t[0] if t.size > 1 else 0.0
    rate = (t.size - 1) / span if span > 0 else 0.0

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=False)
    fig.suptitle(
        f"runtime_probe: status polling latency  "
        f"(n={t.size}, rate={rate:.1f} Hz, blocks={int(blocks.sum())}, "
        f"failures={int(failures.sum())})"
    )
    ax = axes[0]
    ax.plot(t, latency, lw=0.6, label="latency")
    if blocks.any():
        ax.scatter(
            t[blocks], latency[blocks], c="r", s=18, zorder=5,
            label=f"block > {BLOCK_THRESHOLD_MS:.0f} ms",
        )
    if failures.any():
        ax.scatter(
            t[failures], latency[failures], c="orange", marker="x",
            s=30, zorder=6, label="failed call",
        )
    ax.set_ylabel("latency (ms)")
    ax.set_xlabel("time (s)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[1]
    ax.hist(latency, bins=80, log=True)
    ax.axvline(
        BLOCK_THRESHOLD_MS, color="r", ls="--",
        label=f"block threshold {BLOCK_THRESHOLD_MS:.0f} ms",
    )
    ax.set_xlabel("latency (ms)")
    ax.set_ylabel("count (log)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"[runtime] {summarize('latency_ms', latency)}")
    print(f"[runtime] effective_rate={rate:.1f} Hz, "
          f"block_events={int(blocks.sum())}, failures={int(failures.sum())}")
    print(f"[runtime] saved {out_png}")


def plot_push(data, csv_path, out_png):
    t, interval = data["t_sec"], data["interval_ms"]
    valid = interval[1:]  # 首行无间隔
    holes = valid > HOLE_THRESHOLD_MS
    span = t[-1] - t[0] if t.size > 1 else 0.0
    rate = (t.size - 1) / span if span > 0 else 0.0

    # 每秒推送次数
    sec = t.astype(int)
    max_sec = sec.max() if sec.size else 0
    per_sec = np.array([(sec == s).sum() for s in range(max_sec + 1)])

    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    fig.suptitle(
        f"push_probe: joint status push  "
        f"(n={t.size}, rate={rate:.2f} Hz, holes={int(holes.sum())})"
    )
    ax = axes[0]
    ax.plot(t[1:], valid, lw=0.6, label="push interval")
    if holes.any():
        idx = np.where(holes)[0] + 1
        ax.scatter(
            t[idx], interval[idx], c="r", s=18, zorder=5,
            label=f"hole > {HOLE_THRESHOLD_MS:.0f} ms",
        )
    ax.set_ylabel("interval (ms)")
    ax.set_xlabel("time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[1]
    ax.bar(np.arange(per_sec.size), per_sec, width=0.8)
    ax.axhline(rate, color="r", ls="--", label=f"avg {rate:.2f} Hz")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pushes per second")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"[push] {summarize('interval_ms', valid)}")
    print(f"[push] rate={rate:.2f} Hz, hole_events={int(holes.sum())}")
    print(f"[push] saved {out_png}")


def plot_tcp2can(data, csv_path, out_png):
    phase, t, rib = data["phase"], data["t_sec"], data["rib"]
    write_ms, batch = data["write_ms"], data["batch"]
    diag_ms = data["diag_ms"]

    stream = phase == "stream"
    drain = phase == "drain"
    has_write = ~np.isnan(write_ms)
    has_diag = ~np.isnan(diag_ms)
    write_blocks = np.nansum(write_ms > BLOCK_THRESHOLD_MS)
    diag_blocks = np.nansum(diag_ms > BLOCK_THRESHOLD_MS)

    # 产能：stream 阶段写入点数 / 时长
    stream_span = (
        t[stream][-1] - t[stream][0] if stream.sum() > 1 else 0.0
    )
    pts = np.nansum(batch[stream])
    pts_per_s = pts / stream_span if stream_span > 0 else 0.0

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(
        f"tcp2can_probe: command channel (zero motion)  "
        f"(production={pts_per_s:.0f} pts/s, "
        f"write_blocks={int(write_blocks)}, diag_blocks={int(diag_blocks)})"
    )

    ax = axes[0]
    ax.plot(t[stream], rib[stream], lw=0.8, label="RIB level (stream)")
    if drain.any():
        ax.plot(t[drain], rib[drain], lw=0.8, c="g", label="RIB level (drain)")
    ax.set_ylabel("RIB level (slots)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[1]
    ax.plot(t[has_write], write_ms[has_write], lw=0.6, c="tab:red",
            label="write latency")
    ax.axhline(BLOCK_THRESHOLD_MS, color="r", ls="--", alpha=0.5,
               label=f"block threshold {BLOCK_THRESHOLD_MS:.0f} ms")
    ax.set_ylabel("write (ms)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(t[has_diag], diag_ms[has_diag], lw=0.6, c="tab:purple",
            label="diag latency")
    ax.set_ylabel("diag (ms)")
    ax.set_xlabel("time (s)")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"[tcp2can] {summarize('write_ms', write_ms)}")
    print(f"[tcp2can] {summarize('diag_ms', diag_ms)}")
    rib_stream = rib[stream]
    print(f"[tcp2can] rib[min/avg/max]={np.nanmin(rib_stream):.0f}/"
          f"{np.nanmean(rib_stream):.1f}/{np.nanmax(rib_stream):.0f}, "
          f"production={pts_per_s:.0f} pts/s")
    print(f"[tcp2can] saved {out_png}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--results-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "results"),
    )
    parser.add_argument(
        "--show", action="store_true",
        help="保留参数位（当前仅落盘 PNG，不弹窗）",
    )
    args = parser.parse_args()
    results_dir = args.results_dir

    jobs = [
        ("runtime_probe_latest.csv", plot_runtime),
        ("runtime_probe_fast_latest.csv", plot_runtime),
        ("push_probe_latest.csv", plot_push),
        ("tcp2can_probe_latest.csv", plot_tcp2can),
    ]
    found = 0
    for csv_name, plotter in jobs:
        csv_path = os.path.join(results_dir, csv_name)
        data = read_csv(csv_path)
        if data is None:
            print(f"[skip] {csv_name} 不存在或为空")
            continue
        out_png = csv_path.replace(".csv", ".png")
        plotter(data, csv_path, out_png)
        found += 1
    if found == 0:
        print("没有找到任何探针数据，先运行 run_tests.sh", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
