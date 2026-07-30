"""Reproducible end-to-end validation on recorded RGB-D observations.

This is an evaluation entry point, not an online execution path.  It runs the
converged foreground route (SAM mask intersected with dilated measured-depth
component) while keeping detection, measured-depth geometry and safety gates
fixed.  Both target classes are evaluated by default through their respective
pipeline lines: ``peach_bag`` (0, cylinder axis) and ``peach_nobag`` (1,
sphere + stem-cavity axis), sharing the same insertion tool and safety gates.

Example:
    venv/bin/python -m peach_pose.e2e_validate \
        --output e2e_validation.json
"""
from __future__ import annotations

import argparse
import itertools
import json
import platform
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

from .contracts import BagObservation
from .candidates import CandidateEstimator, MODE_IDS, MODE_LABELS
from .inference import InferenceEngine
from .inspector.config import (
    CALIBRATION_VERSION, DATASET_DIR, K_AZURE, MODEL_VERSION, YOLO_MODEL,
)
# e2e 跑的是历史 Azure 录包（1280×720），故用 K_AZURE；实机/本机工具一律 K_PERCIPIO
from .validation import AnnotationMetrics, load_annotations


def _quantiles(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return {
        "n": int(a.size),
        "p10": float(np.percentile(a, 10)),
        "median": float(np.median(a)),
        "p90": float(np.percentile(a, 90)),
    }


def _portable_path(path: Path) -> str:
    """报告内记录相对路径（位于当前工作目录内时），保证报告可移植。"""
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def _summary(items, elapsed, missing_masks=0):
    statuses = Counter(item["status"] for item in items)
    reasons = Counter(flag for item in items for flag in item["flags"])
    kind_statuses = defaultdict(Counter)
    for item in items:
        kind_statuses[item.get("target_kind", "bag")][item["status"]] += 1
    metric_values = defaultdict(list)
    for item in items:
        for key, value in item["metrics"].items():
            if isinstance(value, (int, float)):
                metric_values[key].append(value)
    total = len(items)
    unsafe_flags = {"tool_clearance_failed", "foreground_truncated",
                    "invalid_gravity", "insufficient_measured_points"}
    unsafe_accept_proxy = sum(
        item["status"] == "ACCEPT" and bool(set(item["flags"]) & unsafe_flags)
        for item in items)
    return {
        "targets_evaluated": total,
        "missing_masks": missing_masks,
        "status_counts": dict(statuses),
        "per_kind_status": {k: dict(v) for k, v in kind_statuses.items()},
        "status_rates": {k: v / total for k, v in statuses.items()} if total else {},
        "diagnostic_counts": dict(reasons),
        "unsafe_accept_proxy": unsafe_accept_proxy,
        "metric_quantiles": {k: _quantiles(v) for k, v in metric_values.items()},
        "pose_runtime_s": elapsed,
        "pose_ms_per_target": 1000.0 * elapsed / total if total else None,
    }


def _pose_item(frame_id, index, bbox, detection, result):
    pose = result.grasp_3d
    return {
        "key": f"{frame_id}:{index}",
        "frame_id": frame_id,
        "detection_index": index,
        "bbox": list(bbox),
        "detector_confidence": float(detection["conf"]),
        "status": pose.status,
        "flags": list(pose.diagnostic_flags),
        "mask_source": result.mask_source,
        "target_kind": result.target_kind,
        "entry_start": pose.entry_start.tolist() if pose.entry_start is not None else None,
        "axis": (pose.translation_direction.tolist()
                 if pose.translation_direction is not None else None),
        "metrics": result.metrics,
    }


def _agreements(mode_items):
    reports = []
    for left_mode, right_mode in itertools.combinations(mode_items, 2):
        left = {item["key"]: item for item in mode_items[left_mode]}
        right = {item["key"]: item for item in mode_items[right_mode]}
        shared = sorted(left.keys() & right.keys())
        distances, same_status = [], 0
        for key in shared:
            a, b = left[key], right[key]
            same_status += a["status"] == b["status"]
            if a["entry_start"] is not None and b["entry_start"] is not None:
                distances.append(float(np.linalg.norm(
                    np.asarray(a["entry_start"]) - np.asarray(b["entry_start"]))))
        reports.append({
            "modes": [left_mode, right_mode], "shared_targets": len(shared),
            "status_agreement_rate": same_status / len(shared) if shared else None,
            "entry_start_delta_m": _quantiles(distances),
        })
    return reports


def _ranking(mode_summaries):
    """Safety-first provisional ranking without claiming ground-truth accuracy."""
    rows = []
    for mode, summary in mode_summaries.items():
        statuses = summary["status_counts"]
        total = summary["targets_evaluated"]
        accept = statuses.get("ACCEPT", 0)
        proxy_unsafe = summary["unsafe_accept_proxy"]
        rows.append({
            "mode": mode, "label": MODE_LABELS[mode],
            "unsafe_accept_proxy": proxy_unsafe,
            "accept_rate": accept / total if total else 0.0,
            "pose_ms_per_target": summary["pose_ms_per_target"],
        })
    rows.sort(key=lambda row: (
        row["unsafe_accept_proxy"], -row["accept_rate"],
        row["pose_ms_per_target"] if row["pose_ms_per_target"] is not None else float("inf")))
    return {
        "provisional_only": True,
        "reason": "no ground-truth annotations supplied",
        "ordered_modes": rows,
    }


def _ground_truth_ranking(annotation_report: dict, mode_summaries: dict) -> dict:
    rows = []
    for mode, metrics in annotation_report["modes"].items():
        rows.append({
            "mode": mode, "label": MODE_LABELS[mode],
            "false_accept_count": metrics.get("false_accept_count", 0),
            "false_accept_rate": metrics.get("false_accept_rate", 0.0),
            "status_accuracy": metrics.get("status_correct_rate", 0.0),
            "accepted_rate": metrics.get("accepted_rate", 0.0),
            "false_accept_95pct_upper": metrics.get("false_accept_95pct_upper"),
            "offline_safety_gate_pass": metrics.get("offline_safety_gate_pass", False),
            "bottom_error_p95_px": metrics.get("bottom_px_error_px", {}).get("p95"),
            "pose_ms_per_target": mode_summaries[mode]["pose_ms_per_target"],
        })
    rows.sort(key=lambda row: (
        not row["offline_safety_gate_pass"],
        row["false_accept_95pct_upper"] if row["false_accept_95pct_upper"] is not None else float("inf"),
        row["false_accept_count"], -row["accepted_rate"],
        row["bottom_error_p95_px"] if row["bottom_error_p95_px"] is not None else float("inf"),
        row["pose_ms_per_target"] if row["pose_ms_per_target"] is not None else float("inf")))
    return {
        "provisional_only": False,
        "reason": "safety-first ground-truth lexicographic ranking",
        "ordered_modes": rows,
    }


def _write_markdown(path: Path, report: dict):
    class_id = report['protocol']['class_id']
    class_text = "0(peach_bag) + 1(peach_nobag)" if class_id < 0 else str(class_id)
    lines = ["# PeachPose 桃姿 离线端到端验证", "",
             "> 几何与安全判定仅使用传感器实测深度。", "",
             "## 协议", "",
             f"- 帧数：{report['protocol']['frames']}",
             f"- 目标类别：{class_text}",
             f"- 检测阈值：{report['protocol']['min_conf']}", "",
             "## 候选结果", "",
             "| 模式 | ACCEPT | REOBSERVE | REJECT | P50袋径(mm) | 位姿耗时(ms/目标) |",
             "|---|---:|---:|---:|---:|---:|"]
    for mode, summary in report["modes"].items():
        counts = summary["status_counts"]
        diameter = summary["metric_quantiles"].get("bag_diameter_upper_m")
        median_mm = diameter["median"] * 1000 if diameter else float("nan")
        lines.append(
            f"| {MODE_LABELS[mode]} | {counts.get('ACCEPT', 0)} | "
            f"{counts.get('REOBSERVE', 0)} | {counts.get('REJECT', 0)} | "
            f"{median_mm:.1f} | {summary['pose_ms_per_target']:.2f} |")
        per_kind = summary.get("per_kind_status", {})
        if per_kind:
            kind_text = "；".join(
                f"{kind}: {c.get('ACCEPT', 0)}/{c.get('REOBSERVE', 0)}/{c.get('REJECT', 0)}"
                for kind, c in sorted(per_kind.items()))
            lines.append(f"  - 分线（ACCEPT/REOBSERVE/REJECT）：{kind_text}")
    lines += ["", "## 选择结论", ""]
    if report.get("annotation_metrics"):
        lines.append("已加载人工真值；最终选择必须以 annotation_metrics 的错误 ACCEPT 为首要指标。")
    else:
        best = report["ranking"]["ordered_modes"][0]
        lines.append(
            f"当前无人工真值，仅可给出暂定顺序；按安全代理和覆盖率首位为 **{best['label']}**。")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Ablate RGB-D bag-pose foreground sources")
    parser.add_argument("--dataset", type=Path, default=DATASET_DIR)
    parser.add_argument("--output", type=Path, default=Path("e2e_validation.json"))
    parser.add_argument("--markdown", type=Path, default=None,
                        help="default: output path with .md suffix")
    parser.add_argument("--annotations", type=Path, default=None,
                        help="optional JSONL/JSON ground truth")
    parser.add_argument("--limit", type=int, default=0, help="0 evaluates all frames")
    parser.add_argument("--class-id", type=int, default=-1,
                        help="-1: both classes (peach_bag=0, peach_nobag=1)")
    parser.add_argument("--min-conf", type=float, default=0.5)
    parser.add_argument("--modes", default=",".join(MODE_IDS),
                        help=f"comma list: {','.join(MODE_IDS)}")
    parser.add_argument("--sam-max-bboxes", type=int, default=32)
    args = parser.parse_args()

    modes = [mode.strip() for mode in args.modes.split(",") if mode.strip()]
    unknown = set(modes) - set(MODE_IDS)
    if unknown:
        parser.error(f"unknown modes: {sorted(unknown)}")

    ids = sorted(path.stem for path in (args.dataset / "rgb").glob("*.png"))
    if args.limit:
        ids = ids[:args.limit]
    # 模型在包根 model/（与 YOLO_MODEL 同目录）；勿用 parents[1]（那是内层
    # peach_pose_ros2/ 包目录，移植后不再含 model/）
    engine = InferenceEngine(
        yolo_model=str(YOLO_MODEL),
        sam_model=str(YOLO_MODEL.parent / "mobile_sam.pt"),
        sam_max_bboxes=args.sam_max_bboxes,
    )
    estimator = CandidateEstimator()
    mode_items = {mode: [] for mode in modes}
    mode_elapsed = Counter()
    missing_masks = Counter()
    detection_counts = Counter()
    detector_elapsed = 0.0
    segment_elapsed = 0.0
    annotations = load_annotations(args.annotations)
    annotation_metrics = AnnotationMetrics(annotations, tuple(modes)) if annotations else None

    for frame_id in ids:
        rgb = cv2.imread(str(args.dataset / "rgb" / f"{frame_id}.png"))
        depth = cv2.imread(
            str(args.dataset / "depth" / f"{frame_id}.png"), cv2.IMREAD_UNCHANGED)
        if rgb is None or depth is None:
            raise FileNotFoundError(f"missing RGB-D pair for frame {frame_id}")

        t0 = time.perf_counter()
        detections = engine.detect(rgb)
        detector_elapsed += time.perf_counter() - t0
        detection_counts["all"] += len(detections)
        selected = [d for d in detections
                    if d["conf"] >= args.min_conf
                    and (args.class_id < 0 or d["class_id"] == args.class_id)]
        detection_counts["selected"] += len(selected)
        for d in selected:
            detection_counts[f"class_{d['class_id']}"] += 1

        masks_by_bbox = {}
        if modes and selected:
            bboxes = [d["bbox"] for d in selected]
            t0 = time.perf_counter()
            segmented = engine.segment(rgb, bboxes)
            segment_elapsed += time.perf_counter() - t0
            masks_by_bbox = {tuple(bbox): mask for mask, bbox in segmented}

        frame_results = []
        for index, detection in enumerate(selected):
            bbox = tuple(detection["bbox"])
            obs = BagObservation(
                rgb=rgb, depth=depth, camera_K=K_AZURE,
                detections=[detection],
                metadata={"model_version": MODEL_VERSION,
                          "calibration_version": CALIBRATION_VERSION},
            )
            results = estimator.estimate_modes(
                obs, f"{frame_id}:{index}", bbox, masks_by_bbox.get(bbox), modes)
            frame_results.append(results)
            for mode, result in results.items():
                mode_elapsed[mode] += estimator.last_timings_ms[mode] / 1000.0
                if "mask_unavailable" in result.grasp_3d.diagnostic_flags:
                    missing_masks[mode] += 1
                mode_items[mode].append(
                    _pose_item(frame_id, index, bbox, detection, result))

        if annotation_metrics is not None:
            annotation_metrics.add_frame(
                frame_id, selected, frame_results, depth.shape[:2])

        print(f"{frame_id}: all={len(detections)} selected={len(selected)}")

    report = {
        "protocol": {
            "dataset": _portable_path(args.dataset),
            "frames": len(ids),
            "class_id": args.class_id,
            "min_conf": args.min_conf,
            "modes": modes,
            "measured_depth_only_for_geometry": True,
            "python": platform.python_version(),
            "opencv": cv2.__version__,
        },
        "detection": {
            "counts": dict(detection_counts),
            "runtime_s": detector_elapsed,
            "ms_per_frame": 1000.0 * detector_elapsed / len(ids) if ids else None,
        },
        "segmentation": {
            "runtime_s": segment_elapsed,
            "ms_per_frame": 1000.0 * segment_elapsed / len(ids) if ids else None,
        },
        "modes": {
            mode: _summary(mode_items[mode], mode_elapsed[mode], missing_masks[mode])
            for mode in modes
        },
        "agreements": _agreements(mode_items),
        "items": mode_items,
    }
    report["ranking"] = _ranking(report["modes"])
    if annotation_metrics is not None:
        report["annotation_metrics"] = annotation_metrics.report()
        report["ranking"] = _ground_truth_ranking(
            report["annotation_metrics"], report["modes"])
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_markdown(args.markdown or args.output.with_suffix(".md"), report)
    print(json.dumps({
        "protocol": report["protocol"],
        "detection": report["detection"],
        "segmentation": report["segmentation"],
        "modes": report["modes"],
        "agreements": report["agreements"],
        "ranking": report["ranking"],
        "annotation_metrics": report.get("annotation_metrics"),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
