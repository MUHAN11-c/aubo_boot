#!/usr/bin/env python3
"""采集 percipio 相机彩色图/深度图，用 YOLO26s-depth 估计深度并评估精度。

用法:
    source /opt/ros/jazzy/setup.bash
    scripts/.venv-yolo/bin/python3 scripts/depth_accuracy_eval.py \
        --frames 30 --output depth_eval_out

依赖话题 (驱动 launch 后存在):
    /camera/color/image_raw   (bgr8)
    /camera/depth/image_raw   (16UC1, 单位 mm; depth_registration_enable=true 时已对齐彩色)

输出 (保存到 --output/<timestamp>/):
    color.png           彩色原图
    depth_gt.png        相机深度伪彩色图
    depth_pred.png      YOLO 预测深度伪彩色图
    error_abs.png       绝对误差热力图 (米)
    metrics_per_frame.csv   每帧精度指标
    metrics_summary.json    汇总指标 (均值/标准差)
"""

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters


def depth_metrics(gt: np.ndarray, pred: np.ndarray, mask: np.ndarray) -> dict:
    """标准单目深度评估指标。gt/pred 单位: 米。"""
    g = gt[mask].astype(np.float64)
    p = pred[mask].astype(np.float64)
    if g.size < 100:
        return {}
    diff = p - g
    thresh = np.maximum(g / p, p / g)
    log_g, log_p = np.log(g), np.log(np.maximum(p, 1e-6))
    return {
        "n_pixels": int(g.size),
        "mae": float(np.mean(np.abs(diff))),
        "rmse": float(np.sqrt(np.mean(diff ** 2))),
        "abs_rel": float(np.mean(np.abs(diff) / g)),
        "sq_rel": float(np.mean(diff ** 2 / g)),
        "rmse_log": float(np.sqrt(np.mean((log_p - log_g) ** 2))),
        "delta_1.25": float(np.mean(thresh < 1.25)),
        "delta_1.25^2": float(np.mean(thresh < 1.25 ** 2)),
        "delta_1.25^3": float(np.mean(thresh < 1.25 ** 3)),
    }


def median_scale_align(gt: np.ndarray, pred: np.ndarray, mask: np.ndarray):
    """中位数尺度对齐: 单目深度常有全局尺度漂移, 对齐后评估形状精度。
    返回 (对齐后的 pred, 尺度因子)。"""
    g = gt[mask].astype(np.float64)
    p = pred[mask].astype(np.float64)
    scale = np.median(g / np.maximum(p, 1e-6))
    return pred * scale, float(scale)


def colorize_depth(depth_m: np.ndarray, max_m: float) -> np.ndarray:
    d = np.clip(depth_m / max_m, 0.0, 1.0)
    d8 = (d * 255).astype(np.uint8)
    vis = cv2.applyColorMap(255 - d8, cv2.COLORMAP_TURBO)
    vis[depth_m <= 0] = 0
    return vis


class DepthEvalNode(Node):
    def __init__(self, args):
        super().__init__("depth_accuracy_eval")
        self.args = args
        self.bridge = CvBridge()
        self.frames = []          # (color_bgr, depth_gt_m)
        self.done = False

        self.get_logger().info("加载模型 %s ..." % args.model)
        from ultralytics import YOLO
        self.model = YOLO(args.model)
        self.get_logger().info("模型加载完成, 等待图像帧...")

        color_sub = message_filters.Subscriber(
            self, Image, args.color_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, args.depth_topic, qos_profile=qos_profile_sensor_data)
        sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self.callback)

    def callback(self, color_msg: Image, depth_msg: Image):
        if len(self.frames) >= self.args.frames:
            return
        color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth_raw.dtype == np.uint16:
            # 注意: 驱动发布的是原始深度值, 未乘 TY_FLOAT_SCALE_UNIT。
            # PS800-E1 的 scale unit 为 0.25, 即 1 个原始单位 = 0.25 mm。
            depth_m = depth_raw.astype(np.float32) * self.args.depth_unit_mm / 1000.0
        else:  # 32FC1, 已是米
            depth_m = depth_raw.astype(np.float32)
        self.frames.append((color, depth_m))
        self.get_logger().info("已采集 %d/%d 帧" % (len(self.frames), self.args.frames))

    def run_eval(self, out_dir: Path):
        args = self.args
        rows = []
        last_vis = None
        for i, (color, depth_gt) in enumerate(self.frames):
            t0 = time.time()
            result = self.model.predict(color, verbose=False)[0]
            pred = result.depth.data
            if hasattr(pred, "cpu"):
                pred = pred.cpu().numpy()
            pred = np.asarray(pred, dtype=np.float32).squeeze()
            if pred.shape != depth_gt.shape:
                pred = cv2.resize(pred, (depth_gt.shape[1], depth_gt.shape[0]),
                                  interpolation=cv2.INTER_LINEAR)
            infer_ms = (time.time() - t0) * 1000

            valid = (depth_gt >= args.min_depth) & (depth_gt <= args.max_depth) \
                & np.isfinite(depth_gt) & (pred > 0)
            coverage = float(np.mean(valid))

            m_raw = depth_metrics(depth_gt, pred, valid)
            pred_aligned, scale = median_scale_align(depth_gt, pred, valid)
            m_aligned = depth_metrics(depth_gt, pred_aligned, valid)

            row = {"frame": i, "infer_ms": round(infer_ms, 1),
                   "coverage": round(coverage, 4),
                   "median_scale": round(scale, 4),
                   "gt_median_m": round(float(np.median(depth_gt[valid])), 4),
                   "pred_median_m": round(float(np.median(pred[valid])), 4)}
            for k, v in m_raw.items():
                row["raw_" + k] = v
            for k, v in m_aligned.items():
                row["aligned_" + k] = v
            rows.append(row)

            # 保存最后一帧的可视化
            last_vis = (color, depth_gt, pred, valid)

        if not rows:
            self.get_logger().error("没有有效帧, 无法评估")
            return

        # 汇总
        summary = {}
        for key in rows[0]:
            if key == "frame":
                continue
            vals = np.array([r[key] for r in rows], dtype=np.float64)
            summary[key] = {"mean": float(vals.mean()), "std": float(vals.std())}
        summary["n_frames"] = len(rows)
        summary["model"] = args.model
        summary["eval_range_m"] = [args.min_depth, args.max_depth]

        out_dir.mkdir(parents=True, exist_ok=True)
        with open(out_dir / "metrics_per_frame.csv", "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        with open(out_dir / "metrics_summary.json", "w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        color, depth_gt, pred, valid = last_vis
        cv2.imwrite(str(out_dir / "color.png"), color)
        cv2.imwrite(str(out_dir / "depth_gt.png"),
                    colorize_depth(depth_gt, args.max_depth))
        cv2.imwrite(str(out_dir / "depth_pred.png"),
                    colorize_depth(pred, args.max_depth))
        err = np.zeros_like(depth_gt)
        err[valid] = np.abs(pred[valid] - depth_gt[valid])
        err_vis = colorize_depth(err, 1.0)  # 0~1m 误差热力
        cv2.imwrite(str(out_dir / "error_abs.png"), err_vis)

        # 终端打印关键指标
        def fmt(name):
            s = summary[name]
            return "%.4f ± %.4f" % (s["mean"], s["std"])

        print("\n===== 精度评估结果 (%d 帧, 有效范围 %.2f~%.2f m) ====="
              % (len(rows), args.min_depth, args.max_depth))
        print("像素覆盖率(相机深度有效): %.1f%%" % (summary["coverage"]["mean"] * 100))
        print("推理耗时: %s ms" % fmt("infer_ms"))
        print("GT深度中位数: %s m | 预测中位数: %s m | 中位尺度因子: %s"
              % (fmt("gt_median_m"), fmt("pred_median_m"), fmt("median_scale")))
        print("\n-- 原始输出 (模型声称米制, 直接对比) --")
        print("MAE      : %s m" % fmt("raw_mae"))
        print("RMSE     : %s m" % fmt("raw_rmse"))
        print("AbsRel   : %s" % fmt("raw_abs_rel"))
        print("RMSE log : %s" % fmt("raw_rmse_log"))
        print("δ < 1.25 : %s" % fmt("raw_delta_1.25"))
        print("δ < 1.25²: %s" % fmt("raw_delta_1.25^2"))
        print("δ < 1.25³: %s" % fmt("raw_delta_1.25^3"))
        print("\n-- 中位数尺度对齐后 (评估形状精度, 消除尺度漂移) --")
        print("MAE      : %s m" % fmt("aligned_mae"))
        print("RMSE     : %s m" % fmt("aligned_rmse"))
        print("AbsRel   : %s" % fmt("aligned_abs_rel"))
        print("δ < 1.25 : %s" % fmt("aligned_delta_1.25"))
        print("\n结果已保存到: %s" % out_dir.resolve())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--model", default="yolo26s-depth.pt")
    parser.add_argument("--frames", type=int, default=30, help="采集帧数")
    parser.add_argument("--output", default="depth_eval_out")
    parser.add_argument("--min-depth", type=float, default=0.2, help="有效深度下限(米)")
    parser.add_argument("--max-depth", type=float, default=5.0, help="有效深度上限(米)")
    parser.add_argument("--depth-unit-mm", type=float, default=0.25,
                        help="16UC1 深度每个原始单位对应的毫米数 "
                             "(= 设备 TY_FLOAT_SCALE_UNIT, PS800-E1 为 0.25)")
    parser.add_argument("--stall-timeout", type=float, default=20.0,
                        help="超过该秒数未收到新帧则提前结束采集并用已有帧评估")
    args = parser.parse_args()

    rclpy.init()
    node = DepthEvalNode(args)
    last_progress = time.time()
    last_count = 0
    try:
        while rclpy.ok() and len(node.frames) < args.frames:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.frames) != last_count:
                last_count = len(node.frames)
                last_progress = time.time()
            elif time.time() - last_progress > args.stall_timeout:
                node.get_logger().warn("超过 %.0fs 未收到新帧, 结束采集" % args.stall_timeout)
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    if node.frames:
        out_dir = Path(args.output) / time.strftime("%Y%m%d_%H%M%S")
        node.run_eval(out_dir)
    else:
        node.get_logger().error("未收到任何图像帧, 请确认相机驱动已启动: "
                                "ros2 launch percipio_camera percipio_camera.launch.py")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
