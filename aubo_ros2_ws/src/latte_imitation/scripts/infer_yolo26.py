#!/usr/bin/env python3
"""YOLOv26 推理脚本 — 基于 ultralytics 框架。

用法:
    python3 scripts/infer_yolo26.py --source image.jpg           # 单张图片
    python3 scripts/infer_yolo26.py --source images/             # 目录批量推理
    python3 scripts/infer_yolo26.py --source 0                   # 摄像头 (device 0)
    python3 scripts/infer_yolo26.py --model m --source img.jpg   # yolo26m 权重
    python3 scripts/infer_yolo26.py --source img.jpg --show      # 显示结果
    python3 scripts/infer_yolo26.py --source img.jpg --save-txt  # 保存 txt 标注
"""

import argparse
import os
import sys
import time

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)


def get_weight_path(model_size: str, custom_weight: str = None) -> str:
    """获取模型权重路径。"""
    if custom_weight:
        if os.path.exists(custom_weight):
            return custom_weight
        return custom_weight

    pkg_weight = os.path.join(_PKG_DIR, f"yolo26{model_size}.pt")
    if os.path.exists(pkg_weight):
        return pkg_weight
    return f"yolo26{model_size}.pt"


def main():
    parser = argparse.ArgumentParser(description="YOLOv26 推理")
    parser.add_argument("--source", required=True,
                        help="输入源: 图片路径 / 目录 / 摄像头编号")
    parser.add_argument("--model", default="n", choices=list("nsmlx"),
                        help="模型规模 (default: n)")
    parser.add_argument("--weight", default=None,
                        help="自定义权重路径（覆盖 --model）")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理尺寸 (default: 640)")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="置信度阈值 (default: 0.25)")
    parser.add_argument("--iou", type=float, default=0.7,
                        help="NMS IOU 阈值 (default: 0.7)")
    parser.add_argument("--device", default="0",
                        help="设备 (default: 0)")
    parser.add_argument("--half", action="store_true",
                        help="FP16 推理")
    parser.add_argument("--show", action="store_true",
                        help="显示推理结果")
    parser.add_argument("--save", action="store_true", default=True,
                        help="保存推理结果 (default: True)")
    parser.add_argument("--save-txt", action="store_true",
                        help="保存 YOLO 格式 txt 标注")
    parser.add_argument("--save-crop", action="store_true",
                        help="保存裁剪出的检测目标")
    parser.add_argument("--nosave", action="store_true",
                        help="不保存结果")
    parser.add_argument("--project", default="runs/yolo26",
                        help="输出目录 (default: runs/yolo26)")
    parser.add_argument("--name", default="predict",
                        help="实验名称 (default: predict)")
    args = parser.parse_args()

    from ultralytics import YOLO

    model_path = get_weight_path(args.model, args.weight)
    print(f"[推理配置]")
    print(f"  模型: {model_path}")
    print(f"  输入: {args.source}")
    print(f"  尺寸: {args.imgsz}, conf={args.conf}, iou={args.iou}")
    print(f"  设备: {args.device}, FP16: {args.half}")

    model = YOLO(model_path)

    t0 = time.perf_counter()
    results = model.predict(
        source=args.source,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        device=args.device,
        half=args.half,
        show=args.show,
        save=not args.nosave,
        save_txt=args.save_txt,
        save_crop=args.save_crop,
        project=args.project,
        name=args.name,
        exist_ok=True,
    )
    elapsed = time.perf_counter() - t0

    # 汇总
    total_objs = sum(r.boxes.shape[0] if r.boxes is not None else 0 for r in results)
    n_images = len(results)
    print(f"\n[推理完成] {n_images} 张图片, {total_objs} 个目标, "
          f"耗时 {elapsed:.2f}s ({elapsed/n_images*1000:.1f}ms/张)")

    if not args.nosave:
        print(f"  结果保存至: {results[0].save_dir}")


if __name__ == "__main__":
    main()
