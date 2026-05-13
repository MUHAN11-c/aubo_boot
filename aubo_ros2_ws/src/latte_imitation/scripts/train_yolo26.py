#!/usr/bin/env python3
"""YOLOv26 训练脚本 — 基于 ultralytics 框架。

用法:
    python3 scripts/train_yolo26.py                          # yolo26n, 默认参数
    python3 scripts/train_yolo26.py --model m --batch 64     # yolo26m, batch=64
    python3 scripts/train_yolo26.py --model x --epochs 500   # yolo26x, 500 epochs
    python3 scripts/train_yolo26.py --data custom.yaml       # 自定义数据集
    python3 scripts/train_yolo26.py --device cpu             # CPU 训练
"""

import argparse
import os
import sys

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)


def get_weight_path(model_size: str) -> str:
    """获取预训练权重路径，不存在则用 ultralytics 内置下载。"""
    pkg_weight = os.path.join(_PKG_DIR, f"yolo26{model_size}.pt")
    if os.path.exists(pkg_weight):
        return pkg_weight
    return f"yolo26{model_size}.pt"  # 让 ultralytics 自动下载


def get_data_path(dataset: str) -> str:
    """解析数据集 yaml 路径。"""
    if os.path.isabs(dataset) or os.path.exists(dataset):
        return dataset
    pkg_data = os.path.join(_PKG_DIR, dataset)
    if os.path.exists(pkg_data):
        return pkg_data
    return dataset


def main():
    parser = argparse.ArgumentParser(description="YOLOv26 训练")
    parser.add_argument("--model", default="n", choices=list("nsmlx"),
                        help="模型规模 (default: n)")
    parser.add_argument("--data", default="datasets/coco128.yaml",
                        help="数据集 yaml 路径 (default: datasets/coco128.yaml)")
    parser.add_argument("--epochs", type=int, default=300,
                        help="训练轮数 (default: 300)")
    parser.add_argument("--batch", type=int, default=-1,
                        help="batch size，-1=自动 (default: -1)")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="输入尺寸 (default: 640)")
    parser.add_argument("--device", default="0",
                        help="设备: 0/1/...=GPU 编号, cpu=CPU (default: 0)")
    parser.add_argument("--workers", type=int, default=8,
                        help="DataLoader workers (default: 8)")
    parser.add_argument("--lr0", type=float, default=0.01,
                        help="初始学习率 (default: 0.01)")
    parser.add_argument("--patience", type=int, default=100,
                        help="早停 patience (default: 100)")
    parser.add_argument("--name", default="train",
                        help="实验名称 (default: train)")
    parser.add_argument("--project", default="runs/yolo26",
                        help="输出目录 (default: runs/yolo26)")
    parser.add_argument("--resume", action="store_true",
                        help="从上次 checkpoint 恢复")
    parser.add_argument("--half", action="store_true",
                        help="FP16 半精度训练（节省显存）")
    parser.add_argument("--cache", action="store_true",
                        help="数据集缓存到 RAM（加速训练但占用内存）")
    args = parser.parse_args()

    from ultralytics import YOLO

    model_path = get_weight_path(args.model)
    data_path = get_data_path(args.data)

    print(f"[训练配置]")
    print(f"  模型: {model_path} (yolo26{args.model})")
    print(f"  数据集: {data_path}")
    print(f"  Epochs: {args.epochs}")
    print(f"  Batch: {'auto' if args.batch == -1 else args.batch}")
    print(f"  ImgSz: {args.imgsz}")
    print(f"  Device: {args.device}")
    print(f"  FP16: {args.half}")
    print(f"  输出: {args.project}/{args.name}")

    model = YOLO(model_path)

    results = model.train(
        data=data_path,
        epochs=args.epochs,
        batch=args.batch if args.batch > 0 else 16,
        imgsz=args.imgsz,
        device=args.device,
        workers=args.workers,
        lr0=args.lr0,
        patience=args.patience,
        name=args.name,
        project=args.project,
        resume=args.resume,
        half=args.half,
        cache=args.cache,
        exist_ok=True,
    )

    print(f"\n[训练完成] 最佳权重: {results.save_dir}/weights/best.pt")


if __name__ == "__main__":
    main()
