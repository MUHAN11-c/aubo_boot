# checkpoint-rs.tar 为 RealSense 预训练权重，使用 --camera realsense
DATASET_ROOT="$(dirname "$0")/dataset/data/Benchmark/graspnet"
CUDA_VISIBLE_DEVICES=0 python test.py --dump_dir logs/dump_rs --checkpoint_path logs/log_kn/checkpoint-rs.tar --camera realsense --dataset_root "$DATASET_ROOT"
# 快速推理（不做碰撞检测）可加: --collision_thresh -1
# CUDA_VISIBLE_DEVICES=0 python test.py --dump_dir logs/dump_kn --checkpoint_path logs/log_kn/checkpoint.tar --camera kinect --dataset_root "$DATASET_ROOT"
