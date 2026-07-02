# 数据集根目录：解压后包含 scenes/、grasp_label/、collision_label/ 的目录
DATASET_ROOT="$(dirname "$0")/dataset/data/Benchmark/graspnet"
CUDA_VISIBLE_DEVICES=0 python train.py --camera realsense --log_dir logs/log_rs --batch_size 2 --dataset_root "$DATASET_ROOT"
# CUDA_VISIBLE_DEVICES=0 python train.py --camera kinect --log_dir logs/log_kn --batch_size 2 --dataset_root /data/Benchmark/graspnet
