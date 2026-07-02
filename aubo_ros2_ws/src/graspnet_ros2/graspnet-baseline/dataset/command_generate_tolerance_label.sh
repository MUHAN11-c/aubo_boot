# 将 dataset_root 改为你的数据集根目录（解压后的 graspnet 目录）
python generate_tolerance_label.py --dataset_root "$(pwd)/data/Benchmark/graspnet" --num_workers 8
