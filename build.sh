#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRASPNET_DIR="${SCRIPT_DIR}/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline"
GREEN='\033[0;32m'; NC='\033[0m'
ok() { echo -e "${GREEN}  ✓${NC} $1"; }

export PATH="/usr/local/cuda/bin:$PATH"
export MAX_JOBS="${MAX_JOBS:-$(nproc)}"

TORCH_LIB="$(python3 -c 'import pathlib, torch; print(pathlib.Path(torch.__file__).resolve().parent / "lib")')"
export LD_LIBRARY_PATH="${TORCH_LIB}:${LD_LIBRARY_PATH:-}"

echo -e "${GREEN}== GraspNet 编译 ==${NC}"

for cmd in python3 nvcc ninja; do
    command -v "$cmd" >/dev/null || { echo "❌ 缺失: $cmd"; exit 1; }
done

# 清理
rm -rf "${GRASPNET_DIR}/build"         "${GRASPNET_DIR}/graspnet_baseline.egg-info"
rm -rf "${GRASPNET_DIR}/pointnet2/build" "${GRASPNET_DIR}/pointnet2/pointnet2.egg-info"
rm -rf "${GRASPNET_DIR}/knn/build"       "${GRASPNET_DIR}/knn/knn_pytorch.egg-info"
rm -rf "${GRASPNET_DIR}/graspnetAPI/build" "${GRASPNET_DIR}/graspnetAPI/graspnetAPI.egg-info"

mkdir -p "${GRASPNET_DIR}/pointnet2/pointnet2"
touch "${GRASPNET_DIR}/pointnet2/pointnet2/__init__.py"
mkdir -p "${GRASPNET_DIR}/knn/knn_pytorch"
touch "${GRASPNET_DIR}/knn/knn_pytorch/__init__.py"

echo "→ graspnet-baseline + graspnetAPI ..."
pip3 install -e "${GRASPNET_DIR}" -e "${GRASPNET_DIR}/graspnetAPI" -q
ok "graspnet-baseline + graspnetAPI"

echo "→ pointnet2 ..."
pip3 install --no-build-isolation -e "${GRASPNET_DIR}/pointnet2" -q
ok "pointnet2"

echo "→ knn ..."
pip3 install --no-build-isolation -e "${GRASPNET_DIR}/knn" -q
ok "knn"

echo "→ 验证 ..."
cd /tmp
LD_LIBRARY_PATH="${TORCH_LIB}:${LD_LIBRARY_PATH:-}" python3 - <<PY
import logging; logging.getLogger().setLevel(logging.ERROR)
from graspnetAPI import GraspGroup
import models
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
import pointnet2._ext as p2
import knn_pytorch.knn_pytorch as knn
print(f"  pointnet2._ext:  {p2.__file__}")
print(f"  knn_pytorch:     {knn.__file__}")
print("  ✅ 全部通过")
PY

echo -e "${GREEN}== 编译完成 ==${NC}"
