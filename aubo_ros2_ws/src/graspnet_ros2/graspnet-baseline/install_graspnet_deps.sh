#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PIP_CMD=("${PYTHON_BIN}" -m pip)
CLEAN_BUILD=0
INSTALL_COMMON_DEPS=0

usage() {
  cat <<'EOF'
用法: ./install_graspnet_deps.sh [选项] 喵~

安装 GraspNet baseline 的 editable 包与 CUDA 扩展喵~
脚本可从任意目录执行，所有项目路径都相对脚本所在目录解析喵~

选项喵~
  --clean                 编译前清理本地 build 与 egg-info 产物喵~
  --install-common-deps   安装 torch/torchvision 之外的常用 Python 依赖喵~
  -h, --help              显示帮助喵~

环境变量喵~
  PYTHON_BIN              使用的 Python 可执行文件，默认 python3 喵~
  MAX_JOBS                torch cpp_extension 并行编译任务数喵~
  TORCH_CUDA_ARCH_LIST    无显示 GPU 编译时的 CUDA 架构，例如 "8.9" 喵~
EOF
}

log() {
  printf '[graspnet-install] %s\n' "$*"
}

die() {
  printf '[graspnet-install][ERROR] %s\n' "$*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --clean)
      CLEAN_BUILD=1
      ;;
    --install-common-deps)
      INSTALL_COMMON_DEPS=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "未知选项: $1 喵~"
      ;;
  esac
  shift
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1 喵~"
}

require_dir() {
  [[ -d "$1" ]] || die "缺少目录: $1 喵~"
}

require_file() {
  [[ -f "$1" ]] || die "缺少文件: $1 喵~"
}

require_cmd "${PYTHON_BIN}"
require_cmd c++
require_cmd ninja
require_cmd nvcc

require_file "${SCRIPT_DIR}/setup.py"
require_dir "${SCRIPT_DIR}/models"
require_dir "${SCRIPT_DIR}/utils"
require_dir "${SCRIPT_DIR}/dataset"
require_file "${SCRIPT_DIR}/pointnet2/setup.py"
require_file "${SCRIPT_DIR}/knn/setup.py"
require_file "${SCRIPT_DIR}/graspnetAPI/setup.py"

log "baseline 目录: ${SCRIPT_DIR} 喵~"
log "Python: $(${PYTHON_BIN} -c 'import sys; print(sys.executable)') 喵~"
log "Python 版本: $(${PYTHON_BIN} -c 'import sys; print(".".join(map(str, sys.version_info[:3])))') 喵~"
log "pip 版本: $(${PIP_CMD[@]} --version) 喵~"
log "nvcc 版本: $(nvcc --version | sed -n 's/^Cuda compilation tools, release //p' | head -n 1) 喵~"

"${PYTHON_BIN}" - <<'PY'
import importlib.util
import sys

missing = [name for name in ("pip", "setuptools", "wheel") if importlib.util.find_spec(name) is None]
if missing:
    raise SystemExit("缺少 Python 构建模块: " + ", ".join(missing) + " 喵~")

if sys.version_info < (3, 8):
    raise SystemExit("需要 Python >= 3.8 喵~")
PY

if [[ "${INSTALL_COMMON_DEPS}" == "1" ]]; then
  log "安装常用 Python 依赖，并保持 torch/torchvision 不变喵~"
  "${PIP_CMD[@]}" install open3d scipy Pillow numpy trimesh
fi

"${PYTHON_BIN}" - <<'PY'
import os
import sys

try:
    import torch
except Exception as exc:
    raise SystemExit(
        "无法导入 torch，请先安装匹配本机 CUDA 的 torch 版本喵~ "
        f"原始错误: {exc!r} 喵~"
    )

print(f"[graspnet-install] torch: {torch.__version__}, torch CUDA: {torch.version.cuda} 喵~")
print(f"[graspnet-install] CUDA 可用: {torch.cuda.is_available()} 喵~")

if not torch.cuda.is_available() and not os.environ.get("TORCH_CUDA_ARCH_LIST"):
    raise SystemExit(
        "torch.cuda.is_available() 为 false 且未设置 TORCH_CUDA_ARCH_LIST 喵~ "
        "请在 GPU 机器上运行，或为无头编译显式设置 TORCH_CUDA_ARCH_LIST 喵~"
    )
PY

export MAX_JOBS="${MAX_JOBS:-$(nproc)}"
log "MAX_JOBS=${MAX_JOBS} 喵~"

if [[ "${CLEAN_BUILD}" == "1" ]]; then
  log "清理本地 build 与 egg-info 产物喵~"
  rm -rf \
    "${SCRIPT_DIR}/build" \
    "${SCRIPT_DIR}/graspnet_baseline.egg-info" \
    "${SCRIPT_DIR}/pointnet2/build" \
    "${SCRIPT_DIR}/pointnet2/pointnet2.egg-info" \
    "${SCRIPT_DIR}/knn/build" \
    "${SCRIPT_DIR}/knn/knn_pytorch.egg-info" \
    "${SCRIPT_DIR}/graspnetAPI/build" \
    "${SCRIPT_DIR}/graspnetAPI/graspnetAPI.egg-info"
fi

log "准备扩展包内层目录喵~"
mkdir -p "${SCRIPT_DIR}/pointnet2/pointnet2"
touch "${SCRIPT_DIR}/pointnet2/pointnet2/__init__.py"
mkdir -p "${SCRIPT_DIR}/knn/knn_pytorch"
touch "${SCRIPT_DIR}/knn/knn_pytorch/__init__.py"

log "安装 graspnet-baseline editable 包喵~"
"${PIP_CMD[@]}" install -e "${SCRIPT_DIR}"

log "安装 pointnet2 CUDA 扩展喵~"
"${PIP_CMD[@]}" install -e "${SCRIPT_DIR}/pointnet2"

log "安装 knn 扩展喵~"
"${PIP_CMD[@]}" install -e "${SCRIPT_DIR}/knn"

log "安装 graspnetAPI editable 包喵~"
"${PIP_CMD[@]}" install trimesh
"${PIP_CMD[@]}" install -e "${SCRIPT_DIR}/graspnetAPI"

TORCH_LIB_DIR="$("${PYTHON_BIN}" - <<'PY'
import pathlib
import torch
print(pathlib.Path(torch.__file__).resolve().parent / "lib")
PY
)"

log "验证导入链喵~"
LD_LIBRARY_PATH="${TORCH_LIB_DIR}:${LD_LIBRARY_PATH:-}" "${PYTHON_BIN}" - <<'PY'
import torch
import models
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
from graspnetAPI import GraspGroup
import pointnet2._ext as pointnet2_ext
import knn_pytorch.knn_pytorch as knn_ext

print(f"[graspnet-install] OK torch={torch.__version__} cuda_available={torch.cuda.is_available()} 喵~")
print(f"[graspnet-install] OK pointnet2._ext={pointnet2_ext.__file__} 喵~")
print(f"[graspnet-install] OK knn_pytorch.knn_pytorch={knn_ext.__file__} 喵~")
PY

log "完成喵~"
log "运行时提示: export LD_LIBRARY_PATH=\"${TORCH_LIB_DIR}:\${LD_LIBRARY_PATH:-}\" 喵~"
