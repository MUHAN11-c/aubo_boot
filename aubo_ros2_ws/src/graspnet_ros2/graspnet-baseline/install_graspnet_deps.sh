#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PIP_CMD=("${PYTHON_BIN}" -m pip)
CLEAN_BUILD=0
INSTALL_COMMON_DEPS=0
SKIP_CUDA_EXT=0

usage() {
  cat <<'EOF'
用法: ./install_graspnet_deps.sh [选项]

安装 GraspNet baseline 的 editable 包与 CUDA 扩展。
脚本可从任意目录执行，所有项目路径都相对脚本所在目录解析。

选项:
  --clean                 编译前清理本地 build 与 egg-info 产物
  --install-common-deps   安装 torch/torchvision 之外的常用 Python 依赖
  --skip-cuda-ext         跳过 pointnet2/knn CUDA 扩展编译（仅 Python 代码变更时用）
  -h, --help              显示帮助

环境变量:
  PYTHON_BIN              使用的 Python 可执行文件，默认 python3
  MAX_JOBS                torch cpp_extension 并行编译任务数
  TORCH_CUDA_ARCH_LIST    无显示 GPU 编译时的 CUDA 架构，例如 "8.9"
EOF
}

# ── 工具函数 ───────────────────────────────────────────────

log()  { printf '[graspnet-install] %s\n' "$*"; }
warn() { printf '[graspnet-install][WARN] %s\n' "$*" >&2; }
die()  { printf '[graspnet-install][ERROR] %s\n' "$*" >&2; exit 1; }
step() { printf '\n[graspnet-install] === %s ===\n' "$*"; }

now_ts() { date +%s; }

elapsed() {
  local s=$(( $(now_ts) - $1 ))
  printf '%dm%ds' $((s/60)) $((s%60))
}

require_dir()  { [[ -d "$1" ]] || die "缺少目录: $1"; }
require_file() { [[ -f "$1" ]] || die "缺少文件: $1"; }

# ── 参数解析 ───────────────────────────────────────────────

while (($#)); do
  case "$1" in
    --clean)              CLEAN_BUILD=1 ;;
    --install-common-deps) INSTALL_COMMON_DEPS=1 ;;
    --skip-cuda-ext)      SKIP_CUDA_EXT=1 ;;
    -h|--help)            usage; exit 0 ;;
    *)                    die "未知选项: $1" ;;
  esac
  shift
done

# ── 预检: 系统依赖（批量检查，一次列出所有缺失项）───────

MISSING_SYS=()
check_sys() { command -v "$1" >/dev/null 2>&1 || MISSING_SYS+=("$1"); }

check_sys "${PYTHON_BIN}"
check_sys c++
check_sys ninja
check_sys nvcc

if ((${#MISSING_SYS[@]})); then
  echo "[graspnet-install] 以下系统依赖缺失:"
  printf '  - %s\n' "${MISSING_SYS[@]}"
  echo ""

  local apt_pkgs=()
  for dep in "${MISSING_SYS[@]}"; do
    case "$dep" in
      python3) apt_pkgs+=("python3") ;;
      c++)     apt_pkgs+=("build-essential") ;;
      ninja)   apt_pkgs+=("ninja-build") ;;
      nvcc)    ;;  # CUDA toolkit 需手动安装
    esac
  done

  if [[ " ${MISSING_SYS[*]} " == *" nvcc "* ]]; then
    echo "  [nvcc] 需要安装 CUDA Toolkit: https://developer.nvidia.com/cuda-downloads"
  fi
  if ((${#apt_pkgs[@]})); then
    echo "  一键安装: sudo apt-get install -y ${apt_pkgs[*]}"
  fi
  exit 1
fi

# ── 预检: ccache ───────────────────────────────────────────

if command -v ccache >/dev/null 2>&1; then
  log "检测到 ccache，CUDA 扩展重编译将命中缓存"
  export CCACHE_BASEDIR="${SCRIPT_DIR}"
  export CMAKE_C_COMPILER_LAUNCHER=ccache
  export CMAKE_CXX_COMPILER_LAUNCHER=ccache
  export CMAKE_CUDA_COMPILER_LAUNCHER=ccache
else
  warn "未检测到 ccache，建议安装以加速重编译: sudo apt-get install -y ccache"
fi

# ── 预检: 关键文件/目录 ────────────────────────────────────

require_dir  "${SCRIPT_DIR}/models"
require_dir  "${SCRIPT_DIR}/utils"
require_dir  "${SCRIPT_DIR}/dataset"
require_file "${SCRIPT_DIR}/setup.py"
require_file "${SCRIPT_DIR}/pointnet2/setup.py"
require_file "${SCRIPT_DIR}/knn/setup.py"
require_file "${SCRIPT_DIR}/graspnetAPI/setup.py"

# ── 环境信息 ───────────────────────────────────────────────

log "baseline 目录: ${SCRIPT_DIR}"
log "Python: $(${PYTHON_BIN} -c 'import sys; print(sys.executable)')"
log "Python 版本: $(${PYTHON_BIN} -c 'import sys; print(\".\".join(map(str, sys.version_info[:3])))')"
log "pip 版本: $(${PIP_CMD[@]} --version)"
log "nvcc: $(nvcc --version 2>/dev/null | sed -n 's/^Cuda compilation tools, release //p' | head -n 1 || echo '未知')"
log "ninja: $(ninja --version 2>/dev/null || echo '未知')"
log "MAX_JOBS=${MAX_JOBS:-$(nproc)}"

# ── Python 构建基础依赖（缺失时自动安装，不直接报错）─────

"${PYTHON_BIN}" - <<'PY'
import importlib.util, subprocess, sys

missing = [name for name in ("pip", "setuptools", "wheel") if importlib.util.find_spec(name) is None]
if missing:
    print(f"[graspnet-install] 自动安装缺失的构建模块: {', '.join(missing)}")
    subprocess.check_call([sys.executable, "-m", "pip", "install", *missing])

if sys.version_info < (3, 8):
    raise SystemExit("需要 Python >= 3.8")
PY

# ── torch 检测 ─────────────────────────────────────────────

"${PYTHON_BIN}" - <<'PY'
import os, sys, torch
print(f"[graspnet-install] torch: {torch.__version__}, torch CUDA: {torch.version.cuda}")
print(f"[graspnet-install] CUDA 可用: {torch.cuda.is_available()}")
if not torch.cuda.is_available() and not os.environ.get("TORCH_CUDA_ARCH_LIST"):
    raise SystemExit(
        "torch.cuda.is_available() 为 false 且未设置 TORCH_CUDA_ARCH_LIST。"
        "请在 GPU 机器上运行，或设置 TORCH_CUDA_ARCH_LIST 进行无头编译。"
    )
PY

TORCH_LIB_DIR="$("${PYTHON_BIN}" -c 'import pathlib, torch; print(pathlib.Path(torch.__file__).resolve().parent / "lib")')"
export MAX_JOBS="${MAX_JOBS:-$(nproc)}"

# ── 清理 ───────────────────────────────────────────────────

if [[ "${CLEAN_BUILD}" == "1" ]]; then
  step "清理 build/egg-info 产物"
  for d in \
    "${SCRIPT_DIR}/build"              "${SCRIPT_DIR}/graspnet_baseline.egg-info" \
    "${SCRIPT_DIR}/pointnet2/build"    "${SCRIPT_DIR}/pointnet2/pointnet2.egg-info" \
    "${SCRIPT_DIR}/knn/build"          "${SCRIPT_DIR}/knn/knn_pytorch.egg-info" \
    "${SCRIPT_DIR}/graspnetAPI/build"  "${SCRIPT_DIR}/graspnetAPI/graspnetAPI.egg-info"; do
    rm -rf "$d"
  done
fi

# ── 常用依赖 ───────────────────────────────────────────────

if [[ "${INSTALL_COMMON_DEPS}" == "1" ]]; then
  step "安装常用 Python 依赖"
  "${PIP_CMD[@]}" install open3d scipy Pillow numpy trimesh
fi

# ── editable 包批量安装 ────────────────────────────────────

step "安装 editable 包"
mkdir -p "${SCRIPT_DIR}/pointnet2/pointnet2"
touch "${SCRIPT_DIR}/pointnet2/pointnet2/__init__.py"
mkdir -p "${SCRIPT_DIR}/knn/knn_pytorch"
touch "${SCRIPT_DIR}/knn/knn_pytorch/__init__.py"

_t0=$(now_ts)

# 基础包 + graspnetAPI 合并为一次 pip install
"${PIP_CMD[@]}" install -e "${SCRIPT_DIR}" -e "${SCRIPT_DIR}/graspnetAPI"

if [[ "${SKIP_CUDA_EXT}" != "1" ]]; then
  # 快速检测已安装的 CUDA 扩展，跳过不必要的重编译
  SKIP_POINTNET=0
  SKIP_KNN=0
  if LD_LIBRARY_PATH="${TORCH_LIB_DIR}:${LD_LIBRARY_PATH:-}" "${PYTHON_BIN}" -c 'import pointnet2._ext' 2>/dev/null; then
    log "pointnet2 已可 import，跳过编译"
    SKIP_POINTNET=1
  fi
  if LD_LIBRARY_PATH="${TORCH_LIB_DIR}:${LD_LIBRARY_PATH:-}" "${PYTHON_BIN}" -c 'import knn_pytorch.knn_pytorch' 2>/dev/null; then
    log "knn 已可 import，跳过编译"
    SKIP_KNN=1
  fi

  if [[ "${SKIP_POINTNET}" == "0" ]]; then
    log "编译安装 pointnet2 CUDA 扩展 (MAX_JOBS=${MAX_JOBS})"
    "${PIP_CMD[@]}" install --no-build-isolation -e "${SCRIPT_DIR}/pointnet2" --config-settings editable_mode=compat
  fi
  if [[ "${SKIP_KNN}" == "0" ]]; then
    log "编译安装 knn 扩展 (MAX_JOBS=${MAX_JOBS})"
    "${PIP_CMD[@]}" install --no-build-isolation -e "${SCRIPT_DIR}/knn" --config-settings editable_mode=compat
  fi
else
  warn "跳过 CUDA 扩展编译 (--skip-cuda-ext)，运行时需已有 .so"
fi

log "editable 包安装耗时: $(elapsed $_t0)"

# ── 验证导入链 ─────────────────────────────────────────────

step "验证导入链"
LD_LIBRARY_PATH="${TORCH_LIB_DIR}:${LD_LIBRARY_PATH:-}" "${PYTHON_BIN}" - <<'PY'
import torch
import models
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
from graspnetAPI import GraspGroup

status = []
try:
    import pointnet2._ext as p2
    status.append(f"pointnet2._ext={p2.__file__}")
except Exception as e:
    status.append(f"pointnet2._ext=FAILED ({e})")

try:
    import knn_pytorch.knn_pytorch as knn
    status.append(f"knn_pytorch={knn.__file__}")
except Exception as e:
    status.append(f"knn_pytorch=FAILED ({e})")

print(f"[graspnet-install] torch={torch.__version__} cuda_available={torch.cuda.is_available()}")
for s in status:
    print(f"[graspnet-install] {s}")
PY

log "完成"
log "运行时提示: export LD_LIBRARY_PATH=\"${TORCH_LIB_DIR}:\${LD_LIBRARY_PATH:-}\""
