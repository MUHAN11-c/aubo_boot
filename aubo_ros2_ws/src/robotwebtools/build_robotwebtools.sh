#!/usr/bin/env bash
# 说明：
# 1) 本脚本仅做“本地离线构建”，不做 git clone / pull。
# 2) 仅依赖本地已有源码、node_modules 或 npm 本地缓存。
# 3) 任一步骤失败立即退出，避免产生“部分成功”的不一致状态。
#
# 使用方法：
#   bash ./build_robotwebtools.sh
#
# 常见问题排查：
# 1) 报错“缺少目录”：请先确认 roslibjs/ros3djs/ros2djs 三个源码目录都已存在。
# 2) 报错“离线安装失败”：说明本地 npm 缓存不足且 node_modules 不存在；
#    需要先在可联网环境准备依赖，或把已安装好的 node_modules 同步到本地。
# 3) 构建结束后会自动执行 copy_runtime_js_assets.sh，把可调用 JS 复制到统一目录。
set -euo pipefail

# 脚本所在目录即 robotwebtools 根目录。
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 校验目录是否存在。
# 典型用途：确认三个仓库目录或包目录已提前准备好。
require_dir() {
  local dir="$1"
  if [[ ! -d "${dir}" ]]; then
    echo "[错误] 缺少目录: ${dir}"
    echo "[错误] 本脚本只构建已存在源码，不会拉取仓库。"
    exit 1
  fi
}

# 在离线模式下准备依赖：
# - 若本地已有 node_modules，直接复用（完全本地）。
# - 若没有，则尝试 npm --offline（只能使用本地 npm 缓存，不访问网络）。
# - 失败则直接退出，并提示先准备离线依赖。
ensure_offline_dependencies() {
  local pkg_dir="$1"
  require_dir "${pkg_dir}"

  if [[ -d "${pkg_dir}/node_modules" ]]; then
    echo "[信息] 检测到本地 node_modules，直接复用: ${pkg_dir}"
    return 0
  fi

  echo "[信息] 未找到 node_modules，尝试使用本地缓存离线安装: ${pkg_dir}"
  if ! npm install --offline --no-audit --fund=false; then
    echo "[错误] 离线依赖安装失败: ${pkg_dir}"
    echo "[错误] 请先准备本地 npm 缓存，或预先放置 node_modules 后重试。"
    exit 1
  fi
}

# 构建 roslibjs（monorepo 内的 packages/roslib）。
# 官方构建命令：npm run build
build_roslibjs() {
  local pkg_dir="${ROOT_DIR}/roslibjs/packages/roslib"
  local vendor_dir="${pkg_dir}/vendor"
  local backup_dir="/tmp/roslib_vendor_backup_${USER:-user}_$$"
  require_dir "${pkg_dir}"
  echo "[信息] 开始构建 roslibjs: ${pkg_dir}"
  cd "${pkg_dir}"
  ensure_offline_dependencies "${pkg_dir}"

  # roslib 本地化会在 packages/roslib 下放置 vendor。
  # 构建时 eslint 会扫描该目录导致失败，因此构建前临时移走，构建后恢复。
  if [[ -d "${vendor_dir}" ]]; then
    rm -rf "${backup_dir}"
    mv "${vendor_dir}" "${backup_dir}"
  fi

  if ! npm run build; then
    if [[ -d "${backup_dir}" ]]; then
      mv "${backup_dir}" "${vendor_dir}"
    fi
    return 1
  fi

  if [[ -d "${backup_dir}" ]]; then
    mv "${backup_dir}" "${vendor_dir}"
  fi
}

# 构建 ros3djs。
# 官方构建命令：npm run build（内部执行 grunt build）
build_ros3djs() {
  local pkg_dir="${ROOT_DIR}/ros3djs"
  require_dir "${pkg_dir}"
  echo "[信息] 开始构建 ros3djs: ${pkg_dir}"
  cd "${pkg_dir}"
  ensure_offline_dependencies "${pkg_dir}"
  npm run build
}

# 构建 ros2djs。
# 官方构建命令：npm run build（内部执行 grunt build）
build_ros2djs() {
  local pkg_dir="${ROOT_DIR}/ros2djs"
  require_dir "${pkg_dir}"
  echo "[信息] 开始构建 ros2djs: ${pkg_dir}"
  cd "${pkg_dir}"
  ensure_offline_dependencies "${pkg_dir}"
  npm run build
}

copy_runtime_assets() {
  local script_path="${ROOT_DIR}/copy_runtime_js_assets.sh"
  if [[ ! -f "${script_path}" ]]; then
    echo "[错误] 未找到复制脚本: ${script_path}"
    exit 1
  fi
  echo "[信息] 开始汇总可调用 JS 资产"
  bash "${script_path}"
}

# npm 离线策略：
# - OFFLINE=true：禁止 npm 走网络。
# - PREFER_OFFLINE=true：优先使用本地缓存（与 OFFLINE 配合更稳妥）。
# - AUDIT/FUND=false：关闭网络相关提示与检查，减少干扰。
export NPM_CONFIG_OFFLINE=true
export NPM_CONFIG_PREFER_OFFLINE=true
export NPM_CONFIG_AUDIT=false
export NPM_CONFIG_FUND=false

echo "[信息] RobotWebTools 构建根目录: ${ROOT_DIR}"
echo "[信息] 已启用离线模式: npm 不会访问网络。"
echo "[信息] 默认执行全量构建: roslibjs + ros3djs + ros2djs"

build_roslibjs
build_ros3djs
build_ros2djs
copy_runtime_assets

echo "[信息] 全量构建完成。"
