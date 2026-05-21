#!/usr/bin/env bash
# ============================================================
# build_robotwebtools.sh — RobotWebTools 三库全量构建脚本
# ============================================================
#
# 【功能】
#   对 roslibjs / ros3djs / ros2djs 三个前端库执行官方构建流程
#   （npm run build / grunt build），最后自动调用 copy_runtime_js_assets.sh
#   将产物汇总到 runtime_js_assets/ 目录。
#
# 【npm 依赖策略 — 优先离线，缺失时自动联网】
#   1. 若本地已有 node_modules/ → 直接复用（零网络）
#   2. 若无 node_modules/ → npm install --prefer-offline 安装
#      （优先使用本地缓存，缓存缺失的包自动从 npm registry 下载）喵~
#   3. 完全不联网的环境：请提前在有网机器上 npm install 并同步 node_modules 喵~
#
# 【设计原则】
#   - 仅构建，不拉取（不执行 git clone/pull）
#   - 优先离线（NPM_CONFIG_PREFER_OFFLINE=true，优先本地缓存）
#   - 缺失自动下载（不再硬性禁止联网，缓存不足时回退在线安装）喵~
#   - 原子失败（set -euo pipefail，任一步骤失败立即退出）
#   - 构建完成后自动汇总产物
#
# 【前置条件】
#   roslibjs/、ros3djs/、ros2djs/ 三个源码目录已存在喵~
#
# 【使用方法】
#   cd /home/wjz/aubo_boot/aubo_ros2_ws/src/robotwebtools
#   bash build_robotwebtools.sh
#
# 【构建产物位置】
#   - roslibjs: roslibjs/packages/roslib/dist/
#   - ros3djs:  ros3djs/build/
#   - ros2djs:  ros2djs/build/
#   - 汇总输出: runtime_js_assets/（由 copy_runtime_js_assets.sh 生成）
#
# 【常见问题】
#   Q: 报错"缺少目录" → 确认三个源码目录已存在，本脚本不拉取仓库喵~
#   Q: npm install 失败（无网络 + 无缓存）→ 需在有网环境预装依赖后同步 node_modules 喵~
#   Q: roslib 构建 eslint 报错 → 脚本会自动临时移走 vendor/ 避免扫描，构建后恢复喵~
# ============================================================

set -euo pipefail

# ============================================================
# 全局变量
# ============================================================
# ROOT_DIR: 脚本所在目录，即 robotwebtools 包根目录
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# npm 策略环境变量：
#   PREFER_OFFLINE=true  → 优先使用本地缓存，缓存缺失时自动从 registry 下载
#   AUDIT=false          → 关闭 npm audit 安全检查（加速安装）
#   FUND=false           → 关闭 npm fund 赞助提示（减少输出噪音）
export NPM_CONFIG_PREFER_OFFLINE=true
export NPM_CONFIG_AUDIT=false
export NPM_CONFIG_FUND=false

# ============================================================
# 工具函数
# ============================================================

# require_dir — 校验目录存在，不存在则报错退出
# 参数: $1 — 待校验的目录路径
# 用途: 构建前确认源码目录已就绪（本脚本不负责拉取仓库）喵~
require_dir() {
  local dir="$1"
  if [[ ! -d "${dir}" ]]; then
    echo "[错误] 缺少目录: ${dir}"
    echo "[错误] 本脚本只构建已存在的源码，不会自动拉取仓库喵~"
    echo "[提示] 请先确保 roslibjs/ ros3djs/ ros2djs/ 三个目录已就绪喵~"
    exit 1
  fi
}

# ensure_dependencies — 确保目标包目录的 npm 依赖可用
# 参数: $1 — 包目录路径（含 package.json）
# 策略（优先离线，缺失时自动联网）:
#   1. 若本地已有 node_modules/ → 直接复用（零网络）
#   2. 若无 node_modules/ → npm install --prefer-offline
#      （优先本地缓存，缓存缺失的包自动从 npm registry 下载）喵~
#   3. 若联网 + 离线均失败 → 报错退出喵~
ensure_dependencies() {
  local pkg_dir="$1"
  require_dir "${pkg_dir}"

  if [[ -d "${pkg_dir}/node_modules" ]]; then
    echo "[信息] 检测到本地 node_modules，直接复用: ${pkg_dir}"
    return 0
  fi

  echo "[信息] 未找到 node_modules，安装依赖（优先本地缓存，缺失时联网下载）: ${pkg_dir}"
  cd "${pkg_dir}"
  if ! npm install --prefer-offline --no-audit --fund=false; then
    echo "[错误] 依赖安装失败: ${pkg_dir}"
    echo "[错误] 请检查网络连接，或手动执行 npm install 排查问题喵~"
    exit 1
  fi
}

# ============================================================
# 构建函数 — 三个库各自独立的构建流程
# ============================================================

# build_roslibjs — 构建 roslibjs（monorepo 内 packages/roslib）
# 输入: roslibjs/packages/roslib/（源码 + node_modules）
# 输出: roslibjs/packages/roslib/dist/（构建产物 JS 文件）
# 官方构建命令: npm run build
# 特殊处理: 构建前临时移走 vendor/ 目录，避免 eslint 扫描报错，构建后恢复喵~
build_roslibjs() {
  local pkg_dir="${ROOT_DIR}/roslibjs/packages/roslib"
  local vendor_dir="${pkg_dir}/vendor"
  local backup_dir="/tmp/roslib_vendor_backup_${USER:-user}_$$"
  require_dir "${pkg_dir}"

  echo "============================================================"
  echo "[构建] 开始构建 roslibjs"
  echo "[构建] 源码目录: ${pkg_dir}"
  echo "============================================================"
  cd "${pkg_dir}"
  ensure_dependencies "${pkg_dir}"

  # roslibjs 的 vendor/ 目录存放了 ESM 依赖的本地副本（bson/cbor2/ws 等）。
  # 构建时 eslint 会扫描 vendor/ 导致报错，因此构建前临时移走，构建后恢复喵~
  if [[ -d "${vendor_dir}" ]]; then
    echo "[信息] 检测到 vendor/ 目录，构建前临时移走以避免 eslint 扫描报错喵~"
    rm -rf "${backup_dir}"
    mv "${vendor_dir}" "${backup_dir}"
  fi

  if ! npm run build; then
    echo "[错误] roslibjs 构建失败喵~"
    # 构建失败也要恢复 vendor/，保持目录原样
    if [[ -d "${backup_dir}" ]]; then
      mv "${backup_dir}" "${vendor_dir}"
    fi
    return 1
  fi

  # 构建成功，恢复 vendor/ 目录
  if [[ -d "${backup_dir}" ]]; then
    mv "${backup_dir}" "${vendor_dir}"
    echo "[信息] 已恢复 vendor/ 目录喵~"
  fi
  echo "[完成] roslibjs 构建成功喵~"
}

# build_ros3djs — 构建 ros3djs（Three.js 3D 可视化库）
# 输入: ros3djs/（源码 + node_modules）
# 输出: ros3djs/build/（ros3d.js / ros3d.min.js / ros3d.esm.js / ros3d.cjs.js）
# 官方构建命令: npm run build（内部执行 grunt build → rollup 打包）
# 注意: 本项目已修改 ros3djs/src/urdf/Urdf.js，将 ROSLIB.URDF_* 改为
#        ROSLIB.UrdfType.* 以兼容 roslibjs v2，构建产物已包含此修复喵~
build_ros3djs() {
  local pkg_dir="${ROOT_DIR}/ros3djs"
  require_dir "${pkg_dir}"

  echo "============================================================"
  echo "[构建] 开始构建 ros3djs"
  echo "[构建] 源码目录: ${pkg_dir}"
  echo "============================================================"
  cd "${pkg_dir}"
  ensure_dependencies "${pkg_dir}"
  npm run build
  echo "[完成] ros3djs 构建成功喵~"
}

# build_ros2djs — 构建 ros2djs（EaselJS 2D 可视化库）
# 输入: ros2djs/（源码 + node_modules）
# 输出: ros2djs/build/（ros2d.js / ros2d.min.js）
# 官方构建命令: npm run build（内部执行 grunt build）
build_ros2djs() {
  local pkg_dir="${ROOT_DIR}/ros2djs"
  require_dir "${pkg_dir}"

  echo "============================================================"
  echo "[构建] 开始构建 ros2djs"
  echo "[构建] 源码目录: ${pkg_dir}"
  echo "============================================================"
  cd "${pkg_dir}"
  ensure_dependencies "${pkg_dir}"
  npm run build
  echo "[完成] ros2djs 构建成功喵~"
}

# copy_runtime_assets — 调用 copy_runtime_js_assets.sh 汇总所有 JS 产物
# 作用: 将三个库的构建产物 + 示例依赖 + roslib ESM vendor 统一复制到
#       runtime_js_assets/ 目录，供 web_dashboard 等包直接引用喵~
copy_runtime_assets() {
  local script_path="${ROOT_DIR}/copy_runtime_js_assets.sh"
  if [[ ! -f "${script_path}" ]]; then
    echo "[错误] 未找到资产汇总脚本: ${script_path}"
    exit 1
  fi
  echo "============================================================"
  echo "[汇总] 开始将构建产物复制到 runtime_js_assets/ 喵~"
  echo "============================================================"
  bash "${script_path}"
}

# ============================================================
# 主流程
# ============================================================

echo "============================================================"
echo "  RobotWebTools 全量构建"
echo "  根目录: ${ROOT_DIR}"
echo "  模式: 优先离线（缺失时自动联网下载）"
echo "  构建范围: roslibjs + ros3djs + ros2djs"
echo "============================================================"

build_roslibjs
build_ros3djs
build_ros2djs
copy_runtime_assets

echo "============================================================"
echo "  全量构建完成喵~"
echo "  产物位置: ${ROOT_DIR}/runtime_js_assets/"
echo "============================================================"
