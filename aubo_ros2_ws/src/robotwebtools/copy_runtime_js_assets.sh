#!/usr/bin/env bash
# ============================================================
# copy_runtime_js_assets.sh — 运行时 JS 资产汇总脚本
# ============================================================
#
# 【功能】
#   将 robotwebtools 三个库的构建产物、示例依赖、roslib ESM vendor
#   统一复制到 runtime_js_assets/ 目录，并生成统一的 importmap，
#   供 web_dashboard 等下游包直接引用，无需关心各仓库内部路径。
#
# 【设计原则】
#   - 纯复制/汇总：不修改 JS 文件内容（语义修复在源码+构建阶段完成）喵~
#   - 幂等性：每次执行先清空输出目录再重新生成喵~
#   - 容错：任一文件缺失立即报错退出，避免下游引用到不完整资产喵~
#
# 【输出目录结构】
#   runtime_js_assets/
#   ├── importmap.js              # 统一 importmap，映射 roslib/ros3d 到本地文件
#   ├── build/                    # 三库构建产物
#   │   ├── roslib/               #   roslibjs ESM 产物 (RosLib.js 等)
#   │   ├── ros3d/                #   ros3djs 四种格式 (esm/cjs/iife/min)
#   │   └── ros2d/                #   ros2djs 全局模式产物
#   ├── libs/                     # 示例级全局依赖（传统 <script> 方式）
#   │   ├── three.js              #   Three.js 3D 引擎
#   │   ├── easeljs.js            #   EaselJS 2D 渲染
#   │   ├── roslib.global.js      #   roslib 全局模式（非 ESM）
#   │   ├── eventemitter2.js      #   事件总线
#   │   ├── ColladaLoader.js      #   Three.js Collada 模型加载器
#   │   └── STLLoader.js          #   Three.js STL 模型加载器
#   └── roslib-esm/               # roslib ESM 模块化运行所需文件
#       ├── importmap.js          #   roslib 自身 importmap（映射依赖到 vendor/）
#       └── vendor/               #   浏览器端 ESM 依赖（从 node_modules 提取）
#           ├── bson/lib/         #   BSON 序列化
#           ├── cbor2/lib/        #   CBOR2 序列化
#           ├── eventemitter3/    #   事件总线 v3
#           ├── fast-png/lib/     #   PNG 编解码
#           ├── uuid/dist/        #   UUID 生成
#           ├── ws/               #   WebSocket 浏览器端实现
#           ├── @cto.af/wtf8/lib/ #   UTF-8 编解码
#           ├── fflate/esm/       #   压缩/解压
#           └── iobuffer/lib/     #   二进制缓冲区
#
# 【使用方法】
#   # 单独调用（需先完成构建）:
#   bash copy_runtime_js_assets.sh
#
#   # 通常由 build_robotwebtools.sh 构建完成后自动调用，无需手动执行喵~
#
# 【与 build_robotwebtools.sh 的关系】
#   build_robotwebtools.sh = npm run build × 3 + copy_runtime_js_assets.sh
#   copy_runtime_js_assets.sh 只负责收集产物，不做任何构建喵~
# ============================================================

set -euo pipefail

# ============================================================
# 全局变量
# ============================================================
# ROOT_DIR: 脚本所在目录，即 robotwebtools 包根目录
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# OUT_DIR: 运行时资产汇总输出目录
OUT_DIR="${ROOT_DIR}/runtime_js_assets"

# ============================================================
# 工具函数
# ============================================================

# require_file — 校验单个文件存在，不存在则报错退出
# 参数: $1 — 文件路径
# 用途: 复制前确保源文件已就绪（通常意味着构建已完成）喵~
require_file() {
  local f="$1"
  if [[ ! -f "${f}" ]]; then
    echo "[错误] 缺少文件: ${f}"
    echo "[提示] 请先执行 build_robotwebtools.sh 完成构建喵~"
    exit 1
  fi
}

# copy_file — 安全复制单个文件（校验源存在 + 自动创建父目录）
# 参数: $1 — 源文件路径
#       $2 — 目标文件路径
copy_file() {
  local src="$1"
  local dst="$2"
  require_file "${src}"
  mkdir -p "$(dirname "${dst}")"
  cp "${src}" "${dst}"
}

# copy_dir — 安全复制整个目录（校验源存在 + 自动创建父目录 + 覆盖旧内容）
# 参数: $1 — 源目录路径
#       $2 — 目标目录路径
copy_dir() {
  local src="$1"
  local dst="$2"
  if [[ ! -d "${src}" ]]; then
    echo "[错误] 缺少目录: ${src}"
    echo "[提示] 请先执行 build_robotwebtools.sh 完成构建喵~"
    exit 1
  fi
  mkdir -p "$(dirname "${dst}")"
  rm -rf "${dst}"
  cp -r "${src}" "${dst}"
}

# populate_roslib_vendor_from_node_modules — 从 node_modules 提取 roslib ESM 依赖
# 触发条件: roslibjs/packages/roslib/vendor/ 目录不存在时调用
# 作用: 将 roslib 运行所需的 9 个 npm 包的 ESM 入口文件复制到 vendor/ 目录，
#       使 importmap 能映射到本地文件，实现浏览器端零外网依赖喵~
# 映射关系（npm 包 → 浏览器 ESM 入口）:
#   bson        → bson/lib/bson.mjs
#   cbor2       → cbor2/lib/index.js
#   eventemitter3 → eventemitter3/dist/eventemitter3.esm.js
#   fast-png    → fast-png/lib/index.js
#   uuid        → uuid/dist/index.js
#   ws          → ws/browser.js
#   @cto.af/wtf8 → @cto.af/wtf8/lib/index.js
#   fflate      → fflate/esm/browser.js
#   iobuffer    → iobuffer/lib/iobuffer.js
populate_roslib_vendor_from_node_modules() {
  local roslib_root="${ROOT_DIR}/roslibjs"
  local nm="${roslib_root}/node_modules"

  if [[ ! -d "${nm}" ]]; then
    echo "[错误] roslibjs vendor 缺失，且未找到 node_modules: ${nm}"
    echo "[提示] 请先执行 build_robotwebtools.sh 完成构建，脚本会自动联网安装缺失依赖喵~"
    exit 1
  fi

  echo "[信息] 未找到 roslib vendor 目录，改为从 roslibjs/node_modules 提取运行时依赖喵~"

  copy_dir "${nm}/bson/lib"            "${OUT_DIR}/roslib-esm/vendor/bson/lib"
  copy_dir "${nm}/cbor2/lib"           "${OUT_DIR}/roslib-esm/vendor/cbor2/lib"
  copy_file "${nm}/eventemitter3/dist/eventemitter3.esm.js" "${OUT_DIR}/roslib-esm/vendor/eventemitter3/dist/eventemitter3.esm.js"
  copy_dir "${nm}/fast-png/lib"        "${OUT_DIR}/roslib-esm/vendor/fast-png/lib"
  copy_dir "${nm}/uuid/dist"           "${OUT_DIR}/roslib-esm/vendor/uuid/dist"
  copy_file "${nm}/ws/browser.js"      "${OUT_DIR}/roslib-esm/vendor/ws/browser.js"
  copy_dir "${nm}/@cto.af/wtf8/lib"    "${OUT_DIR}/roslib-esm/vendor/@cto.af/wtf8/lib"
  copy_dir "${nm}/fflate/esm"          "${OUT_DIR}/roslib-esm/vendor/fflate/esm"
  copy_dir "${nm}/iobuffer/lib"        "${OUT_DIR}/roslib-esm/vendor/iobuffer/lib"
}

# ============================================================
# 主流程
# ============================================================

echo "============================================================"
echo "  运行时 JS 资产汇总"
echo "  源目录: ${ROOT_DIR}"
echo "  输出目录: ${OUT_DIR}"
echo "============================================================"

# 初始化输出目录（清空旧内容，确保幂等性）喵~
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

# ============================================================
# 第 1 步：复制三库构建产物 → build/
# ============================================================
echo "[步骤 1/4] 复制构建产物..."

# 1a. roslibjs 构建产物（ESM 格式，供 importmap 引用）喵~
mkdir -p "${OUT_DIR}/build/roslib"
cp "${ROOT_DIR}"/roslibjs/packages/roslib/dist/*.js "${OUT_DIR}/build/roslib/"
echo "  → roslibjs 产物已复制喵~"

# 1b. ros3djs 构建产物（四种格式：ESM / CJS / IIFE / minified）喵~
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.js"       "${OUT_DIR}/build/ros3d/ros3d.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.min.js"    "${OUT_DIR}/build/ros3d/ros3d.min.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.esm.js"    "${OUT_DIR}/build/ros3d/ros3d.esm.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.cjs.js"    "${OUT_DIR}/build/ros3d/ros3d.cjs.js"
echo "  → ros3djs 产物已复制喵~"

# 1c. ros2djs 构建产物（IIFE 全局模式 + minified）喵~
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.js"        "${OUT_DIR}/build/ros2d/ros2d.js"
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.min.js"    "${OUT_DIR}/build/ros2d/ros2d.min.js"
echo "  → ros2djs 产物已复制喵~"

# ============================================================
# 第 2 步：复制示例级全局依赖 → libs/
# ============================================================
# 这些文件用于传统 <script> 标签加载方式（非 ESM 工程化方式）。
# 来源是各库 examples/vendor/ 目录中已本地化的第三方库喵~
echo "[步骤 2/4] 复制示例全局依赖..."

copy_file "${ROOT_DIR}/ros3djs/examples/vendor/three.js"          "${OUT_DIR}/libs/three.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/eventemitter2.js"  "${OUT_DIR}/libs/eventemitter2.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/roslib.js"         "${OUT_DIR}/libs/roslib.global.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/ColladaLoader.js"  "${OUT_DIR}/libs/ColladaLoader.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/STLLoader.js"      "${OUT_DIR}/libs/STLLoader.js"
copy_file "${ROOT_DIR}/ros2djs/examples/vendor/easeljs.js"        "${OUT_DIR}/libs/easeljs.js"
echo "  → 全局依赖已复制喵~"

# ============================================================
# 第 3 步：复制 roslib ESM 依赖 → roslib-esm/
# ============================================================
echo "[步骤 3/4] 复制 roslib ESM 依赖..."

# 3a. roslib 自身 importmap（映射 roslib 入口 + 各依赖到 vendor/）喵~
copy_file "${ROOT_DIR}/roslibjs/packages/roslib/importmap.js" "${OUT_DIR}/roslib-esm/importmap.js"

# 3b. roslib ESM vendor 依赖（浏览器端运行 roslib ESM 所需的所有 npm 包）喵~
if [[ -d "${ROOT_DIR}/roslibjs/packages/roslib/vendor" ]]; then
  # 理想路径：vendor 已由之前构建过程准备就绪喵~
  copy_dir "${ROOT_DIR}/roslibjs/packages/roslib/vendor" "${OUT_DIR}/roslib-esm/vendor"
  echo "  → roslib vendor 已从 packages/roslib/vendor 复制喵~"
else
  # 兜底路径：vendor 目录缺失时，直接从 node_modules 提取依赖喵~
  populate_roslib_vendor_from_node_modules
  echo "  → roslib vendor 已从 node_modules 提取喵~"
fi

# ============================================================
# 第 4 步：生成统一 importmap → importmap.js
# ============================================================
# 这是给下游（web_dashboard 等）使用的顶层 importmap。
# 页面只需加载这一个文件，即可用 import 'roslib' / import 'ros3d' 引用本地产物喵~
echo "[步骤 4/4] 生成统一 importmap..."
cat > "${OUT_DIR}/importmap.js" <<'EOF'
(map => {
  const mapUrl = document.currentScript.src;
  const resolve = imports =>
    Object.fromEntries(Object.entries(imports).map(([k, v]) => [k, new URL(v, mapUrl).href]));
  document.head.appendChild(
    Object.assign(document.createElement("script"), {
      type: "importmap",
      innerHTML: JSON.stringify({ imports: resolve(map.imports) })
    })
  );
})({
  "imports": {
    "roslib": "./build/roslib/RosLib.js",
    "ros3d": "./build/ros3d/ros3d.esm.js",
    "bson": "./roslib-esm/vendor/bson/lib/bson.mjs",
    "cbor2": "./roslib-esm/vendor/cbor2/lib/index.js",
    "eventemitter3": "./roslib-esm/vendor/eventemitter3/dist/eventemitter3.esm.js",
    "fast-png": "./roslib-esm/vendor/fast-png/lib/index.js",
    "uuid": "./roslib-esm/vendor/uuid/dist/index.js",
    "ws": "./roslib-esm/vendor/ws/browser.js",
    "@cto.af/wtf8": "./roslib-esm/vendor/@cto.af/wtf8/lib/index.js",
    "fflate": "./roslib-esm/vendor/fflate/esm/browser.js",
    "iobuffer": "./roslib-esm/vendor/iobuffer/lib/iobuffer.js"
  }
});
EOF
echo "  → 统一 importmap 已生成喵~"

echo "============================================================"
echo "  运行时 JS 资产汇总完成喵~"
echo "  输出目录: ${OUT_DIR}"
echo "============================================================"
