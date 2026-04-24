#!/usr/bin/env bash
# 用途：
# 将“本地化示例涉及的 JS 库”以及“构建产物 JS 文件”统一复制到一个可分发目录，
# 方便其他功能包直接引用，不需要再关心各仓库内部路径。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${ROOT_DIR}/runtime_js_assets"

require_file() {
  local f="$1"
  if [[ ! -f "${f}" ]]; then
    echo "[错误] 缺少文件: ${f}"
    exit 1
  fi
}

copy_file() {
  local src="$1"
  local dst="$2"
  require_file "${src}"
  mkdir -p "$(dirname "${dst}")"
  cp "${src}" "${dst}"
}

copy_dir() {
  local src="$1"
  local dst="$2"
  if [[ ! -d "${src}" ]]; then
    echo "[错误] 缺少目录: ${src}"
    exit 1
  fi
  mkdir -p "$(dirname "${dst}")"
  rm -rf "${dst}"
  cp -r "${src}" "${dst}"
}

populate_roslib_vendor_from_node_modules() {
  local roslib_root="${ROOT_DIR}/roslibjs"
  local nm="${roslib_root}/node_modules"
  if [[ ! -d "${nm}" ]]; then
    echo "[错误] roslibjs vendor 缺失，且未找到 node_modules: ${nm}"
    exit 1
  fi
  echo "[信息] 未找到 roslib vendor，改为从 roslibjs/node_modules 生成运行时 vendor"
  copy_dir "${nm}/bson/lib" "${OUT_DIR}/roslib-esm/vendor/bson/lib"
  copy_dir "${nm}/cbor2/lib" "${OUT_DIR}/roslib-esm/vendor/cbor2/lib"
  copy_file "${nm}/eventemitter3/dist/eventemitter3.esm.js" "${OUT_DIR}/roslib-esm/vendor/eventemitter3/dist/eventemitter3.esm.js"
  copy_dir "${nm}/fast-png/lib" "${OUT_DIR}/roslib-esm/vendor/fast-png/lib"
  copy_dir "${nm}/uuid/dist" "${OUT_DIR}/roslib-esm/vendor/uuid/dist"
  copy_file "${nm}/ws/browser.js" "${OUT_DIR}/roslib-esm/vendor/ws/browser.js"
  copy_dir "${nm}/@cto.af/wtf8/lib" "${OUT_DIR}/roslib-esm/vendor/@cto.af/wtf8/lib"
  copy_dir "${nm}/fflate/esm" "${OUT_DIR}/roslib-esm/vendor/fflate/esm"
  copy_dir "${nm}/iobuffer/lib" "${OUT_DIR}/roslib-esm/vendor/iobuffer/lib"
}

echo "[信息] 开始复制运行时 JS 资产"
echo "[信息] 输出目录: ${OUT_DIR}"
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

# 1) 编译构建后的 JS 文件
mkdir -p "${OUT_DIR}/build/roslib"
cp "${ROOT_DIR}"/roslibjs/packages/roslib/dist/*.js "${OUT_DIR}/build/roslib/"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.js" "${OUT_DIR}/build/ros3d/ros3d.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.min.js" "${OUT_DIR}/build/ros3d/ros3d.min.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.esm.js" "${OUT_DIR}/build/ros3d/ros3d.esm.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.cjs.js" "${OUT_DIR}/build/ros3d/ros3d.cjs.js"
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.js" "${OUT_DIR}/build/ros2d/ros2d.js"
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.min.js" "${OUT_DIR}/build/ros2d/ros2d.min.js"

# 2) 本地化示例涉及的 JS 库（统一沉淀为 libs）
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/three.js" "${OUT_DIR}/libs/three.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/eventemitter2.js" "${OUT_DIR}/libs/eventemitter2.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/roslib.js" "${OUT_DIR}/libs/roslib.global.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/ColladaLoader.js" "${OUT_DIR}/libs/ColladaLoader.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/STLLoader.js" "${OUT_DIR}/libs/STLLoader.js"
copy_file "${ROOT_DIR}/ros2djs/examples/vendor/easeljs.js" "${OUT_DIR}/libs/easeljs.js"

# 3) roslib ESM 本地 importmap 运行所需文件
copy_file "${ROOT_DIR}/roslibjs/packages/roslib/importmap.js" "${OUT_DIR}/roslib-esm/importmap.js"
if [[ -d "${ROOT_DIR}/roslibjs/packages/roslib/vendor" ]]; then
  copy_dir "${ROOT_DIR}/roslibjs/packages/roslib/vendor" "${OUT_DIR}/roslib-esm/vendor"
else
  populate_roslib_vendor_from_node_modules
fi

# 4) dashboard 直接可用的统一 importmap（映射 roslib / ros3d）
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

echo "[信息] 运行时 JS 资产复制完成。"
