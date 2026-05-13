#!/usr/bin/env bash
# 用途：
# 将"本地化示例涉及的 JS 库"以及"构建产物 JS 文件"统一复制到一个可分发目录，
# 方便其他功能包直接引用，不需要再关心各仓库内部路径。
set -euo pipefail

# ═══════════════════════════════════════════════════════════════
# 颜色与常量
# ═══════════════════════════════════════════════════════════════
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${ROOT_DIR}/runtime_js_assets"

# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

require_file() {
    local f="$1"
    if [[ ! -f "${f}" ]]; then
        echo -e "${RED}[错误] 缺少文件: ${f}${NC}"
        return 1
    fi
}

copy_file() {
    local src="$1"
    local dst="$2"

    require_file "${src}" || return 1
    mkdir -p "$(dirname "${dst}")"
    cp "${src}" "${dst}"
    echo -e "  ${BLUE}→${NC} ${dst##*/}"
}

copy_dir() {
    local src="$1"
    local dst="$2"

    if [[ ! -d "${src}" ]]; then
        echo -e "${RED}[错误] 缺少目录: ${src}${NC}"
        return 1
    fi

    mkdir -p "$(dirname "${dst}")"
    rm -rf "${dst}"
    cp -r "${src}" "${dst}"
    echo -e "  ${BLUE}→${NC} ${dst##*/}/ (目录)"
}

# 预检：批量检查源文件/目录是否存在，缺失时统一报错
precheck_sources() {
    local missing=0
    for src in "$@"; do
        if [[ ! -e "${src}" ]]; then
            echo -e "${RED}[缺失] ${src}${NC}"
            missing=1
        fi
    done
    if [[ "${missing}" -eq 1 ]]; then
        echo -e "${RED}请先运行 build_robotwebtools.sh 完成构建。${NC}"
        exit 1
    fi
}

# 从 node_modules 生成 roslib ESM vendor（离线兜底）
populate_roslib_vendor_from_node_modules() {
    local roslib_root="${ROOT_DIR}/roslibjs"
    local nm="${roslib_root}/node_modules"

    if [[ ! -d "${nm}" ]]; then
        echo -e "${RED}[错误] roslibjs node_modules 缺失，无法生成 vendor${NC}"
        exit 1
    fi

    echo -e "${YELLOW}  从 roslibjs/node_modules 生成运行时 vendor...${NC}"

    copy_dir "${nm}/bson/lib"             "${OUT_DIR}/roslib-esm/vendor/bson/lib"
    copy_dir "${nm}/cbor2/lib"            "${OUT_DIR}/roslib-esm/vendor/cbor2/lib"
    copy_file "${nm}/eventemitter3/dist/eventemitter3.esm.js" "${OUT_DIR}/roslib-esm/vendor/eventemitter3/dist/eventemitter3.esm.js"
    copy_dir "${nm}/fast-png/lib"         "${OUT_DIR}/roslib-esm/vendor/fast-png/lib"
    copy_dir "${nm}/uuid/dist"            "${OUT_DIR}/roslib-esm/vendor/uuid/dist"
    copy_file "${nm}/ws/browser.js"       "${OUT_DIR}/roslib-esm/vendor/ws/browser.js"
    copy_dir "${nm}/@cto.af/wtf8/lib"    "${OUT_DIR}/roslib-esm/vendor/@cto.af/wtf8/lib"
    copy_dir "${nm}/fflate/esm"           "${OUT_DIR}/roslib-esm/vendor/fflate/esm"
    copy_dir "${nm}/iobuffer/lib"         "${OUT_DIR}/roslib-esm/vendor/iobuffer/lib"
}

# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

echo -e "${BLUE}[RobotWebTools]${NC} 复制运行时 JS 资产"
echo -e "  输出目录: ${BLUE}${OUT_DIR}${NC}"

# 阶段 0: 预检所有源文件
echo -e "${BLUE}[预检]${NC} 检查源文件..."

SOURCES=(
    "${ROOT_DIR}/roslibjs/packages/roslib/dist/"
    "${ROOT_DIR}/ros3djs/build/ros3d.js"
    "${ROOT_DIR}/ros3djs/build/ros3d.min.js"
    "${ROOT_DIR}/ros3djs/build/ros3d.esm.js"
    "${ROOT_DIR}/ros3djs/build/ros3d.cjs.js"
    "${ROOT_DIR}/ros2djs/build/ros2d.js"
    "${ROOT_DIR}/ros2djs/build/ros2d.min.js"
    "${ROOT_DIR}/ros3djs/examples/vendor/three.js"
    "${ROOT_DIR}/ros3djs/examples/vendor/eventemitter2.js"
    "${ROOT_DIR}/ros3djs/examples/vendor/roslib.js"
    "${ROOT_DIR}/ros3djs/examples/vendor/ColladaLoader.js"
    "${ROOT_DIR}/ros3djs/examples/vendor/STLLoader.js"
    "${ROOT_DIR}/ros2djs/examples/vendor/easeljs.js"
    "${ROOT_DIR}/roslibjs/packages/roslib/importmap.js"
)

precheck_sources "${SOURCES[@]}"
echo -e "${GREEN}  ✓ 源文件检查通过${NC}"

# 清空旧输出
rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"

# 阶段 1: 编译构建后的 JS 文件
echo -e "${BLUE}[1/4]${NC} 构建产物..."
mkdir -p "${OUT_DIR}/build/roslib"
cp "${ROOT_DIR}"/roslibjs/packages/roslib/dist/*.js "${OUT_DIR}/build/roslib/"
echo -e "  ${BLUE}→${NC} build/roslib/ ($(ls "${OUT_DIR}/build/roslib/" | wc -l) 个文件)"

copy_file "${ROOT_DIR}/ros3djs/build/ros3d.js"      "${OUT_DIR}/build/ros3d/ros3d.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.min.js"  "${OUT_DIR}/build/ros3d/ros3d.min.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.esm.js"  "${OUT_DIR}/build/ros3d/ros3d.esm.js"
copy_file "${ROOT_DIR}/ros3djs/build/ros3d.cjs.js"  "${OUT_DIR}/build/ros3d/ros3d.cjs.js"
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.js"      "${OUT_DIR}/build/ros2d/ros2d.js"
copy_file "${ROOT_DIR}/ros2djs/build/ros2d.min.js"  "${OUT_DIR}/build/ros2d/ros2d.min.js"

# 阶段 2: 第三方 vendor 库
echo -e "${BLUE}[2/4]${NC} 第三方 vendor 库..."
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/three.js"         "${OUT_DIR}/libs/three.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/eventemitter2.js" "${OUT_DIR}/libs/eventemitter2.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/roslib.js"        "${OUT_DIR}/libs/roslib.global.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/ColladaLoader.js" "${OUT_DIR}/libs/ColladaLoader.js"
copy_file "${ROOT_DIR}/ros3djs/examples/vendor/STLLoader.js"     "${OUT_DIR}/libs/STLLoader.js"
copy_file "${ROOT_DIR}/ros2djs/examples/vendor/easeljs.js"       "${OUT_DIR}/libs/easeljs.js"

# 阶段 3: roslib ESM 本地 importmap 运行所需文件
echo -e "${BLUE}[3/4]${NC} roslib ESM 运行时..."
copy_file "${ROOT_DIR}/roslibjs/packages/roslib/importmap.js" "${OUT_DIR}/roslib-esm/importmap.js"

if [[ -d "${ROOT_DIR}/roslibjs/packages/roslib/vendor" ]]; then
    copy_dir "${ROOT_DIR}/roslibjs/packages/roslib/vendor" "${OUT_DIR}/roslib-esm/vendor"
else
    populate_roslib_vendor_from_node_modules
fi

# 阶段 4: 统一 importmap（原子写入）
echo -e "${BLUE}[4/4]${NC} 生成统一 importmap..."
TMP_IMPORTMAP="${OUT_DIR}/.importmap.js.tmp"

cat > "${TMP_IMPORTMAP}" <<'EOF'
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

# 原子替换：只有写入成功才覆盖旧文件
mv "${TMP_IMPORTMAP}" "${OUT_DIR}/importmap.js"
echo -e "  ${GREEN}✓${NC} importmap.js (原子写入)"

echo ""
echo -e "${GREEN}[完成]${NC} 运行时 JS 资产已就绪"
echo -e "  路径: ${BLUE}${OUT_DIR}${NC}"
echo -e "  文件数: $(find "${OUT_DIR}" -type f | wc -l)"
