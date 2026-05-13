#!/usr/bin/env bash
# 说明：
# 1) 本脚本仅做"本地离线构建"，不做 git clone / pull。
# 2) 仅依赖本地已有源码、node_modules 或 npm 本地缓存。
# 3) 任一步骤失败立即退出，避免产生"部分成功"的不一致状态。
# 4) 三个包并行构建（互不依赖），支持增量跳过。
#
# 使用方法：
#   bash ./build_robotwebtools.sh
#
# 常见问题排查：
# 1) 报错"缺少目录"：请先确认 roslibjs/ros3djs/ros2djs 三个源码目录都已存在。
# 2) 报错"离线安装失败"：说明本地 npm 缓存不足且 node_modules 不存在；
#    需要先在可联网环境准备依赖，或把已安装好的 node_modules 同步到本地。
# 3) 构建结束后会自动执行 copy_runtime_js_assets.sh，把可调用 JS 复制到统一目录。
set -euo pipefail

# ═══════════════════════════════════════════════════════════════
# 颜色与常量
# ═══════════════════════════════════════════════════════════════
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATUS_DIR="$(mktemp -d -t rwt_build_XXXXXX)"
trap "rm -rf ${STATUS_DIR}" EXIT

# ═══════════════════════════════════════════════════════════════
# 预检
# ═══════════════════════════════════════════════════════════════

for tool in node npm; do
    if ! command -v "$tool" &>/dev/null; then
        echo -e "${RED}[错误] 未找到 ${tool}，请先安装 Node.js${NC}"
        echo -e "${RED}  sudo apt install nodejs npm  或  https://nodejs.org/${NC}"
        exit 1
    fi
done

# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

require_dir() {
    local dir="$1"
    if [[ ! -d "${dir}" ]]; then
        echo -e "${RED}[错误] 缺少目录: ${dir}${NC}"
        echo -e "${RED}  本脚本只构建已存在源码，不会拉取仓库。${NC}"
        exit 1
    fi
}

# 增量构建判断：对比源文件与产物标记文件的时间戳
# 返回 0 = 需要构建，1 = 可跳过
needs_rebuild() {
    local src_dir="$1"
    local out_dir="$2"
    local marker="${out_dir}/.rwt_build_marker"

    if [[ ! -f "${marker}" ]]; then
        return 0
    fi

    # 查找比 marker 更新的源文件（.ts / .js / .json / Gruntfile）
    local newer
    newer=$(find "${src_dir}" -type f \
        \( -name "*.ts" -o -name "*.js" -o -name "*.json" \) \
        -newer "${marker}" 2>/dev/null | head -1)

    if [[ -n "${newer}" ]]; then
        return 0
    fi

    return 1
}

# 标记构建完成
mark_built() {
    local out_dir="$1"
    touch "${out_dir}/.rwt_build_marker"
}

# 验证产物非空
verify_output() {
    local out_dir="$1" label="$2"
    if [[ ! -d "${out_dir}" ]] || [[ -z "$(ls -A "${out_dir}" 2>/dev/null)" ]]; then
        echo -e "${RED}[错误] ${label} 构建产物为空: ${out_dir}${NC}"
        return 1
    fi
}

# 确保 npm 依赖就绪
ensure_offline_dependencies() {
    local pkg_dir="$1"

    # 处理 monorepo：roslibjs node_modules 在根目录而非 packages/roslib/
    local nm_dir="${pkg_dir}/node_modules"
    if [[ ! -d "${nm_dir}" ]]; then
        local parent_nm="$(dirname "${pkg_dir}")/node_modules"
        if [[ -d "${parent_nm}" ]]; then
            nm_dir="${parent_nm}"
        fi
    fi

    if [[ -d "${nm_dir}" ]] && [[ -n "$(ls -A "${nm_dir}" 2>/dev/null)" ]]; then
        echo -e "  ${GREEN}✓${NC} node_modules 已就绪"
        return 0
    fi

    cd "${pkg_dir}"
    echo -e "  ${YELLOW}安装 npm 依赖...${NC}"

    if npm install --no-audit --fund=false 2>&1; then
        echo -e "  ${GREEN}✓${NC} npm 依赖安装成功"
        return 0
    fi

    echo -e "  ${YELLOW}联网安装失败，尝试离线安装...${NC}"
    if ! npm install --offline --no-audit --fund=false; then
        echo -e "${RED}[错误] 依赖安装失败（联网和离线均失败）: ${pkg_dir}${NC}"
        echo -e "${RED}  请检查网络连接或本地 npm 缓存。${NC}"
        return 1
    fi
    echo -e "  ${GREEN}✓${NC} npm 离线安装成功"
}

# ═══════════════════════════════════════════════════════════════
# 构建函数（每个在独立 subshell 中执行，支持并行）
# ═══════════════════════════════════════════════════════════════

build_roslibjs() {
    local pkg_dir="${ROOT_DIR}/roslibjs/packages/roslib"
    local src_dir="${pkg_dir}/src"
    local out_dir="${pkg_dir}/dist"
    local label="roslibjs"
    local status_file="${STATUS_DIR}/roslibjs.status"
    local vendor_dir="${pkg_dir}/vendor"
    local backup_dir="/tmp/roslib_vendor_backup_${USER:-user}_$$"

    require_dir "${pkg_dir}"

    if ! needs_rebuild "${src_dir}" "${out_dir}"; then
        echo -e "${GREEN}[跳过]${NC} ${label} — 产物已是最新"
        echo "0" > "${status_file}"
        return 0
    fi

    echo -e "${BLUE}[构建]${NC} ${label} (Vite)..."
    local t0=$SECONDS

    cd "${pkg_dir}"
    ensure_offline_dependencies "${pkg_dir}" || { echo "1" > "${status_file}"; return 1; }

    # vendor 目录会导致 eslint 扫描失败，构建前临时移走
    local _vendor_restored=false
    trap 'if [ "${_vendor_restored}" = false ] && [ -d "${backup_dir}" ]; then
              mv "${backup_dir}" "${vendor_dir}" 2>/dev/null || true
          fi
          rm -rf "${backup_dir}" 2>/dev/null || true' EXIT

    if [[ -d "${vendor_dir}" ]]; then
        rm -rf "${backup_dir}"
        mv "${vendor_dir}" "${backup_dir}"
    fi

    if npm run build; then
        if [[ -d "${backup_dir}" ]]; then
            mv "${backup_dir}" "${vendor_dir}"
        fi
        _vendor_restored=true
        trap - EXIT
        rm -rf "${backup_dir}" 2>/dev/null || true

        verify_output "${out_dir}" "${label}" || { echo "1" > "${status_file}"; return 1; }
        mark_built "${out_dir}"
        echo -e "${GREEN}[完成]${NC} ${label} (${BLUE}$((SECONDS - t0))s${NC})"
        echo "0" > "${status_file}"
    else
        if [[ -d "${backup_dir}" ]]; then
            mv "${backup_dir}" "${vendor_dir}"
        fi
        _vendor_restored=true
        trap - EXIT
        rm -rf "${backup_dir}" 2>/dev/null || true

        echo -e "${RED}[失败]${NC} ${label} 构建失败"
        echo "1" > "${status_file}"
        return 1
    fi
}

build_ros3djs() {
    local pkg_dir="${ROOT_DIR}/ros3djs"
    local src_dir="${pkg_dir}/src"
    local out_dir="${pkg_dir}/build"
    local label="ros3djs"
    local status_file="${STATUS_DIR}/ros3djs.status"

    require_dir "${pkg_dir}"

    if ! needs_rebuild "${src_dir}" "${out_dir}"; then
        echo -e "${GREEN}[跳过]${NC} ${label} — 产物已是最新"
        echo "0" > "${status_file}"
        return 0
    fi

    echo -e "${BLUE}[构建]${NC} ${label} (Grunt + Rollup)..."
    local t0=$SECONDS

    cd "${pkg_dir}"
    ensure_offline_dependencies "${pkg_dir}" || { echo "1" > "${status_file}"; return 1; }

    if npm run build; then
        verify_output "${out_dir}" "${label}" || { echo "1" > "${status_file}"; return 1; }
        mark_built "${out_dir}"
        echo -e "${GREEN}[完成]${NC} ${label} (${BLUE}$((SECONDS - t0))s${NC})"
        echo "0" > "${status_file}"
    else
        echo -e "${RED}[失败]${NC} ${label} 构建失败"
        echo "1" > "${status_file}"
        return 1
    fi
}

build_ros2djs() {
    local pkg_dir="${ROOT_DIR}/ros2djs"
    local src_dir="${pkg_dir}/src"
    local out_dir="${pkg_dir}/build"
    local label="ros2djs"
    local status_file="${STATUS_DIR}/ros2djs.status"

    require_dir "${pkg_dir}"

    if ! needs_rebuild "${src_dir}" "${out_dir}"; then
        echo -e "${GREEN}[跳过]${NC} ${label} — 产物已是最新"
        echo "0" > "${status_file}"
        return 0
    fi

    echo -e "${BLUE}[构建]${NC} ${label} (Grunt)..."
    local t0=$SECONDS

    cd "${pkg_dir}"
    ensure_offline_dependencies "${pkg_dir}" || { echo "1" > "${status_file}"; return 1; }

    if npm run build; then
        verify_output "${out_dir}" "${label}" || { echo "1" > "${status_file}"; return 1; }
        mark_built "${out_dir}"
        echo -e "${GREEN}[完成]${NC} ${label} (${BLUE}$((SECONDS - t0))s${NC})"
        echo "0" > "${status_file}"
    else
        echo -e "${RED}[失败]${NC} ${label} 构建失败"
        echo "1" > "${status_file}"
        return 1
    fi
}

# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

export NPM_CONFIG_AUDIT=false
export NPM_CONFIG_FUND=false

echo -e "${BLUE}══════════════════════════════════════${NC}"
echo -e "${BLUE}  RobotWebTools 离线构建${NC}"
echo -e "${BLUE}  根目录: ${ROOT_DIR}${NC}"
echo -e "${BLUE}══════════════════════════════════════${NC}"
echo ""

TOTAL_START=$SECONDS

# 校验必要目录存在
require_dir "${ROOT_DIR}/roslibjs/packages/roslib"
require_dir "${ROOT_DIR}/ros3djs"
require_dir "${ROOT_DIR}/ros2djs"

# 并行构建（三个包互不依赖）
echo -e "${BLUE}启动并行构建 (roslibjs | ros3djs | ros2djs)...${NC}"
echo ""

build_roslibjs &
build_ros3djs &
build_ros2djs &

wait

# 收集构建结果
FAILED=0
for status_file in "${STATUS_DIR}"/*.status; do
    if [[ -f "${status_file}" ]] && [[ "$(cat "${status_file}")" != "0" ]]; then
        FAILED=1
    fi
done

if [[ "${FAILED}" -eq 1 ]]; then
    echo ""
    echo -e "${RED}══════════════════════════════════════${NC}"
    echo -e "${RED}  构建失败！请检查上方错误信息。${NC}"
    echo -e "${RED}══════════════════════════════════════${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}══════════════════════════════════════${NC}"
echo -e "${GREEN}  全量构建完成 (总耗时 ${BLUE}$((SECONDS - TOTAL_START))s${NC}${GREEN})${NC}"
echo -e "${GREEN}══════════════════════════════════════${NC}"
echo ""
echo -e "产物位置:"
echo -e "  roslibjs → ${BLUE}roslibjs/packages/roslib/dist/${NC}"
echo -e "  ros3djs  → ${BLUE}ros3djs/build/${NC}"
echo -e "  ros2djs  → ${BLUE}ros2djs/build/${NC}"

# ═══════════════════════════════════════════════════════════════
# 汇总可调用 JS 资产
# ═══════════════════════════════════════════════════════════════

COPY_SCRIPT="${ROOT_DIR}/copy_runtime_js_assets.sh"
if [[ ! -f "${COPY_SCRIPT}" ]]; then
    echo -e "${RED}[错误] 未找到复制脚本: ${COPY_SCRIPT}${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}[汇总]${NC} 复制可调用 JS 资产到 runtime_js_assets/ ..."
bash "${COPY_SCRIPT}"
echo -e "${GREEN}[完成]${NC} 运行时 JS 资产已就绪: ${BLUE}${ROOT_DIR}/runtime_js_assets/${NC}"
