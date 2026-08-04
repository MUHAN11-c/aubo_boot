#!/usr/bin/env bash
# 制作商业化交付源码归档（显式清单制）。
# 打包内容 = 交付标准结构：README/LICENSE/AGENTS/requirements + src + docs +
# tools + diagnostics + scripts；其余（构建产物、运行时输出、本机环境、
# 厂商原始资料）一律不进入归档。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_name="aubo_e5_jazzy_ws"
output_dir="${1:-${workspace_dir}/release}"
staging_dir="$(mktemp -d)"

cleanup() {
  rm -rf "${staging_dir}"
}
trap cleanup EXIT

mkdir -p "${output_dir}" "${staging_dir}/${package_name}"

# ---- 版本号命名：扫描输出目录已有 V<n> 归档，取最大编号 +1 ----
next_version=1
for existing in "${output_dir}/${package_name}_source_V"*.tar.gz; do
  [[ -e "${existing}" ]] || continue
  version="${existing##*_source_V}"
  version="${version%.tar.gz}"
  if [[ "${version}" =~ ^[0-9]+$ ]] && (( version >= next_version )); then
    next_version=$(( version + 1 ))
  fi
done
archive="${output_dir}/${package_name}_source_V${next_version}.tar.gz"

# ---- 显式打包清单（交付标准结构）----
copy_if_present() {
  local item="$1"
  if [[ -e "${workspace_dir}/${item}" ]]; then
    cp -a "${workspace_dir}/${item}" "${staging_dir}/${package_name}/"
  else
    printf 'WARN: 清单项缺失：%s\n' "${item}" >&2
  fi
}

copy_if_present README.md
copy_if_present LICENSE
copy_if_present AGENTS.md
copy_if_present requirements.txt
copy_if_present .gitignore
copy_if_present src
copy_if_present docs
copy_if_present tools
copy_if_present diagnostics
copy_if_present scripts

# 剔除随目录带入的本地产物（.gitignore 覆盖的 gitignored 项与缓存）
find "${staging_dir}" -type d \( -name "__pycache__" -o -name ".pytest_cache" \
       -o -name ".venv-yolo" -o -name ".playwright-mcp" \) -prune -exec rm -rf {} +
find "${staging_dir}" -type f -name "*.pyc" -delete
rm -rf "${staging_dir}/${package_name}/diagnostics/build" \
       "${staging_dir}/${package_name}/diagnostics/results" \
       "${staging_dir}/${package_name}/src/hand_eye" \
       "${staging_dir}/${package_name}/src/percipio_camera/depth_eval_out"

tar -C "${staging_dir}" -czf "${archive}" "${package_name}"

printf 'Created source archive: %s\n' "${archive}"
printf '%s\n' 'Excluded: build/ install/ log/ release/（构建产物）、test_results/ hand_eye/（运行时输出）、aubo_py3.12/（本机 venv）、SDK资料/（厂商原始资料）。'
printf '%s\n' 'Note: src/aubo_e5_hardware/vendor 与 src/percipio_camera 为厂商专有代码；交付前请确认再分发权。'
