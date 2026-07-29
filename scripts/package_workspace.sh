#!/usr/bin/env bash
# 制作商业化交付源码归档（显式清单制）。
# 打包内容 = 交付标准结构：README/LICENSE/AGENTS/requirements + src + docs +
# tools + diagnostics + scripts；其余（构建产物、运行时输出、本机环境、
# 厂商原始资料）一律不进入归档。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
package_name="aubo_e5_jazzy_ws"
output_dir="${1:-${workspace_dir}/release}"
timestamp="$(date +%Y%m%d_%H%M%S)"
archive="${output_dir}/${package_name}_source_${timestamp}.tar.gz"
staging_dir="$(mktemp -d)"

cleanup() {
  rm -rf "${staging_dir}"
}
trap cleanup EXIT

mkdir -p "${output_dir}" "${staging_dir}/${package_name}"

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

# 剔除随目录带入的本地产物（.gitignore 覆盖的 gitignored 项）
find "${staging_dir}" -type d -name "__pycache__" -prune -exec rm -rf {} +
find "${staging_dir}" -type d -name ".venv-yolo" -prune -exec rm -rf {} +
find "${staging_dir}" -type f -name "*.pyc" -delete
rm -rf "${staging_dir}/${package_name}/diagnostics/build" \
       "${staging_dir}/${package_name}/diagnostics/results"

tar -C "${staging_dir}" -czf "${archive}" "${package_name}"

printf 'Created source archive: %s\n' "${archive}"
printf '%s\n' 'Excluded: build/ install/ log/ release/（构建产物）、test_results/ hand_eye/（运行时输出）、aubo_py3.12/（本机 venv）、SDK资料/（厂商原始资料）。'
printf '%s\n' 'Note: src/aubo_e5_hardware/vendor 与 src/percipio_camera 为厂商专有代码；交付前请确认再分发权。'
