#!/usr/bin/env bash
# Package the reproducible source workspace for transfer; generated ROS artifacts are excluded.
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

copy_if_present() {
  local item="$1"
  if [[ -e "${workspace_dir}/${item}" ]]; then
    cp -a "${workspace_dir}/${item}" "${staging_dir}/${package_name}/"
  fi
}

copy_if_present README.md
copy_if_present docs
copy_if_present src
copy_if_present colcon.meta
copy_if_present scripts

tar -C "${staging_dir}" -czf "${archive}" "${package_name}"

printf 'Created source archive: %s\n' "${archive}"
printf '%s\n' 'Excluded generated directories: build/, install/, log/, release/.'
printf '%s\n' 'Note: src/aubo_e5_hardware/vendor may contain proprietary AUBO SDK files; confirm redistribution rights before sharing the archive.'
