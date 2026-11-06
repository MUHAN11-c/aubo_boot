#!/usr/bin/env bash
# Build the complete Peach dependency closure and the browser assets from a fresh checkout.
set -eo pipefail
PEACH_BUILD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PEACH_BUILD_ROOT"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy is required. See docs/deployment.md.' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
node -e 'if (Number(process.versions.node.split(".")[0]) < 22) process.exit(2)'
for package in src/ros2_web_link src/peach/peach_web; do
  npm ci --prefix "$package" --no-audit --no-fund
done
npm --prefix src/ros2_web_link run build
npm --prefix src/peach/peach_web run build:vendor
npm --prefix run build
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
colcon build --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation \
  --parallel-workers "${PEACH_BUILD_WORKERS:-2}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
