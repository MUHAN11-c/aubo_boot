#!/usr/bin/env bash
set -euo pipefail

# 手动测试入口脚本
# 示例:
#   ./run_manual_moveitpy_test.sh joints
#   ./run_manual_moveitpy_test.sh pose
#   ./run_manual_moveitpy_test.sh joints --joint-index 1 --joint-delta 0.03

MODE="${1:-joints}"
shift || true

source /opt/ros/humble/setup.bash
source /home/wjz/ws_moveit/install/setup.bash
source /home/wjz/aubo_boot/aubo_ros2_ws/install/setup.bash

python3 /home/wjz/aubo_boot/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline/manual_moveitpy_test.py \
  --mode "${MODE}" "$@"
