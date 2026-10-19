#!/usr/bin/env bash
# 完整官方仿真 + ros_web_link 独立监控 E2E（数据面只用 src/examples 官方项目）。
# 需要 /dev/shm（FastDDS）与 /dev/dri（gz 渲染）；容器/沙箱请在完整权限 shell 下运行。
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_HOME="${ROS_HOME:-$WS/.ros-home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/.ros-log}"
export PYTHONPATH="$WS/src/ros_web_link:${PYTHONPATH:-}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export RWL_FULL_SIM=1
exec ./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k full_sim -v "$@"
