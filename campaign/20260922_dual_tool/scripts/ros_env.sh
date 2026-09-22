#!/usr/bin/env bash
# 战役 ROS 环境包装：source 完整 overlay 后执行传入命令（域 61）。
# 用法: ros_env.sh <命令> [参数...]   例: ros_env.sh ros2 lifecycle get /peach_arm
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
# 不能 set -u：ROS setup.bash 依赖未绑定变量探测
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec "$@"
