#!/usr/bin/env bash
# 战役注入矩阵栈启动器（后台日志 /tmp/campaign_m1_stack.log）
# 用法: launch_stack.sh <tool_profile> [额外 launch 参数...]（由调用方传 camera/imu 等）
# 注意不能 set -u：ROS setup.bash 依赖未绑定变量探测（AMENT_TRACE_SETUP_FILES）。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
PROFILE="${1:?用法: launch_stack.sh <tool_profile> [extra launch args...]}"
shift
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"  # 战役独立 DDS 域（并发会话在 33/46，勿混用）
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true \
  imu_enabled:=false tool_profile:="$PROFILE" "$@"
