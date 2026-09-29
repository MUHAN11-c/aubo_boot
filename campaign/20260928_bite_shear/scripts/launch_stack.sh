#!/usr/bin/env bash
# bite 主测轮栈启动器（后台日志 /tmp/campaign_bite_stack.log）
# 用法: launch_stack.sh <tool_profile> [额外 launch 参数...]（由调用方传 camera/imu 等）
# 默认 mock + skip_reconstruction，同 20260922_dual_tool 的 launch_stack 模式；
# 三把剪切手档案（shear_v1 / bite_shear_v1 / adaptive_shear_v1）均可传。
# 注意不能 set -u：ROS setup.bash 依赖未绑定变量探测（AMENT_TRACE_SETUP_FILES）。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
PROFILE="${1:?用法: launch_stack.sh <tool_profile> [extra launch args...]}"
shift
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-62}"  # 本战役独立 DDS 域（61=旧 dual_tool 战役，勿混用）
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=mock camera_enabled:=false skip_reconstruction:=true \
  imu_enabled:=false tool_profile:="$PROFILE" "$@"
