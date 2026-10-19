#!/usr/bin/env bash
# 可选 demo：MoveIt 2 官方 Panda 示例 + ros_web_link（不是 core 运行依赖）。
# 用法: bash run_panda_demo.sh [web_port] [bridge_port] [domain_id]
# 前置: 工作区已构建 ros_web_link / ivg_foxglove_bridge /
#       moveit_resources_panda_{description,moveit_config}，并 source install/setup.bash。
set -euo pipefail

WEB_PORT="${1:-8095}"
BRIDGE_PORT="${2:-18766}"          # 8765 常被 preview_server 占用
DOMAIN="${3:-80}"

need_prefix() {
  local pkg="$1"
  ros2 pkg prefix "$pkg" 2>/dev/null || {
    echo "[run_panda_demo] 缺少包: $pkg（先构建并 source 工作区）" >&2
    exit 2
  }
}

PREFIX="$(need_prefix ros_web_link)"
BRIDGE_PREFIX="$(need_prefix ivg_foxglove_bridge)"
PANDA_DESC="$(need_prefix moveit_resources_panda_description)"
PANDA_MOVEIT="$(need_prefix moveit_resources_panda_moveit_config)"
WS="$(dirname "$(dirname "$PREFIX")")"

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DOMAIN}"
export AMENT_PREFIX_PATH="$PREFIX:$BRIDGE_PREFIX:$PANDA_DESC:$PANDA_MOVEIT:/opt/ros/jazzy"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/ros_web_link:${PYTHONPATH:-}"

# vendor 库链（sdformat14→gz-math7、ompl18 等）：MoveIt 解析/规划插件必需
VENDOR_LIBS="$(ls -d /opt/ros/jazzy/opt/*/lib 2>/dev/null | tr '\n' ':' || true)"
export LD_LIBRARY_PATH="${VENDOR_LIBS}/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:$BRIDGE_PREFIX/lib"

# 网关优先工作区 venv（系统 fastapi/starlette 组合可能不兼容）
if [[ -x "$WS/aubo_py3.12/bin/python" ]]; then
  export PATH="$WS/aubo_py3.12/bin:/opt/ros/jazzy/bin:$PATH"
else
  export PATH="/opt/ros/jazzy/bin:$PATH"
  echo "[run_panda_demo] 未找到 $WS/aubo_py3.12，使用系统 python3（需已装 fastapi/uvicorn/websockets）" >&2
fi

exec ros2 launch ros_web_link panda_moveit_web.launch.py \
  web_port:="$WEB_PORT" bridge_port:="$BRIDGE_PORT"
