#!/bin/bash
# run_panda_demo.sh — 启动 MoveIt 2 官方 Panda 示例 + ros_web_link 端到端栈
# 用法: bash run_panda_demo.sh [web_port] [bridge_port] [domain_id]
# 注意：桥端口默认 18766（8765 常被本机 preview_server 外来进程占用）
set -u
WEB_PORT="${1:-8095}"
BRIDGE_PORT="${2:-18766}"
DOMAIN="${3:-80}"
WS=/home/mu/Desktop/aubo_e5_jazzy_ws

export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID="$DOMAIN"
export AMENT_PREFIX_PATH="$WS/install/ros_web_link:$WS/install/ivg_foxglove_bridge:$WS/install/moveit_resources_panda_description:$WS/install/moveit_resources_panda_moveit_config:/opt/ros/jazzy"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/ros_web_link"
# vendor 库链（sdformat14→gz-math7、ompl18 等）：MoveIt 解析/规划插件必需
VENDOR_LIBS=$(ls -d /opt/ros/jazzy/opt/*/lib 2>/dev/null | tr '\n' ':')
export LD_LIBRARY_PATH="${VENDOR_LIBS}/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:$WS/install/ivg_foxglove_bridge/lib"
# 网关须跑 aubo_py3.12 venv（系统 fastapi/starlette 版本组合会崩）
export PATH="$WS/aubo_py3.12/bin:/opt/ros/jazzy/bin:$PATH"

exec ros2 launch ros_web_link panda_moveit_web.launch.py \
  web_port:="$WEB_PORT" bridge_port:="$BRIDGE_PORT"
