#!/bin/bash
# 移植前启动：三步——ROS1、桥接、ROS2 MoveIt（simulator_in_ros2:=false）
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

PROJECT_ROOT="/home/mu/IVG2.0"
AUBO_ROS2_WS="${PROJECT_ROOT}/aubo_ros2_ws"
ROS1_ROOT="${ROS1_ROOT:-/home/mu/IVG}"
ROS_CATKIN_WS="${ROS1_ROOT}/ros_catkin_ws/install_isolated/setup.bash"
WS_MOVEIT="${ROS1_ROOT}/ws_moveit/install_isolated/setup.bash"
AUBO_WS="${PROJECT_ROOT}/aubo_ws"
ROS2_WS="${ROS1_ROOT}/ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"

TERMINATOR=""
if command -v terminator &>/dev/null; then TERMINATOR="terminator"; elif [ -x /usr/bin/terminator ]; then TERMINATOR="/usr/bin/terminator"; fi
if [ -z "$TERMINATOR" ]; then
    echo -e "${RED}错误: 未找到 terminator${NC}"
    exit 1
fi

launch_in_terminator() {
    local title=$1
    local cmd=$2
    local escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5
}

echo -e "${GREEN}[1/3] 启动 ROS1（含 aubo_robot_simulator）...${NC}"
ROS1_CMD="cd $AUBO_WS && source $ROS_CATKIN_WS && source $WS_MOVEIT && catkin_make_isolated && source devel_isolated/setup.bash && roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge_before_migration.launch"
launch_in_terminator "ROS1 MoveIt (before)" "$ROS1_CMD"
sleep 3

echo -e "${GREEN}[2/3] 启动 ROS1-ROS2 桥接...${NC}"
BRIDGE_CMD="cd $ROS2_WS && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && source install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test"
launch_in_terminator "ROS1-ROS2 Bridge" "$BRIDGE_CMD"
sleep 3

echo -e "${GREEN}[3/3] 启动 ROS2 MoveIt（simulator_in_ros2:=false）...${NC}"
MOVEIT_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py simulator_in_ros2:=false"
launch_in_terminator "ROS2 MoveIt (before)" "$MOVEIT_CMD"
sleep 3
