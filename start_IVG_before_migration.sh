#!/bin/bash
# 移植前启动：轨迹插值在 ROS1（aubo_robot_simulator），并记录关键数据到 .cursor/motion_log_before.ndjson 用于对比分析抖动
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_CATKIN_WS="/home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash"
WS_MOVEIT="/home/mu/IVG/ws_moveit/install_isolated/setup.bash"
AUBO_WS="/home/mu/IVG/aubo_ws"
AUBO_ROS2_WS="/home/mu/IVG/aubo_ros2_ws"
ROS2_WS="/home/mu/IVG/ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"

# 关键数据日志：移植前由 ROS1 aubo_robot_simulator 写入
export MOTION_LOG_FILE="/home/mu/IVG/.cursor/motion_log_before.ndjson"
mkdir -p /home/mu/IVG/.cursor
: > "$MOTION_LOG_FILE"
echo -e "${BLUE}移植前模式：关键数据将写入 ${MOTION_LOG_FILE}${NC}"
echo ""

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

# 步骤1: ROS1 含 aubo_robot_simulator（移植前）
echo -e "${GREEN}[1/3] 启动 ROS1（含 aubo_robot_simulator，写日志到 MOTION_LOG_FILE）...${NC}"
ROS1_CMD="export MOTION_LOG_FILE='$MOTION_LOG_FILE' && cd $AUBO_WS && source $ROS_CATKIN_WS && source $WS_MOVEIT && catkin_make_isolated && source devel_isolated/setup.bash && roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge_before_migration.launch"
launch_in_terminator "ROS1 MoveIt (before)" "$ROS1_CMD"
echo -e "${GREEN}  ✓ 已启动${NC}"
sleep 3

# 步骤2: 桥接
echo -e "${GREEN}[2/3] 启动 ROS1-ROS2 桥接...${NC}"
BRIDGE_CMD="cd $ROS2_WS && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && source install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test"
launch_in_terminator "ROS1-ROS2 Bridge" "$BRIDGE_CMD"
echo -e "${GREEN}  ✓ 已启动${NC}"
sleep 3

# 步骤3: ROS2 MoveIt 不启动 aubo_robot_simulator_ros2，轨迹经 bridge 到 ROS1 插值
echo -e "${GREEN}[3/3] 启动 ROS2 MoveIt（simulator_in_ros2:=false，插值在 ROS1）...${NC}"
MOVEIT_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py simulator_in_ros2:=false"
launch_in_terminator "ROS2 MoveIt (before)" "$MOVEIT_CMD"
echo -e "${GREEN}  ✓ 已启动${NC}"
sleep 3

echo ""
echo -e "${GREEN}移植前启动完成。执行若干轨迹后，关键数据在: ${MOTION_LOG_FILE}${NC}"
echo -e "${YELLOW}可再运行 start_IVG_after_migration.sh 生成 motion_log_after.ndjson，用 analyze_motion_logs.md 对比。${NC}"
echo ""
trap "exit 0" INT
while true; do sleep 1; done
