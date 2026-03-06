#!/bin/bash
# 请使用 bash 运行（./start_aubo_moveit_bridge_ros2.sh 或 bash start_aubo_moveit_bridge_ros2.sh），勿用 sh
# 若已用 sh 启动，则自动用 bash 重新执行
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

# 仅启动 ROS2 MoveIt Bridge（aubo_driver_ros2 模式）
# 用法: ./start_aubo_moveit_bridge_ros2.sh
# 对应: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py use_aubo_driver_ros2:=true

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 环境路径（与 start_IVG.sh 一致）
AUBO_ROS2_WS="/home/mu/IVG/aubo_ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS2 MoveIt Bridge (aubo_driver_ros2)${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 可选：在 terminator 新标签页中启动
USE_TERMINATOR="${USE_TERMINATOR:-0}"
if [ "$USE_TERMINATOR" = "1" ]; then
    TERMINATOR=""
    if command -v terminator &>/dev/null; then
        TERMINATOR="terminator"
    elif [ -x /usr/bin/terminator ]; then
        TERMINATOR="/usr/bin/terminator"
    fi
    if [ -z "$TERMINATOR" ]; then
        echo -e "${RED}错误: USE_TERMINATOR=1 但未找到 terminator${NC}"
        exit 1
    fi
    CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py use_aubo_driver_ros2:=true"
    escaped_cmd=$(echo "$CMD" | sed "s/'/'\"'\"'/g")
    "$TERMINATOR" --new-tab --title="ROS2 MoveIt Bridge" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    echo -e "${GREEN}已在 terminator 新标签页中启动 ROS2 MoveIt Bridge${NC}"
    echo -e "${YELLOW}用法: USE_TERMINATOR=1 ./start_aubo_moveit_bridge_ros2.sh${NC}"
    exit 0
fi

# 默认：当前终端直接运行
echo -e "${BLUE}正在构建并启动 ROS2 MoveIt Bridge (use_aubo_driver_ros2:=true)...${NC}"
cd "$AUBO_ROS2_WS" || exit 1
source "$ROS2_SETUP"
colcon build
source install/setup.bash
exec $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py use_aubo_driver_ros2:=true
