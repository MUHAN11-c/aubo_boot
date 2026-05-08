#!/bin/bash
# 手眼标定：纯 ROS2 启动（MoveIt + demo 服务 + 相机 + ImageData 桥接 + 标定节点）
# 请使用 bash 运行

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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUBO_ROS2_WS="${AUBO_ROS2_WS:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS2_BASE_ENV="source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && if [ -f ~/ws_moveit/install/setup.bash ]; then source ~/ws_moveit/install/setup.bash; fi"
WS_ENV="cd $AUBO_ROS2_WS && $ROS2_BASE_ENV && if [ -f install/setup.bash ]; then source install/setup.bash; fi"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统启动（纯 ROS2）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

TERMINATOR=""
if command -v terminator &>/dev/null; then
    TERMINATOR="terminator"
elif [ -x /usr/bin/terminator ]; then
    TERMINATOR="/usr/bin/terminator"
fi
if [ -z "$TERMINATOR" ]; then
    echo -e "${RED}错误: 未找到 terminator，请先安装: sudo apt install terminator${NC}"
    exit 1
fi

launch_in_terminator() {
    local title=$1
    local cmd=$2
    local escaped_cmd
    escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5
}

echo -e "${GREEN}[1/6] 启动 MoveIt（aubo_new_driver）...${NC}"
launch_in_terminator "Aubo MoveIt New Driver" "$WS_ENV && ros2 launch aubo_moveit_config aubo_new_driver.launch.py"
sleep 3

echo -e "${GREEN}[2/6] 启动机器人驱动服务...${NC}"
launch_in_terminator "Robot Driver" "$WS_ENV && ros2 launch aubo_moveit_config demo_driver_services.launch.py"
sleep 3

echo -e "${GREEN}[3/6] 启动相机节点...${NC}"
launch_in_terminator "Percipio Camera" "$WS_ENV && ros2 launch percipio_camera percipio_camera.launch.py"
sleep 5

echo -e "${GREEN}[4/6] 启动相机控制节点...${NC}"
launch_in_terminator "Camera Control" "$WS_ENV && ros2 launch percipio_camera_interface camera_control.launch.py"
sleep 2

echo -e "${GREEN}[5/6] 启动图像数据桥接（image_data_bridge）...${NC}"
launch_in_terminator "Image Data Bridge" "$WS_ENV && ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
sleep 2

echo -e "${GREEN}[6/6] 启动手眼标定节点...${NC}"
launch_in_terminator "Hand Eye Calibration" "$WS_ENV && ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有节点已启动完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Web 界面（若已配置）: http://localhost:8080${NC}"
echo -e "${YELLOW}停止示例: pkill -f 'aubo_moveit_pure_ros2'; pkill -f 'demo_driver_services'; pkill -f 'percipio_camera'; pkill -f 'camera_control'; pkill -f 'image_data_bridge'; pkill -f 'hand_eye_calibration'${NC}"
echo ""
echo -e "${BLUE}按 Ctrl+C 退出本脚本（不停止已启动节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出${NC}'; exit 0" INT
while true; do sleep 1; done
