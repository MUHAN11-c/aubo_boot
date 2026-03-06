#!/bin/bash
# 移植后启动脚本：全 ROS2 模式（不启动 ROS1、不启动 ros1_bridge）
# 轨迹与驱动均在 ROS2 闭环：aubo_driver_ros2 + aubo_robot_simulator_ros2 + aubo_ros2_trajectory_action
#
# 用法: ./start_IVG_ros2.sh  或  bash start_IVG_ros2.sh
# 请使用 bash 运行，勿用 sh（避免 echo -e 等不生效）
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

AUBO_ROS2_WS="/home/mu/IVG/aubo_ros2_ws"
ROS2_WS="/home/mu/IVG/ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统启动脚本（移植后 / 全 ROS2）${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}模式: 不启动 ROS1，不启动 ros1_bridge，使用 aubo_driver_ros2${NC}"
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

echo -e "${BLUE}使用 terminator 分屏终端，每个节点独立标签页${NC}"
echo ""

launch_in_terminator() {
    local title=$1
    local cmd=$2
    local escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5
}

# [1/9] ROS2 MoveIt（含 aubo_driver_ros2 + aubo_robot_simulator_ros2 + trajectory_action）
echo -e "${GREEN}[1/9] 启动 ROS2 MoveIt（全 ROS2 driver）...${NC}"
MOVEIT_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py use_aubo_driver_ros2:=true"
launch_in_terminator "ROS2 MoveIt (aubo_driver_ros2)" "$MOVEIT_CMD"
echo -e "${GREEN}  ✓ ROS2 MoveIt 已启动${NC}"
sleep 3

# [2/9] MoveIt 工作空间限制
echo -e "${GREEN}[2/9] 启动 MoveIt 工作空间限制（边界墙）...${NC}"
LIMIT_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && /usr/bin/python3 src/aubo_ros2_driver/aubo_moveit_config/scripts/limit_workspace.py"
launch_in_terminator "MoveIt Workspace Limit" "$LIMIT_CMD"
echo -e "${GREEN}  ✓ 工作空间限制已启动${NC}"
sleep 3

# [3/9] 机器人驱动服务
echo -e "${GREEN}[3/9] 启动机器人驱动服务...${NC}"
DRIVER_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# [4/9] 相机节点
echo -e "${GREEN}[4/9] 启动相机节点...${NC}"
CAMERA_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select percipio_camera && source install/setup.bash && $ROS2_BIN/ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5

# [5/9] 相机控制节点
echo -e "${GREEN}[5/9] 启动相机控制节点...${NC}"
CAMERA_CTRL_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select percipio_camera_interface && source install/setup.bash && $ROS2_BIN/ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CTRL_CMD"
echo -e "${GREEN}  ✓ 相机控制已启动${NC}"
sleep 2

# [6/9] 图像数据桥接
echo -e "${GREEN}[6/9] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select image_data_bridge && source install/setup.bash && $ROS2_BIN/ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像桥接已启动${NC}"
sleep 2

# [7/9] 手眼标定节点
echo -e "${GREEN}[7/9] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select hand_eye_calibration && source install/setup.bash && $ROS2_BIN/ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定已启动${NC}"
sleep 2

# [8/9] 视觉姿态估计算法
echo -e "${GREEN}[8/9] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select visual_pose_estimation_python && source install/setup.bash && export PATH=\"/usr/bin:\$PATH\" && $ROS2_BIN/ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计已启动${NC}"
sleep 2

# [9/9] HTTP 桥接服务器
echo -e "${GREEN}[9/9] 启动 HTTP 桥接服务器...${NC}"
WEB_UI_DIR="$AUBO_ROS2_WS/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui"
HTTP_BRIDGE_CMD="cd $WEB_UI_DIR && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && /usr/bin/python3 scripts/http_bridge_server.py"
launch_in_terminator "HTTP Bridge Server" "$HTTP_BRIDGE_CMD"
echo -e "${GREEN}  ✓ HTTP 桥接服务器已启动${NC}"
sleep 2

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有节点已启动完成（移植后 / 全 ROS2）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}各节点在 terminator 独立标签页中运行，可查看实时日志${NC}"
echo ""
echo -e "${GREEN}等待节点就绪...${NC}"
sleep 5
echo -e "${GREEN}启动完成。手眼标定系统已就绪。${NC}"
echo -e "${GREEN}手眼标定 Web: http://localhost:8080${NC}"
echo -e "${GREEN}视觉姿态估计 Web: http://localhost:8088/index.html${NC}"
echo ""
echo -e "${YELLOW}停止节点示例（按需执行）:${NC}"
echo "  pkill -f 'aubo_moveit_bridge_ros1'"
echo "  pkill -f 'demo_driver_services'"
echo "  pkill -f 'percipio_camera'"
echo "  pkill -f 'camera_control'"
echo "  pkill -f 'image_data_bridge'"
echo "  pkill -f 'hand_eye_calibration'"
echo "  pkill -f 'visual_pose_estimation_python'"
echo "  pkill -f 'http_bridge_server'"
echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（已启动的节点继续运行）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
