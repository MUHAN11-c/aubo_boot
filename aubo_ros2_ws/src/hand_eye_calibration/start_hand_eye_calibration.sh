#!/bin/bash

# 手眼标定完整启动脚本
# 按顺序启动所有必要的节点和服务
# 使用 terminator 创建分屏终端

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 不再需要日志目录，直接在 terminator 标签页中查看

# 环境路径
ROS1_SETUP="/opt/ros/noetic/setup.bash"
ROS2_SETUP="/opt/ros/foxy/setup.bash"
AUBO_WS="/home/mu/IVG/aubo_ws"
AUBO_ROS2_WS="/home/mu/IVG/aubo_ros2_ws"
ROS2_WS="/home/mu/ros2_ws"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检测 terminator
if ! command -v terminator &> /dev/null; then
    echo -e "${RED}错误: 未找到 terminator，请先安装: sudo apt install terminator${NC}"
    exit 1
fi

echo -e "${BLUE}将使用 terminator 创建分屏终端${NC}"
echo -e "${BLUE}每个节点在独立的标签页中运行，可直接查看日志${NC}"
echo ""


# 启动函数：在 terminator 标签页中运行
launch_in_terminator() {
    local title=$1
    local cmd=$2
    
    # 转义命令中的特殊字符
    local escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")
    
    # 使用 terminator 创建新标签页，直接显示输出（不保存日志文件）
    terminator --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5  # 给 terminator 一点时间创建窗口
}

# 步骤1: 启动 ROS1 MoveIt 配置
echo -e "${GREEN}[1/8] 启动 ROS1 MoveIt 配置...${NC}"
ROS1_CMD="cd $AUBO_WS && source $ROS1_SETUP && catkin_make && source devel/setup.bash && roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge.launch"
launch_in_terminator "ROS1 MoveIt" "$ROS1_CMD"
echo -e "${GREEN}  ✓ ROS1 MoveIt 已启动${NC}"
sleep 3

# 步骤2: 启动 ROS1-ROS2 桥接
echo -e "${GREEN}[2/8] 启动 ROS1-ROS2 桥接...${NC}"
BRIDGE_CMD="cd $ROS2_WS && source $ROS1_SETUP && source $AUBO_WS/devel/setup.bash && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure && source install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test"
launch_in_terminator "ROS1-ROS2 Bridge" "$BRIDGE_CMD"
echo -e "${GREEN}  ✓ ROS1-ROS2 桥接已启动${NC}"
sleep 3

# 步骤3: 启动 ROS2 MoveIt 桥接
echo -e "${GREEN}[3/8] 启动 ROS2 MoveIt 桥接...${NC}"
MOVEIT_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py"
launch_in_terminator "ROS2 MoveIt Bridge" "$MOVEIT_BRIDGE_CMD"
echo -e "${GREEN}  ✓ ROS2 MoveIt 桥接已启动${NC}"
sleep 3

# 步骤4: 启动机器人驱动服务
echo -e "${GREEN}[4/8] 启动机器人驱动服务...${NC}"
DRIVER_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤5: 启动相机节点
echo -e "${GREEN}[5/8] 启动相机节点...${NC}"
CAMERA_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5  # 相机需要更多时间初始化

# 步骤6: 启动相机控制节点
echo -e "${GREEN}[6/8] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤7: 启动图像数据桥接节点
echo -e "${GREEN}[7/8] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤8: 启动手眼标定节点
echo -e "${GREEN}[8/8] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有节点已启动完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

echo -e "${BLUE}每个节点都在独立的 terminator 标签页中运行${NC}"
echo -e "${BLUE}您可以在各自的标签页中查看实时日志${NC}"

echo ""
echo -e "${GREEN}等待所有节点完全启动...${NC}"
sleep 5

echo -e "${GREEN}启动完成！手眼标定系统已就绪。${NC}"
echo -e "${GREEN}Web界面地址: http://localhost:8080${NC}"
echo ""
echo -e "${YELLOW}提示: 关闭 terminator 标签页或按 Ctrl+C 停止对应节点${NC}"
echo -e "${YELLOW}或使用以下命令停止所有节点:${NC}"
echo "  pkill -f 'aubo_e5_moveit_bridge'"
echo "  pkill -f 'ros1_bridge'"
echo "  pkill -f 'aubo_moveit_bridge_ros1'"
echo "  pkill -f 'demo_driver_services'"
echo "  pkill -f 'percipio_camera'"
echo "  pkill -f 'camera_control'"
echo "  pkill -f 'image_data_bridge'"
echo "  pkill -f 'hand_eye_calibration'"


# 保持脚本运行
echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（不会停止已启动的节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
