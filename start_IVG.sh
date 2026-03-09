#!/bin/bash
# 请使用 bash 运行（./start_IVG.sh 或 bash start_IVG.sh），勿用 sh start_IVG.sh
# 若已用 sh 启动，则自动用 bash 重新执行（避免 echo -e 等不生效）
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

# 手眼标定完整启动脚本
# 按顺序启动所有必要的节点和服务
# 使用 terminator 创建分屏终端
#
# 全 ROS2 driver 模式：无需启动 ROS1 与 ros1_bridge，轨迹全在 ROS2 闭环
#   用法: USE_AUBO_DRIVER_ROS2=1 ./start_IVG.sh
#   将跳过步骤 1、2，步骤 3 使用 use_aubo_driver_ros2:=true（aubo_driver_ros2 + 无 feedback_bridge）

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 不再需要日志目录，直接在 terminator 标签页中查看

# 环境路径（IVG 项目：ROS1 自编译链 + ROS2 Humble）
ROS_CATKIN_WS="/home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash"
WS_MOVEIT="/home/mu/IVG/ws_moveit/install_isolated/setup.bash"
AUBO_WS="/home/mu/IVG/aubo_ws"
AUBO_ROS2_WS="/home/mu/IVG/aubo_ros2_ws"
ROS2_WS="/home/mu/IVG/ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检测 terminator（兼容 PATH 未含 /usr/bin 或 sh 下 command -v 行为）
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
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5  # 给 terminator 一点时间创建窗口
}

# 全 ROS2 driver 模式：不启动 ROS1 与 bridge，由 aubo_driver_ros2 直接发布 joint_states / aubo/feedback_states
USE_FULL_ROS2_DRIVER="${USE_AUBO_DRIVER_ROS2:-0}"

if [ "$USE_FULL_ROS2_DRIVER" != "1" ]; then
  # 步骤1: 启动 ROS1 MoveIt 配置（IVG：ros_catkin_ws → ws_moveit → aubo_ws，使用 catkin_make_isolated）
  echo -e "${GREEN}[1/10] 启动 ROS1 MoveIt 配置...${NC}"
  ROS1_CMD="cd $AUBO_WS && source $ROS_CATKIN_WS && source $WS_MOVEIT && catkin_make_isolated && source devel_isolated/setup.bash && roslaunch aubo_e5_moveit_config aubo_e5_moveit_bridge.launch"
  launch_in_terminator "ROS1 MoveIt" "$ROS1_CMD"
  echo -e "${GREEN}  ✓ ROS1 MoveIt 已启动${NC}"
  sleep 3

  # 步骤2: 启动 ROS1-ROS2 桥接（仅需 ROS2 环境：Humble + aubo_ros2_ws + ros2_ws）
  echo -e "${GREEN}[2/10] 启动 ROS1-ROS2 桥接...${NC}"
  BRIDGE_CMD="cd $ROS2_WS && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && source install/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --ros-args -r __node:=ros_bridge_test"
  launch_in_terminator "ROS1-ROS2 Bridge" "$BRIDGE_CMD"
  echo -e "${GREEN}  ✓ ROS1-ROS2 桥接已启动${NC}"
  sleep 3
fi

# 步骤3: 启动 ROS2 MoveIt（含轨迹插值 aubo_robot_simulator_ros2；全 ROS2 时含 aubo_driver_ros2，无 feedback_bridge）
echo -e "${GREEN}[3/11] 启动 ROS2 MoveIt...${NC}"
if [ "$USE_FULL_ROS2_DRIVER" = "1" ]; then
  echo -e "${BLUE}  全 ROS2 driver 模式：aubo_driver_ros2，不启动 feedback_bridge${NC}"
  MOVEIT_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py use_aubo_driver_ros2:=true"
else
  MOVEIT_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py"
fi
launch_in_terminator "ROS2 MoveIt Bridge" "$MOVEIT_BRIDGE_CMD"
echo -e "${GREEN}  ✓ ROS2 MoveIt 已启动${NC}"
sleep 3

# 步骤4: 启动 MoveIt 工作空间限制（边界墙）- 暂时不启动
# echo -e "${GREEN}[4/11] 启动 MoveIt 工作空间限制（边界墙）...${NC}"
# LIMIT_WORKSPACE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash && /usr/bin/python3 src/aubo_ros2_driver/aubo_moveit_config/scripts/limit_workspace.py"
# launch_in_terminator "MoveIt Workspace Limit" "$LIMIT_WORKSPACE_CMD"
# echo -e "${GREEN}  ✓ MoveIt 工作空间限制已启动${NC}"
# sleep 3

# 步骤5: 启动机器人驱动服务
echo -e "${GREEN}[5/11] 启动机器人驱动服务...${NC}"
DRIVER_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build && source install/setup.bash && $ROS2_BIN/ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤6: 启动相机节点
echo -e "${GREEN}[6/11] 启动相机节点...${NC}"
CAMERA_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select percipio_camera && source install/setup.bash && $ROS2_BIN/ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5  # 相机需要更多时间初始化

# 步骤7: 启动相机控制节点
echo -e "${GREEN}[7/11] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select percipio_camera_interface && source install/setup.bash && $ROS2_BIN/ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤8: 启动图像数据桥接节点
echo -e "${GREEN}[8/11] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select image_data_bridge && source install/setup.bash && $ROS2_BIN/ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤9: 启动手眼标定节点
echo -e "${GREEN}[9/11] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select hand_eye_calibration && source install/setup.bash && $ROS2_BIN/ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"
sleep 2

# 步骤10: 启动视觉姿态估计算法节点（RemBG 子进程使用 conda 环境 ros2_env）
echo -e "${GREEN}[10/11] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="cd $AUBO_ROS2_WS && source $ROS2_SETUP && colcon build --packages-select visual_pose_estimation_python && source install/setup.bash && export PATH=\"/usr/bin:\$PATH\" && $ROS2_BIN/ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计算法节点已启动${NC}"
sleep 2

# 步骤11: 启动HTTP桥接服务器（RemBG：当前 Python 有 rembg 则进程内使用，否则子进程回退到 conda ros2_env）
echo -e "${GREEN}[11/11] 启动HTTP桥接服务器...${NC}"
WEB_UI_DIR="$AUBO_ROS2_WS/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui"
# 使用系统 Python 3.8（可导入 rclpy）；若已安装 rembg 则进程内使用，否则子进程用 conda ros2_env
HTTP_BRIDGE_CMD="cd $WEB_UI_DIR && source $ROS2_SETUP && source $AUBO_ROS2_WS/install/setup.bash && /usr/bin/python3 scripts/http_bridge_server.py"
launch_in_terminator "HTTP Bridge Server" "$HTTP_BRIDGE_CMD"
echo -e "${GREEN}  ✓ HTTP桥接服务器已启动${NC}"
sleep 2

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
echo -e "${GREEN}手眼标定Web界面地址: http://localhost:8080${NC}"
echo -e "${GREEN}视觉姿态估计Web界面地址: http://localhost:8088/index.html${NC}"
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
echo "  pkill -f 'visual_pose_estimation_python'"
echo "  pkill -f 'http_bridge_server'"


# 保持脚本运行
echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（不会停止已启动的节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
