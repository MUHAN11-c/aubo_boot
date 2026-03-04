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
# 使用纯 ROS2 启动：aubo_moveit_pure_ros2.launch.py（aubo_driver_ros2 + move_group + RViz + 轨迹插值），无需 ROS1 与 ros1_bridge

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 不再需要日志目录，直接在 terminator 标签页中查看

# 环境路径（IVG2.0 项目，纯 ROS2 Humble）
AUBO_ROS2_WS="/home/mu/IVG2.0/aubo_ros2_ws"
ROS2_SETUP="/opt/ros/humble/setup.bash"
ROS2_BIN="/opt/ros/humble/bin"
# 各步骤共用：进入工作空间并 source 环境（不再每步重复 colcon build）
WS_ENV="cd $AUBO_ROS2_WS && source $ROS2_SETUP && source install/setup.bash"

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

# 预先构建工作空间一次，后续各步骤仅 source，避免每步重复 colcon build
echo -e "${GREEN}[0/9] 构建工作空间...${NC}"
(cd "$AUBO_ROS2_WS" && source "$ROS2_SETUP" && colcon build)
echo -e "${GREEN}  ✓ 构建完成${NC}"
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

# 步骤1: 启动 ROS2 MoveIt（纯 ROS2：aubo_moveit_pure_ros2.launch.py，含 aubo_driver_ros2、轨迹插值、move_group、RViz）
echo -e "${GREEN}[1/9] 启动 ROS2 MoveIt（纯 ROS2）...${NC}"
MOVEIT_CMD="$WS_ENV && $ROS2_BIN/ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py"
launch_in_terminator "ROS2 MoveIt (Pure ROS2)" "$MOVEIT_CMD"
echo -e "${GREEN}  ✓ ROS2 MoveIt（纯 ROS2）已启动${NC}"
sleep 3

# 步骤2: 启动 MoveIt 工作空间限制（边界墙）
echo -e "${GREEN}[2/9] 启动 MoveIt 工作空间限制（边界墙）...${NC}"
LIMIT_WORKSPACE_CMD="$WS_ENV && /usr/bin/python3 src/aubo_ros2_driver/aubo_moveit_config/scripts/limit_workspace.py"
launch_in_terminator "MoveIt Workspace Limit" "$LIMIT_WORKSPACE_CMD"
echo -e "${GREEN}  ✓ MoveIt 工作空间限制已启动${NC}"
sleep 3

# 步骤3: 启动机器人驱动服务
echo -e "${GREEN}[3/9] 启动机器人驱动服务...${NC}"
DRIVER_CMD="$WS_ENV && $ROS2_BIN/ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤4: 启动相机节点
echo -e "${GREEN}[4/9] 启动相机节点...${NC}"
CAMERA_CMD="$WS_ENV && $ROS2_BIN/ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5  # 相机需要更多时间初始化

# 步骤5: 启动相机控制节点
echo -e "${GREEN}[5/9] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="$WS_ENV && $ROS2_BIN/ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤6: 启动图像数据桥接节点
echo -e "${GREEN}[6/9] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="$WS_ENV && $ROS2_BIN/ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤7: 启动手眼标定节点
echo -e "${GREEN}[7/9] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="$WS_ENV && $ROS2_BIN/ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"
sleep 2

# 步骤8: 启动视觉姿态估计算法节点（RemBG 子进程使用 conda 环境 ros2_env）
echo -e "${GREEN}[8/9] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="$WS_ENV && export PATH=\"/usr/bin:\$PATH\" && $ROS2_BIN/ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计算法节点已启动${NC}"
sleep 2

# 步骤9: 启动HTTP桥接服务器（RemBG：当前 Python 有 rembg 则进程内使用，否则子进程回退到 conda ros2_env）
echo -e "${GREEN}[9/9] 启动HTTP桥接服务器...${NC}"
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
echo "  pkill -f 'aubo_moveit_pure_ros2'"
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
