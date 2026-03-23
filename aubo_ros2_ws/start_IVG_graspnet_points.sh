#!/bin/bash
# 请使用 bash 运行（./start_IVG_graspnet_points.sh 或 bash start_IVG_graspnet_points.sh），勿用 sh
# 若已用 sh 启动，则自动用 bash 重新执行（避免 echo -e 等不生效）
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

# 手眼标定完整启动脚本（GraspNet Points 版本）
# 按顺序启动所有必要的节点和服务
# 使用 terminator 创建分屏终端

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 环境路径（IVG2.0 项目，纯 ROS2 Humble）
AUBO_ROS2_WS="/home/mu/IVG2.0/aubo_ros2_ws"
# 固定使用 ROS2 环境链
ROS2_ENV_CHAIN="source /opt/ros/humble/setup.bash && source ~/ws_moveit/install/setup.bash && source install/setup.bash"
# 各步骤共用：进入工作空间并 source 环境（不再每步重复 colcon build）
WS_ENV="cd $AUBO_ROS2_WS && $ROS2_ENV_CHAIN"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}手眼标定系统启动脚本（GraspNet Points）${NC}"
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
echo -e "${GREEN}[0/13] 构建工作空间...${NC}"
(cd "$AUBO_ROS2_WS" && source /opt/ros/humble/setup.bash && source ~/ws_moveit/install/setup.bash && source install/setup.bash && colcon build)
echo -e "${GREEN}  ✓ 构建完成${NC}"
echo ""

# 启动函数：在 terminator 标签页中运行
launch_in_terminator() {
    local title=$1
    local cmd=$2

    # 转义命令中的特殊字符
    local escaped_cmd
    escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")

    # 使用 terminator 创建新标签页，直接显示输出（不保存日志文件）
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5  # 给 terminator 一点时间创建窗口
}

# 步骤1: 启动真实机械臂驱动（aubo_moveit_pure_ros2）
echo -e "${GREEN}[1/13] 启动真实机械臂驱动（aubo_moveit_pure_ros2）...${NC}"
AUBO_PURE_ROS2_CMD="$WS_ENV && ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py"
launch_in_terminator "Aubo MoveIt Pure ROS2" "$AUBO_PURE_ROS2_CMD"
echo -e "${GREEN}  ✓ 真实机械臂驱动已启动${NC}"
sleep 3

# 步骤2: 启动机器人驱动服务
echo -e "${GREEN}[2/13] 启动机器人驱动服务...${NC}"
DRIVER_CMD="$WS_ENV && ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤3: 启动相机节点
echo -e "${GREEN}[3/13] 启动相机节点...${NC}"
CAMERA_CMD="$WS_ENV && ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5  # 相机需要更多时间初始化

# 步骤4: 启动相机控制节点
echo -e "${GREEN}[4/13] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="$WS_ENV && ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤5: 启动图像数据桥接节点
echo -e "${GREEN}[5/13] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="$WS_ENV && ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤6: 启动手眼标定节点
echo -e "${GREEN}[6/13] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="$WS_ENV && ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"
sleep 2

# 步骤7: 启动视觉姿态估计算法节点（RemBG 子进程使用 conda 环境 ros2_env）
echo -e "${GREEN}[7/13] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="$WS_ENV && export PATH=\"/usr/bin:\$PATH\" && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计算法节点已启动${NC}"
sleep 2

# 步骤8: 启动 GraspNet 点云节点（提供 /graspnet_capture_control 服务）
echo -e "${GREEN}[8/13] 启动 GraspNet 点云节点（with_tf）...${NC}"
GRASPNET_WITH_TF_CMD="$WS_ENV && ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py launch_camera:=false launch_hand_eye_tf:=true"
launch_in_terminator "GraspNet Points With TF" "$GRASPNET_WITH_TF_CMD"
echo -e "${GREEN}  ✓ GraspNet 点云节点已启动${NC}"
sleep 2

# 步骤9: 启动执行抓取位姿服务节点（gripper0 模板匹配循环抓取链路）
echo -e "${GREEN}[9/13] 启动执行抓取位姿服务节点...${NC}"
GRASP_WORKER_CMD="$WS_ENV && ros2 launch demo_driver execute_grasp_pose_worker.launch.py"
launch_in_terminator "Execute Grasp Worker" "$GRASP_WORKER_CMD"
echo -e "${GREEN}  ✓ 执行抓取位姿服务节点已启动${NC}"
sleep 2

# 步骤10: 启动夹爪切换服务节点（gripper0 -> gripper2）
echo -e "${GREEN}[10/13] 启动夹爪切换服务节点...${NC}"
GRIPPER_SWAP_CMD="$WS_ENV && ros2 run demo_driver gripper_swap_worker_node"
launch_in_terminator "Gripper Swap Worker" "$GRIPPER_SWAP_CMD"
echo -e "${GREEN}  ✓ 夹爪切换服务节点已启动${NC}"
sleep 2

# 步骤11: 启动 GraspNet 循环抓取 Worker（gripper2 抓取链路）
echo -e "${GREEN}[11/13] 启动 GraspNet 循环抓取 Worker...${NC}"
# 注意：该节点内部使用了 `automatically_declare_parameters_from_overrides(true)` 并手动 declare_parameter，
# 若此处再通过 `--ros-args -p ...` 传入同名参数，会触发 ParameterAlreadyDeclaredException。
# 这里直接使用节点默认参数（当前默认值已满足本计划验证链路）。
PUBLISH_GRASPS_WORKER_CMD="$WS_ENV && ros2 run demo_driver publish_grasps_client_worker_node"
launch_in_terminator "Publish Grasps Worker" "$PUBLISH_GRASPS_WORKER_CMD"
echo -e "${GREEN}  ✓ GraspNet 循环抓取 Worker 已启动${NC}"
sleep 2

# 步骤12: 校验关键服务是否已就绪（便于计划验证）
echo -e "${GREEN}[12/13] 校验关键服务...${NC}"
SERVICE_CHECK_CMD="$WS_ENV && ros2 service list | rg '/graspnet_capture_control|/publish_grasps_worker_loop_control|/loop_grasp_control|/run_gripper_swap' || true"
launch_in_terminator "Service Check" "$SERVICE_CHECK_CMD"
echo -e "${GREEN}  ✓ 关键服务校验标签页已启动${NC}"
sleep 1

# 步骤13: 启动HTTP桥接服务器
echo -e "${GREEN}[13/13] 启动HTTP桥接服务器...${NC}"
WEB_UI_DIR="$AUBO_ROS2_WS/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui"
HTTP_BRIDGE_CMD="$WS_ENV && cd $WEB_UI_DIR && /usr/bin/python3 scripts/http_bridge_server.py"
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
echo "  pkill -f 'aubo_moveit_pure_ros2.launch.py'"
echo "  pkill -f 'demo_driver_services'"
echo "  pkill -f 'percipio_camera'"
echo "  pkill -f 'camera_control'"
echo "  pkill -f 'image_data_bridge'"
echo "  pkill -f 'hand_eye_calibration'"
echo "  pkill -f 'visual_pose_estimation_python'"
echo "  pkill -f 'graspnet_demo_points_with_tf.launch.py'"
echo "  pkill -f 'execute_grasp_pose_worker'"
echo "  pkill -f 'gripper_swap_worker_node'"
echo "  pkill -f 'publish_grasps_client_worker_node'"
echo "  pkill -f 'publish_grasps_worker_loop_control_client'"
echo "  pkill -f 'http_bridge_server'"

# 保持脚本运行
echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（不会停止已启动的节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
