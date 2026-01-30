#!/bin/bash

###############################################################################
# Visual Pose Estimation Python - Web UI 启动脚本
# 用于启动HTTP桥接服务器和Web UI界面
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WEB_UI_DIR="${SCRIPT_DIR}"
SCRIPTS_DIR="${WEB_UI_DIR}/scripts"

# 启动函数：在 terminator 标签页中运行；若无 terminator 则后台运行并记录日志
launch_in_terminator() {
    local title=$1
    local cmd=$2

    # 转义单引号，避免 bash -c 解析错误
    local escaped_cmd
    escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")

    if command -v terminator >/dev/null 2>&1; then
        terminator --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
        sleep 0.5
    else
        echo -e "${YELLOW}未找到 terminator，${title} 将在后台运行，日志保存在 .${title// /_}.log${NC}"
        bash -c "$cmd" > "${WEB_UI_DIR}/.${title// /_}.log" 2>&1 &
    fi
}

# 端口配置
HTTP_PORT=8088  # Web UI服务器端口（与http_bridge_server.py保持一致）

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Visual Pose Estimation Python - Web UI${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告: ROS2环境未设置${NC}"
    echo -e "${YELLOW}正在尝试 source ROS2 环境...${NC}"
    
    # 尝试常见的ROS2安装路径
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ ROS2 Humble 环境已加载${NC}"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        echo -e "${GREEN}✓ ROS2 Foxy 环境已加载${NC}"
    else
        echo -e "${RED}✗ 未找到ROS2环境，请手动source${NC}"
        exit 1
    fi
fi

# 检查工作空间
# 从当前目录向上查找包含 install/setup.bash 的工作空间
CURRENT_DIR="${SCRIPT_DIR}"
WORKSPACE_DIR=""

# 最多向上查找10层
for i in {1..10}; do
    CURRENT_DIR="$(dirname "${CURRENT_DIR}")"
    if [ -f "${CURRENT_DIR}/install/setup.bash" ]; then
        WORKSPACE_DIR="${CURRENT_DIR}"
        break
    fi
done

if [ -n "${WORKSPACE_DIR}" ] && [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    echo -e "${BLUE}加载工作空间: ${WORKSPACE_DIR}${NC}"
    source "${WORKSPACE_DIR}/install/setup.bash"
    echo -e "${GREEN}✓ 工作空间环境已加载${NC}"
else
    echo -e "${YELLOW}警告: 未找到工作空间 install/setup.bash${NC}"
    echo -e "${YELLOW}请确保已编译工作空间: colcon build${NC}"
    echo -e "${YELLOW}或手动 source install/setup.bash${NC}"
    echo ""
    echo -e "${RED}✗ 无法继续，缺少必需的ROS2包${NC}"
    exit 1
fi

# 记录工作空间和 ROS2 setup 路径，供后续节点启动使用
AUBO_ROS2_WS="${WORKSPACE_DIR}"
ROS2_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ ! -f "${ROS2_SETUP}" ]; then
    ROS2_SETUP="/opt/ros/foxy/setup.bash"
fi

echo ""

# 检查Python依赖
echo -e "${BLUE}检查Python依赖...${NC}"
python3 -c "import rclpy" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}✗ rclpy未安装${NC}"
    exit 1
fi

python3 -c "import cv_bridge" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}✗ cv_bridge未安装${NC}"
    exit 1
fi

python3 -c "import cv2" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${RED}✗ OpenCV (cv2)未安装${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Python依赖检查通过${NC}"
echo ""

# 启动相机节点（复用手眼标定脚本步骤）
echo -e "${GREEN}[1/3] 启动相机节点...${NC}"
CAMERA_CMD="cd ${AUBO_ROS2_WS} && source ${ROS2_SETUP} && colcon build --packages-select percipio_camera && source install/setup.bash && ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5  # 相机需要时间初始化

echo -e "${GREEN}[2/3] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="cd ${AUBO_ROS2_WS} && source ${ROS2_SETUP} && colcon build --packages-select percipio_camera_interface && source install/setup.bash && ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 启动视觉位姿估计节点（提供 /estimate_pose 等服务）
echo -e "${GREEN}[3/3] 启动 visual_pose_estimation_python 节点...${NC}"
VPE_CMD="cd ${AUBO_ROS2_WS} && source ${ROS2_SETUP} && colcon build --packages-select visual_pose_estimation_python && source install/setup.bash && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ visual_pose_estimation_python 已启动${NC}"
sleep 2

# 检查端口是否被占用
echo -e "${BLUE}检查端口 ${HTTP_PORT}...${NC}"
if lsof -Pi :${HTTP_PORT} -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    echo -e "${YELLOW}警告: 端口 ${HTTP_PORT} 已被占用${NC}"
    echo -e "${YELLOW}正在尝试终止占用进程...${NC}"
    
    PID=$(lsof -ti:${HTTP_PORT})
    if [ ! -z "$PID" ]; then
        kill -9 $PID 2>/dev/null
        sleep 2
        echo -e "${GREEN}✓ 端口已清理${NC}"
    fi
else
    echo -e "${GREEN}✓ 端口 ${HTTP_PORT} 可用${NC}"
fi

echo ""

# 检查ROS2服务是否运行
echo -e "${BLUE}检查ROS2服务...${NC}"
ros2 service list | grep -q "/estimate_pose"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ /estimate_pose 服务已运行${NC}"
else
    echo -e "${YELLOW}警告: /estimate_pose 服务未运行${NC}"
    echo -e "${YELLOW}请先启动 visual_pose_estimation_python 节点${NC}"
    echo -e "${YELLOW}提示: ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py${NC}"
fi

ros2 service list | grep -q "/list_templates"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ /list_templates 服务已运行${NC}"
else
    echo -e "${YELLOW}警告: /list_templates 服务未运行${NC}"
fi

echo ""

# 启动HTTP桥接服务器
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}启动HTTP桥接服务器...${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

cd "${WEB_UI_DIR}"

python3 "${SCRIPTS_DIR}/http_bridge_server.py" &
HTTP_PID=$!

# 等待服务器启动
sleep 3

# 检查服务器是否成功启动
if ps -p $HTTP_PID > /dev/null; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}✓ Web UI 启动成功！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${GREEN}访问地址:${NC}"
    echo -e "  ${BLUE}http://localhost:${HTTP_PORT}${NC}"
    echo -e "  ${BLUE}http://localhost:${HTTP_PORT}/index.html${NC}"
    echo ""
    echo -e "${YELLOW}提示:${NC}"
    echo -e "  - 使用浏览器访问上述地址"
    echo -e "  - 确保 visual_pose_estimation_python 节点正在运行"
    echo -e "  - 按 Ctrl+C 停止服务"
    echo ""
    echo -e "${GREEN}========================================${NC}"
    
    # 保存PID到文件
    echo $HTTP_PID > "${WEB_UI_DIR}/.web_ui.pid"
    
    # 等待用户中断
    trap "echo ''; echo -e '${YELLOW}正在停止服务...${NC}'; kill $HTTP_PID 2>/dev/null; rm -f '${WEB_UI_DIR}/.web_ui.pid'; echo -e '${GREEN}服务已停止${NC}'; exit 0" INT TERM
    
    wait $HTTP_PID
else
    echo -e "${RED}✗ HTTP桥接服务器启动失败${NC}"
    exit 1
fi
