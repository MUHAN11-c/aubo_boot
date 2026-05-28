#!/bin/bash
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; exit 1; fi

# ═══════════════════════════════════════════════════════════════
# IVG 完整启动 (新框架机械臂 + Vision/GraspNet/Web)
#
# 启动策略:
#   - launch 文件自动 TCP 探测机器人是否可达
#       - 可达   → 真实硬件模式
#       - 不可达 → 仿真模式 (ros2_control + mock_components)
#   - 主动轮询替代固定 sleep，根据实际就绪状态推进
#   - 独立步骤组内并行启动，组间串行等待
#
# 路径自定位 — 脚本放在工作空间根目录即可喵~
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

# 退出清理（Ctrl+C / kill 时终止所有已启动的 ROS 2 进程）
cleanup() {
    echo -e "\n${YELLOW}正在终止所有启动的进程...${NC}"
    pkill -f rosbridge_websocket 2>/dev/null || true
    pkill -f rosapi_node 2>/dev/null || true
    pkill -f aubo_driver_ros2 2>/dev/null || true
    pkill -f aubo_state_broadcaster 2>/dev/null || true
    pkill -f aubo_dashboard_node 2>/dev/null || true
    pkill -f joint_trajectory_controller 2>/dev/null || true
    pkill -f ros2_control_node 2>/dev/null || true
    pkill -f controller_manager 2>/dev/null || true
    pkill -f move_group 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true
    pkill -f web_video_server 2>/dev/null || true
    pkill -f 'uvicorn.*8090' 2>/dev/null || true
    pkill -f 'ros2 bag' 2>/dev/null || true
    pkill -f 'ros2 lifecycle' 2>/dev/null || true
    pkill -f 'latte_imitation' 2>/dev/null || true
    pkill -f 'latte_cartesian_planner' 2>/dev/null || true
    pkill -f 'latte_workflow_node' 2>/dev/null || true
    sleep 0.5
    echo -e "${GREEN}  ✓ 清理完成${NC}"
}
trap cleanup INT TERM

# ═══════════════════════════════════════════════════════════════
# 路径自定位
# ═══════════════════════════════════════════════════════════════
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${AUBO_ROS2_WS:-${SCRIPT_DIR}}"
ROS_DISTRO="${ROS_DISTRO_NAME:-humble}"
ROS2_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WAIT_SVC="${WS}/wait_for_service.sh"

# ═══════════════════════════════════════════════════════════════
# 用户可配环境变量
# ═══════════════════════════════════════════════════════════════
AUBO_IP="${AUBO_IP:-169.254.10.98}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8088}"
IVG_WEB_RELOAD="${IVG_WEB_RELOAD:-true}"
WEB_DASH_HOST="${WEB_DASH_HOST:-0.0.0.0}"
WEB_DASH_PORT="${WEB_DASH_PORT:-8090}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
IVG_ROSBAG_DIR="${IVG_ROSBAG_DIR:-rosbags/ivg_session}"
IVG_ROSBAG_TOPICS="${IVG_ROSBAG_TOPICS:-}"
HAND_EYE_PORT="${HAND_EYE_PORT:-8070}"
WEB_VIDEO_PORT="${WEB_VIDEO_PORT:-8089}"
SKIP_BUILD="${SKIP_BUILD:-0}"
SKIP_ROSBAG="${SKIP_ROSBAG:-0}"

# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

ivg_lan_ipv4_addrs() {
    if command -v ip >/dev/null 2>&1; then
        ip -o -4 addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1
    else hostname -I 2>/dev/null | tr ' ' '\n'; fi | grep -vE '^(127\.|169\.254\.)' | sort -u
}

ivg_rosbridge_query() {
    if [ "${ROSBRIDGE_PORT}" != "9090" ]; then printf '%s' "?rosbridge_port=${ROSBRIDGE_PORT}"
    else printf '%s' ""; fi
}

ivg_web_dash_url() {
    printf 'http://%s:%s/%s%s' "$1" "$WEB_DASH_PORT" "$2" "$(ivg_rosbridge_query)"
}

ivg_print_access_urls() {
    local lh="127.0.0.1"
    echo ""; echo -e "${BLUE}──────── 本机浏览器 ────────${NC}"
    # 代理警告 — WebSocket 连接走代理会导致 rosbridge 连接失败
    local proxy_warn=""
    [ -n "${http_proxy:-}${HTTP_PROXY:-}" ] && proxy_warn="1"
    if [ -n "$proxy_warn" ]; then
        echo -e "${YELLOW}⚠ 检测到系统代理 (http_proxy/HTTP_PROXY)，WebSocket 可能被拦截！${NC}"
        echo -e "${YELLOW}  请确保浏览器对 127.0.0.1 / localhost 绕过代理，否则页面无法连接 rosbridge。${NC}"
        echo -e "${YELLOW}  Chrome: 设置→系统→打开代理设置→绕过本地地址 或 启动加 --no-proxy-server${NC}"
        echo -e "${YELLOW}  当前代理: http_proxy=${http_proxy:-${HTTP_PROXY:-未设置}}${NC}"
        echo ""
    fi
    echo -e "${GREEN}手眼标定:     http://${lh}:${HAND_EYE_PORT}/${NC}"
    echo -e "${GREEN}VPE FastAPI:  http://${lh}:${WEB_PORT}/${NC}"
    echo -e "${GREEN}IVG 门户:     $(ivg_web_dash_url "$lh" "index.html")${NC}"
    echo -e "${GREEN}视觉抓取:     $(ivg_web_dash_url "$lh" "vision_grasp_panel.html")${NC}"
    echo -e "${GREEN}咖啡拉花:     $(ivg_web_dash_url "$lh" "coffee_latte_panel.html")${NC}"
    echo ""
    local -a lan_ips=()
    mapfile -t lan_ips < <(ivg_lan_ipv4_addrs)
    for ip in "${lan_ips[@]}"; do
        echo -e "${GREEN}  [${ip}] 门户: $(ivg_web_dash_url "$ip" "index.html")${NC}"
    done
}

# active_wait: 使用 wait_for_service.sh 进行主动轮询等待
# 如果 wait_for_service.sh 不可用，回退到固定 sleep
active_wait() {
    local type="$1" pattern="$2" timeout="${3:-30}" desc="${4:-$pattern}"
    echo -e "${BLUE}  → 等待 ${desc} (超时 ${timeout}s)...${NC}"
    if [ -x "$WAIT_SVC" ]; then
        if "$WAIT_SVC" "$type" "$pattern" "$timeout"; then
            echo -e "${GREEN}    ✓ ${desc} 就绪${NC}"
            return 0
        else
            echo -e "${YELLOW}    ⚠ ${desc} 超时，继续...${NC}"
            return 1
        fi
    else
        echo -e "${YELLOW}    wait_for_service.sh 不可用，sleep ${timeout}s${NC}"
        sleep "$timeout"
        return 0
    fi
}

# 在 terminator 新标签页中启动节点
# 每个标签页先 cd 到工作空间根目录，再 source ROS 2 + install/setup.bash
launch() {
    local title="$1" cmd="$2"
    local full
    full=""
    # 清除代理 — 所有 ROS 2 节点和 WebSocket 均为本地通信，走代理反而阻断连接
    full+="unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy FTP_PROXY ftp_proxy; "
    full+="export NO_PROXY=\"127.0.0.1,localhost,0.0.0.0,::1\${NO_PROXY:+, \${NO_PROXY}}\"; export no_proxy=\"\$NO_PROXY\"; "
    # PyTorch 绑定的 CUDA 库（需优先于系统 CUDA，避免版本冲突）
    local nvidia_libs
    nvidia_libs=$(echo "$HOME"/.local/lib/python3.10/site-packages/nvidia/*/lib | tr ' ' ':')
    full+="export LD_LIBRARY_PATH=\"\$HOME/.local/lib/python3.10/site-packages/torch/lib:${nvidia_libs}:${WS}/src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/aubocontroller:${WS}/src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/log4cplus:${WS}/src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/config:${WS}/src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/protobuf:\$LD_LIBRARY_PATH\" && cd \"${WS}\" && source \"${ROS2_SETUP}\" && source install/setup.bash && ${cmd}; exec bash"
    "$TERMINATOR" --new-tab --title="$title" \
        -e "bash -c '${full}'" &
}

# ═══════════════════════════════════════════════════════════════
# 预检
# ═══════════════════════════════════════════════════════════════

TERMINATOR=""
if command -v terminator &>/dev/null; then TERMINATOR="terminator"
elif [ -x /usr/bin/terminator ]; then TERMINATOR="/usr/bin/terminator"; fi
if [ -z "$TERMINATOR" ]; then
    echo -e "${RED}未找到 terminator, sudo apt install terminator${NC}"; exit 1
fi

if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${RED}未找到 ROS 2 安装: ${ROS2_SETUP}${NC}"
    echo -e "${RED}请先安装 ROS 2 ${ROS_DISTRO}: sudo apt install ros-${ROS_DISTRO}-desktop${NC}"
    exit 1
fi

if [ ! -f "${WS}/install/setup.bash" ]; then
    echo -e "${YELLOW}首次运行: install/setup.bash 不存在${NC}"
    FIRST_BUILD=1
else
    FIRST_BUILD=0
fi

# ═══════════════════════════════════════════════════════════════
# 清理旧进程（首次启动前执行，与 trap 共享同一清理逻辑）
# ═══════════════════════════════════════════════════════════════
echo -e "${YELLOW}清理旧进程...${NC}"
cleanup

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IVG 完整启动 (新框架机械臂)${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}工作空间: ${WS}${NC}"
echo -e "${BLUE}机器人 IP: ${AUBO_IP}${NC}"
echo -e "${BLUE}ROS 版本: ${ROS_DISTRO}${NC}"
echo ""

# ═══════════════════════════════════════════════════════════════
# [0] 构建
# ═══════════════════════════════════════════════════════════════

if [ "$SKIP_BUILD" = "1" ]; then
    echo -e "${YELLOW}[0] 跳过构建 (SKIP_BUILD=1)${NC}"
else
    echo -e "${GREEN}[0] 构建 (colcon)...${NC}"
    (
        cd "$WS"
        source "$ROS2_SETUP"
        if [ "$FIRST_BUILD" = "0" ]; then
            source install/setup.bash
        fi
        # Web Dashboard 使用纯 HTML/JS (web/public/)，无需 npm 构建喵~
        colcon build
    )
    echo -e "${GREEN}  ✓ 构建完成${NC}"
fi

# ═══════════════════════════════════════════════════════════════
# [1] 机械臂核心 (MoveIt2 + Dashboard + StateBroadcaster + JTC)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[1] 机械臂核心 (Controller + Dashboard + StateBroadcaster + MoveIt2)...${NC}"
launch "Aubo Driver" "ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=${AUBO_IP}"

# Dashboard 生命周期激活（后台轮询，30s 超时）
(
    cd "$WS"
    source "$ROS2_SETUP"
    source install/setup.bash
    echo "[dashboard] 等待 /aubo_dashboard 节点就绪 (超时 30s)..." >&2
    tick=0
    while ! ros2 node list 2>/dev/null | grep -q "/aubo_dashboard"; do
        sleep 0.5; tick=$((tick + 1))
        if [ "$tick" -ge 60 ]; then
            echo "[dashboard] ⚠ 超时 — /aubo_dashboard 未就绪，跳过生命周期激活" >&2
            exit 0
        fi
    done
    echo "[dashboard] ✓ 节点就绪，激活生命周期..." >&2
    ros2 lifecycle set /aubo_dashboard configure 2>/dev/null || true
    ros2 lifecycle set /aubo_dashboard activate 2>/dev/null || true
    echo "[dashboard] ✓ 生命周期激活完成" >&2
) &

# 等待 MoveIt2 就绪（move_group 是后续所有步骤的基础依赖）
active_wait node "/move_group" 30 "move_group" || true

# ═══════════════════════════════════════════════════════════════
# [2] Demo Driver 服务 (依赖 move_group)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[2] Demo Driver 服务...${NC}"
launch "Robot Driver" "ros2 launch aubo_moveit_config demo_driver_services.launch.py"
active_wait service "/execute_trajectory" 20 "execute_trajectory 服务" || true

# ═══════════════════════════════════════════════════════════════
# [15] IVG Web 网关 (提前启动 — 仅依赖 build 产物，与相机/视觉并行)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[15] IVG Web 网关...${NC}"
WEB_DASH_CMD="ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py web_host:=${WEB_DASH_HOST} web_port:=${WEB_DASH_PORT} rosbridge_port:=${ROSBRIDGE_PORT} web_video_port:=${WEB_VIDEO_PORT}"
launch "IVG Web Dashboard" "${WEB_DASH_CMD}"

# ═══════════════════════════════════════════════════════════════
# [16] rosbag 录制 (提前启动 — 仅需 ROS 2 运行，尽早开始录制)
# ═══════════════════════════════════════════════════════════════

if [ "$SKIP_ROSBAG" = "1" ]; then
    echo -e "${YELLOW}[16] 跳过 rosbag 录制 (SKIP_ROSBAG=1)${NC}"
else
    echo -e "${GREEN}[16] rosbag 录制...${NC}"
    mkdir -p "$(dirname "${WS}/${IVG_ROSBAG_DIR}")"
    rm -rf "${WS}/${IVG_ROSBAG_DIR}"
    if [ -n "$IVG_ROSBAG_TOPICS" ]; then
        launch "ROS2 Bag" "ros2 bag record -o \"${IVG_ROSBAG_DIR}\" ${IVG_ROSBAG_TOPICS}"
    else
        launch "ROS2 Bag" "ros2 bag record -o \"${IVG_ROSBAG_DIR}\" -a -x '/state$'"
    fi
fi

# ═══════════════════════════════════════════════════════════════
# [3-5] 相机栈 (并行启动，减少串行等待)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[3] 相机节点...${NC}"
launch "Percipio Camera" "ros2 launch percipio_camera percipio_camera.launch.py"

echo -e "${GREEN}[4] 相机控制...${NC}"
launch "Camera Control" "ros2 launch percipio_camera camera_control.launch.py"

active_wait topic "/camera/color/image_raw" 15 "相机图像话题" || true
active_wait topic "/camera_status" 10 "相机状态话题" || true

# ═══════════════════════════════════════════════════════════════
# [6-7] 视觉栈 (并行启动)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[6] 手眼标定...${NC}"
launch "Hand Eye" "ros2 launch hand_eye_calibration hand_eye_calibration_launch.py enable_image_data_converter:=true web_port:=${HAND_EYE_PORT}"

echo -e "${GREEN}[7] 视觉姿态估计...${NC}"
launch "VPE" "export PATH=\"/usr/bin:\$PATH\" && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"

active_wait service "/estimate_pose" 15 "estimate_pose 服务" || true

# ═══════════════════════════════════════════════════════════════
# [14] FastAPI Web (提前启动 — 依赖 estimate_pose，与 GraspNet/Workers 并行)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[14] FastAPI Web...${NC}"
launch "FastAPI Web" "ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py host:=${WEB_HOST} port:=${WEB_PORT} reload:=${IVG_WEB_RELOAD}"

# ═══════════════════════════════════════════════════════════════
# [8] GraspNet (依赖相机点云 + 手眼标定 TF)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[8] GraspNet 点云...${NC}"
launch "GraspNet" "ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py launch_camera:=false launch_hand_eye_tf:=true"
active_wait topic "/grasp_poses_base" 20 "grasp_poses_base 话题" || true

# ═══════════════════════════════════════════════════════════════
# [9-12] 工位与执行 (并行启动)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[9] 抓取 Worker...${NC}"
launch "Grasp Worker" "ros2 launch demo_driver execute_grasp_pose_worker.launch.py"

echo -e "${GREEN}[10] 夹爪快换...${NC}"
launch "Tool Changer" "ros2 launch tool_changer gripper_swap_worker.launch.py"

echo -e "${GREEN}[11] 咖啡拉花 (DO 开关)...${NC}"
launch "Coffee Latte" "ros2 launch latte_imitation latte_io.launch.py"

echo -e "${GREEN}[11b] 拉花轨迹回放 (MoveIt2 标准管线)...${NC}"
launch "Latte Imitation" "ros2 launch latte_imitation start_latte_pour.launch.py"

echo -e "${GREEN}[11c] 拉花工作流编排...${NC}"
launch "Latte Workflow" "ros2 launch aubo_moveit_config latte_workflow.launch.py"

echo -e "${GREEN}[12] GraspNet 循环抓取...${NC}"
launch "Publish Grasps" "ros2 run demo_driver publish_grasps_client_worker_node"

# 等待关键服务就绪
active_wait service "/run_gripper_swap" 10 "run_gripper_swap 服务" || true
active_wait service "/change_tool" 15  "change_tool 服务" || true
active_wait service "/set_latte_do2" 5 "set_latte_do2 服务" || true
active_wait service "/latte/run_workflow" 30 "latte/run_workflow 服务" || true

# ═══════════════════════════════════════════════════════════════
# [13] 综合校验 (ROS 服务 + Web 健康检查)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[13] 综合校验...${NC}"
(
    cd "$WS"
    source "$ROS2_SETUP"
    source install/setup.bash
    echo -e "${BLUE}──────── 已注册服务 ────────${NC}"
    ros2 service list | grep -E '/estimate_pose|/list_templates|/graspnet_capture_control|/publish_grasps_worker_loop_control|/loop_grasp_control|/run_gripper_swap|/set_latte_do2|/set_latte_do4|/change_tool|/get_current_tool|/latte/run_workflow' || true
    echo -e "${GREEN}  ✓ 服务校验完成${NC}"
)

# 等待提前启动的 Web 服务就绪
active_wait http "http://${WEB_HOST}:${WEB_PORT}/health" 30 "FastAPI /health" || true
active_wait http "http://127.0.0.1:${WEB_DASH_PORT}/health" 20 "Web Dashboard /health" || true

# ═══════════════════════════════════════════════════════════════
# 完成
# ═══════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  IVG 启动完成！${NC}"
echo -e "${GREEN}========================================${NC}"
ivg_print_access_urls
echo ""
echo -e "${YELLOW}提示: 在 terminator 各标签页中查看组件日志喵~${NC}"
