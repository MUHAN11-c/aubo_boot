#!/bin/bash
# 请使用 bash 运行（./start_IVG_graspnet_points_fastapi_web_dashboard.sh 或 bash …），勿用 sh
# 若已用 sh 启动，则自动用 bash 重新执行（避免 echo -e 等不生效）
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

# IVG 完整启动脚本（GraspNet Points + FastAPI Web + aubo_ros2_web_dashboard）
# 在 start_IVG_graspnet_points_fastapi.sh 基础上增加：rosbridge + tf2_web_republisher + 静态 ROS Web 控制台（默认 8090/9090）
# 使用 terminator 创建分屏终端

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 环境路径（默认使用脚本所在工作空间，可通过环境变量覆盖）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUBO_ROS2_WS="${AUBO_ROS2_WS:-${SCRIPT_DIR}}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
ROS2_BASE_ENV="source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && if [ -f ~/ws_moveit/install/setup.bash ]; then source ~/ws_moveit/install/setup.bash; fi"
WS_ENV="cd $AUBO_ROS2_WS && $ROS2_BASE_ENV && if [ -f install/setup.bash ]; then source install/setup.bash; fi"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8088}"
# FastAPI/uvicorn 开发热重载：默认开启；设为 false 可关闭（例如生产或稳定对比）
IVG_WEB_RELOAD="${IVG_WEB_RELOAD:-true}"
WEB_URL="http://${WEB_HOST}:${WEB_PORT}"
# aubo_ros2_web_dashboard（与 README 中 WEB_DASH_* / ROSBRIDGE_PORT 约定一致）
WEB_DASH_HOST="${WEB_DASH_HOST:-0.0.0.0}"
WEB_DASH_PORT="${WEB_DASH_PORT:-8090}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
IVG_STRIP_PROXY_FOR_DASH_LAUNCH="${IVG_STRIP_PROXY_FOR_DASH_LAUNCH:-true}"
# 步骤15 rosbag：输出目录；启动前会 rm -rf 实现覆盖。话题默认为 -a 全部；可设 IVG_ROSBAG_TOPICS="/t1 /t2"
IVG_ROSBAG_DIR="${IVG_ROSBAG_DIR:-${AUBO_ROS2_WS}/rosbags/ivg_session}"
IVG_ROSBAG_TOPICS="${IVG_ROSBAG_TOPICS:-}"
# 手眼 Web 端口（仅用于结束时的链接提示；实际以 hand_eye launch 为准）
HAND_EYE_PORT="${HAND_EYE_PORT:-8080}"

if [ "${IVG_STRIP_PROXY_FOR_DASH_LAUNCH}" = "true" ]; then
    WEB_DASH_UNSET_PROXY='unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY no_proxy; '
else
    WEB_DASH_UNSET_PROXY=''
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IVG 系统启动脚本（GraspNet + FastAPI + Web 控制台）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检测 terminator（兼容 PATH 未含 /usr/bin）
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

# 预先构建工作空间一次，后续各步骤仅 source
echo -e "${GREEN}[0/15] 构建工作空间...${NC}"
(
    cd "$AUBO_ROS2_WS" && \
    $ROS2_BASE_ENV && \
    if [ -f install/setup.bash ]; then source install/setup.bash; fi && \
    colcon build
)
echo -e "${GREEN}  ✓ 构建完成${NC}"
echo ""

launch_in_terminator() {
    local title=$1
    local cmd=$2
    local escaped_cmd
    escaped_cmd=$(echo "$cmd" | sed "s/'/'\"'\"'/g")
    "$TERMINATOR" --new-tab --title="$title" -e "bash -c 'eval \"$escaped_cmd\"; exec bash'" &
    sleep 0.5
}

# 本机 / 平板访问链接：收集非回环全局 IPv4（常见为有线或 Wi‑Fi）
ivg_lan_ipv4_addrs() {
    if command -v ip >/dev/null 2>&1; then
        ip -o -4 addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1
    else
        hostname -I 2>/dev/null | tr ' ' '\n'
    fi | grep -vE '^(127\.|169\.254\.)' | grep -v '^$' | sort -u
}

ivg_topics_lab_url() {
    local host="$1"
    local u="http://${host}:${WEB_DASH_PORT}/topics_lab.html"
    if [ "${ROSBRIDGE_PORT}" != "9090" ]; then
        u="${u}?rosbridge_port=${ROSBRIDGE_PORT}"
    fi
    printf '%s' "$u"
}

ivg_print_access_urls() {
    local lh="127.0.0.1"
    echo ""
    echo -e "${BLUE}──────── 本机浏览器 ────────${NC}"
    echo -e "${GREEN}手眼标定 Web:       http://${lh}:${HAND_EYE_PORT}/${NC}"
    echo -e "${GREEN}VPE FastAPI:        http://${lh}:${WEB_PORT}/${NC}"
    echo -e "${GREEN}VPE 旧版 UI:        http://${lh}:${WEB_PORT}/legacy-ui/index.html${NC}"
    echo -e "${GREEN}ROS Web（RWT）:     http://${lh}:${WEB_DASH_PORT}/${NC}"
    echo -e "${GREEN}ROS 控制台 topics_lab: $(ivg_topics_lab_url "$lh")${NC}"

    local -a lan_ips=()
    mapfile -t lan_ips < <(ivg_lan_ipv4_addrs)
    if [ "${#lan_ips[@]}" -eq 0 ]; then
        echo ""
        echo -e "${YELLOW}未检测到局域网 IPv4（有线/Wi‑Fi）；若已联网，可稍后手动查 ip addr 后替换下方地址。${NC}"
        return 0
    fi

    echo ""
    echo -e "${BLUE}──────── 局域网（手机/平板等与 PC 同 Wi‑Fi）────────${NC}"
    echo -e "${GREEN}ROS Web / topics_lab 默认监听 0.0.0.0，下列链接一般可直接用；${NC}"
    echo -e "${GREEN}页面会通过当前主机名连接 rosbridge（ws://同一 IP:${ROSBRIDGE_PORT}）。${NC}"
    local ip
    for ip in "${lan_ips[@]}"; do
        echo -e "${GREEN}  [${ip}] ROS Web:     http://${ip}:${WEB_DASH_PORT}/${NC}"
        echo -e "${GREEN}  [${ip}] topics_lab:  $(ivg_topics_lab_url "$ip")${NC}"
    done

    if [ "${WEB_HOST}" = "127.0.0.1" ] || [ "${WEB_HOST}" = "localhost" ]; then
        echo ""
        echo -e "${YELLOW}VPE FastAPI 当前 WEB_HOST=${WEB_HOST}，仅本机可访；平板要打开 FastAPI 请:${NC}"
        echo -e "${YELLOW}  export WEB_HOST=0.0.0.0 后重新运行本脚本（或自行改 launch）。${NC}"
    else
        for ip in "${lan_ips[@]}"; do
            echo -e "${GREEN}  [${ip}] VPE FastAPI: http://${ip}:${WEB_PORT}/${NC}"
            echo -e "${GREEN}  [${ip}] VPE 旧版 UI: http://${ip}:${WEB_PORT}/legacy-ui/index.html${NC}"
        done
    fi

    echo ""
    echo -e "${GREEN}手眼（默认端口 ${HAND_EYE_PORT}）若 launch 只绑 127.0.0.1，平板无法访问；需绑 0.0.0.0 时可用:${NC}"
    for ip in "${lan_ips[@]}"; do
        echo -e "${GREEN}  [${ip}] 手眼 Web:    http://${ip}:${HAND_EYE_PORT}/${NC}"
    done
}

# 步骤1: 启动真实机械臂驱动
echo -e "${GREEN}[1/15] 启动真实机械臂驱动（aubo_moveit_pure_ros2）...${NC}"
AUBO_PURE_ROS2_CMD="$WS_ENV && ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py"
launch_in_terminator "Aubo MoveIt Pure ROS2" "$AUBO_PURE_ROS2_CMD"
echo -e "${GREEN}  ✓ 真实机械臂驱动已启动${NC}"
sleep 3

# 步骤2: 启动机器人驱动服务
echo -e "${GREEN}[2/15] 启动机器人驱动服务...${NC}"
DRIVER_CMD="$WS_ENV && ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤3: 启动相机节点
echo -e "${GREEN}[3/15] 启动相机节点...${NC}"
CAMERA_CMD="$WS_ENV && ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5

# 步骤4: 启动相机控制节点
echo -e "${GREEN}[4/15] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="$WS_ENV && ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤5: 启动图像数据桥接节点
echo -e "${GREEN}[5/15] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="$WS_ENV && ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤6: 启动手眼标定节点
echo -e "${GREEN}[6/15] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="$WS_ENV && ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"
sleep 2

# 步骤7: 启动视觉姿态估计算法节点
echo -e "${GREEN}[7/15] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="$WS_ENV && export PATH=\"/usr/bin:\$PATH\" && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计算法节点已启动${NC}"
sleep 2

# 步骤8: 启动 GraspNet 点云节点
echo -e "${GREEN}[8/15] 启动 GraspNet 点云节点（with_tf）...${NC}"
GRASPNET_WITH_TF_CMD="$WS_ENV && ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py launch_camera:=false launch_hand_eye_tf:=true"
launch_in_terminator "GraspNet Points With TF" "$GRASPNET_WITH_TF_CMD"
echo -e "${GREEN}  ✓ GraspNet 点云节点已启动${NC}"
sleep 2

# 步骤9: 启动执行抓取位姿服务节点
echo -e "${GREEN}[9/15] 启动执行抓取位姿服务节点...${NC}"
GRASP_WORKER_CMD="$WS_ENV && ros2 launch demo_driver execute_grasp_pose_worker.launch.py"
launch_in_terminator "Execute Grasp Worker" "$GRASP_WORKER_CMD"
echo -e "${GREEN}  ✓ 执行抓取位姿服务节点已启动${NC}"
sleep 2

# 步骤10: 启动夹爪切换服务节点
echo -e "${GREEN}[10/15] 启动夹爪切换服务节点...${NC}"
GRIPPER_SWAP_CMD="$WS_ENV && ros2 run demo_driver gripper_swap_worker_node"
launch_in_terminator "Gripper Swap Worker" "$GRIPPER_SWAP_CMD"
echo -e "${GREEN}  ✓ 夹爪切换服务节点已启动${NC}"
sleep 2

# 步骤11: 启动 GraspNet 循环抓取 Worker
echo -e "${GREEN}[11/15] 启动 GraspNet 循环抓取 Worker...${NC}"
PUBLISH_GRASPS_WORKER_CMD="$WS_ENV && ros2 run demo_driver publish_grasps_client_worker_node"
launch_in_terminator "Publish Grasps Worker" "$PUBLISH_GRASPS_WORKER_CMD"
echo -e "${GREEN}  ✓ GraspNet 循环抓取 Worker 已启动${NC}"
sleep 2

# 步骤12: 校验关键服务是否已就绪
echo -e "${GREEN}[12/15] 校验关键服务...${NC}"
SERVICE_CHECK_CMD="$WS_ENV && ros2 service list | rg '/estimate_pose|/list_templates|/graspnet_capture_control|/publish_grasps_worker_loop_control|/loop_grasp_control|/run_gripper_swap' || true"
launch_in_terminator "Service Check" "$SERVICE_CHECK_CMD"
echo -e "${GREEN}  ✓ 关键服务校验标签页已启动${NC}"
sleep 1

# 步骤13: 启动 FastAPI Web 服务
echo -e "${GREEN}[13/15] 启动 FastAPI Web 服务...${NC}"
FASTAPI_WEB_CMD="$WS_ENV && ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py host:=${WEB_HOST} port:=${WEB_PORT} reload:=${IVG_WEB_RELOAD}"
launch_in_terminator "Visual Pose Web FastAPI" "$FASTAPI_WEB_CMD"
echo -e "${GREEN}  ✓ FastAPI Web 服务已启动${NC}"
sleep 3

# 步骤14: rosbridge + tf2_web_republisher + 静态页（aubo_ros2_web_dashboard）
echo -e "${GREEN}[14/15] 启动 ROS Web 控制台（rosbridge + tf2_web + 静态站 ${WEB_DASH_HOST}:${WEB_DASH_PORT}，WS ${ROSBRIDGE_PORT}）...${NC}"
WEB_DASH_CMD="$WS_ENV && ${WEB_DASH_UNSET_PROXY}ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py web_host:=${WEB_DASH_HOST} web_port:=${WEB_DASH_PORT} rosbridge_port:=${ROSBRIDGE_PORT}"
launch_in_terminator "IVG Web Dashboard (rosbridge)" "$WEB_DASH_CMD"
echo -e "${GREEN}  ✓ ROS Web 控制台已启动${NC}"
sleep 2

# 步骤15: rosbag 录制（删除已有同名目录后重新录，实现覆盖）
echo -e "${GREEN}[15/15] rosbag 录制数据（覆盖: ${IVG_ROSBAG_DIR}）...${NC}"
mkdir -p "$(dirname "$IVG_ROSBAG_DIR")"
rm -rf "$IVG_ROSBAG_DIR"
if [ -n "$IVG_ROSBAG_TOPICS" ]; then
    # shellcheck disable=SC2086
    ROSBAG_CMD="$WS_ENV && ros2 bag record -o \"$IVG_ROSBAG_DIR\" $IVG_ROSBAG_TOPICS"
else
    ROSBAG_CMD="$WS_ENV && ros2 bag record -o \"$IVG_ROSBAG_DIR\" -a"
fi
launch_in_terminator "ROS2 Bag Record IVG" "$ROSBAG_CMD"
echo -e "${GREEN}  ✓ rosbag 已在独立标签页启动（Ctrl+C 停止录制）${NC}"
sleep 1

FASTAPI_HEALTH_URL="${WEB_URL}/health"
if command -v curl >/dev/null 2>&1; then
    if curl --fail --silent --max-time 5 "$FASTAPI_HEALTH_URL" >/dev/null; then
        echo -e "${GREEN}  ✓ FastAPI 健康检查通过: ${FASTAPI_HEALTH_URL}${NC}"
    else
        echo -e "${YELLOW}  ! FastAPI 健康检查暂未通过，请查看 'Visual Pose Web FastAPI' 标签页日志${NC}"
    fi
else
    echo -e "${YELLOW}  ! 未找到 curl，跳过 FastAPI 健康检查${NC}"
fi

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

echo -e "${GREEN}启动完成！IVG 系统已就绪。${NC}"
ivg_print_access_urls
echo ""
echo -e "${YELLOW}提示: 关闭 terminator 标签页或按 Ctrl+C 停止对应节点${NC}"
echo -e "${YELLOW}或使用以下命令停止所有节点:${NC}"
echo "  pkill -f 'aubo_moveit_pure_ros2.launch.py'"
echo "  pkill -f 'demo_driver_services.launch.py'"
echo "  pkill -f 'percipio_camera.launch.py'"
echo "  pkill -f 'camera_control.launch.py'"
echo "  pkill -f 'image_data_bridge.launch.py'"
echo "  pkill -f 'hand_eye_calibration_launch.py'"
echo "  pkill -f 'visual_pose_estimation_python.launch.py'"
echo "  pkill -f 'graspnet_demo_points_with_tf.launch.py'"
echo "  pkill -f 'execute_grasp_pose_worker.launch.py'"
echo "  pkill -f 'gripper_swap_worker_node'"
echo "  pkill -f 'publish_grasps_client_worker_node'"
echo "  pkill -f 'visual_pose_estimation_web.launch.py'"
echo "  pkill -f 'visual_pose_estimation_web'"
echo "  pkill -f 'web_dashboard.launch.py'"
echo "  pkill -f 'ros2 bag record'"

echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（不会停止已启动的节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
