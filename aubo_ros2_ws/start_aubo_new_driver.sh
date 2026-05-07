#!/bin/bash
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; exit 1; fi

# ═══════════════════════════════════════════════════════════════
# IVG 完整启动 (新框架机械臂 + Vision/GraspNet/Web)
#
# 基于 start_IVG_graspnet_points_fastapi_web_legacy.sh
# 机械臂部分替换为新框架: Controller + Dashboard + StateBroadcaster
# 其余步骤(相机/GraspNet/Web/抓取)保持不变
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${AUBO_ROS2_WS:-${SCRIPT_DIR}}"
ROS_DISTRO="${ROS_DISTRO_NAME:-humble}"
AUBO_IP="${AUBO_IP:-169.254.10.98}"

# Web / rosbag 等配置 (沿用 legacy 脚本)
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8088}"
IVG_WEB_RELOAD="${IVG_WEB_RELOAD:-true}"
WEB_DASH_HOST="${WEB_DASH_HOST:-0.0.0.0}"
WEB_DASH_PORT="${WEB_DASH_PORT:-8090}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
IVG_STRIP_PROXY_FOR_DASH_LAUNCH="${IVG_STRIP_PROXY_FOR_DASH_LAUNCH:-true}"
IVG_ROSBAG_DIR="${IVG_ROSBAG_DIR:-${WS}/rosbags/ivg_session}"
IVG_ROSBAG_TOPICS="${IVG_ROSBAG_TOPICS:-}"
HAND_EYE_PORT="${HAND_EYE_PORT:-8080}"
WEB_VIDEO_PORT="${WEB_VIDEO_PORT:-8089}"
IVG_INCLUDE_POINTCLOUD_WEB_BRIDGE="${IVG_INCLUDE_POINTCLOUD_WEB_BRIDGE:-true}"
IVG_POINTCLOUD_WEB_MAX_POINTS="${IVG_POINTCLOUD_WEB_MAX_POINTS:-15000}"
SKIP_RVIZ="${SKIP_RVIZ:-0}"
ROBOTWEBTOOLS_ROOT="${ROBOTWEBTOOLS_ROOT:-${WS}/src/robotwebtools}"
ROBOTWEBTOOLS_BUILD_SCRIPT="${ROBOTWEBTOOLS_ROOT}/build_robotwebtools.sh"
ROBOTWEBTOOLS_ASSETS_DIR="${ROBOTWEBTOOLS_ASSETS_DIR:-${ROBOTWEBTOOLS_ROOT}/runtime_js_assets}"

source_env="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS}/install/setup.bash"

WEB_DASH_NO_PROXY_EXPORT='export NO_PROXY="127.0.0.1,localhost,0.0.0.0,::1${NO_PROXY:+,${NO_PROXY}}"; export no_proxy="$NO_PROXY"; '
if [ "${IVG_STRIP_PROXY_FOR_DASH_LAUNCH}" = "true" ]; then
    WEB_DASH_UNSET_PROXY="unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY all_proxy FTP_PROXY ftp_proxy; ${WEB_DASH_NO_PROXY_EXPORT}"
else
    WEB_DASH_UNSET_PROXY="${WEB_DASH_NO_PROXY_EXPORT}"
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IVG 完整启动 (新框架机械臂)${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}机器人 IP: ${AUBO_IP}${NC}"
echo ""

TERMINATOR=""
if command -v terminator &>/dev/null; then TERMINATOR="terminator"
elif [ -x /usr/bin/terminator ]; then TERMINATOR="/usr/bin/terminator"; fi
if [ -z "$TERMINATOR" ]; then
    echo -e "${RED}未找到 terminator, sudo apt install terminator${NC}"; exit 1
fi

launch() {
    local title="$1" cmd="$2"
    local full="cd ${WS} && ${source_env} && ${cmd}"
    "$TERMINATOR" --new-tab --title="$title" \
        -e "bash -c '${full}; exec bash'" &
    sleep 0.5
}

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

# ═══════════════════════════════════════════════════════════════
# 构建
# ═══════════════════════════════════════════════════════════════
echo -e "${GREEN}[0/16] 构建...${NC}"
( cd "$WS" && eval "$source_env" && colcon build )
echo -e "${GREEN}  ✓ 构建完成${NC}"
if [ -x "$ROBOTWEBTOOLS_BUILD_SCRIPT" ]; then
    echo -e "${GREEN}  → 导出 RobotWebTools 产物${NC}"
    ( cd "$ROBOTWEBTOOLS_ROOT" && bash "$ROBOTWEBTOOLS_BUILD_SCRIPT" )
fi

# ═══════════════════════════════════════════════════════════════
# 新框架机械臂 (替代旧步骤 1-5)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[1/16] 新框架机械臂 (Controller + Dashboard + StateBroadcaster + MoveIt2)...${NC}"
launch "Aubo New Driver" "ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=${AUBO_IP}"
sleep 5
# 激活 Dashboard 生命周期
( cd "$WS" && eval "$source_env" && sleep 3 && \
  ros2 lifecycle set /aubo_dashboard configure && \
  ros2 lifecycle set /aubo_dashboard activate )
echo -e "${GREEN}  ✓ 新框架机械臂已启动${NC}"
sleep 3

# ═══════════════════════════════════════════════════════════════
# 以下步骤 5-16 与 legacy 脚本保持一致
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[2/16] Demo Driver 服务...${NC}"
launch "Robot Driver" "ros2 launch aubo_moveit_config demo_driver_services.launch.py"
sleep 3

echo -e "${GREEN}[3/16] 相机节点...${NC}"
launch "Percipio Camera" "ros2 launch percipio_camera percipio_camera.launch.py"
sleep 5

echo -e "${GREEN}[4/16] 相机控制...${NC}"
launch "Camera Control" "ros2 launch percipio_camera_interface camera_control.launch.py"
sleep 2

echo -e "${GREEN}[5/16] 图像桥接...${NC}"
launch "Image Bridge" "ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
sleep 2

echo -e "${GREEN}[6/16] 手眼标定...${NC}"
launch "Hand Eye" "ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
sleep 2

echo -e "${GREEN}[7/16] 视觉姿态估计...${NC}"
launch "VPE" "export PATH=\"/usr/bin:\$PATH\" && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
sleep 2

echo -e "${GREEN}[8/16] GraspNet 点云...${NC}"
launch "GraspNet" "ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py launch_camera:=false launch_hand_eye_tf:=true"
sleep 2

echo -e "${GREEN}[9/16] 抓取 Worker...${NC}"
launch "Grasp Worker" "ros2 launch demo_driver execute_grasp_pose_worker.launch.py"
sleep 2

echo -e "${GREEN}[10/16] 夹爪快换...${NC}"
launch "Tool Changer" "ros2 launch tool_changer gripper_swap_worker.launch.py"
sleep 2

echo -e "${GREEN}[11/16] 咖啡拉花...${NC}"
launch "Coffee Latte" "ros2 launch coffee_latte_demo coffee_latte_demo.launch.py"
sleep 2

echo -e "${GREEN}[12/16] FastAPI Web + IVG 网关...${NC}"
launch "FastAPI Web" "ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py host:=${WEB_HOST} port:=${WEB_PORT} reload:=${IVG_WEB_RELOAD}"
sleep 3

WEB_DASH_PC_WEB_ARGS=""
if [ "${IVG_INCLUDE_POINTCLOUD_WEB_BRIDGE}" = "true" ]; then
    WEB_DASH_PC_WEB_ARGS=" include_pointcloud_web_bridge:=true pointcloud_web_max_points:=${IVG_POINTCLOUD_WEB_MAX_POINTS}"
fi
WEB_DASH_CMD="${WEB_DASH_UNSET_PROXY}ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py web_host:=${WEB_DASH_HOST} web_port:=${WEB_DASH_PORT} rosbridge_port:=${ROSBRIDGE_PORT} web_video_port:=${WEB_VIDEO_PORT} robotwebtools_assets_dir:=${ROBOTWEBTOOLS_ASSETS_DIR}${WEB_DASH_PC_WEB_ARGS}"
launch "IVG Web Dashboard" "${WEB_DASH_CMD}"
sleep 2

echo -e "${GREEN}[13/16] rosbag 录制...${NC}"
mkdir -p "$(dirname "$IVG_ROSBAG_DIR")"
rm -rf "$IVG_ROSBAG_DIR"
if [ -n "$IVG_ROSBAG_TOPICS" ]; then
    launch "ROS2 Bag" "ros2 bag record -o \"$IVG_ROSBAG_DIR\" $IVG_ROSBAG_TOPICS"
else
    launch "ROS2 Bag" "ros2 bag record -o \"$IVG_ROSBAG_DIR\" -a"
fi
sleep 1

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}启动完成 (新框架机械臂 + IVG 全家桶)${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}机械臂数据流:${NC}"
echo -e "  MoveIt2 → Action → JointTrajectoryController → HardwareInterface → 机器人"
echo -e "  joint_states ← StateBroadcaster (RoadPoint + JointStatus 回调)"
echo -e "  SDK 全功能 ← Dashboard (20 ROS2 服务)"
echo ""

ivg_print_access_urls

echo ""
echo -e "${YELLOW}提示: Ctrl+C 各标签页停止对应节点${NC}"
trap "echo -e '\n${YELLOW}脚本退出, 节点继续运行${NC}'; exit 0" INT
while true; do sleep 1; done
