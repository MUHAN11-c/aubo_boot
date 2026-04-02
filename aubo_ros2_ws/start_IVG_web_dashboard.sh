#!/bin/bash
# 请使用 bash 运行（./start_IVG_web_dashboard.sh 或 bash start_IVG_web_dashboard.sh），勿用 sh
# 若已用 sh 启动，则自动用 bash 重新执行（避免 echo -e 等不生效）
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
    exit 1
fi

# IVG 完整启动脚本（GraspNet Points + 统一 Web 网关 aubo_ros2_web_dashboard）
# 与 start_IVG_graspnet_points_fastapi.sh 步骤 0–12、14 一致；第 13 步为 web_dashboard.launch.py：
#   - 默认 WEB_DASH_PORT=8090、ROSBRIDGE_PORT=9090、WEB_DASH_HOST=0.0.0.0（手机/平板用局域网 IP 访问）
#   - LAUNCH_VPE_WEB=true（默认）：launch 内 Include visual_pose_estimation_web → 127.0.0.1:8088
#   - LAUNCH_VPE_WEB=false：不在此 launch 内起 8088，须已有 VPE Web 且与 VPE_UPSTREAM 一致（避免双占 8088）
#
# 系统依赖（未装则第 13 步会失败；发行版默认 humble，可设 ROS_DISTRO_NAME 覆盖）：
#   sudo apt install ros-humble-rosapi ros-humble-rosbridge-suite ros-humble-tf2-web-republisher terminator
# Humble 下 tf2 可执行名为 tf2_web_republisher_node（launch 已按仓库修正，更新后请 colcon build 本包）
#
# 代理与本机 HTTP 自检：若 export 了 http_proxy/ALL_PROXY（含 socks://），curl 访问 127.0.0.1 可能假 502；
#   脚本内 curl 已加 --noproxy '*'；第 13 步默认对 ros2 launch 剥离代理（见 IVG_STRIP_PROXY_FOR_DASH_LAUNCH），
#   与网关内 httpx trust_env=false 一致。建议 NO_PROXY 含 localhost,127.0.0.1,192.168.0.0/16
#
# rosbridge 报 Address already in use：旧进程占用 ROSBRIDGE_PORT，ss -tlnp | grep 9090 后结束旧 launch 或改 ROSBRIDGE_PORT
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

# 统一网关（Robot Web Tools + VPE 反向代理）
WEB_DASH_HOST="${WEB_DASH_HOST:-0.0.0.0}"
WEB_DASH_PORT="${WEB_DASH_PORT:-8090}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
# 须与 launch_vpe_web 拉起的 VPE 端口一致（launch 内固定 127.0.0.1:8088）
VPE_UPSTREAM="${VPE_UPSTREAM:-http://127.0.0.1:8088}"
# 若为 true，web_dashboard.launch 会 Include visual_pose_estimation_web（8088）
LAUNCH_VPE_WEB="${LAUNCH_VPE_WEB:-true}"
# 第 13 步执行 ros2 launch 前剥离代理变量，避免子进程（uvicorn/httpx 等）受 socks:// 等影响；设 false 则保留环境代理
IVG_STRIP_PROXY_FOR_DASH_LAUNCH="${IVG_STRIP_PROXY_FOR_DASH_LAUNCH:-true}"

WEB_DASH_URL="http://127.0.0.1:${WEB_DASH_PORT}"
VPE_WEB_URL="http://127.0.0.1:8088"

# 步骤14 rosbag：输出目录；启动前会 rm -rf 实现覆盖。话题默认为 -a 全部；可设 IVG_ROSBAG_TOPICS="/t1 /t2"
IVG_ROSBAG_DIR="${IVG_ROSBAG_DIR:-${AUBO_ROS2_WS}/rosbags/ivg_session}"
IVG_ROSBAG_TOPICS="${IVG_ROSBAG_TOPICS:-}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IVG 系统启动脚本（GraspNet Points + 统一 Web 网关）${NC}"
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
echo -e "${GREEN}[0/14] 构建工作空间...${NC}"
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

# 第 13 步前预检（仅提示，不中断）
preflight_web_dashboard() {
    (
        cd "$AUBO_ROS2_WS" || exit 0
        # shellcheck disable=SC1090
        eval "$ROS2_BASE_ENV"
        if [ -f install/setup.bash ]; then
            # shellcheck disable=SC1091
            source install/setup.bash
        fi
        if ! ros2 pkg prefix rosapi &>/dev/null; then
            echo -e "${YELLOW}  ! 未检测到 rosapi。请安装: sudo apt install ros-${ROS_DISTRO_NAME}-rosapi ros-${ROS_DISTRO_NAME}-rosbridge-suite${NC}"
        fi
        local tf2_exe="/opt/ros/${ROS_DISTRO_NAME}/lib/tf2_web_republisher/tf2_web_republisher_node"
        if [ ! -x "$tf2_exe" ]; then
            echo -e "${YELLOW}  ! 未检测到 tf2_web_republisher_node。请安装: sudo apt install ros-${ROS_DISTRO_NAME}-tf2-web-republisher${NC}"
        fi
        if ! ros2 pkg prefix aubo_ros2_web_dashboard &>/dev/null; then
            echo -e "${YELLOW}  ! 工作空间中未找到 aubo_ros2_web_dashboard（是否未 colcon build？）${NC}"
        fi
    ) || true
    if command -v ss &>/dev/null; then
        if ss -tlnp 2>/dev/null | grep -q ":${ROSBRIDGE_PORT}\\b"; then
            echo -e "${YELLOW}  ! 端口 ${ROSBRIDGE_PORT} 已被监听：若随后 rosbridge 报 Address already in use，请结束旧进程或设置 ROSBRIDGE_PORT=9091${NC}"
        fi
    fi
}

# 步骤1: 启动真实机械臂驱动
echo -e "${GREEN}[1/14] 启动真实机械臂驱动（aubo_moveit_pure_ros2）...${NC}"
AUBO_PURE_ROS2_CMD="$WS_ENV && ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py"
launch_in_terminator "Aubo MoveIt Pure ROS2" "$AUBO_PURE_ROS2_CMD"
echo -e "${GREEN}  ✓ 真实机械臂驱动已启动${NC}"
sleep 3

# 步骤2: 启动机器人驱动服务
echo -e "${GREEN}[2/14] 启动机器人驱动服务...${NC}"
DRIVER_CMD="$WS_ENV && ros2 launch aubo_moveit_config demo_driver_services.launch.py"
launch_in_terminator "Robot Driver" "$DRIVER_CMD"
echo -e "${GREEN}  ✓ 机器人驱动服务已启动${NC}"
sleep 3

# 步骤3: 启动相机节点
echo -e "${GREEN}[3/14] 启动相机节点...${NC}"
CAMERA_CMD="$WS_ENV && ros2 launch percipio_camera percipio_camera.launch.py"
launch_in_terminator "Percipio Camera" "$CAMERA_CMD"
echo -e "${GREEN}  ✓ 相机节点已启动${NC}"
sleep 5

# 步骤4: 启动相机控制节点
echo -e "${GREEN}[4/14] 启动相机控制节点...${NC}"
CAMERA_CONTROL_CMD="$WS_ENV && ros2 launch percipio_camera_interface camera_control.launch.py"
launch_in_terminator "Camera Control" "$CAMERA_CONTROL_CMD"
echo -e "${GREEN}  ✓ 相机控制节点已启动${NC}"
sleep 2

# 步骤5: 启动图像数据桥接节点
echo -e "${GREEN}[5/14] 启动图像数据桥接节点...${NC}"
IMAGE_BRIDGE_CMD="$WS_ENV && ros2 launch image_data_bridge image_data_bridge.launch.py input_image_topic:=/camera/color/image_raw"
launch_in_terminator "Image Data Bridge" "$IMAGE_BRIDGE_CMD"
echo -e "${GREEN}  ✓ 图像数据桥接节点已启动${NC}"
sleep 2

# 步骤6: 启动手眼标定节点
echo -e "${GREEN}[6/14] 启动手眼标定节点...${NC}"
HAND_EYE_CMD="$WS_ENV && ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
launch_in_terminator "Hand Eye Calibration" "$HAND_EYE_CMD"
echo -e "${GREEN}  ✓ 手眼标定节点已启动${NC}"
sleep 2

# 步骤7: 启动视觉姿态估计算法节点
echo -e "${GREEN}[7/14] 启动视觉姿态估计算法节点...${NC}"
VPE_CMD="$WS_ENV && export PATH=\"/usr/bin:\$PATH\" && ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
launch_in_terminator "Visual Pose Estimation" "$VPE_CMD"
echo -e "${GREEN}  ✓ 视觉姿态估计算法节点已启动${NC}"
sleep 2

# 步骤8: 启动 GraspNet 点云节点
echo -e "${GREEN}[8/14] 启动 GraspNet 点云节点（with_tf）...${NC}"
GRASPNET_WITH_TF_CMD="$WS_ENV && ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py launch_camera:=false launch_hand_eye_tf:=true"
launch_in_terminator "GraspNet Points With TF" "$GRASPNET_WITH_TF_CMD"
echo -e "${GREEN}  ✓ GraspNet 点云节点已启动${NC}"
sleep 2

# 步骤9: 启动执行抓取位姿服务节点
echo -e "${GREEN}[9/14] 启动执行抓取位姿服务节点...${NC}"
GRASP_WORKER_CMD="$WS_ENV && ros2 launch demo_driver execute_grasp_pose_worker.launch.py"
launch_in_terminator "Execute Grasp Worker" "$GRASP_WORKER_CMD"
echo -e "${GREEN}  ✓ 执行抓取位姿服务节点已启动${NC}"
sleep 2

# 步骤10: 启动夹爪切换服务节点
echo -e "${GREEN}[10/14] 启动夹爪切换服务节点...${NC}"
GRIPPER_SWAP_CMD="$WS_ENV && ros2 run demo_driver gripper_swap_worker_node"
launch_in_terminator "Gripper Swap Worker" "$GRIPPER_SWAP_CMD"
echo -e "${GREEN}  ✓ 夹爪切换服务节点已启动${NC}"
sleep 2

# 步骤11: 启动 GraspNet 循环抓取 Worker
echo -e "${GREEN}[11/14] 启动 GraspNet 循环抓取 Worker...${NC}"
PUBLISH_GRASPS_WORKER_CMD="$WS_ENV && ros2 run demo_driver publish_grasps_client_worker_node"
launch_in_terminator "Publish Grasps Worker" "$PUBLISH_GRASPS_WORKER_CMD"
echo -e "${GREEN}  ✓ GraspNet 循环抓取 Worker 已启动${NC}"
sleep 2

# 步骤12: 校验关键服务是否已就绪
echo -e "${GREEN}[12/14] 校验关键服务...${NC}"
SERVICE_CHECK_CMD="$WS_ENV && ros2 service list | rg '/estimate_pose|/list_templates|/graspnet_capture_control|/publish_grasps_worker_loop_control|/loop_grasp_control|/run_gripper_swap' || true"
launch_in_terminator "Service Check" "$SERVICE_CHECK_CMD"
echo -e "${GREEN}  ✓ 关键服务校验标签页已启动${NC}"
sleep 1

# 步骤13: 启动统一 Web 网关（rosbridge + rosapi + FastAPI；可选一并启动 VPE Web 8088）
echo -e "${GREEN}[13/14] 启动统一 Web 网关（aubo_ros2_web_dashboard）...${NC}"
preflight_web_dashboard
if [ "${IVG_STRIP_PROXY_FOR_DASH_LAUNCH}" = "true" ]; then
    WEB_DASH_PROXY_STRIP="env -u ALL_PROXY -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u all_proxy "
else
    WEB_DASH_PROXY_STRIP=""
fi
WEB_DASH_CMD="$WS_ENV && ${WEB_DASH_PROXY_STRIP}ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py web_host:=${WEB_DASH_HOST} web_port:=${WEB_DASH_PORT} rosbridge_port:=${ROSBRIDGE_PORT} vpe_upstream:=${VPE_UPSTREAM} launch_vpe_web:=${LAUNCH_VPE_WEB}"
launch_in_terminator "IVG Web Dashboard" "$WEB_DASH_CMD"
echo -e "${GREEN}  ✓ 统一 Web 网关已启动${NC}"
sleep 3

# 步骤14: rosbag 录制（删除已有同名目录后重新录，实现覆盖）
echo -e "${GREEN}[14/14] rosbag 录制数据（覆盖: ${IVG_ROSBAG_DIR}）...${NC}"
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

# 注意：/health 经网关反向代理到 VPE，上游未就绪时为 502；本网关进程存活请用 /gateway/health
# curl 加 --noproxy '*'：若环境中有 http_proxy（如 7890 本地代理），否则访问 127.0.0.1 可能被误转发而得到假 502
DASH_LIVENESS_URL="${WEB_DASH_URL}/gateway/health"
CURL_LOCAL=(curl --noproxy '*' --fail --silent --max-time 8)
if command -v curl >/dev/null 2>&1; then
    if "${CURL_LOCAL[@]}" "$DASH_LIVENESS_URL" >/dev/null; then
        echo -e "${GREEN}  ✓ 网关 Liveness 通过: ${DASH_LIVENESS_URL}${NC}"
    else
        echo -e "${YELLOW}  ! 网关 Liveness 暂未通过，请查看「IVG Web Dashboard」标签页日志${NC}"
    fi
    if [ "${LAUNCH_VPE_WEB}" = "true" ]; then
        if "${CURL_LOCAL[@]}" "${VPE_WEB_URL}/health" >/dev/null; then
            echo -e "${GREEN}  ✓ VPE Web 健康检查通过: ${VPE_WEB_URL}/health${NC}"
        else
            echo -e "${YELLOW}  ! VPE Web 健康检查暂未通过（若未安装 visual_pose_estimation_python 请将 LAUNCH_VPE_WEB=false）${NC}"
        fi
    else
        if "${CURL_LOCAL[@]}" "${VPE_WEB_URL}/health" >/dev/null; then
            echo -e "${GREEN}  ✓ 检测到 ${VPE_WEB_URL}/health（LAUNCH_VPE_WEB=false 时已存在上游，与 vpe_upstream 一致即可）${NC}"
        else
            echo -e "${YELLOW}  ! LAUNCH_VPE_WEB=false 且 ${VPE_WEB_URL} 未响应：请自行启动 VPE Web 或核对 VPE_UPSTREAM${NC}"
        fi
    fi
else
    echo -e "${YELLOW}  ! 未找到 curl，跳过 HTTP 健康检查${NC}"
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
echo -e "${GREEN}手眼标定 Web 界面地址: http://localhost:8080${NC}"
echo -e "${GREEN}统一 Web 网关（RWT 建议 /rwt/；Liveness: ${WEB_DASH_URL}/gateway/health）: ${WEB_DASH_URL}/${NC}"
if [ "${LAUNCH_VPE_WEB}" = "true" ]; then
    echo -e "${GREEN}视觉位姿 FastAPI（本 launch 已起 8088 时；Legacy 代理上游）: ${VPE_WEB_URL}/${NC}"
else
    echo -e "${GREEN}视觉位姿 FastAPI: 由外部进程提供，须与 VPE_UPSTREAM=${VPE_UPSTREAM} 一致${NC}"
fi
echo ""
echo -e "${BLUE}自检（建议加 --noproxy '*'，勿用 /health 判断网关进程）:${NC}"
echo "  curl -sS --noproxy '*' ${WEB_DASH_URL}/gateway/health"
echo "  ss -tlnp | grep -E ':${WEB_DASH_PORT}|:${ROSBRIDGE_PORT}'"
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
echo "  pkill -f 'web_dashboard.launch.py'"
echo "  pkill -f 'visual_pose_estimation_web.launch.py'"
echo "  pkill -f 'tf2_web_republisher'"
echo "  pkill -f 'ros2 bag record'"

echo ""
echo -e "${BLUE}按 Ctrl+C 退出此脚本（不会停止已启动的节点）${NC}"
trap "echo -e '\n${YELLOW}脚本已退出，节点继续运行${NC}'; exit 0" INT
while true; do
    sleep 1
done
