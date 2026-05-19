#!/bin/bash
# shellcheck shell=bash
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; exit 1; fi

# ═══════════════════════════════════════════════════════════════
# latte_imitation 一键测试启动脚本 v3.0 (MoveIt2 6 阶段管线)
#
# 参考 start_aubo_new_driver.sh 的多终端架构:
#   终端1 = 仿真环境 (move_group + ros2_control)
#   终端2 = latte_imitation 节点 (默认 preview 模式)
#   终端3 = 交互式预览/测试面板 (test_latte_pour.py)
#   终端4 = RViz2 可视化 (加载 latte_preview.rviz)
#   终端5 = 快捷命令速查
#
# 管线: _execute_pipeline → _pipeline (6 阶段)
#   ① Load npz → ② SE(3) Retarget → ③ Preview (RViz2 markers)
#   ④ Safety Check → ⑤ computeCartesianPath → ⑥ executeTrajectory
# 轨迹起点: 自动从 TF (base_link→tool_tcp) 获取当前 EE 位姿
#
# 用法:
#   ./start_latte_test.sh                  # 默认仿真模式
#   ./start_latte_test.sh --skip-build     # 跳过构建
#   ./start_latte_test.sh --real           # 真机模式 (需 AUBO IP 可达)
#   ./start_latte_test.sh --help           # 查看帮助
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

# ── 默认配置 ──────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCRIPT_DIR}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
AUBO_IP="169.254.10.98"
SKIP_BUILD=0
REAL_MODE=0

# ── 启动变量 (可通过 launch 文件覆盖) ──
EPISODE_IDX="${EPISODE_IDX:-0}"
ARM="${ARM:-right}"
MODE="${MODE:-preview}"                          # preview | debug | action
SPEED_SCALE="${SPEED_SCALE:-1.0}"

for arg in "$@"; do
    case "$arg" in
        --skip-build) SKIP_BUILD=1 ;;
        --real)       REAL_MODE=1 ;;
        --action)     MODE="action" ;;
        --help|-h)
            cat << 'HELP'
用法: ./start_latte_test.sh [选项]

选项:
  --skip-build    跳过 colcon build
  --real          真机模式 (需 AUBO IP 可达)
  --action        直接进入 action 模式 (真机执行)
  --help          显示此帮助

5 个 terminator 标签页:
  [Sim]           仿真环境 (move_group + ros2_control)
  [Latte Node]    latte_imitation 节点 (默认 preview 模式)
  [Preview Panel] 交互式预览/测试面板 (test_latte_pour.py)
  [RViz2]         RViz2 可视化 (加载 latte_preview.rviz)
  [Cmd Ref]       手动命令速查 (可直接复制执行)

Preview 模式说明:
  - 节点启动后自动发布 RViz2 markers: TCP 轨迹 (绿色)
    方向箭头 (绿)、spout 轨迹 (蓝)、杯子位置 (黄)、安全框 (红)
  - 在 [Preview Panel] 中调整 RPY 参数后按 [p] 刷新
  - 满意后按 [e] 进入 action 模式真机执行

管线流程 (trajectory_pipeline.py v3.0):
  ① Load npz  → ② SE(3) Retarget → ③ Preview (RViz2 markers)
  ④ Safety Check → ⑤ computeCartesianPath → ⑥ executeTrajectory

理论依据:
  SPOT (arXiv:2411.00965): Object-centric SE(3) 轨迹
  Isaac Teleop: Se3RelRetargeter delta 语义
  SO(3) Action Repr. (Savva 2025): Hamilton 四元数约定
  SVRC: Object-relative Cartesian → Very High generalization
HELP
            exit 0 ;;
    esac
done

# ── 清理函数 ──────────────────────────────────────────────────

cleanup() {
    echo -e "\n${YELLOW}终止所有进程...${NC}"
    pkill -f "latte_imitation"       2>/dev/null || true
    pkill -f "latte_debug_panel"     2>/dev/null || true
    pkill -f "test_latte_pour"       2>/dev/null || true
    pkill -f "test_replay_service"   2>/dev/null || true
    pkill -f "rviz2"                 2>/dev/null || true
    pkill -f "move_group"            2>/dev/null || true
    pkill -f "ros2_control_node"     2>/dev/null || true
    pkill -f "controller_manager"    2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "aubo_mode"             2>/dev/null || true
    pkill -f "spawner"               2>/dev/null || true
    pkill -f "joint_state_broadcaster" 2>/dev/null || true
    sleep 0.5
    echo -e "${GREEN}  ✓ 清理完成${NC}"
}
trap cleanup INT TERM EXIT

# ── 预检 ──────────────────────────────────────────────────────

TERMINATOR=""
if command -v terminator &>/dev/null; then TERMINATOR="terminator"
elif [ -x /usr/bin/terminator ]; then TERMINATOR="/usr/bin/terminator"; fi
if [ -z "$TERMINATOR" ]; then
    echo -e "${RED}未找到 terminator, sudo apt install terminator${NC}"; exit 1
fi

if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${RED}未找到 ROS 2: ${ROS2_SETUP}${NC}"; exit 1
fi

WS_SETUP="${WS}/install/setup.bash"
if [ ! -f "$WS_SETUP" ] && [ "$SKIP_BUILD" = "1" ]; then
    echo -e "${RED}install/setup.bash 不存在, 不能跳过构建${NC}"; exit 1
fi

LAUNCH_FILE="${WS}/src/latte_imitation/launch/start_latte_pour.launch.py"
PANEL_SCRIPT="${WS}/src/latte_imitation/scripts/test_latte_pour.py"
RVIZ_CONFIG="${WS}/src/latte_imitation/config/latte_preview.rviz"

# ── terminator 标签页启动函数 ─────────────────────────────────

launch() {
    local title="$1" cmd="$2"
    local full
    full="cd \"${WS}\" && unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY && source \"${ROS2_SETUP}\" && source \"${WS_SETUP}\" && ${cmd}; exec bash"
    "$TERMINATOR" --new-tab --title="$title" \
        -e "bash -c '${full}'" &
}

# ═══════════════════════════════════════════════════════════════
# 启动
# ═══════════════════════════════════════════════════════════════

cleanup

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}latte_imitation 测试启动 v3.0${NC}"
echo -e "${GREEN}  (6 阶段管线 + RViz2 Preview)${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}工作空间:   ${WS}${NC}"
echo -e "${BLUE}机器人IP:   ${AUBO_IP}${NC}"
echo -e "${BLUE}Episode:    ${EPISODE_IDX}   arm: ${ARM}   mode: ${MODE}   speed: ${SPEED_SCALE}x${NC}"
echo ""

# ═══════════════════════════════════════════════════════════════
# [0] 构建
# ═══════════════════════════════════════════════════════════════

if [ "$SKIP_BUILD" = "1" ]; then
    echo -e "${YELLOW}[0] 跳过构建 (--skip-build)${NC}"
else
    echo -e "${GREEN}[0] 构建...${NC}"
    (
        cd "$WS"
        source "$ROS2_SETUP"
        if [ -f "$WS_SETUP" ]; then source "$WS_SETUP"; fi
        colcon build --packages-select ivg_interfaces latte_imitation
    )
    echo -e "${GREEN}  ✓ 构建完成${NC}"
fi

# ═══════════════════════════════════════════════════════════════
# [1] 终端1 — 仿真环境
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[1] 终端1 — 仿真环境...${NC}"
launch "Sim" "ros2 launch aubo_moveit_config aubo_new_driver.launch.py server_host:=${AUBO_IP}"

# 等待 move_group 就绪
echo -e "${BLUE}  → 等待 /move_group 节点 (超时 30s)...${NC}"
TICK=0
while [ "$TICK" -lt 60 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 node list 2>/dev/null | grep -q "/move_group"; then
        echo -e "${GREEN}    ✓ /move_group 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# 等待 /execute_trajectory action
echo -e "${BLUE}  → 等待 /execute_trajectory action (超时 60s)...${NC}"
TICK=0
while [ "$TICK" -lt 120 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 action list 2>/dev/null | grep -q "/execute_trajectory"; then
        echo -e "${GREEN}    ✓ /execute_trajectory 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# 等待 /compute_cartesian_path 服务
echo -e "${BLUE}  → 等待 /compute_cartesian_path 服务 (超时 30s)...${NC}"
TICK=0
while [ "$TICK" -lt 60 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 service list 2>/dev/null | grep -q "/compute_cartesian_path"; then
        echo -e "${GREEN}    ✓ /compute_cartesian_path 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# ═══════════════════════════════════════════════════════════════
# [2] 终端2 — latte_imitation 节点 (preview 模式)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[2] 终端2 — latte_imitation 节点 (mode=${MODE})...${NC}"
launch "Latte Node" "ros2 launch latte_imitation start_latte_pour.launch.py \
    episode_idx:=${EPISODE_IDX} arm:=${ARM} mode:=${MODE} speed_scale:=${SPEED_SCALE}"

# 等待服务就绪
echo -e "${BLUE}  → 等待 /latte_imitation/replay_trajectory 服务 (超时 15s)...${NC}"
TICK=0
while [ "$TICK" -lt 30 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 service list 2>/dev/null | grep -q "replay_trajectory"; then
        echo -e "${GREEN}    ✓ 服务就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# ═══════════════════════════════════════════════════════════════
# [3] 终端3 — 交互式预览/测试面板
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[3] 终端3 — 交互式预览/测试面板...${NC}"
if [ -f "$PANEL_SCRIPT" ]; then
    launch "Preview Panel" "python3 ${PANEL_SCRIPT}"
else
    echo -e "${RED}  ✗ ${PANEL_SCRIPT} 不存在${NC}"
fi

# ═══════════════════════════════════════════════════════════════
# [4] 终端4 — RViz2 可视化 (加载 latte_preview.rviz)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[4] 终端4 — RViz2 可视化...${NC}"
if [ -f "$RVIZ_CONFIG" ]; then
    launch "RViz2" "rviz2 -d ${RVIZ_CONFIG}"
else
    echo -e "${YELLOW}  ⚠ latte_preview.rviz 不存在, 启动默认 RViz2${NC}"
    launch "RViz2" "rviz2"
fi

# ═══════════════════════════════════════════════════════════════
# [5] 终端5 — 手动命令速查 (v3.0 新字段)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[5] 终端5 — 快捷命令速查...${NC}"
launch "Cmd Ref" "cat << 'CMDS'
═══════════════════════════════════════════════════════════════
  latte_imitation v3.0 命令速查 (6 阶段管线 + RViz2 Preview)
═══════════════════════════════════════════════════════════════

=== Preview 模式 (RViz2 可视化, 不规划/执行) ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: preview,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== Action 模式 (完整管线, 真机执行, 2倍速) ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 2.0, mode: action,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== Preview + Yaw 90° 旋转 ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: preview,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 90.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== Preview + Roll 前倾 10° ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: preview,
    roll_deg: 10.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== 指定杯子位置 (position 非零) ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: preview,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== 错误处理 — 不存在 episode ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 999, arm: right, speed_scale: 1.0, mode: preview,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"
# 预期: success=false, message=episode_000999.npz ... 未找到

=== 左臂持杯轨迹 ===
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: left, speed_scale: 1.0, mode: preview,
    roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
    tool_offset_id: default,
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"
# 预期: success=true, num_frames=400, path_length~0.31

═══════════════════════════════════════════════════════════════
  请求字段说明 (v3.0 新增字段标 ★)
═══════════════════════════════════════════════════════════════
episode_idx    0~39               Episode 编号
arm            right=拉花臂 left=持杯臂
speed_scale    0.01~10.0          播放速度倍率
mode           preview/debug/action  (preview=RViz2可视化)
★ roll_deg     绕 X 轴旋转角度 (度, 默认 0)
★ pitch_deg    绕 Y 轴旋转角度 (度, 默认 0)
★ yaw_deg      绕 Z 轴旋转角度 (度, 默认 0)
★ tool_offset_id 工具偏移ID (config/tool_offset.yaml, 默认 default)
start_pose:
  position     0,0,0→TF自动获取   非零→指定杯子位置
  orientation  0,0,0,1→纯平移    非identity→叠加旋转

═══════════════════════════════════════════════════════════════
  响应字段
═══════════════════════════════════════════════════════════════
success / message / num_frames / path_length
ik_success_count = int(fraction * num_frames)
collision_count = 0, collision_details = []

═══════════════════════════════════════════════════════════════
  6 阶段管线 (trajectory_pipeline.py v3.0)
═══════════════════════════════════════════════════════════════
_execute_pipeline (加锁)
  └─ _pipeline:
      ① _load_cartesian()        加载 npz 文件
      ② retarget_trajectory()     SE(3) 重定目标 + RPY
      ③ _publish_preview()       RViz2 markers (mode=preview 返回)
      ④ check_workspace_bounds() 工作空间安全检查
      ⑤ _compute_cartesian_path() MoveIt2 笛卡尔规划
      ⑥ _execute_trajectory()    MoveIt2 轨迹执行

═══════════════════════════════════════════════════════════════
  理论依据
═══════════════════════════════════════════════════════════════
SPOT (arXiv:2411.00965):           Object-centric SE(3) 轨迹
Isaac Teleop (NVIDIA):             Se3RelRetargeter delta 语义
SO(3) Action Repr. (Savva 2025):   Hamilton 四元数约定
SVRC:                              Object-relative > absolute
FluidLab (ICLR 2023):              可微物理拉花仿真
CMDS
echo '按 Ctrl+C 或直接关终端退出喵~'"

# ═══════════════════════════════════════════════════════════════
# 完成
# ═══════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  latte_imitation v3.0 测试环境就绪${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}5 个 terminator 标签页:${NC}"
echo -e "  ${CYAN}[Sim]${NC}           仿真环境 (move_group + ros2_control)"
echo -e "  ${CYAN}[Latte Node]${NC}    latte_imitation 节点 (mode=${MODE})"
echo -e "  ${CYAN}[Preview Panel]${NC} 交互式预览/测试面板 (test_latte_pour.py)"
echo -e "  ${CYAN}[RViz2]${NC}         RViz2 可视化 (latte_preview.rviz)"
echo -e "  ${CYAN}[Cmd Ref]${NC}       手动命令速查"
echo ""
echo -e "${YELLOW}Preview 使用流程:${NC}"
echo -e "  1. 在 [Preview Panel] 中调整 RPY 角度"
echo -e "  2. 按 [p] 刷新 → ${CYAN}RViz2 即时更新轨迹预览${NC}"
echo -e "  3. 满意后按 [e] → 输入 yes → ${CYAN}真机执行${NC}"
echo ""
echo -e "${YELLOW}RViz2 Display 提示 (已预配置):${NC}"
echo -e "  Path (绿)     → /latte_imitation/preview/tcp_path"
echo -e "  PoseArray (绿) → /latte_imitation/preview/tcp_waypoints"
echo -e "  Marker (蓝)    → /latte_imitation/preview/spout_path"
echo -e "  Marker (黄)    → /latte_imitation/preview/cup_pose"
echo -e "  Marker (红)    → /latte_imitation/preview/workspace_bounds"
echo ""
echo -e "${YELLOW}Ctrl+C 或直接关闭 terminator → 自动清理所有进程喵~${NC}"

# 阻塞主进程, 等待用户 Ctrl+C
wait
