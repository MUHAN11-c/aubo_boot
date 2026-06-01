#!/bin/bash
# shellcheck shell=bash
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; exit 1; fi

# ═══════════════════════════════════════════════════════════════
# latte_backend 一键测试启动脚本 v4.0 (C++ 5 步工作流)
#
# latte_imitation (Python) + latte_cartesian_planner 已删除
# 替代: latte_backend — 独立 C++ 包, 依赖 demo_driver::robot_controller
#
# 终端1 = 仿真环境 (move_group + ros2_control)
# 终端2 = latte_backend 工作流节点
# 终端3 = 命令速查
#
# 用法:
#   ./start_latte_test.sh                  # 默认仿真模式
#   ./start_latte_test.sh --skip-build     # 跳过构建
#   ./start_latte_test.sh --help           # 查看帮助
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SCRIPT_DIR}"
ROS2_SETUP="/opt/ros/humble/setup.bash"
SKIP_BUILD=0

for arg in "$@"; do
    case "$arg" in
        --skip-build) SKIP_BUILD=1 ;;
        --help|-h)
            cat << 'HELP'
用法: ./start_latte_test.sh [选项]

选项:
  --skip-build    跳过 colcon build
  --help          显示此帮助

3 个 terminator 标签页:
  [Sim]           仿真环境 (move_group + ros2_control)
  [Latte Backend] latte_backend 工作流节点
  [Cmd Ref]       手动命令速查

启动后执行工作流:
  ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow "{}"
HELP
            exit 0 ;;
    esac
done

# ── 清理函数 ──────────────────────────────────────────────────

cleanup() {
    echo -e "\n${YELLOW}终止所有进程...${NC}"
    pkill -f "latte_workflow_node"    2>/dev/null || true
    pkill -f "rviz2"                  2>/dev/null || true
    pkill -f "move_group"             2>/dev/null || true
    pkill -f "ros2_control_node"      2>/dev/null || true
    pkill -f "controller_manager"     2>/dev/null || true
    pkill -f "robot_state_publisher"  2>/dev/null || true
    pkill -f "aubo_mode"              2>/dev/null || true
    pkill -f "spawner"                2>/dev/null || true
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
echo -e "${GREEN}latte_backend 测试启动 v4.0${NC}"
echo -e "${GREEN}  (C++ 5 步拉花工作流)${NC}"
echo -e "${GREEN}========================================${NC}"

# [0] 构建
if [ "$SKIP_BUILD" = "1" ]; then
    echo -e "${YELLOW}[0] 跳过构建 (--skip-build)${NC}"
else
    echo -e "${GREEN}[0] 构建...${NC}"
    (
        cd "$WS"
        source "$ROS2_SETUP"
        if [ -f "$WS_SETUP" ]; then source "$WS_SETUP"; fi
        colcon build --packages-select demo_driver latte_backend
    )
    echo -e "${GREEN}  ✓ 构建完成${NC}"
fi

# [1] 终端1 — 仿真环境
echo -e "${GREEN}[1] 终端1 — 仿真环境...${NC}"
launch "Sim" "ros2 launch aubo_moveit_config aubo_new_driver.launch.py use_fake_hardware:=true"

echo -e "${BLUE}  → 等待 /joint_states (超时 30s)...${NC}"
TICK=0
while [ "$TICK" -lt 60 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 topic list 2>/dev/null | grep -q "/joint_states"; then
        echo -e "${GREEN}    ✓ /joint_states 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# [2] 终端2 — latte_backend 工作流节点
echo -e "${GREEN}[2] 终端2 — latte_backend 工作流节点...${NC}"
launch "Latte Backend" "ros2 launch latte_backend latte_workflow.launch.py"

echo -e "${BLUE}  → 等待 /latte/run_workflow 服务 (超时 60s)...${NC}"
TICK=0
while [ "$TICK" -lt 120 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 service list 2>/dev/null | grep -q "/latte/run_workflow"; then
        echo -e "${GREEN}    ✓ /latte/run_workflow 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# [3] 终端3 — 命令速查
echo -e "${GREEN}[3] 终端3 — 命令速查...${NC}"
launch "Cmd Ref" "cat << 'CMDS'
═══════════════════════════════════════════════════════════════
  latte_backend v4.0 — C++ 5 步咖啡拉花工作流
═══════════════════════════════════════════════════════════════

=== 执行完整 5 步工作流 ===
ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow \"{}\"

=== 仅执行 step5 拉花轨迹 (手动完成 step1-4 后) ===
ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow \"{data: ''}\"

=== 跳过 step5 拉花执行 ===
ros2 param set /latte_workflow_node lwf_execute_latte false

=== 切换拉花图案 ===
ros2 param set /latte_workflow_node lwf_pattern_type heart

=== 调整拉花速度 ===
ros2 param set /latte_workflow_node lwf_heart_velocity 0.3

=== 禁用动态 Roll 渐变 (IK 失败时回退) ===
ros2 param set /latte_workflow_node lwf_heart_roll_draw_dynamic false

=== 查看所有拉花参数 ===
ros2 param dump /latte_workflow_node | grep lwf

═══════════════════════════════════════════════════════════════
  5 步工作流 (handleRunWorkflow)
═══════════════════════════════════════════════════════════════
[1/5] 取牛奶杯 → moveToJoints 预教位姿 → 抓取
[2/5] 打奶泡   → 笛卡尔直线去/回喷嘴 → 等待2s
[3/5] 转腕朝上 → 笛卡尔 slerp (FK rotate_up_joints)
      放置咖啡杯 → 笛卡尔 slerp (FK place_coffee_joints)
[4/5] 嘴口倾倒 → 绕世界X轴前倾45° (倾倒基准)
[5/5] 心形拉花 → 融合画圈→成形注入→划穿收尾

═══════════════════════════════════════════════════════════════
  坐标系 (世界坐标系, base_link)
═══════════════════════════════════════════════════════════════
X 轴 (前) → 倾倒倾角轴 (绕此轴旋转 = 奶缸前倾/后仰)
Y 轴 (左) → 划穿方向轴
Z 轴 (上) → 高度轴 (液面上方距离)

═══════════════════════════════════════════════════════════════
  心形轨迹分段 (step5, 基于 Barista Hustle MSLA)
═══════════════════════════════════════════════════════════════
[5a] 融合画圈  Z=80mm roll=45°      r=10mm ×2圈 (25%)
[5b] 成形注入  Z=5mm  roll=60°→45°  定点不动 (55%)
[5c] 划穿收尾  Z=80mm roll=50°      Y轴推进15mm (20%)

CMDS
echo '按 Ctrl+C 或直接关终端退出喵~'"

# ═══════════════════════════════════════════════════════════════
# 完成
# ═══════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  latte_backend v4.0 测试环境就绪${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}3 个 terminator 标签页:${NC}"
echo -e "  ${CYAN}[Sim]${NC}            仿真环境 (move_group + ros2_control)"
echo -e "  ${CYAN}[Latte Backend]${NC}  latte_backend 工作流节点"
echo -e "  ${CYAN}[Cmd Ref]${NC}        手动命令速查"
echo ""
echo -e "${YELLOW}执行工作流:${NC}"
echo -e "  ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow \"{}\""
echo ""
echo -e "${YELLOW}Ctrl+C 或直接关闭 terminator → 自动清理所有进程喵~${NC}"

wait
