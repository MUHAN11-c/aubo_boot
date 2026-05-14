#!/bin/bash
if [ -z "${BASH_VERSION:-}" ]; then exec /bin/bash "$0" "$@"; exit 1; fi

# ═══════════════════════════════════════════════════════════════
# latte_imitation 一键测试启动脚本 (MoveIt2 新管线)
#
# 参考 start_aubo_new_driver.sh 的多终端架构:
#   终端1 = 仿真环境 (ros2 launch aubo_new_driver)
#   终端2 = latte_imitation 服务节点
#   终端3 = 交互式测试菜单
#   终端4 = 快捷命令速查 (可选)
#
# 新管线: MoveIt2 computeCartesianPath → execute (不再使用自定义 IK)
# 轨迹起点: 自动从 TF (base_link→tool_tcp) 获取当前 EE 位姿
#
# 用法:
#   ./start_latte_test.sh                  # 默认仿真模式
#   ./start_latte_test.sh --skip-build     # 跳过构建
#   ./start_latte_test.sh --real           # 真机模式 (需AUBO IP可达)
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
SKIP_RVIZ=0
REAL_MODE=0

for arg in "$@"; do
    case "$arg" in
        --skip-build) SKIP_BUILD=1 ;;
        --skip-rviz)  SKIP_RVIZ=1 ;;
        --real)       REAL_MODE=1 ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo "  --skip-build  跳过 colcon build"
            echo "  --skip-rviz   不启动 RViz2"
            echo "  --real        真机模式 (需 AUBO IP=${AUBO_IP} 可达)"
            echo "  --help        显示此帮助"
            exit 0 ;;
    esac
done

# ── 清理函数 (Ctrl+C 时终止 terminator 中的所有子进程) ──────

cleanup() {
    echo -e "\n${YELLOW}终止所有进程...${NC}"
    pkill -f "latte_imitation"       2>/dev/null || true
    pkill -f "test_replay_service"   2>/dev/null || true
    pkill -f "rviz2"                 2>/dev/null || true
    pkill -f "move_group"            2>/dev/null || true
    pkill -f "ros2_control_node"     2>/dev/null || true
    pkill -f "controller_manager"    2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "aubo_mode"             2>/dev/null || true
    pkill -f "spawner"               2>/dev/null || true
    sleep 0.5
    echo -e "${GREEN}  ✓ 清理完成${NC}"
}
trap cleanup INT TERM EXIT

# ── 路径自定位 ────────────────────────────────────────────────

WS_SETUP="${WS}/install/setup.bash"

# ── terminator 标签页启动函数 (参考 start_aubo_new_driver.sh) ──

launch() {
    local title="$1" cmd="$2"
    local full
    full="cd \"${WS}\" && source \"${ROS2_SETUP}\" && source \"${WS_SETUP}\" && ${cmd}; exec bash"
    "$TERMINATOR" --new-tab --title="$title" \
        -e "bash -c '${full}'" &
}

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
if [ ! -f "$WS_SETUP" ] && [ "$SKIP_BUILD" = "1" ]; then
    echo -e "${RED}install/setup.bash 不存在, 不能跳过构建${NC}"; exit 1
fi

# ── 清理旧进程 ────────────────────────────────────────────────

echo -e "${YELLOW}清理旧进程...${NC}"
cleanup

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}latte_imitation 测试启动 (MoveIt2 新管线)${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}工作空间: ${WS}${NC}"
echo -e "${BLUE}机器人IP: ${AUBO_IP}${NC}"
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
# [1] 终端1 — 仿真环境 (move_group + ros2_control + RViz)
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

# 等待 joint_trajectory_controller action 就绪 (仿真 ros2_control)
echo -e "${BLUE}  → 等待 /joint_trajectory_controller/follow_joint_trajectory (超时 60s)...${NC}"
TICK=0
while [ "$TICK" -lt 120 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 action list 2>/dev/null | grep -q "follow_joint_trajectory"; then
        echo -e "${GREEN}    ✓ JTC 控制器就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# 等待 MoveIt2 /execute_trajectory action 就绪 (latte_imitation 新管线依赖)
echo -e "${BLUE}  → 等待 /execute_trajectory action (超时 30s)...${NC}"
TICK=0
while [ "$TICK" -lt 60 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 action list 2>/dev/null | grep -q "/execute_trajectory"; then
        echo -e "${GREEN}    ✓ /execute_trajectory 就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# ═══════════════════════════════════════════════════════════════
# [2] 终端2 — latte_imitation 服务节点
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[2] 终端2 — latte_imitation 服务节点...${NC}"
launch "Latte Svc" "ros2 run latte_imitation latte_imitation_node --ros-args -p mode:=debug"

# 等待服务就绪
echo -e "${BLUE}  → 等待 /latte_imitation/replay_trajectory (超时 15s)...${NC}"
TICK=0
while [ "$TICK" -lt 30 ]; do
    sleep 0.5; TICK=$((TICK + 1))
    if ros2 service list 2>/dev/null | grep -q "replay_trajectory"; then
        echo -e "${GREEN}    ✓ 服务就绪 (${TICK}x0.5s)${NC}"
        break
    fi
done

# ═══════════════════════════════════════════════════════════════
# [3] 终端3 — 交互式测试菜单
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[3] 终端3 — 交互式测试菜单...${NC}"
TEST_SCRIPT="${WS}/src/latte_imitation/scripts/test_replay_service.py"
if [ -f "$TEST_SCRIPT" ]; then
    launch "Test" "python3 ${TEST_SCRIPT}"
else
    echo -e "${YELLOW}  ⚠ ${TEST_SCRIPT} 不存在, 跳过${NC}"
fi

# ═══════════════════════════════════════════════════════════════
# [4] 终端4 — 手动命令速查 (可选)
# ═══════════════════════════════════════════════════════════════

echo -e "${GREEN}[4] 终端4 — 快捷命令速查...${NC}"
launch "Cmd Ref" "cat << 'CMDS'
=== 测试命令速查 (MoveIt2 新管线) ===

Test 1 — Debug (自动从 TF 获取起点):
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: debug, \\
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

Test 2 — Action (MoveIt2 CartesianPath→Execute, 2倍速~10s):
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 2.0, mode: action, \\
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

Test 3 — start_pose 覆盖: 杯子在 (0.45,0,0.50), 杯口Z轴朝上:
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: action, \\
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

Test 4 — start_pose 覆盖: 杯子绕Z轴90°倾斜 (z=0.707,w=0.707):
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: right, speed_scale: 1.0, mode: action, \\
    start_pose: {position: {x: 0.45, y: 0.0, z: 0.50}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}\"

Test 5 — 错误处理:
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 999, arm: right, speed_scale: 1.0, mode: debug, \\
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

Test 6 — 左臂:
ros2 service call /latte_imitation/replay_trajectory \\
  ivg_interfaces/srv/ReplayLatteTrajectory \\
  \"{episode_idx: 0, arm: left, speed_scale: 1.0, mode: debug, \\
    start_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \\
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\"

=== 参数速查 (MoveIt2 新管线) ===
episode_idx  : 0~39 轨迹编号
arm          : right=拉花臂(~1.5m)  left=持杯臂(~0.3m)
speed_scale  : 1.0=20秒  2.0=10秒  0.5=40秒
mode         : debug=只发布PoseStamped/Path  action=MoveIt2 CartesianPath→Execute
start_pose   : 零=自动从TF获取当前EE位姿作起点  非零=手动指定起点(杯子位姿)

=== 废弃字段 (保留在接口但不再生效) ===
pos_only     : no-op (MoveIt2 始终全6-DOF IK)
collision_check : no-op (MoveIt2 内置 avoid_collisions)
ik_success_count 语义: fraction * num_frames (规划成功等效帧数)
collision_count 始终为0 (MoveIt2内部处理)
CMDS
echo '按 Ctrl+C 或直接关终端退出喵~'"

# ═══════════════════════════════════════════════════════════════
# 完成
# ═══════════════════════════════════════════════════════════════

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  latte_imitation 测试环境启动完成${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}4 个 terminator 标签页:${NC}"
echo -e "  ${CYAN}[Sim]${NC}       仿真环境 (move_group + ros2_control + RViz)"
echo -e "  ${CYAN}[Latte Svc]${NC}  latte_imitation 服务节点 (MoveIt2 新管线)"
echo -e "  ${CYAN}[Test]${NC}       交互式测试菜单 (输入编号 1~7)"
echo -e "  ${CYAN}[Cmd Ref]${NC}    手动命令速查 (可直接复制执行)"
echo ""
echo -e "${YELLOW}提示: 在 terminator 各标签页中查看日志喵~${NC}"
echo -e "${YELLOW}      Ctrl+C 或直接关闭 terminator → 自动清理所有进程喵~${NC}"

# 阻塞主进程, 等待用户 Ctrl+C
wait
