#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ROS2 官方示例启动脚本 — foxglove_bridge 收发测试用
#
# 启动: bash start_demo_nodes.sh
# 停止: bash start_demo_nodes.sh stop
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

ROS2_SETUP="/opt/ros/humble/setup.bash"
PID_DIR="/tmp/ros2_demo_pids"

# ── 清理 ──
cleanup() {
    echo -e "${YELLOW}正在停止所有示例节点...${NC}"
    if [ -d "$PID_DIR" ]; then
        for f in "$PID_DIR"/*.pid; do
            [ -f "$f" ] || continue
            local pid; pid=$(cat "$f")
            local name; name=$(basename "$f" .pid)
            if kill -0 "$pid" 2>/dev/null; then
                kill -TERM "$pid" 2>/dev/null || true
                echo "  ✓ 已停止 ${name} (pid=${pid})"
            fi
        done
        rm -rf "$PID_DIR"
    fi
    echo -e "${GREEN}  ✓ 清理完成${NC}"
}

if [ "${1:-}" = "stop" ]; then
    cleanup
    exit 0
fi

# ── 启动 ──
trap cleanup INT TERM HUP
mkdir -p "$PID_DIR"

if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${RED}未找到 ROS 2: ${ROS2_SETUP}${NC}"
    exit 1
fi
source "$ROS2_SETUP"

echo -e "${GREEN}═══════════════════════════════════════${NC}"
echo -e "${GREEN}  ROS2 官方示例启动${NC}"
echo -e "${GREEN}═══════════════════════════════════════${NC}"
echo ""

# 启动单个节点并记录 PID
run_node() {
    local name="$1"; shift
    echo -e "${BLUE}  → 启动 ${name}...${NC}"
    "$@" &
    local pid=$!
    echo "$pid" > "${PID_DIR}/${name}.pid"
    sleep 0.5
    if kill -0 "$pid" 2>/dev/null; then
        echo -e "${GREEN}    ✓ ${name} 已启动 (pid=${pid})${NC}"
    else
        echo -e "${RED}    ✗ ${name} 启动失败${NC}"
        return 1
    fi
}

# ── 1. 话题: talker + listener ──
echo -e "${GREEN}[1] 话题 (Topic) — talker / listener${NC}"
run_node "talker"   ros2 run demo_nodes_cpp talker
run_node "listener" ros2 run demo_nodes_cpp listener
echo ""

# ── 2. 服务: add_two_ints ──
echo -e "${GREEN}[2] 服务 (Service) — add_two_ints_server${NC}"
run_node "add_two_ints_server" ros2 run demo_nodes_cpp add_two_ints_server
echo ""

# ── 3. 动作: fibonacci ──
echo -e "${GREEN}[3] 动作 (Action) — fibonacci_action_server${NC}"
run_node "fibonacci_action_server" ros2 run action_tutorials_py fibonacci_action_server
echo ""

# ── 4. 参数: parameter_blackboard ──
echo -e "${GREEN}[4] 参数 (Parameter) — parameter_blackboard${NC}"
run_node "parameter_blackboard" ros2 run demo_nodes_cpp parameter_blackboard
echo ""

# ── 5. foxglove_bridge ──
echo -e "${GREEN}[5] foxglove_bridge (端口 8765)${NC}"
run_node "foxglove_bridge" ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0
echo ""

# ── 状态 ──
echo -e "${GREEN}═══════════════════════════════════════${NC}"
echo -e "${GREEN}  所有节点已启动 (共 6 个)${NC}"
echo -e "${GREEN}═══════════════════════════════════════${NC}"
echo ""
echo -e "  Pid 目录: ${PID_DIR}"
echo ""
echo -e "  后续操作:"
echo -e "    ${BLUE}ros2 topic list${NC}              # 查看话题"
echo -e "    ${BLUE}ros2 topic echo /chatter${NC}      # 监听 talker"
echo -e "    ${BLUE}ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 3, b: 4}\"${NC}"
echo -e "    ${BLUE}npm run send${NC}                  # 运行 foxglove_bridge 收发测试"
echo ""
echo -e "  ${YELLOW}停止: bash $0 stop${NC}"
echo ""

# 保持前台运行, 等待用户 Ctrl+C
echo -e "${YELLOW}按 Ctrl+C 停止所有节点...${NC}"
wait
