#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# start_ivg_demo.sh — IVG 演示栈一键启动（2026-09-30 自 aubo_boot
# start_aubo_new_driver.sh 移植重构：无 terminator 依赖，后台进程 +
# trap 全清 + 结束 pgrep 复核，符合本仓 MUST「测完即停」）。
#
# 用法:
#   scripts/start_ivg_demo.sh                 # mock 全栈（默认）
#   scripts/start_ivg_demo.sh real            # 真机栈（=操作员授权，须示教器已上电）
#   scripts/start_ivg_demo.sh mock --no-web   # 不起 Web 控制台
#
# 组件: bringup(mock/real) + moveit + 演示服务(move/grasp_trigger/system_monitor)
#       + tool_changer + latte_backend + aubo_mode + Web(rosbridge+网关8095
#       +foxglove+web_video)。peach harvest_system 不在此栈。
# ═══════════════════════════════════════════════════════════════════
set -u

MODE="${1:-mock}"
shift || true
NO_WEB=false
for a in "$@"; do
  [ "$a" = "--no-web" ] && NO_WEB=true
done

if [ "$MODE" != "mock" ] && [ "$MODE" != "real" ]; then
  echo "用法: $0 [mock|real] [--no-web]" >&2
  exit 2
fi

WS=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

# IVG/peach 隔离（用户裁定 2026-09-30）：演示栈默认跑独立 DDS 域（96），
# 与 peach 现役域互不可见、互不干扰；其他终端执行 ros2 命令前先
# export ROS_DOMAIN_ID=96（环境变量 IVG_DEMO_DOMAIN_ID 可覆盖）。
export ROS_DOMAIN_ID="${IVG_DEMO_DOMAIN_ID:-96}"

# 启动前预检（MUST）：清残留（cursorsandbox 外壳命令串内嵌同类文本，已知假阳性须排除）。
# rviz2：moveit.launch.py 无条件随栈起一个 rviz2——运行期本栈恰有一个属正常，
# 预检遇到任何存量 rviz2 都拦（避免叠窗口），停栈清扫也会带走自己的那个。
echo "── 预检: 检查残留进程 ──"
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|rosbridge_websocket|foxglove_bridge|web_video_server|rviz2' | grep -v cursorsandbox && {
  echo "!! 发现残留进程（见上）。请先按 docs/testing.md 清理再启动。" >&2
  exit 1
}

PIDS=()
LOGDIR=$(mktemp -d /tmp/ivg_demo_XXXX)
start() {  # start <name> <cmd...>
  local name="$1"; shift
  echo "── 启动 $name (日志 $LOGDIR/$name.log)"
  "$@" >"$LOGDIR/$name.log" 2>&1 &
  PIDS+=($!)
}
cleanup() {
  echo ""
  echo "── 停栈: 结束 ${#PIDS[@]} 个进程 ──"
  for pid in "${PIDS[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    kill -9 "$pid" 2>/dev/null
  done
  # MUST：复核无残留（launch 的子进程随父退出；仍存活的按 PID 清）
  sleep 1
  local rest
  rest=$(pgrep -af 'ros2 launch ivg_demo|ros2 launch latte|ros2 launch tool_changer|ros2 launch aubo_ros2_web_dashboard|ros2 launch aubo_e5_bringup|rosbridge_websocket|foxglove_bridge|web_video_server|grasp_trigger_node|move_service_node|latte_workflow_node|latte_io_node|gripper_swap_worker|scene_attach_worker|system_monitor_node|extrinsics_publisher|rviz2' || true)
  if [ -n "$rest" ]; then
    echo "!! 仍有残留，按 PID 清理:" >&2
    echo "$rest" >&2
    echo "$rest" | awk '{print $1}' | xargs -r kill -TERM 2>/dev/null
    sleep 2
    echo "$rest" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
  fi
  echo "── 已清空（日志保留在 $LOGDIR）──"
}
trap cleanup INT TERM

WAIT="$WS/scripts/wait_for_service.sh"

# ── 1. 机械臂栈 + MoveIt（一体：bringup 默认 moveit_enabled:=true，
#        勿另起第二份 moveit——双 move_group 参数分裂会让 MGI/Pilz 失败）──
if [ "$MODE" = "real" ]; then
  echo "╔══════════════════════════════════════════╗"
  echo "║ 真机模式：操作员起本栈=授权。             ║"
  echo "║ 急停手须能摸到；工作空间无人。            ║"
  echo "╚══════════════════════════════════════════╝"
  start bringup ros2 launch aubo_e5_bringup bringup.launch.py \
    hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
else
  start bringup ros2 launch aubo_e5_bringup bringup.launch.py \
    hardware_mode:=mock camera_enabled:=false
fi
$WAIT topic /joint_states 40 || echo "[WARN] /joint_states 40s 未就绪，继续"
$WAIT node /move_group 90 || echo "[WARN] move_group 未就绪，演示服务将自行重试"

# ── 2. 演示服务 + 工具快换 + 拉花 ─────────────────────────────
IO_SIM=$([ "$MODE" = "mock" ] && echo true || echo false)
start demo_services ros2 launch ivg_demo_services demo_services.launch.py \
  io_simulated:="$IO_SIM"
start tool_changer ros2 launch tool_changer gripper_swap_worker.launch.py \
  io_simulated:="$IO_SIM"
start latte ros2 launch latte_backend latte_workflow.launch.py \
  io_simulated:="$IO_SIM"
start aubo_mode ros2 run ivg_demo_services aubo_mode.py --ros-args \
  -p aubo_driver_mode:="$([ "$MODE" = mock ] && echo simulation || echo real)"

# ── 3. Web 控制台（8095；foxglove/web_video 独立进程，同旧栈分工）──
if [ "$NO_WEB" = false ]; then
  start dashboard ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
  start foxglove ros2 launch foxglove_bridge foxglove_bridge_launch.xml
  start web_video ros2 run web_video_server web_video_server \
    --ros-args -p port:=8089
  $WAIT http http://127.0.0.1:8095/health 30 \
    && echo "✓ Web 控制台: http://127.0.0.1:8095/" \
    || echo "[WARN] 网关 30s 未就绪（rosbridge/uvicorn 依赖是否安装？）"
fi

echo ""
echo "════ IVG 演示栈已启动 ($MODE, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ════"
echo "  其他终端先: export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  单次抓取:  ros2 service call /execute_single_grasp ivg_interfaces/srv/ExecuteGraspPose '{object_id: demo, use_visual_estimation: false}'"
echo "  拉花工作流: ros2 service call /latte/run_workflow ivg_interfaces/srv/RunLatteWorkflow '{}'"
echo "  换刀:      ros2 run tool_changer test_tool_change.py gripper2"
echo "  Ctrl+C 停栈并清理"
echo "════════════════════════════════"

wait
