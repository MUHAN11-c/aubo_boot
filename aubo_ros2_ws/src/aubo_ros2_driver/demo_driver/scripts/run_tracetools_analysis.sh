#!/bin/bash
#
# Tracetools 分析工作流脚本
#
# 用法:
#   ./run_tracetools_analysis.sh status     # 检查 tracetools 是否启用
#   ./run_tracetools_analysis.sh trace     # 启动追踪（需手动按 Enter 开始/停止）
#   ./run_tracetools_analysis.sh view NAME # 查看已有追踪
#   ./run_tracetools_analysis.sh help      # 显示完整流程
#
# 前置: source aubo_ros2_ws/install/setup.bash
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
TRACE_DIR="${HOME}/.ros/tracing"

cmd="${1:-help}"
session_name="${2:-aubo_grasp}"

case "$cmd" in
  status)
    echo "=== 检查 tracetools 状态 ==="
    if ! command -v ros2 &>/dev/null; then
      echo "错误: ros2 未找到，请先 source workspace"
      echo "  source $WS_ROOT/install/setup.bash"
      exit 1
    fi
    ros2 run tracetools status || true
    ;;
  trace)
    echo "=== 启动追踪会话: $session_name ==="
    echo "按 Enter 开始记录，再次按 Enter 停止"
    echo "追踪数据将保存到: $TRACE_DIR/$session_name"
    echo ""
    ros2 trace --session-name "$session_name" --list
    ;;
  view)
    if [[ -z "$session_name" || "$session_name" == "trace" || "$session_name" == "help" ]]; then
      echo "用法: $0 view <session_name>"
      echo "示例: $0 view aubo_grasp"
      exit 1
    fi
    path="$TRACE_DIR/$session_name"
    if [[ ! -d "$path" ]]; then
      echo "追踪目录不存在: $path"
      echo "请先运行: $0 trace  # 按 Enter 开始记录，执行系统后再按 Enter 停止"
      exit 1
    fi
    echo "=== 查看追踪: $path ==="
    # LTTng 追踪结构: path/ust/uid/UID/64-bit/ 含 metadata 与 stream 文件
    # babeltrace2 需指向物理 CTF 目录，尝试根目录与 ust 子目录
    ctf_dirs=()
    if [[ -f "$path/ust/uid/1000/64-bit/metadata" ]]; then
      ctf_dirs=("$path/ust/uid/1000/64-bit")
    fi
    # 若未找到，尝试根目录（部分 LTTng 版本结构不同）
    if [[ ${#ctf_dirs[@]} -eq 0 && -f "$path/metadata" ]]; then
      ctf_dirs=("$path")
    fi
    if [[ ${#ctf_dirs[@]} -eq 0 ]]; then
      first_ctf=$(find "$path" -name metadata -type f 2>/dev/null | head -1)
      if [[ -n "$first_ctf" ]]; then
        ctf_dirs=("$(dirname "$first_ctf")")
      fi
    fi

    if command -v babeltrace2 &>/dev/null; then
      if [[ ${#ctf_dirs[@]} -gt 0 ]]; then
        out=$(timeout 15 babeltrace2 "${ctf_dirs[@]}" 2>&1 | head -300)
      else
        out=$(timeout 15 babeltrace2 "$path" 2>&1 | head -300)
      fi
    elif command -v babeltrace &>/dev/null; then
      out=$(timeout 15 babeltrace "$path" 2>&1 | head -300)
    else
      echo "请安装 babeltrace2: sudo apt install babeltrace2"
      exit 1
    fi

    if [[ -z "${out//[[:space:]]/}" ]]; then
      echo "(追踪输出为空)"
      echo ""
      echo "可能原因:"
      echo "  1. ROS2 核心包（rclcpp/rcl）来自 apt，未带 LTTng 插桩"
      echo "     需从源码构建完整 ROS2 并启用 tracing 才能记录回调事件"
      echo "  2. 追踪期间无 ROS2 节点运行"
      echo "  3. 追踪会话刚创建即停止，未捕获到事件"
      echo ""
      echo "建议: 重新运行追踪，确保在按 Enter 开始后、停止前有节点在运行"
    else
      echo "$out" | less
    fi
    ;;
  help|*)
    echo "Tracetools 分析工作流"
    echo ""
    echo "用法: $0 <command> [session_name]"
    echo ""
    echo "命令:"
    echo "  status           检查 tracetools 是否启用 (Tracing enabled)"
    echo "  trace [NAME]     启动交互式追踪，默认会话名 aubo_grasp"
    echo "  view NAME        用 babeltrace 查看已有追踪"
    echo "  help             显示本帮助"
    echo ""
    echo "完整流程:"
    echo "  1. source $WS_ROOT/install/setup.bash"
    echo "  2. $0 status          # 确认 Tracing enabled"
    echo "  3. 终端1: $0 trace    # 按 Enter 开始"
    echo "  4. 终端2: ros2 launch aubo_moveit_config aubo_moveit_pure_ros2.launch.py"
    echo "  5. 终端3: ros2 launch graspnet_ros2 graspnet_demo_points_with_tf.launch.py"
    echo "  6. 终端4: ros2 run demo_driver publish_grasps_client_worker_node"
    echo "  7. 执行 1-2 个抓取周期"
    echo "  8. 终端1: 按 Enter 停止追踪"
    echo "  9. $0 view aubo_grasp # 查看追踪"
    echo ""
    echo "分析回调耗时: 使用 tracetools_analysis 的 callback_duration.ipynb"
    echo "  path = '$TRACE_DIR/aubo_grasp'"
    ;;
esac
