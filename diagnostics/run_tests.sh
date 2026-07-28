#!/usr/bin/env bash
# diagnostics 总测试脚本：构建探针 → 按序运行（全部零运动）→ 生成可视化报告。
#
# 用法:
#   ./diagnostics/run_tests.sh [host] [quick|full]
#     host   控制器 IP，默认 169.254.10.98
#     quick  短测试（约 30s 总计）：10Hz 轮询 10s + 推送 10s + TCP2CAN 10s
#     full   完整测试（默认，约 90s）：追加 2ms 极限轮询，时长翻倍
#
# 输出（每次运行覆盖最新）:
#   diagnostics/results/*.csv   逐样本原始数据
#   diagnostics/results/*.png   可视化曲线
#   diagnostics/results/abi_probe_latest.txt
#
# 注意：tcp2can 探针会进入 TCP2CAN 独占模式（测试结束后退出并恢复），
# 期间示教器运动暂停；设定点=当前位置，机械臂不会运动。

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIAG_DIR="$WS_DIR/diagnostics"
BUILD_DIR="$DIAG_DIR/build"
RESULTS_DIR="$DIAG_DIR/results"
PY="$WS_DIR/aubo_py3.12/bin/python"
# SDK 按进程 CWD 读取 ./config/auborobot.conf，探针须以 install share 目录为 CWD
SDK_CWD="$WS_DIR/install/aubo_e5_hardware/share/aubo_e5_hardware"

HOST="${1:-169.254.10.98}"
MODE="${2:-full}"

if [[ ! -x "$PY" ]]; then
  echo "error: 项目 Python 环境缺失: $PY" >&2
  exit 1
fi
if [[ ! -d "$SDK_CWD/config" ]]; then
  echo "error: SDK 配置目录缺失: $SDK_CWD/config（先 colcon build aubo_e5_hardware）" >&2
  exit 1
fi

echo "==> [1/4] 构建探针"
cmake -S "$DIAG_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$BUILD_DIR" -j"$(nproc)"

mkdir -p "$RESULTS_DIR"

echo "==> [2/4] ABI 探针（不连机器人）"
"$BUILD_DIR/aubo_sdk_abi_probe" | tee "$RESULTS_DIR/abi_probe_latest.txt"

run_probe() {
  (cd "$SDK_CWD" && "$@")
}

if [[ "$MODE" == "quick" ]]; then
  RUNTIME_SAMPLES=100; RUNTIME_PERIOD=100
  PUSH_SEC=10
  TCP2CAN_SEC=10
else
  RUNTIME_SAMPLES=300; RUNTIME_PERIOD=100
  PUSH_SEC=30
  TCP2CAN_SEC=30
fi

echo "==> [3/4] 状态通道测试（零运动）"
echo "--- 轮询延迟 ${RUNTIME_SAMPLES} 样本 @ ${RUNTIME_PERIOD}ms 周期"
run_probe "$BUILD_DIR/aubo_sdk_runtime_probe" "$HOST" \
  "$RUNTIME_SAMPLES" "$RUNTIME_PERIOD" "$RESULTS_DIR/runtime_probe_latest.csv" \
  | grep -v -e log4cplus -e "sdk log"

if [[ "$MODE" == "full" ]]; then
  echo "--- 极限轮询 2000 样本 @ 2ms 周期（测节流/阻塞）"
  run_probe "$BUILD_DIR/aubo_sdk_runtime_probe" "$HOST" \
    2000 2 "$RESULTS_DIR/runtime_probe_fast_latest.csv" \
    | grep -v -e log4cplus -e "sdk log"
fi

echo "--- 推送频率 ${PUSH_SEC}s"
run_probe "$BUILD_DIR/aubo_sdk_push_probe" "$HOST" \
  "$PUSH_SEC" "$RESULTS_DIR/push_probe_latest.csv" \
  | grep -v -e log4cplus -e "sdk log"

echo "==> [4/4] TCP2CAN 指令通道测试 ${TCP2CAN_SEC}s（零运动，独占模式）"
run_probe "$BUILD_DIR/aubo_sdk_tcp2can_probe" "$HOST" \
  "$TCP2CAN_SEC" 8 1 "$RESULTS_DIR/tcp2can_probe_latest.csv" \
  | grep -v -e log4cplus -e "sdk log"

echo "==> 生成可视化报告"
"$PY" "$DIAG_DIR/plot_results.py" --results-dir "$RESULTS_DIR"

echo "完成。数据与曲线: $RESULTS_DIR"
