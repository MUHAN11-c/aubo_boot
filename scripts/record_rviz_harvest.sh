#!/bin/bash
# harvest RViz 窗口录像：x11grab 当前 DISPLAY 上 MoveIt RViz 窗。
# x11grab 录的是屏幕像素：窗被挡住就会进别的窗口。录前把 RViz 提到前台。
# 用法: scripts/record_rviz_harvest.sh <request_id> [duration_s]
# 自检：窗存在；相机开时 /peach/perception/debug_image 至少一帧才开录。
# 产物: runs/<request_id>/rvizwin.mp4
set -euo pipefail
RID=${1:?request_id}
DUR=${2:-120}
export DISPLAY="${DISPLAY:-:0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/runs/$RID"
mkdir -p "$OUT"

# 结束即复核：本脚本只拉 ffmpeg，退出时杀子进程。
cleanup() {
  if [[ -n "${FPID:-}" ]]; then
    kill -INT "$FPID" 2>/dev/null || true
    sleep 1
    kill -9 "$FPID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

pick_rviz_id() {
  local tree
  tree=$(xwininfo -root -tree 2>/dev/null || true)
  echo "$tree" | grep -i 'moveit.rviz' | head -1 \
    | grep -oE '0x[0-9a-f]+' | head -1 || true
}

RW=$(pick_rviz_id)
if [[ -z "$RW" ]]; then
  RW=$(xwininfo -root -tree 2>/dev/null | grep -i ' - RViz' | head -1 \
    | grep -oE '0x[0-9a-f]+' | head -1 || true)
fi
if [[ -z "$RW" ]]; then
  echo "[rviz-rec] FATAL: 未找到 RViz 窗（harvest_system moveit_enabled:=true？）"
  exit 1
fi

# 提到前台，避免 x11grab 录到叠在 RViz 上的 IDE/浏览器。
if command -v wmctrl >/dev/null 2>&1; then
  wmctrl -i -a "$RW" 2>/dev/null || true
elif command -v xdotool >/dev/null 2>&1; then
  xdotool windowactivate "$RW" 2>/dev/null || true
fi
sleep 0.4

read -r RX RY RWd RHt <<< "$(xwininfo -id "$RW" \
  | grep -E 'Absolute upper-left X|Absolute upper-left Y|^  Width|^  Height' \
  | grep -oE '[0-9]+' | tr '\n' ' ')"
echo "[rviz-rec] rviz ${RWd}x${RHt}+${RX}+${RY} dur=${DUR}s -> $OUT/rvizwin.mp4"
echo "[rviz-rec] 请保持该窗不被挡住（x11grab 录屏幕像素）"

if command -v ros2 >/dev/null 2>&1; then
  if timeout 3 ros2 topic echo --once /peach/perception/debug_image >/dev/null 2>&1; then
    echo '[rviz-rec] debug_image 有帧'
  else
    echo '[rviz-rec] WARN: debug_image 无帧（无相机网格跑仍可录臂/markers）'
  fi
fi

ffmpeg -y -loglevel error -f x11grab -framerate 15 \
  -video_size "${RWd}x${RHt}" -i "${DISPLAY}.0+${RX},${RY}" \
  -t "$DUR" "$OUT/rvizwin.mp4" &
FPID=$!
wait "$FPID" || true
FPID=""
echo "[rviz-rec] done $OUT/rvizwin.mp4"
ls -lh "$OUT/rvizwin.mp4"
