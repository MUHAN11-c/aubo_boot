#!/bin/bash
# 一条链录制会话：采集器(GUI composite 全尺寸窗) → 后起 rviz → 截图自检门 → 30s 双窗录制 → 收尾复核
# 用法: record_session.sh <tag> <collect_duration_s>   （相机前端须已在跑）
# 自检门不过则拒绝录制并退出非零——杜绝"录完才发现面板无标注"（09-21 末轮教训）。
TAG=${1:?tag}
DUR=${2:-75}
ROOT=/tmp/e2e_live
REPORT=$ROOT/report
export DISPLAY=:0
. /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77
SD=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$ROOT/$TAG" "$REPORT"

# 1) 清本轮旧产物
rm -f "$ROOT/$TAG/frames.jsonl" "$ROOT/${TAG}_frames.jsonl"
rm -rf "$ROOT/$TAG/img" "$ROOT/$TAG/npz"
rm -f "$REPORT/${TAG}_composite.mp4" "$REPORT/${TAG}_rvizwin.mp4" "$ROOT/${TAG}_gate.png"

# 2) 起采集器（E2E_GUI=1 开 1920x480 composite 窗）
E2E_GUI=1 nohup bash "$SD/run_collect.sh" "$TAG" "$DUR" \
  > "$ROOT/$TAG/frames.jsonl" 2> "$ROOT/${TAG}_collect.log" &
CPID=$!
echo "[rec] collect pid=$CPID tag=$TAG dur=$DUR"

# 3) 等管线出帧（jsonl >=5 行，最长 90s）
n=0
for i in $(seq 1 90); do
  n=$(cat "$ROOT/$TAG/frames.jsonl" 2>/dev/null | grep -c '^{' || true)
  [ "${n:-0}" -ge 5 ] && break
  sleep 1
done
if [ "${n:-0}" -lt 5 ]; then
  echo '[rec] FATAL: 采集器 90s 内未出帧'; tail -3 "$ROOT/${TAG}_collect.log"
  kill -INT "$CPID" 2>/dev/null; exit 1
fi
echo "[rec] 帧已流出（$n 行）"

# 4) 后起 rviz（TRANSIENT_LOCAL 闩锁首帧即带标注）
nohup bash "$SD/run_rviz_fixed.sh" > "$ROOT/${TAG}_rviz.log" 2>&1 &
RVPID=$!
sleep 5

# 5) 取双窗几何
CW=$(xwininfo -root -tree 2>/dev/null | grep -i 'e2e_demo' | head -1 | grep -oE '0x[0-9a-f]+' | head -1)
RW=$(xwininfo -root -tree 2>/dev/null | grep -i ' - RViz' | head -1 | grep -oE '0x[0-9a-f]+' | head -1)
if [ -z "$CW" ]; then
  echo '[rec] FATAL: 未找到 e2e_demo 窗（GUI 未开？查 collect.log）'
  kill -INT "$CPID" "$RVPID" 2>/dev/null; exit 1
fi
read -r CX CY CWd CHt <<< "$(xwininfo -id "$CW" | grep -E 'Absolute upper-left X|Absolute upper-left Y|^  Width|^  Height' | grep -oE '[0-9]+' | tr '\n' ' ')"
read -r RX RY RWd RHt <<< "$(xwininfo -id "${RW:-root}" | grep -E 'Absolute upper-left X|Absolute upper-left Y|^  Width|^  Height' | grep -oE '[0-9]+' | tr '\n' ' ')"
echo "[rec] composite ${CWd}x${CHt}+${CX}+${CY}; rviz ${RWd}x${RHt}+${RX}+${RY}"

# 6) 自检门：composite 窗抽帧验检测框/分割轮廓（最多 5 次）
ok=0
for i in 1 2 3 4 5; do
  sleep 2
  ffmpeg -y -loglevel error -f x11grab -video_size "${CWd}x${CHt}" -i ":0.0+${CX},${CY}" \
    -frames:v 1 -update 1 "$ROOT/${TAG}_gate.png" 2>/dev/null
  if python3 "$SD/check_frame.py" "$ROOT/${TAG}_gate.png"; then ok=1; break; fi
  echo "[rec] 自检未过（第 $i 次），重试…"
done
if [ "$ok" != 1 ]; then
  echo '[rec] FATAL: 自检门未过——composite 窗无检测框/掩膜，拒绝录制'
  kill -INT "$CPID" "$RVPID" 2>/dev/null; exit 1
fi
echo '[rec] 自检门通过：框/掩膜可见'

# 7) 30s 双窗并行录制
ffmpeg -y -loglevel error -f x11grab -framerate 15 -video_size "${CWd}x${CHt}" \
  -i ":0.0+${CX},${CY}" -t 30 "$REPORT/${TAG}_composite.mp4" &
F1=$!
if [ -n "$RW" ]; then
  ffmpeg -y -loglevel error -f x11grab -framerate 15 -video_size "${RWd}x${RHt}" \
    -i ":0.0+${RX},${RY}" -t 30 "$REPORT/${TAG}_rvizwin.mp4" &
  F2=$!
fi
wait "$F1" ${F2:-}
echo '[rec] 双窗 30s 录制完成'

# 8) 等采集自然退出 + 收尾复核（测试进程清理规则）
wait "$CPID" 2>/dev/null
kill -INT "$RVPID" 2>/dev/null
sleep 4
kill -INT "$RVPID" 2>/dev/null
sleep 2
frames=$(grep -c '^{' "$ROOT/$TAG/frames.jsonl" 2>/dev/null || echo 0)
echo "[rec] DONE tag=$TAG frames=$frames"
stat -c '[rec] %n %s' "$REPORT/${TAG}_composite.mp4" "$REPORT/${TAG}_rvizwin.mp4" 2>/dev/null
sleep 2
if pgrep -af 'collect\.py|rviz2 -d' | grep -v record_session; then
  echo '[rec] WARN: 有残留进程，按 PID 清杀后复验'
  exit 2
fi
echo '[rec] 无残留'
