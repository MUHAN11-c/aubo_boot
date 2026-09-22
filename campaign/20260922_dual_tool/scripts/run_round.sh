#!/usr/bin/env bash
# 战役单轮编排 wrapper（薄封装，不造第二套）：
#   run_round.sh <request_id> [--survey|--pregrasp|--full] [--level std|all] [--rviz-dur N]
# 前清 pgrep → 提示 launch（栈由操作员终端起，避免后台孤儿）→ 起rviz录屏 →
# 发意图 → 轮毕提示停栈 → collect_round。
# MUST：未授权不真机运动；tool 恒关；每轮前后 pgrep 复核。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
RID="${1:?用法: run_round.sh <request_id> [--survey|--pregrasp|--full] [--level std|all] [--rviz-dur N]}"
shift
KIND="--pregrasp"; LEVEL="std"; RVIZ_DUR=0
while [ $# -gt 0 ]; do
  case "$1" in
    --survey|--pregrasp|--full) KIND="$1"; shift;;
    --level) LEVEL="$2"; shift;;
    --rviz-dur) RVIZ_DUR="$2"; shift;;
    *) echo "未知参数: $1" >&2; exit 2;;
  esac
done

echo "== [1/5] 前清残留 =="
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|collect|probe|ffmpeg' && {
  echo "发现残留进程，先清理再跑（见 AGENTS.md 第 9 章）" >&2; exit 1; } || echo "无残留"

echo "== [2/5] 确认栈在跑 =="
ros2 lifecycle get /peach_arm 2>/dev/null | grep -q active || {
  echo "peach_arm 非 active。先在独立终端起栈：" >&2
  echo "  ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \\" >&2
  echo "    camera_enabled:=true camera_frontend:=stereo skip_reconstruction:=true \\" >&2
  echo "    tool_profile:=hollow_cylinder_v1 record_level:=$LEVEL autostart:=false" >&2
  exit 1; }
echo "栈就绪（tool_profile 须与 launch 一致，sim 脚本会 fail-fast 核对）"

echo "== [3/5] 起 rviz 录屏（${RVIZ_DUR}s，0=由操作员 Ctrl+C 停）=="
if [ "$RVIZ_DUR" -gt 0 ]; then
  scripts/record_rviz_harvest.sh "$RID" "$RVIZ_DUR" &
  RVIZ_PID=$!
else
  scripts/record_rviz_harvest.sh "$RID" &
  RVIZ_PID=$!
fi
sleep 2

echo "== [4/5] 发意图 ($KIND) =="
case "$KIND" in
  --survey)   INTENT=2; EXTRA="";;
  --pregrasp) INTENT=0; EXTRA="view_policy: 0";;
  --full)     INTENT=0; EXTRA="view_policy: 0";;
esac
ros2 action send_goal /peach_supervisor/run_harvest peach_interfaces/action/RunHarvest \
  "{request_id: '$RID', scene_key: 'lab', profile_id: 'default', intent: $INTENT$([ -n "$EXTRA" ] && echo ", $EXTRA")}" || true

echo "== [5/5] 轮毕收尾 =="
[ -n "${RVIZ_PID:-}" ] && kill "$RVIZ_PID" 2>/dev/null || true
echo "1) 操作员 Ctrl+C 停 launch（停栈）"
echo "2) pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|bag record|ffmpeg'  复核无残留"
echo "3) python3 scripts/collect_round.py $RID"
echo "4) ros2 run peach_observability peach_bag_report runs/session_*/bag"
echo "5) python3 campaign/20260922_dual_tool/scripts/per_round_summary.py $RID"
echo "6) 更新 campaign/20260922_dual_tool/README.md 轮次表 + bags.md"
echo "7) python3 scripts/purge_analyzed_bags.py runs/session_<该轮>"
