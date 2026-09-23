#!/usr/bin/env bash
# S6 五轮完整抓取批链（核心门）：e2e_full_unrefined_ × ≥5 轮。
# 前提：真相机+mock 栈已起（camera_enabled:=true skip_reconstruction:=true
# tool_profile:=hollow_cylinder_v1），已 Survey 对拍照位自洽，
# FULL 干跑须运行期放开 execute_pregrasp_only=false。
# 用法: run_5rounds.sh <轮数默认5> [每轮rviz录屏秒数默认600]
# 每轮: 发 RunHarvest(FULL) → 等终态 → 停录 → collect+summary → 登记提示。
# 场景布置（每轮不同：基线/遮挡/多目标/光照/异常）由操作员在轮间人工完成。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
OUT="campaign/20260922_dual_tool"
mkdir -p "$OUT/analysis"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROUNDS="${1:-5}"
RVIZ_DUR="${2:-600}"
TS="$(date +%Y%m%d)"
for i in $(seq 1 "$ROUNDS"); do
  RID="e2e_full_unrefined_${TS}T$(date +%H%M%S)_r${i}"
  echo "=== 第 ${i}/${ROUNDS} 轮: ${RID}（先布置好场景再回车）==="
  read -r || true  # EOF（无人值守）不中断，布置确认靠操作员
  # FULL 干跑：调度运行期放开（真机 KEEP 门在操作台/授权，mock 自由）
  ros2 param set /peach_supervisor execute_pregrasp_only false
  # rviz 录屏（后台；录不到窗口时降级只告警）
  scripts/record_rviz_harvest.sh "$RID" "$RVIZ_DUR" \
    > "$OUT/analysis/${RID}_rviz.log" 2>&1 &
  RVIZ_PID=$!
  # FULL hang 防线：3600s 无终态即 INT（执行超时护栏应在臂侧先兜住）
  timeout -s INT 3600 ros2 action send_goal /peach_supervisor/run_harvest \
    peach_interfaces/action/RunHarvest \
    "{request_id: '$RID', scene_key: 'lab', profile_id: 'default', intent: 0, view_policy: 0}" \
    || true
  python3 scripts/collect_round.py "$RID" || true
  python3 "$OUT/scripts/per_round_summary.py" "$RID" || true
  # 杀录屏：脚本与 ffmpeg 子进程一起清（杀 launch 不带走子进程的坑）
  pkill -P "$RVIZ_PID" 2>/dev/null || true
  kill "$RVIZ_PID" 2>/dev/null || true
  echo "--- 轮毕 ${RID}：确认停栈需求/登记 README；下一轮前 pgrep 复核 ---"
done
echo "=== 5 轮链完成：逐轮 purge 见 README runbook ==="
