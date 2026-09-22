#!/usr/bin/env bash
# M1–M5 注入矩阵一键执行（栈已起 + hollow 档后运行；域 61 由 launch_stack.sh 定）
#   run_m1_m5.sh m1                      # 网格 20 例 FULL
#   run_m1_m5.sh m2 | m3 | m4            # random 30 pregrasp / full / algorithm
#   run_m1_m5.sh m3pick rand_07 rand_39  # M5 失败例重测（--pick）
# 结果 jsonl 落 runs/，日志落 campaign/20260922_dual_tool/analysis/injection/
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
OUT="campaign/20260922_dual_tool/analysis/injection"
mkdir -p "$OUT"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
# 自含环境：脚本内 source（不能 set -u，ROS setup 依赖未绑定变量探测）
source /opt/ros/jazzy/setup.bash
source install/setup.bash
MODE="${1:?用法: run_m1_m5.sh m1|m2|m3|m4|m3pick [ids...]}"
shift || true
case "$MODE" in
  m1) exec python3 scripts/sim_field_targets.py --grid --mode full \
        --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 | tee "$OUT/m1_grid.log" ;;
  m2) exec python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
        --mode pregrasp --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 | tee "$OUT/m2_pregrasp30.log" ;;
  m3) exec python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
        --mode full --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 | tee "$OUT/m3_full30.log" ;;
  m4) exec python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
        --envelope algorithm --mode full --tool-profile hollow_cylinder_v1 \
        --velocity 1.0 2>&1 | tee "$OUT/m4_algo30.log" ;;
  m3pick) exec python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
        --mode full --tool-profile hollow_cylinder_v1 --velocity 1.0 \
        --pick "$@" 2>&1 | tee "$OUT/m5_pick_$(date +%H%M%S).log" ;;
  *) echo "未知模式 $MODE" >&2; exit 2;;
esac
