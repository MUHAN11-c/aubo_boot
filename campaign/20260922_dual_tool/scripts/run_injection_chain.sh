#!/usr/bin/env bash
# 注入矩阵全链：M1 网格 → M2 随机30 PREGRASP → M3 随机30 FULL → M4 algorithm30。
# 前提：hollow 栈已 active（域 61）。日志 tee 到 analysis/injection/。
# 单 M 失配（sim 非零退出，如 grid 19/20）不拖垮链——门判定归 m1_m5_report。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
OUT="campaign/20260922_dual_tool/analysis/injection"
mkdir -p "$OUT"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "=== M1 grid $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --grid --mode full \
  --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 || echo "[M1 exit=$?]"; } | tee "$OUT/m1_grid.log"
echo "=== M2 pregrasp30 $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
  --mode pregrasp --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 || echo "[M2 exit=$?]"; } | tee "$OUT/m2_pregrasp30.log"
echo "=== M3 full30 $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
  --mode full --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 || echo "[M3 exit=$?]"; } | tee "$OUT/m3_full30.log"
echo "=== M4 algorithm30 $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
  --envelope algorithm --mode full --tool-profile hollow_cylinder_v1 \
  --velocity 1.0 2>&1 || echo "[M4 exit=$?]"; } | tee "$OUT/m4_algo30.log"
echo "=== ALL-DONE $(date +%T) ==="
