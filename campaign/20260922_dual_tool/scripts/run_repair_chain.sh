#!/usr/bin/env bash
# 补验链：M1 网格（deny 决策层校验）→ M3 FULL 29 例（超时 ACK 修复后；rand_03 剔除：
# 该例周期>300s，取消触发 move_group TEM stop 风暴楔死——MoveIt 缺陷 P3 上游追）→ M4 algorithm30。
# M2（27/30）已有干净结果不重跑。单 M 容错同 run_injection_chain。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
OUT="campaign/20260922_dual_tool/analysis/injection"
mkdir -p "$OUT"
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-61}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "=== M1r grid $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --grid --mode full \
  --tool-profile hollow_cylinder_v1 --velocity 1.0 2>&1 || echo "[M1r exit=$?]"; } | tee "$OUT/m1_grid.log"
echo "=== M3r full30 $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
  --mode full --tool-profile hollow_cylinder_v1 --velocity 1.0 \
  --pick rand_00 rand_01 rand_02 rand_04 rand_05 rand_06 rand_07 rand_08 rand_09 rand_10 rand_11 rand_12 rand_13 rand_14 rand_15 rand_16 rand_17 rand_18 rand_19 rand_20 rand_21 rand_22 rand_23 rand_24 rand_25 rand_26 rand_27 rand_28 rand_29 2>&1 || echo "[M3r exit=$?]"; } | tee "$OUT/m3_full30.log"
echo "=== M4r algorithm30 $(date +%T) ==="
{ python3 scripts/sim_field_targets.py --random 30 --seed 20260922 \
  --envelope algorithm --mode full --tool-profile hollow_cylinder_v1 \
  --velocity 1.0 2>&1 || echo "[M4r exit=$?]"; } | tee "$OUT/m4_algo30.log"
echo "=== ALL-DONE $(date +%T) ==="
