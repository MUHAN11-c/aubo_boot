#!/bin/bash
# e2e 分析总跑器：统计 + 全部图件（数字打 stdout，图进 /tmp/e2e_live/report）
# 2026-09-21 重录轮定版：E2E_FRAME 取两档 jsonl 帧号内的共同 img/npz 交集帧（≡1 mod 30）
set -e
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
PY=./aubo_py3.12/bin/python
S=$(cd "$(dirname "$0")" && pwd)
export E2E_ROOT=/tmp/e2e_live
export E2E_FRAME=91
$PY $S/analyze.py
echo '=== deep_analysis ==='
$PY $S/deep_analysis.py
echo '=== fig_a rebuild ==='
$PY $S/make_fig_annot.py
echo '=== fig_e (from window videos) ==='
$PY $S/make_fig_e.py
echo '=== fig_f (stitch) ==='
bash $S/make_fig_f.sh
