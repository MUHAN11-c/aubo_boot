#!/usr/bin/env bash
# 网格单轮：约束网格 --mode full（须先在另一终端 launch_stack.sh <同档> 起栈）
# 用法: run_grid.sh <tool_profile> [额外 sim_field_targets 参数...]
# 输出：sim jsonl 落 runs/（脚本自身行为），控制台日志 tee 到 campaign analysis/<日期>/。
# 注意不能 set -u：ROS setup.bash 依赖未绑定变量探测（AMENT_TRACE_SETUP_FILES）。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
PROFILE="${1:?用法: run_grid.sh <tool_profile> [extra sim args...]}"
shift
export ROS_DOMAIN_ID="${CAMPAIGN_DOMAIN:-62}"
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source install/setup.bash
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="campaign/20260928_bite_shear/analysis/${STAMP}_${PROFILE}"
mkdir -p "$OUT_DIR"
# ensure_profile_match 会在启动时比对 /peach_arm tool.profile_id，错配即拒跑（须整栈同档重启）
python3 scripts/sim_field_targets.py \
  --grid --mode full --velocity 1.0 --tool-profile "$PROFILE" "$@" 2>&1 \
  | tee "$OUT_DIR/grid_console.log"
