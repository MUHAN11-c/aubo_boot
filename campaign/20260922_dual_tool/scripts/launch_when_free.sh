#!/usr/bin/env bash
# 栈空抢启器：进程表一空立即 launch（消除 check/launch 竞态窗口）。
# 用法: launch_when_free.sh <tool_profile> [extra launch args...]
# 每 20s 巡检；preflight 拒绝（又被人抢）则继续等；最长 2h。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
PROFILE="${1:?用法: launch_when_free.sh <tool_profile> [extra...]}"
shift || true
export PYTHONDONTWRITEBYTECODE=1
for i in $(seq 1 360); do
  clean=$(python3 -c "
import sys
sys.path.insert(0, 'src/peach_bringup')
from peach_bringup.preflight import running_stack_pids
print(0 if not running_stack_pids() else 1)" 2>/dev/null || echo 1)
  if [ "$clean" = "0" ]; then
    echo "[cycle $i] 栈空，立即启动"
    exec campaign/20260922_dual_tool/scripts/launch_stack.sh "$PROFILE" "$@"
  fi
  sleep 20
done
echo "2h 内未等到空位" >&2
exit 1
