#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# ROS2 就绪等待工具 — 以主动轮询替代固定 sleep
#
# 用法:
#   wait_for_service.sh service  <pattern> [timeout_sec]
#   wait_for_service.sh topic    <pattern> [timeout_sec]
#   wait_for_service.sh node     <pattern> [timeout_sec]
#   wait_for_service.sh http     <url>      [timeout_sec]
#
# 返回 0 = 就绪, 1 = 超时
# ═══════════════════════════════════════════════════════════════

set -e

TYPE="${1:-}"
PATTERN="${2:-}"
TIMEOUT="${3:-30}"

if [ -z "$TYPE" ] || [ -z "$PATTERN" ]; then
    echo "用法: $0 <service|topic|node|http> <pattern> [timeout_sec]" >&2
    exit 2
fi

START=$(date +%s)

while true; do
    case "$TYPE" in
        service)
            if ros2 service list 2>/dev/null | grep -q "$PATTERN"; then
                exit 0
            fi
            ;;
        topic)
            if ros2 topic list 2>/dev/null | grep -q "$PATTERN"; then
                exit 0
            fi
            ;;
        node)
            if ros2 node list 2>/dev/null | grep -q "$PATTERN"; then
                exit 0
            fi
            ;;
        http)
            if curl -s --noproxy '*' --connect-timeout 2 "$PATTERN" >/dev/null 2>&1; then
                exit 0
            elif python3 -c "
import urllib.request, sys
try:
    urllib.request.urlopen('$PATTERN', timeout=2)
    sys.exit(0)
except Exception:
    sys.exit(1)
" 2>/dev/null; then
                exit 0
            fi
            ;;
        *)
            echo "未知类型: $TYPE (支持: service|topic|node|http)" >&2
            exit 2
            ;;
    esac

    ELAPSED=$(($(date +%s) - START))
    if [ "$ELAPSED" -ge "$TIMEOUT" ]; then
        echo "[WARN] 等待超时: $TYPE '$PATTERN' (${TIMEOUT}s)" >&2
        exit 1
    fi
    sleep 0.5
done
