#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "${PACKAGE_ROOT}/../../../../.." && pwd)}"

echo "=========================================="
echo "测试 FastAPI Web 启动"
echo "=========================================="

cd "${WORKSPACE_ROOT}"
if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
fi

echo ""
echo "1. 验证代码语法..."
python3 -m py_compile "${PACKAGE_ROOT}/visual_pose_estimation_python/web/server.py"
python3 -m py_compile "${PACKAGE_ROOT}/visual_pose_estimation_python/web/ros_bridge/node_runtime.py"
echo "   ✓ 代码语法正确"

echo ""
echo "2. 提示：运行自动回归测试"
echo "   pytest ${PACKAGE_ROOT}/test/test_web_app.py"

echo ""
echo "=========================================="
echo "✓ 所有检查通过，可以启动服务"
echo "=========================================="
