#!/bin/bash
echo "=========================================="
echo "测试 HTTP 桥接启动"
echo "=========================================="

cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash

echo ""
echo "1. 检查导入..."
python3 << 'PYEOF'
import sys
sys.path.insert(0, '/home/mu/IVG/aubo_ros2_ws/install/interface/lib/python3.8/site-packages')
try:
    from interface.srv import UpdateParams
    print("   ✓ UpdateParams 导入成功")
except ImportError:
    print("   ⚠️  UpdateParams 不可用（降级模式）")
PYEOF

echo ""
echo "2. 验证代码语法..."
python3 -m py_compile src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts/http_bridge_server.py
if [ $? -eq 0 ]; then
    echo "   ✓ 代码语法正确"
else
    echo "   ✗ 代码语法错误"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ 所有检查通过，可以启动服务"
echo "=========================================="
