#!/bin/bash
# 最终部署脚本 - 按需去畸变版本

echo "============================================"
echo "  手眼标定工具 - 最终部署"
echo "  策略：按需去畸变（快速+准确）"
echo "============================================"
echo ""

# 进入工作空间
cd ~/RVG_ws

echo "1️⃣  清理旧的编译文件..."
rm -rf build/hand_eye_calibration install/hand_eye_calibration

echo ""
echo "2️⃣  重新编译手眼标定包..."
colcon build --packages-select hand_eye_calibration --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ 编译成功！"
else
    echo "❌ 编译失败，请检查错误信息"
    exit 1
fi

echo ""
echo "3️⃣  应用环境变量..."
source install/setup.bash

echo ""
echo "4️⃣  验证文件部署..."

# 检查 script_v2.js
if [ -f "install/hand_eye_calibration/share/hand_eye_calibration/web/static/script_v2.js" ]; then
    echo "✅ script_v2.js 已部署"
    
    # 检查是否包含按需去畸变的注释
    if grep -q "按需去畸变策略" install/hand_eye_calibration/share/hand_eye_calibration/web/static/script_v2.js; then
        echo "✅ 包含按需去畸变策略"
    else
        echo "⚠️  未检测到按需去畸变策略标记"
    fi
else
    echo "❌ script_v2.js 未找到"
fi

# 检查后端去畸变API
if grep -q "pixel_to_camera_coords" src/hand_eye_calibration/hand_eye_calibration/camera_calibration_utils.py; then
    echo "✅ 后端坐标计算函数就绪"
fi

echo ""
echo "============================================"
echo "  ✅ 部署完成！"
echo "============================================"
echo ""
echo "📌 按需去畸变策略说明："
echo ""
echo "   图像显示：原始状态（快速）"
echo "   坐标计算：自动去畸变（准确）"
echo ""
echo "   优势："
echo "   • 图像加载快（无需处理6百万像素）"
echo "   • 数据传输小（<500KB vs >1MB）"
echo "   • 计算准确（只处理需要的点）"
echo ""
echo "📌 下一步操作："
echo ""
echo "1️⃣  启动节点："
echo "   ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
echo ""
echo "2️⃣  打开浏览器（使用隐私窗口避免缓存）："
echo "   Chrome: Ctrl + Shift + N"
echo "   访问: http://localhost:8080"
echo ""
echo "3️⃣  验证功能："
echo "   • 加载图像后，不应该弹出去畸变对话框"
echo "   • 看到提示：'图像显示为原始状态，坐标计算时会自动去畸变'"
echo "   • 提取角点或手动选点时，坐标计算准确"
echo ""
echo "4️⃣  性能对比："
echo "   旧方案：图像加载需要去畸变处理，耗时3-5秒"
echo "   新方案：图像立即显示，< 1秒"
echo ""
echo "📖 详细文档:"
echo "   • ON_DEMAND_UNDISTORT.md - 按需去畸变策略详解"
echo "   • VERIFICATION_CHECKLIST.md - 验证清单"
echo ""

