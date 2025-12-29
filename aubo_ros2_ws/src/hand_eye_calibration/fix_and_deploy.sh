#!/bin/bash
# 修复并部署新功能

echo "============================================"
echo "  手眼标定工具 - 修复并部署新功能"
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
echo "4️⃣  验证文件是否正确部署..."

# 检查 script_v2.js 是否包含新功能
if grep -q "autoLoadDefaultCameraParams" install/hand_eye_calibration/share/hand_eye_calibration/web/static/script_v2.js; then
    echo "✅ script_v2.js 包含新功能代码"
else
    echo "❌ script_v2.js 未包含新功能，可能需要手动复制"
    exit 1
fi

# 检查去畸变API
if grep -q "undistort_image" src/hand_eye_calibration/hand_eye_calibration/hand_eye_calibration_node.py; then
    echo "✅ 后端包含去畸变API"
else
    echo "❌ 后端未包含去畸变API"
    exit 1
fi

echo ""
echo "============================================"
echo "  ✅ 修复和部署完成！"
echo "============================================"
echo ""
echo "📌 现在请执行以下步骤："
echo ""
echo "1️⃣  启动节点："
echo "   ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
echo ""
echo "2️⃣  打开浏览器（新的隐私窗口，避免缓存问题）："
echo "   - Chrome: Ctrl + Shift + N"
echo "   - Firefox: Ctrl + Shift + P"
echo "   然后访问: http://localhost:8080"
echo ""
echo "   或者在当前浏览器强制刷新:"
echo "   - 按 Ctrl + Shift + R 或 Ctrl + F5"
echo ""
echo "3️⃣  打开浏览器开发者工具（F12），查看Console输出："
echo "   应该看到："
echo "   🚀 script_v2.js 加载成功 - 新功能版本 v2.0"
echo "   ✅ 包含功能：自动加载标定参数、文件选择加载、去畸变弹窗"
echo ""
echo "4️⃣  在操作日志中查看："
echo "   应该看到："
echo "   [时间] 尝试自动加载默认相机参数..."
echo "   [时间] ✅ 已自动加载默认相机参数：fx=..., fy=..."
echo ""
echo "5️⃣  测试文件选择功能："
echo "   - 点击'加载相机参数'按钮"
echo "   - 应该弹出文件选择对话框"
echo ""
echo "6️⃣  测试去畸变弹窗："
echo "   - 上传一张图像或点击'采集验证图像'"
echo "   - 图像加载后应该弹出："
echo "     '🔧 图像去畸变'"
echo "     '检测到已加载相机标定参数，是否对图像进行去畸变处理？'"
echo ""
echo "📖 详细文档: NEW_FEATURES_v2.md"
echo ""

