#!/bin/bash
# 部署新功能脚本

echo "========================================"
echo "  手眼标定工具 - 新功能部署脚本"
echo "========================================"
echo ""

# 进入工作空间
cd ~/RVG_ws

echo "1️⃣  清理旧的编译文件..."
rm -rf build/hand_eye_calibration install/hand_eye_calibration

echo ""
echo "2️⃣  重新编译手眼标定包（使用symlink模式）..."
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
echo "========================================"
echo "  ✅ 部署完成！"
echo "========================================"
echo ""
echo "📌 下一步操作："
echo ""
echo "1. 启动节点："
echo "   ros2 launch hand_eye_calibration hand_eye_calibration_launch.py"
echo ""
echo "2. 打开浏览器，访问："
echo "   http://localhost:8080"
echo ""
echo "3. 强制刷新浏览器清除缓存："
echo "   按 Ctrl + Shift + R 或 Ctrl + F5"
echo ""
echo "4. 检查新功能："
echo "   - 查看操作日志，应该看到自动加载标定参数的提示"
echo "   - 点击'加载相机参数'按钮，应该弹出文件选择框"
echo "   - 拍照或上传图像后，应该弹出去畸变确认对话框"
echo ""
echo "📖 详细文档: NEW_FEATURES_v2.md"
echo ""

