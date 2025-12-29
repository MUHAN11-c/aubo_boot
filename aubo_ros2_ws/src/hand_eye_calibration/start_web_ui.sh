#!/bin/bash

echo "=========================================="
echo "  启动手眼标定Web界面"
echo "=========================================="
echo ""

# 切换到工作空间
cd /home/nvidia/RVG_ws

# 加载ROS2环境
echo "🔧 加载ROS2环境..."
source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "✅ 环境加载完成"
echo ""

# 检查8080端口是否被占用
if netstat -tlnp 2>/dev/null | grep -q ":8080 " || ss -tlnp 2>/dev/null | grep -q ":8080 "; then
    echo "⚠️  警告：8080端口已被占用"
    echo "   正在停止旧进程..."
    pkill -f hand_eye_calibration_node
    sleep 2
fi

echo "🚀 启动手眼标定节点..."
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 启动节点
ros2 launch hand_eye_calibration hand_eye_calibration_launch.py





