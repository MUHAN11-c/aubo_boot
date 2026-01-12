#!/bin/bash

# 启动HTTP桥接服务器（用于Web UI）
echo "启动HTTP桥接服务器..."
echo "服务地址: http://localhost:8088/"
echo "按 Ctrl+C 停止服务器"

# 确保ROS2环境已设置
if [ -f ~/RVG_ws/install/setup.bash ]; then
    source ~/RVG_ws/install/setup.bash
fi

# 启动算法服务器
cd "$(dirname "$0")"
python3 algorithm_server.py
