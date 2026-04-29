#!/bin/bash
# 测试 demo_driver 服务

echo "=========================================="
echo "测试 demo_driver 服务"
echo "=========================================="

# 检查服务是否可用
echo ""
echo "1. 检查服务是否可用..."
ros2 service list | grep -E "move_to_pose|plan_trajectory|execute_trajectory|get_current_state|set_speed_factor"

if [ $? -ne 0 ]; then
    echo "错误: 服务未找到，请确保 launch 文件已启动"
    exit 1
fi

echo ""
echo "2. 测试 /get_current_state 服务..."
ros2 service call /get_current_state demo_interface/srv/GetCurrentState "{}"

echo ""
echo "3. 测试 /set_speed_factor 服务 (设置为 0.5)..."
ros2 service call /set_speed_factor demo_interface/srv/SetSpeedFactor "{velocity_factor: 0.5}"

echo ""
echo "=========================================="
echo "服务测试完成"
echo "=========================================="

