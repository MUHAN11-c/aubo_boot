#!/bin/bash
# 测试所有 demo_driver 服务

set -e  # 遇到错误立即退出

echo "=========================================="
echo "测试 demo_driver 服务"
echo "=========================================="
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 请先 source ROS2 环境"
    exit 1
fi

# 等待服务启动
echo "等待服务启动（最多 15 秒）..."
timeout=15
elapsed=0
while [ $elapsed -lt $timeout ]; do
    # 尝试调用服务来检查是否可用
    if timeout 2 ros2 service call /get_current_state demo_interface/srv/GetCurrentState "{}" >/dev/null 2>&1; then
        echo "服务已就绪"
        break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
    echo -n "."
done
echo ""

# 检查服务是否可用（使用 service call 而不是 service list，因为服务发现可能有延迟）
echo ""
echo "1. 检查服务是否可用..."
services=(
    "/get_current_state:demo_interface/srv/GetCurrentState:{}"
    "/move_to_pose:demo_interface/srv/MoveToPose:{target_pose: {position: {x: 0.3, y: 0.2, z: 0.5}, orientation: {w: 1.0}}, use_joints: false, velocity_factor: 0.5, acceleration_factor: 0.5}"
    "/plan_trajectory:demo_interface/srv/PlanTrajectory:{target_pose: {position: {x: 0.3, y: 0.2, z: 0.5}, orientation: {w: 1.0}}, use_joints: false}"
    "/execute_trajectory:demo_interface/srv/ExecuteTrajectory:{trajectory: {joint_names: ['shoulder_joint'], points: []}}"
    "/set_speed_factor:demo_interface/srv/SetSpeedFactor:{velocity_factor: 0.5}"
)

all_available=true
for service_info in "${services[@]}"; do
    service_name=$(echo "$service_info" | cut -d: -f1)
    service_type=$(echo "$service_info" | cut -d: -f2)
    service_request=$(echo "$service_info" | cut -d: -f3-)
    
    # 使用 service call 来测试服务是否可用（超时 3 秒）
    if timeout 3 ros2 service call "$service_name" "$service_type" "$service_request" >/dev/null 2>&1; then
        echo "  ✓ $service_name 可用"
    else
        # 如果 service call 失败，再尝试用 service list 检查
        if ros2 service list 2>/dev/null | grep -q "$service_name"; then
            echo "  ✓ $service_name 可用（在服务列表中）"
        else
            echo "  ✗ $service_name 不可用"
            all_available=false
        fi
    fi
done

if [ "$all_available" = false ]; then
    echo ""
    echo "警告: 部分服务可能不可用，但服务可能仍在启动中"
    echo "提示: 可以稍后重试，或检查 launch 文件日志"
fi

echo ""
echo "2. 测试 /get_current_state 服务..."
ros2 service call /get_current_state demo_interface/srv/GetCurrentState "{}" 2>&1 | head -20

echo ""
echo "3. 测试 /set_speed_factor 服务 (设置为 0.5)..."
ros2 service call /set_speed_factor demo_interface/srv/SetSpeedFactor "{velocity_factor: 0.5}" 2>&1

echo ""
echo "=========================================="
echo "基本服务测试完成"
echo "=========================================="
echo ""
echo "提示: 可以使用以下命令测试其他服务:"
echo "  - ros2 run aubo_demo get_current_pose_client"
echo "  - ros2 run aubo_demo move_to_pose_client"
echo "  - ros2 run aubo_demo plan_trajectory_client"

