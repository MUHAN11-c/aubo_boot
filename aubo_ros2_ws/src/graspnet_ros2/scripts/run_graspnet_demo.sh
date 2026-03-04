#!/bin/bash
# 启动 GraspNet Demo（含 move_to_pose_server）
# - graspnet_demo.launch.py：机械臂模型、ros2_control、MoveIt2、graspnet_demo_node、RViz2，
#   以及 move_to_pose_server_node（由 launch 内 use_move_to_pose_server 控制，默认 true）
# - move_to_pose_server 对应 demo_driver 包中的 C++ 节点，由本 launch 一并启动，无需单独运行

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 工作空间根目录：scripts -> graspnet_ros2 -> src -> aubo_ros2_ws（回退三级）
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo "错误: 未找到工作空间 install，请先执行: cd $WS_ROOT && colcon build"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"

echo "工作空间: $WS_ROOT"
echo "启动: ros2 launch graspnet_ros2 graspnet_demo.launch.py $*"
echo "（move_to_pose_server_node 已由该 launch 启动，无需单独运行）"
echo ""

exec ros2 launch graspnet_ros2 graspnet_demo.launch.py "$@"
