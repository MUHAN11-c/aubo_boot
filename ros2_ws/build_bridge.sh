#!/bin/bash
# 使用 aubo_ws (ROS1) 和 aubo_ros2_ws (ROS2) 编译桥接工作空间 ros2_ws
# ROS Noetic: /home/mu/IVG/ros_catkin_ws
# ROS Humble: /opt/ros/humble/setup.bash

set -e
export ROS1_SETUP=/home/mu/IVG/ros_catkin_ws/install_isolated/setup.bash
export AUBO_WS=/home/mu/IVG/aubo_ws
export ROS2_SETUP=/opt/ros/humble/setup.bash
export AUBO_ROS2_WS=/home/mu/IVG/aubo_ros2_ws
export ROS2_WS=/home/mu/IVG/ros2_ws

# ROS1 链：Noetic (ros_catkin_ws) -> ws_moveit -> aubo_ws
source "$ROS1_SETUP"
source /home/mu/IVG/ws_moveit/install_isolated/setup.bash 2>/dev/null || true
source "$AUBO_WS/devel_isolated/setup.bash"

# ROS2 链：Humble -> aubo_ros2_ws
source "$ROS2_SETUP"
source "$AUBO_ROS2_WS/install/setup.bash" 2>/dev/null || true

cd "$ROS2_WS"
colcon build --symlink-install "$@"
echo "Done. Source: source $ROS2_WS/install/setup.bash"
