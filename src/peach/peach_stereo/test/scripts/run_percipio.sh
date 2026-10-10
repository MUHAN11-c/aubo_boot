#!/bin/bash
# 原始 percipio 相机驱动前端（RGB-D 配准版，harvest_system 同款）启动器
# 用法: run_percipio.sh [额外 launch 参数...]  如: run_percipio.sh depth_resolution:=640x480
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
. install/setup.bash
export ROS_DOMAIN_ID=77
exec ros2 launch percipio_camera percipio_rgbd.launch.py "$@"
