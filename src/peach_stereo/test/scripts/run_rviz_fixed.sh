#!/bin/bash
# rviz2 启动器（固定几何配置 perception_fixed.rviz；几何由配置内 Window Geometry 决定）
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
. install/setup.bash
export ROS_DOMAIN_ID=77
export QT_X11_NO_MITSHM=1
SD=$(cd "$(dirname "$0")" && pwd)
exec rviz2 -d "$SD/perception_fixed.rviz"
