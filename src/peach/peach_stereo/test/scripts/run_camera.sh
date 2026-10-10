#!/bin/bash
# 以指定 sgbm.mode 起相机节点（与 launch 等价的直接运行形式；产物日志到 /tmp/e2e_live）
# 用法: run_camera.sh <mode:hh4|3way|hh|sgbm> <logfile>
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
. install/setup.bash
export ROS_DOMAIN_ID=77
MODE="${1:-hh4}"
LOG="${2:-/tmp/e2e_live/camera.log}"
exec ros2 run peach_stereo stereo_camera_node --ros-args \
  -r __ns:=/camera \
  --params-file /home/mu/Desktop/aubo_e5_jazzy_ws/install/peach_stereo/share/peach_stereo/config/stereo_camera.yaml \
  -p color_camera_info_file:=/home/mu/Desktop/aubo_e5_jazzy_ws/install/percipio_camera/share/percipio_camera/config/color_camera_info.yaml \
  -p sgbm.mode:="$MODE" > "$LOG" 2>&1
