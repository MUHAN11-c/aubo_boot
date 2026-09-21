#!/bin/bash
# 原始深度探针启动器（source overlay + venv python）
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
. install/setup.bash
export ROS_DOMAIN_ID=77
exec ./aubo_py3.12/bin/python /tmp/e2e_live/probe_raw_depth.py
