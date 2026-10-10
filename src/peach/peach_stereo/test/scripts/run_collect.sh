#!/bin/bash
# 采集器启动器（先 source ROS overlay 再用 venv python；stdout/stderr 由调用方重定向）
# 用法: run_collect.sh <run_tag> <duration_s>
. /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
. install/setup.bash
export ROS_DOMAIN_ID=77
SD=$(cd "$(dirname "$0")" && pwd)
exec ./aubo_py3.12/bin/python "$SD/collect.py" "$1" "${2:-70}"
