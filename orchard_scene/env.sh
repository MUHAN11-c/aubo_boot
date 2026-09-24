#!/usr/bin/env bash
# orchard_scene 一键环境：Gazebo Harmonic 随 ROS Jazzy vendor 安装，
# gz CLI 需要 GZ_CONFIG_PATH 指向各 vendor 的 share/gz，HD 世界还需要
# GZ_SIM_RESOURCE_PATH 解析 model:// 网格。
# 用法：先 source /opt/ros/jazzy/setup.bash（补齐 LD_LIBRARY_PATH），再 source 本文件。
export GZ_CONFIG_PATH="/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/sdformat_vendor/share/gz"
export GZ_SIM_RESOURCE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
