# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/mu/IVG/aubo_ws/devel_isolated/industrial_trajectory_filters;/home/mu/IVG/aubo_ws/devel_isolated/industrial_robot_simulator;/home/mu/IVG/aubo_ws/devel_isolated/demo_driver;/home/mu/IVG/aubo_ws/devel_isolated/aubo_driver;/home/mu/IVG/aubo_ws/devel_isolated/industrial_msgs;/home/mu/IVG/aubo_ws/devel_isolated/industrial_deprecated;/home/mu/IVG/aubo_ws/devel_isolated/industrial_core;/home/mu/IVG/aubo_ws/devel_isolated/demo_interface;/home/mu/IVG/aubo_ws/devel_isolated/aubo_urdf;/home/mu/IVG/aubo_ws/devel_isolated/aubo_robot;/home/mu/IVG/aubo_ws/devel_isolated/aubo_msgs;/home/mu/IVG/aubo_ws/devel_isolated/aubo_kinematics;/home/mu/IVG/aubo_ws/devel_isolated/aubo_gazebo;/home/mu/IVG/aubo_ws/devel_isolated/aubo_e5_moveit_config;/home/mu/IVG/aubo_ws/devel_isolated/aubo_description;/home/mu/IVG/aubo_ws/devel_isolated/aubo_demo;/home/mu/IVG/ws_moveit/install_isolated;/home/mu/IVG/ros_catkin_ws/install_isolated'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/mu/IVG/aubo_ws/devel_isolated/industrial_utils/env.sh')

output_filename = '/home/mu/IVG/aubo_ws/build_isolated/industrial_utils/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
