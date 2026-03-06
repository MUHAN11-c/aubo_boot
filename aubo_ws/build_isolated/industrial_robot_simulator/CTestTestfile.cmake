# CMake generated Testfile for 
# Source directory: /home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_robot_simulator
# Build directory: /home/mu/IVG/aubo_ws/build_isolated/industrial_robot_simulator
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_industrial_robot_simulator_roslaunch-check_launch "/home/mu/IVG/aubo_ws/build_isolated/industrial_robot_simulator/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/aubo_ws/build_isolated/industrial_robot_simulator/test_results/industrial_robot_simulator/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/mu/IVG/aubo_ws/build_isolated/industrial_robot_simulator/test_results/industrial_robot_simulator" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/mu/IVG/aubo_ws/build_isolated/industrial_robot_simulator/test_results/industrial_robot_simulator/roslaunch-check_launch.xml\" \"/home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_robot_simulator/launch\" ")
set_tests_properties(_ctest_industrial_robot_simulator_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_robot_simulator/CMakeLists.txt;15;roslaunch_add_file_check;/home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_robot_simulator/CMakeLists.txt;0;")
subdirs("gtest")
