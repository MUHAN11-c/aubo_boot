# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ws_moveit/src/moveit/moveit_ros/visualization
# Build directory: /home/mu/IVG/ws_moveit/build_isolated/moveit_ros_visualization
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_moveit_ros_visualization_rostest_test_moveit_joy.test "/home/mu/IVG/ws_moveit/build_isolated/moveit_ros_visualization/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ws_moveit/build_isolated/moveit_ros_visualization/test_results/moveit_ros_visualization/rostest-test_moveit_joy.xml" "--return-code" "/usr/bin/python3 /home/mu/IVG/ros_catkin_ws/install_isolated/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/mu/IVG/ws_moveit/src/moveit/moveit_ros/visualization --package=moveit_ros_visualization --results-filename test_moveit_joy.xml --results-base-dir \"/home/mu/IVG/ws_moveit/build_isolated/moveit_ros_visualization/test_results\" /home/mu/IVG/ws_moveit/src/moveit/moveit_ros/visualization/test/moveit_joy.test ")
set_tests_properties(_ctest_moveit_ros_visualization_rostest_test_moveit_joy.test PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/mu/IVG/ws_moveit/src/moveit/moveit_ros/visualization/CMakeLists.txt;113;add_rostest;/home/mu/IVG/ws_moveit/src/moveit/moveit_ros/visualization/CMakeLists.txt;0;")
subdirs("gtest")
subdirs("rviz_plugin_render_tools")
subdirs("robot_state_rviz_plugin")
subdirs("planning_scene_rviz_plugin")
subdirs("motion_planning_rviz_plugin")
subdirs("trajectory_rviz_plugin")
