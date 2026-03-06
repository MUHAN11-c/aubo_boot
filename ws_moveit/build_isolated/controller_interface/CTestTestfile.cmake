# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ws_moveit/src/ros_control/controller_interface
# Build directory: /home/mu/IVG/ws_moveit/build_isolated/controller_interface
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_controller_interface_gtest_controller_base_test "/home/mu/IVG/ws_moveit/build_isolated/controller_interface/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ws_moveit/build_isolated/controller_interface/test_results/controller_interface/gtest-controller_base_test.xml" "--return-code" "/home/mu/IVG/ws_moveit/devel_isolated/controller_interface/lib/controller_interface/controller_base_test --gtest_output=xml:/home/mu/IVG/ws_moveit/build_isolated/controller_interface/test_results/controller_interface/gtest-controller_base_test.xml")
set_tests_properties(_ctest_controller_interface_gtest_controller_base_test PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;63;_catkin_add_google_test;/home/mu/IVG/ws_moveit/src/ros_control/controller_interface/CMakeLists.txt;33;catkin_add_gmock;/home/mu/IVG/ws_moveit/src/ros_control/controller_interface/CMakeLists.txt;0;")
subdirs("gtest")
