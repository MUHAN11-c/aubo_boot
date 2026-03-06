# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ws_moveit/src/moveit/moveit_setup_assistant
# Build directory: /home/mu/IVG/ws_moveit/build_isolated/moveit_setup_assistant
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_moveit_setup_assistant_gtest_test_reading_writing_config "/home/mu/IVG/ws_moveit/build_isolated/moveit_setup_assistant/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ws_moveit/build_isolated/moveit_setup_assistant/test_results/moveit_setup_assistant/gtest-test_reading_writing_config.xml" "--return-code" "/home/mu/IVG/ws_moveit/devel_isolated/moveit_setup_assistant/lib/moveit_setup_assistant/test_reading_writing_config --gtest_output=xml:/home/mu/IVG/ws_moveit/build_isolated/moveit_setup_assistant/test_results/moveit_setup_assistant/gtest-test_reading_writing_config.xml")
set_tests_properties(_ctest_moveit_setup_assistant_gtest_test_reading_writing_config PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/mu/IVG/ws_moveit/src/moveit/moveit_setup_assistant/CMakeLists.txt;165;catkin_add_gtest;/home/mu/IVG/ws_moveit/src/moveit/moveit_setup_assistant/CMakeLists.txt;0;")
subdirs("gtest")
