# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/roslint
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/roslint
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_roslint_roslint_package "/home/mu/IVG/ros_catkin_ws/build_isolated/roslint/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/roslint/test_results/roslint/roslint-roslint.xml" "--working-dir" "/home/mu/IVG/ros_catkin_ws/build_isolated/roslint" "--return-code" "/home/mu/IVG/ros_catkin_ws/src/roslint/scripts/test_wrapper /home/mu/IVG/ros_catkin_ws/build_isolated/roslint/test_results/roslint/roslint-roslint.xml make roslint_roslint")
set_tests_properties(_ctest_roslint_roslint_package PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/build_isolated/roslint/cmake/roslint-extras.cmake;67;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/roslint/CMakeLists.txt;28;roslint_add_test;/home/mu/IVG/ros_catkin_ws/src/roslint/CMakeLists.txt;0;")
subdirs("gtest")
