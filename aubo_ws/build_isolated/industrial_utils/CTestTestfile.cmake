# CMake generated Testfile for 
# Source directory: /home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_utils
# Build directory: /home/mu/IVG/aubo_ws/build_isolated/industrial_utils
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_industrial_utils_gtest_utest_inds_utils "/home/mu/IVG/aubo_ws/build_isolated/industrial_utils/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/aubo_ws/build_isolated/industrial_utils/test_results/industrial_utils/gtest-utest_inds_utils.xml" "--return-code" "/home/mu/IVG/aubo_ws/devel_isolated/industrial_utils/lib/industrial_utils/utest_inds_utils --gtest_output=xml:/home/mu/IVG/aubo_ws/build_isolated/industrial_utils/test_results/industrial_utils/gtest-utest_inds_utils.xml")
set_tests_properties(_ctest_industrial_utils_gtest_utest_inds_utils PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_utils/CMakeLists.txt;24;catkin_add_gtest;/home/mu/IVG/aubo_ws/src/aubo_robot/industrial_core/industrial_utils/CMakeLists.txt;0;")
subdirs("gtest")
