# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/ros_comm/rosnode
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/rosnode
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rosnode_rostest_test_rosnode.test "/home/mu/IVG/ros_catkin_ws/build_isolated/rosnode/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/rosnode/test_results/rosnode/rostest-test_rosnode.xml" "--return-code" "/usr/bin/python3 /home/mu/IVG/ros_catkin_ws/install_isolated/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/mu/IVG/ros_catkin_ws/src/ros_comm/rosnode --package=rosnode --results-filename test_rosnode.xml --results-base-dir \"/home/mu/IVG/ros_catkin_ws/build_isolated/rosnode/test_results\" /home/mu/IVG/ros_catkin_ws/src/ros_comm/rosnode/test/rosnode.test ")
set_tests_properties(_ctest_rosnode_rostest_test_rosnode.test PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/ros_comm/rosnode/CMakeLists.txt;10;add_rostest;/home/mu/IVG/ros_catkin_ws/src/ros_comm/rosnode/CMakeLists.txt;0;")
subdirs("gtest")
