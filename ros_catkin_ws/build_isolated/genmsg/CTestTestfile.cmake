# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/genmsg
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/genmsg
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_genmsg_nosetests_test "/home/mu/IVG/ros_catkin_ws/build_isolated/genmsg/catkin_generated/env_cached.sh" "/usr/bin/python3.10" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/genmsg/test_results/genmsg/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/mu/IVG/ros_catkin_ws/build_isolated/genmsg/test_results/genmsg" "/usr/bin/nosetests3 -P --process-timeout=60 --where=/home/mu/IVG/ros_catkin_ws/src/genmsg/test --with-xunit --xunit-file=/home/mu/IVG/ros_catkin_ws/build_isolated/genmsg/test_results/genmsg/nosetests-test.xml")
set_tests_properties(_ctest_genmsg_nosetests_test PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/genmsg/CMakeLists.txt;23;catkin_add_nosetests;/home/mu/IVG/ros_catkin_ws/src/genmsg/CMakeLists.txt;0;")
subdirs("gtest")
