# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/rqt_dep
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_dep
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rqt_dep_nosetests_test.dotcode_pack_test.py "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_dep/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_dep/test_results/rqt_dep/nosetests-test.dotcode_pack_test.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_dep/test_results/rqt_dep" "/usr/bin/nosetests3 -P --process-timeout=60 /home/mu/IVG/ros_catkin_ws/src/rqt_dep/test/dotcode_pack_test.py --with-xunit --xunit-file=/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_dep/test_results/rqt_dep/nosetests-test.dotcode_pack_test.py.xml")
set_tests_properties(_ctest_rqt_dep_nosetests_test.dotcode_pack_test.py PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/rqt_dep/CMakeLists.txt;9;catkin_add_nosetests;/home/mu/IVG/ros_catkin_ws/src/rqt_dep/CMakeLists.txt;0;")
subdirs("gtest")
