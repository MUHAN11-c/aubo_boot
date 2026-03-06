# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/rqt_graph
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_graph
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rqt_graph_nosetests_test.dotcode_test.py "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_graph/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_graph/test_results/rqt_graph/nosetests-test.dotcode_test.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_graph/test_results/rqt_graph" "/usr/bin/nosetests3 -P --process-timeout=60 /home/mu/IVG/ros_catkin_ws/src/rqt_graph/test/dotcode_test.py --with-xunit --xunit-file=/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_graph/test_results/rqt_graph/nosetests-test.dotcode_test.py.xml")
set_tests_properties(_ctest_rqt_graph_nosetests_test.dotcode_test.py PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/rqt_graph/CMakeLists.txt;9;catkin_add_nosetests;/home/mu/IVG/ros_catkin_ws/src/rqt_graph/CMakeLists.txt;0;")
subdirs("gtest")
