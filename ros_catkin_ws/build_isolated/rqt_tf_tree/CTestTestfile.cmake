# CMake generated Testfile for 
# Source directory: /home/mu/IVG/ros_catkin_ws/src/rqt_tf_tree
# Build directory: /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_tf_tree
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_rqt_tf_tree_nosetests_test.dotcode_tf_test.py "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_tf_tree/catkin_generated/env_cached.sh" "/usr/bin/python3" "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/run_tests.py" "/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_tf_tree/test_results/rqt_tf_tree/nosetests-test.dotcode_tf_test.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/mu/IVG/ros_catkin_ws/build_isolated/rqt_tf_tree/test_results/rqt_tf_tree" "/usr/bin/nosetests3 -P --process-timeout=60 /home/mu/IVG/ros_catkin_ws/src/rqt_tf_tree/test/dotcode_tf_test.py --with-xunit --xunit-file=/home/mu/IVG/ros_catkin_ws/build_isolated/rqt_tf_tree/test_results/rqt_tf_tree/nosetests-test.dotcode_tf_test.py.xml")
set_tests_properties(_ctest_rqt_tf_tree_nosetests_test.dotcode_tf_test.py PROPERTIES  _BACKTRACE_TRIPLES "/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/tests.cmake;160;add_test;/home/mu/IVG/ros_catkin_ws/install_isolated/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/mu/IVG/ros_catkin_ws/src/rqt_tf_tree/CMakeLists.txt;21;catkin_add_nosetests;/home/mu/IVG/ros_catkin_ws/src/rqt_tf_tree/CMakeLists.txt;0;")
subdirs("gtest")
