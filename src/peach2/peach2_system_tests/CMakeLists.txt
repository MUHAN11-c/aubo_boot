cmake_minimum_required(VERSION 3.16)
project(peach2_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Jazzy launch_testing_ament_cmake has no add_ros_isolated_launch_test (Kilted+): each test
  # gets its own ROS_DOMAIN_ID (95..97, v1 system tests use 89..94) and localhost-only DDS.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bringup_mock.py TIMEOUT 420
    ENV "ROS_DOMAIN_ID=95" "ROS_LOCALHOST_ONLY=1")
  add_launch_test(test/test_plan_only_chain.py TIMEOUT 720
    ENV "ROS_DOMAIN_ID=96" "ROS_LOCALHOST_ONLY=1")
  add_launch_test(test/test_pregrasp_mock_exec.py TIMEOUT 900
    ENV "ROS_DOMAIN_ID=97" "ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
