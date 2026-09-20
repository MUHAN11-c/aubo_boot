cmake_minimum_required(VERSION 3.16)
project(peach_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_preflight test/test_preflight.py)
  ament_add_pytest_test(test_replay_approach test/test_replay_approach.py)
  ament_add_pytest_test(test_perf_baseline test/test_perf_baseline.py)
  add_launch_test(test/test_mock_launch.py TIMEOUT 300)
endif()

ament_package()
