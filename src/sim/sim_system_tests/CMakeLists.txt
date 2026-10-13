cmake_minimum_required(VERSION 3.8)
project(sim_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_controllers.py TIMEOUT 420)
endif()

ament_package()
