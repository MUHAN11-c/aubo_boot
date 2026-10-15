cmake_minimum_required(VERSION 3.8)
project(sim_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # 超时须覆盖：重世界首载 40-90 s + 两个用例各 300 s 语义等待 + 拆栈；
  # 420 s 会在慢机/并发负载下先于用例自身超时杀进程，产出 missing_result
  add_launch_test(test/test_sim_controllers.py TIMEOUT 900)
endif()

ament_package()
