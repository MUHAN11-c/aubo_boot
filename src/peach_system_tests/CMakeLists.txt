cmake_minimum_required(VERSION 3.16)
project(peach_system_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(test_preflight test/test_preflight.py)
  ament_add_pytest_test(test_replay_approach test/test_replay_approach.py)
  ament_add_pytest_test(test_perf_baseline test/test_perf_baseline.py)
  ament_add_pytest_test(test_baseline_inventory test/test_baseline_inventory.py)
  # Jazzy 的 launch_testing_ament_cmake 无 add_ros_isolated_launch_test
  # （Kilted+ 新增宏）；用 add_launch_test 的 ENV 注入实现等效隔离域。
  # 域 89：mock 冒烟；域 90：E1 批次链（注入器 perception_sim + RunHarvest）。
  add_launch_test(test/test_mock_launch.py TIMEOUT 420
    ENV "ROS_DOMAIN_ID=89" "ROS_LOCALHOST_ONLY=1")
  add_launch_test(test/test_e1_supervisor_chain.py TIMEOUT 600
    ENV "ROS_DOMAIN_ID=90" "ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
