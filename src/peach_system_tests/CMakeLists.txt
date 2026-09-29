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
  # 场景障碍快照链（2026-09-29 避障只为保护相机）：合成点云+假目标+触发，
  # 假 apply_planning_scene 端断言对象内容/目标邻域空洞/原子替换。域 94。
  add_launch_test(test/test_scene_obstacles.py TIMEOUT 300
    ENV "ROS_DOMAIN_ID=94" "ROS_LOCALHOST_ONLY=1")
  # 三把末端逐档冒烟（bite 主测轮）：TF 对档案 + 标签对齐 + imu_follow 接线，
  # 收编 09-28 手工 _tools/smoke_shear.sh。域 91/92/93 = shear/bite/adaptive。
  add_launch_test(test/test_tool_profile_smoke.py
    TARGET test_tool_profile_smoke_shear_v1 TIMEOUT 420
    ENV "ROS_DOMAIN_ID=91" "ROS_LOCALHOST_ONLY=1" "PEACH_LT_TOOL_PROFILE=shear_v1")
  add_launch_test(test/test_tool_profile_smoke.py
    TARGET test_tool_profile_smoke_bite_shear_v1 TIMEOUT 420
    ENV "ROS_DOMAIN_ID=92" "ROS_LOCALHOST_ONLY=1" "PEACH_LT_TOOL_PROFILE=bite_shear_v1")
  add_launch_test(test/test_tool_profile_smoke.py
    TARGET test_tool_profile_smoke_adaptive_shear_v1 TIMEOUT 420
    ENV "ROS_DOMAIN_ID=93" "ROS_LOCALHOST_ONLY=1" "PEACH_LT_TOOL_PROFILE=adaptive_shear_v1")
endif()

# 系统测驱动脚本（ros2 run peach_system_tests <script>）：对已起栈的
# 人工复核工具——goal 发送 / 相机流探针 / PlanningScene+ACM 巡检。
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
