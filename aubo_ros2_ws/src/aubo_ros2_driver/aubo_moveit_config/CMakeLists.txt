cmake_minimum_required(VERSION 3.16.3)
project(aubo_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
install(PROGRAMS scripts/aubo_mode.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/simulation_state_publisher.py DESTINATION lib/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
