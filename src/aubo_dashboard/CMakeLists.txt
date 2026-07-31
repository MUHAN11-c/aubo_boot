cmake_minimum_required(VERSION 3.22)
project(aubo_dashboard)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(aubo_msgs REQUIRED)

add_executable(aubo_dashboard_node src/aubo_dashboard_node.cpp)
target_compile_features(aubo_dashboard_node PRIVATE cxx_std_17)
target_include_directories(aubo_dashboard_node PRIVATE vendor vendor/aubo_driver)
ament_target_dependencies(aubo_dashboard_node rclcpp std_srvs aubo_msgs)

set(AUBO_SDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/aubocontroller/libauborobotcontroller.so)
if(NOT EXISTS ${AUBO_SDK_LIB})
  message(FATAL_ERROR "missing vendored AUBO SDK: ${AUBO_SDK_LIB}")
endif()
target_link_libraries(aubo_dashboard_node ${AUBO_SDK_LIB} pthread)
# DT_RPATH (old dtags) so the rpath also covers transitive deps (libprotobuf.so.9
# is NEEDED by libauborobotcontroller, not by this executable).
target_link_options(aubo_dashboard_node PRIVATE -Wl,--disable-new-dtags)
set(AUBO_DASHBOARD_VENDOR_LIBDIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/aubocontroller
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/config
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/log4cplus
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/protobuf)
set_target_properties(aubo_dashboard_node PROPERTIES
  BUILD_RPATH "${AUBO_DASHBOARD_VENDOR_LIBDIRS}"
  INSTALL_RPATH "$ORIGIN/vendor")

install(TARGETS aubo_dashboard_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY vendor/lib64/aubocontroller/ vendor/lib64/config/ vendor/lib64/log4cplus/ vendor/lib64/protobuf/
  DESTINATION lib/${PROJECT_NAME}/vendor
  FILES_MATCHING PATTERN "*.so*" PATTERN "*.a" EXCLUDE PATTERN "*.la" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
