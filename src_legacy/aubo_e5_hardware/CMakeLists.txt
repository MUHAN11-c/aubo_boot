cmake_minimum_required(VERSION 3.22)
project(aubo_e5_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/aubo_ros2_system.cpp
  src/aubo_hardware_interface.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE vendor/aubo_sdk_1_3_1/include PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp)

# AUBO SDK v1.3.1（2026-07-24 晚由 2.5.3 换回）：写延迟 avg ~5ms（2.5.3 为
# ~41ms），TCP2CAN 产能 ~660 pts/s 可灌满 RIB 让控制器匀速消费——低速
# 运动平滑的结构前提。2.5.3 保留在 vendor/aubo_sdk_2_5_3 仅作对照。
set(AUBO_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/aubo_sdk_1_3_1)
set(AUBO_VENDOR_LIBRARIES
  ${AUBO_SDK_DIR}/lib/aubocontroller/libauborobotcontroller.so.1.3.1
  ${AUBO_SDK_DIR}/lib/log4cplus/liblog4cplus-1.2.so.5.1.4
  ${AUBO_SDK_DIR}/lib/config/libconfig.so.11.0.2
  ${AUBO_SDK_DIR}/lib/protobuf/libprotobuf.so.9.0.1
  ${AUBO_SDK_DIR}/lib/libotgLib.a)
foreach(lib ${AUBO_VENDOR_LIBRARIES})
  if(NOT EXISTS ${lib})
    message(FATAL_ERROR "The bundled AUBO SDK 1.3.1 library is missing: ${lib}")
  endif()
endforeach()
target_link_libraries(${PROJECT_NAME}
  "-Wl,--push-state,--no-as-needed"
  ${AUBO_VENDOR_LIBRARIES}
  "-Wl,--pop-state")
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${AUBO_SDK_DIR}/lib/aubocontroller;${AUBO_SDK_DIR}/lib/log4cplus;${AUBO_SDK_DIR}/lib/config;${AUBO_SDK_DIR}/lib/protobuf"
  INSTALL_RPATH "$ORIGIN/aubo_e5_hardware/vendor/aubocontroller;$ORIGIN/aubo_e5_hardware/vendor/log4cplus;$ORIGIN/aubo_e5_hardware/vendor/config;$ORIGIN/aubo_e5_hardware/vendor/protobuf")

pluginlib_export_plugin_description_file(hardware_interface aubo_e5_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${AUBO_SDK_DIR}/lib/
  DESTINATION lib/aubo_e5_hardware/vendor
  FILES_MATCHING PATTERN "*.so*")
install(FILES aubo_e5_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_package()
