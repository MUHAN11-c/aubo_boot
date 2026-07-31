cmake_minimum_required(VERSION 3.22)
project(aubo_e5_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/aubo_e5_hardware.cpp src/aubo_e5_sim_hardware.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# 旧式 export_state_interfaces()/export_command_interfaces() 在 Jazzy 已标记
# deprecated，但它仍是 UR 验证过的、能让硬件把 passthrough 状态机反馈回写进
# command 接口的写法，故保留并仅屏蔽该告警。
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_include_directories(${PROJECT_NAME} PRIVATE vendor vendor/aubo_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs)

set(AUBO_VENDOR_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/aubocontroller/libauborobotcontroller.so)
if(NOT EXISTS ${AUBO_VENDOR_LIBRARY})
  message(FATAL_ERROR "The bundled 64-bit AUBO SDK library is missing: ${AUBO_VENDOR_LIBRARY}")
endif()
target_link_libraries(${PROJECT_NAME} ${AUBO_VENDOR_LIBRARY} pthread)
# RPATH 的坑：必须强制生成 DT_RPATH（旧式 dtags）而不是默认的 DT_RUNPATH。
# 动态链接器解析"传递依赖"（libauborobotcontroller.so 自己 NEEDED 的
# libprotobuf.so.9 / libconfig / liblog4cplus）时不使用可执行文件/上层库的
# RUNPATH —— RUNPATH 只对直接依赖生效，DT_RPATH 才会沿依赖链向下继承。
# 不加 --disable-new-dtags 的话运行期会报 libprotobuf.so.9 找不到。
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--disable-new-dtags)

# 与上文同理：这里设置的是 DT_RPATH（不是 RUNPATH），保证搜索路径同时覆盖
# libauborobotcontroller.so 的传递 NEEDED 项（libprotobuf.so.9、libconfig、
# liblog4cplus）。INSTALL_RPATH 用 $ORIGIN 相对本库位置，便于整体搬迁。
set(AUBO_VENDOR_LIBDIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/aubocontroller
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/protobuf
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/config
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/lib64/log4cplus)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${AUBO_VENDOR_LIBDIRS}"
  INSTALL_RPATH "$ORIGIN/aubo_e5_hardware/vendor")

pluginlib_export_plugin_description_file(hardware_interface aubo_e5_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  vendor/lib64/aubocontroller/
  vendor/lib64/protobuf/
  vendor/lib64/config/
  vendor/lib64/log4cplus/
  DESTINATION lib/aubo_e5_hardware/vendor
  FILES_MATCHING PATTERN "*.so*" PATTERN "*.a" EXCLUDE PATTERN "*.la" EXCLUDE)
# SDK 按进程 CWD 相对路径读取 ./config/auborobot.conf；bringup 以
# cwd=share/aubo_e5_hardware 启动 ros2_control_node，故把 config 装到 share。
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES aubo_e5_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
