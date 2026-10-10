cmake_minimum_required(VERSION 3.16)
project(ivg_foxglove_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ZLIB REQUIRED)

# ---- vendored websocketpp 0.8.2（BSD，header-only；asio 走 Boost）----
add_library(websocketpp_headers INTERFACE)
target_include_directories(websocketpp_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/websocketpp>)
target_link_libraries(websocketpp_headers INTERFACE Boost::headers ZLIB::ZLIB)

# ---- vendored foxglove-websocket 服务器库（MIT，ws-protocol f3b4135）----
add_library(foxglove_websocket STATIC
  vendor/foxglove-websocket/src/base64.cpp
  vendor/foxglove-websocket/src/parameter.cpp
  vendor/foxglove-websocket/src/serialization.cpp
  vendor/foxglove-websocket/src/server_factory.cpp)
set_target_properties(foxglove_websocket PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(foxglove_websocket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/foxglove-websocket/include>)
target_link_libraries(foxglove_websocket
  PUBLIC websocketpp_headers nlohmann_json::nlohmann_json)

# ---- ROS 2 glue（上游 0.8.5 ros2_foxglove_bridge 原样 + 版本符号本地实现）----
add_library(${PROJECT_NAME} SHARED
  src/version.cpp
  src/generic_client.cpp
  src/message_definition_cache.cpp
  src/parameter_interface.cpp
  src/param_utils.cpp
  src/ros2_foxglove_bridge.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} foxglove_websocket)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp rclcpp rclcpp_components rcl_interfaces resource_retriever rosgraph_msgs)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "foxglove_bridge::FoxgloveBridge"
  EXECUTABLE foxglove_bridge)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_index_cpp rclcpp rcl_interfaces resource_retriever rosgraph_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 全包 vendored（foxglove ws-protocol MIT + websocketpp BSD + 上游 0.8.5 ros2 glue
  # 原样，见 VENDOR_PROVENANCE.md）：上游自带风格/版权体系，ROS lint 门不适用，
  # 显式豁免（ament 惯例：FOUND TRUE 短路对应测试），避免每次 colcon test 千余红
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
