cmake_minimum_required(VERSION 3.16)
project(visual_pose_estimation)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 查找YAML和JSON库
# 优先使用新编译的 yaml-cpp 库
set(YAML_CPP_CUSTOM_PATH "$ENV{HOME}/Depends/yaml-cpp/build")
if(EXISTS "${YAML_CPP_CUSTOM_PATH}/libyaml-cpp.a")
    message(STATUS "Found custom yaml-cpp library at: ${YAML_CPP_CUSTOM_PATH}")
    set(YAML_CPP_LIBRARIES "${YAML_CPP_CUSTOM_PATH}/libyaml-cpp.a")
    set(YAML_CPP_INCLUDE_DIRS "$ENV{HOME}/Depends/yaml-cpp/include")
    include_directories(${YAML_CPP_INCLUDE_DIRS})
else()
    # 回退到系统安装的库
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
    include_directories(${YAML_CPP_INCLUDE_DIRS})
endif()

find_package(jsoncpp REQUIRED)

# 包含目录
include_directories(include)

# 查找interface包
find_package(interface REQUIRED)

# 创建库（算法实现将在此添加）
add_library(visual_pose_estimation_lib
    src/ros2_communication.cpp
    src/config_reader.cpp
    src/preprocessor.cpp
    src/feature_extractor.cpp
    src/template_standardizer.cpp
    # 算法实现文件将在此添加
    # src/pose_estimation.cpp
    # src/template_creation.cpp
    # src/pose_conversion.cpp
)

target_include_directories(visual_pose_estimation_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(visual_pose_estimation_lib
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    jsoncpp_lib
)

ament_target_dependencies(visual_pose_estimation_lib
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    interface
)

# 创建主节点可执行文件
add_executable(visual_pose_estimation_node
    src/main.cpp
)

target_link_libraries(visual_pose_estimation_node
    visual_pose_estimation_lib
    ${rclcpp_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    jsoncpp_lib
)

ament_target_dependencies(visual_pose_estimation_node
    rclcpp
    interface
    ament_index_cpp
)

# 安装规则
install(TARGETS
    visual_pose_estimation_lib
    visual_pose_estimation_node
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY configs/
    DESTINATION share/${PROJECT_NAME}/configs
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 Python 脚本（使用 setup.py 中定义的脚本）
# 对于混合包（C++ + Python），直接使用 install(PROGRAMS ...) 安装脚本
install(PROGRAMS
    scripts/trigger_depth.py
    DESTINATION lib/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# 导出
ament_export_include_directories(include)
ament_export_libraries(visual_pose_estimation_lib)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    OpenCV
    tf2
    tf2_geometry_msgs
    interface
    ament_index_cpp
)

ament_package()

