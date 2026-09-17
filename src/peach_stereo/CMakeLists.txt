cmake_minimum_required(VERSION 3.16)
project(peach_stereo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

# camport4 SDK 随 percipio_camera 源码树 vendored（未安装头文件），跨包引用源码路径；
# 运行时 .so 由 install/percipio_camera/lib 经 LD_LIBRARY_PATH 解析
set(TYCAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../percipio_camera/camport4)
if(NOT EXISTS ${TYCAM_DIR}/include/TYApi.h)
  message(FATAL_ERROR "camport4 SDK not found at ${TYCAM_DIR}（需先检出 percipio_camera）")
endif()
find_library(LIB_TYCAM tycam PATHS ${TYCAM_DIR}/lib/linux/lib_x64 NO_DEFAULT_PATH REQUIRED)
find_library(LIB_TYIMGPROC tyimgproc PATHS ${TYCAM_DIR}/lib/linux/lib_x64 NO_DEFAULT_PATH REQUIRED)

add_executable(stereo_camera_node src/stereo_camera_node.cpp)
target_include_directories(stereo_camera_node PRIVATE ${TYCAM_DIR}/include)
target_link_libraries(stereo_camera_node ${LIB_TYCAM} ${LIB_TYIMGPROC} ${OpenCV_LIBS})
ament_target_dependencies(stereo_camera_node
  rclcpp sensor_msgs std_msgs cv_bridge image_transport tf2_ros)

install(TARGETS stereo_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
