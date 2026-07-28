# AUBO 旧 SDK v1.3.1 探针独立构建（不参与 colcon）。
# 链接仓库内 vendored 二进制 SDK（aubo_e5_hardware/vendor），无源码构建。
#
# 构建:
#   cmake -S diagnostics -B diagnostics/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build diagnostics/build -j
# 通常直接走 ./diagnostics/run_tests.sh（会先构建再按序运行探针）。
cmake_minimum_required(VERSION 3.16)
project(aubo_sdk_diagnostics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vendored SDK 位置（相对本目录）
set(AUBO_VENDOR ${CMAKE_CURRENT_SOURCE_DIR}/../src/aubo_e5_hardware/vendor)
set(AUBO_SDK_INCLUDE ${AUBO_VENDOR}/aubo_driver)
set(AUBO_SDK_LIBRARY ${AUBO_VENDOR}/lib64/aubocontroller/libauborobotcontroller.so)
if(NOT EXISTS ${AUBO_SDK_LIBRARY})
  message(FATAL_ERROR "vendored AUBO SDK 缺失: ${AUBO_SDK_LIBRARY}")
endif()

# 与 aubo_e5_hardware/CMakeLists.txt 相同的 RPATH 坑：必须 DT_RPATH
# （--disable-new-dtags），否则传递依赖 libprotobuf.so.9 / libconfig /
# liblog4cplus 解析不到（RUNPATH 不沿依赖链继承）。
set(AUBO_VENDOR_LIBDIRS
  ${AUBO_VENDOR}/lib64/aubocontroller
  ${AUBO_VENDOR}/lib64/protobuf
  ${AUBO_VENDOR}/lib64/config
  ${AUBO_VENDOR}/lib64/log4cplus)

foreach(probe IN ITEMS
    aubo_sdk_abi_probe
    aubo_sdk_runtime_probe
    aubo_sdk_push_probe
    aubo_sdk_tcp2can_probe)
  add_executable(${probe} ${probe}.cpp)
  target_include_directories(${probe} PRIVATE ${AUBO_SDK_INCLUDE})
  target_link_libraries(${probe} ${AUBO_SDK_LIBRARY} pthread)
  set_target_properties(${probe} PROPERTIES
    LINK_OPTIONS "-Wl,--disable-new-dtags"
    BUILD_RPATH "${AUBO_VENDOR_LIBDIRS}"
    INSTALL_RPATH "${AUBO_VENDOR_LIBDIRS}")
endforeach()
