#!/bin/bash
# PS800-E1 评测工具集构建脚本（不入 colcon 构建，独立 g++ 编译）
# 用法: bash build.sh   （产物 ./bin/）
set -e
cd "$(dirname "$0")"
PKG_ROOT="$(cd ../.. && pwd)"                 # src/percipio_camera
SDK_INC="$PKG_ROOT/camport4/include"
PKG_INC="$PKG_ROOT/include"
SDK_LIB="$PKG_ROOT/camport4/lib/linux/lib_x64"
THREAD_SRC="$PKG_ROOT/src/TYThread.cpp"

mkdir -p bin
build() {  # build <src> <out> [extra_libs]
  g++ -O3 -std=c++17 -I"$SDK_INC" -I"$PKG_INC" "$1" "$THREAD_SRC" \
      -L"$SDK_LIB" -ltycam -ltyimgproc -lpthread ${3:-} -o "bin/$2"
}
build feature_dump.cpp  feature_dump
build write_test.cpp    write_test
build laser_check.cpp   laser_check
build raw_ir_test.cpp   raw_ir_test
build stereo_grab.cpp   stereo_grab
build stereo_live.cpp   stereo_live "$(pkg-config --cflags --libs opencv4)"

echo "built: $(ls bin | tr '\n' ' ')"
echo "run with: LD_LIBRARY_PATH=$SDK_LIB ./bin/<tool>"
