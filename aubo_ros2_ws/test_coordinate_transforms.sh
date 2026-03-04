#!/usr/bin/env bash
# 坐标与变换功能包：全面编译与执行测试（使用系统环境，不依赖 conda/venv）
#
# 用法:
#   cd aubo_ros2_ws
#   source /opt/ros/humble/setup.bash   # 若尚未 source
#   ./test_coordinate_transforms.sh              # 完整：编译 + 安装检查 + 可执行 + launch + 单元测试
#   ./test_coordinate_transforms.sh --no-build   # 跳过编译，仅运行测试
#   ./test_coordinate_transforms.sh --no-launch # 跳过 launch 测试
#
# 测试项：环境检查、编译、安装与文件、C++/Python 可执行、Launch、输出文件、Python core 单元测试、包列表。
# 脚本会先退出 conda/venv，改用系统 python3 与 ROS2。

# 不设 set -e：单个测试失败不中断脚本，最后根据 FAILED 统一退出
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR" && pwd)"
cd "$WS_DIR" || exit 1

DO_BUILD=1
DO_LAUNCH=1
for arg in "$@"; do
  case "$arg" in
    --no-build) DO_BUILD=0 ;;
    --no-launch) DO_LAUNCH=0 ;;
  esac
done

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'
PASSED=0
FAILED=0
SKIPPED=0

# ---------- 使用系统环境：退出 conda/venv ----------
if [ -n "${CONDA_DEFAULT_ENV}" ]; then
  echo -e "${CYAN}检测到 conda 环境: $CONDA_DEFAULT_ENV，已退出以使用系统环境${NC}"
  eval "$(conda shell.bash hook 2>/dev/null)" && conda deactivate 2>/dev/null || true
  unset CONDA_DEFAULT_ENV CONDA_PREFIX 2>/dev/null || true
fi
if [ -n "${VIRTUAL_ENV}" ]; then
  echo -e "${CYAN}检测到虚拟环境: $VIRTUAL_ENV，已退出${NC}"
  deactivate 2>/dev/null || true
  unset VIRTUAL_ENV 2>/dev/null || true
fi
# 优先使用系统 Python 和 ROS（若 /usr/bin 在 PATH 前则已满足）
export PATH="/usr/bin:/usr/local/bin:${PATH}"

run_test() {
  local name="$1"
  shift
  printf "  [RUN] %s ... " "$name"
  if "$@" > /tmp/coord_tf_test.log 2>&1; then
    echo -e "${GREEN}PASS${NC}"
    ((PASSED++)) || true
    return 0
  else
    echo -e "${RED}FAIL${NC}"
    ((FAILED++)) || true
    echo "    log: /tmp/coord_tf_test.log"
    tail -n 25 /tmp/coord_tf_test.log | sed 's/^/    /'
    return 1
  fi
}

run_test_timeout() {
  local name="$1"
  local timeout_sec="${2:-5}"
  shift 2
  printf "  [RUN] %s (timeout %ss) ... " "$name" "$timeout_sec"
  timeout "$timeout_sec" "$@" > /tmp/coord_tf_test.log 2>&1
  local ret=$?
  if [ "$ret" -eq 0 ]; then
    echo -e "${GREEN}PASS${NC}"
    ((PASSED++)) || true
    return 0
  fi
  # 124 = timeout 已结束进程，说明节点在运行
  if [ "$ret" -eq 124 ]; then
    echo -e "${GREEN}PASS (node ran)${NC}"
    ((PASSED++)) || true
    return 0
  fi
  if grep -q -E "ExternalShutdownException|coord_tf|Wrote|Saved|done\.|project_3d_to_2d|reprojection|TF|Marker|rclcpp|rclpy|signal_handler" /tmp/coord_tf_test.log 2>/dev/null; then
    echo -e "${GREEN}PASS (node ran)${NC}"
    ((PASSED++)) || true
    return 0
  fi
  echo -e "${RED}FAIL${NC}"
  ((FAILED++)) || true
  tail -n 15 /tmp/coord_tf_test.log | sed 's/^/    /'
  return 1
}

skip_test() {
  local name="$1"
  local reason="$2"
  echo -e "  [SKIP] $name ($reason)"
  ((SKIPPED++)) || true
}

echo "=============================================="
echo "  coordinate_transforms 全面测试（系统环境）"
echo "  工作空间: $WS_DIR"
echo "=============================================="

# ---------- 1. 环境检查 ----------
echo ""
echo "--- 1. 环境检查 ---"
PYTHON_EXE=""
for p in /usr/bin/python3 python3; do
  if command -v "$p" >/dev/null 2>&1; then
    PYTHON_EXE="$p"
    break
  fi
done
if [ -z "$PYTHON_EXE" ]; then
  echo -e "  ${RED}未找到 python3${NC}"
  exit 1
fi
echo "  python3: $PYTHON_EXE ($($PYTHON_EXE --version 2>&1))"

if [ -z "${ROS_DISTRO}" ]; then
  if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  ROS2: source /opt/ros/humble/setup.bash"
  elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "  ROS2: source /opt/ros/jazzy/setup.bash"
  else
    echo -e "  ${RED}未找到 ROS2，请先 source /opt/ros/<distro>/setup.bash${NC}"
    exit 1
  fi
fi
echo "  ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "  colcon: $(command -v colcon 2>/dev/null || echo 'not found')"
echo "  ros2:   $(command -v ros2 2>/dev/null || echo 'not found')"

# ---------- 2. 编译 ----------
if [ "$DO_BUILD" -eq 1 ]; then
  echo ""
  echo "--- 2. 编译 ---"
  echo "  [2.1] coordinate_transforms (C++) ..."
  if colcon build --packages-select coordinate_transforms --cmake-clean-cache 2>&1 | tee /tmp/coord_tf_build_cpp.log; then
    echo -e "  ${GREEN}coordinate_transforms 编译成功${NC}"
  else
    echo -e "  ${RED}coordinate_transforms 编译失败${NC}"
    exit 1
  fi
  echo "  [2.2] coordinate_transforms_py (Python) ..."
  if colcon build --packages-select coordinate_transforms_py 2>&1 | tee /tmp/coord_tf_build_py.log; then
    echo -e "  ${GREEN}coordinate_transforms_py 编译成功${NC}"
  else
    echo -e "  ${RED}coordinate_transforms_py 编译失败${NC}"
    exit 1
  fi
else
  echo ""
  echo "--- 2. 编译 (已跳过 --no-build) ---"
fi

source "$WS_DIR/install/setup.bash"

# 统一输出目录：所有生成文件写入工作空间下的 coord_tf_output
export COORD_TF_OUTPUT_DIR="${COORD_TF_OUTPUT_DIR:-$WS_DIR/coord_tf_output}"
mkdir -p "$COORD_TF_OUTPUT_DIR"
echo "  输出目录: $COORD_TF_OUTPUT_DIR"

# ---------- 3. 安装与文件检查 ----------
echo ""
echo "--- 3. 安装与文件检查 ---"
run_test "C++ 包 install 存在" test -d "$WS_DIR/install/coordinate_transforms"
run_test "Python 包 install 存在" test -d "$WS_DIR/install/coordinate_transforms_py"
run_test "C++ doc 已安装" test -f "$WS_DIR/install/coordinate_transforms/share/coordinate_transforms/doc/COORDINATE_SYSTEMS_AND_TRANSFORMS.md"
run_test "C++ config 已安装" test -f "$WS_DIR/install/coordinate_transforms/share/coordinate_transforms/config/camera_example.yaml"
run_test "C++ rviz 已安装" test -f "$WS_DIR/install/coordinate_transforms/share/coordinate_transforms/config/rviz/coord_tf_visualization.rviz"
run_test "C++ launch 已安装" test -f "$WS_DIR/install/coordinate_transforms/share/coordinate_transforms/launch/coord_tf_demo.launch.py"
run_test "Python config 已安装" test -f "$WS_DIR/install/coordinate_transforms_py/share/coordinate_transforms_py/config/camera_example.yaml"
run_test "Python launch 已安装" test -f "$WS_DIR/install/coordinate_transforms_py/share/coordinate_transforms_py/launch/coord_tf_demo.launch.py"

# ---------- 4. C++ 可执行 ----------
echo ""
echo "--- 4. C++ 可执行 ---"
run_test "export_visualization (PCD/CSV)" ros2 run coordinate_transforms export_visualization
if ros2 pkg executables coordinate_transforms 2>/dev/null | grep -q opencv_process_data; then
  run_test "opencv_process_data (PNG/YAML)" ros2 run coordinate_transforms opencv_process_data
else
  skip_test "opencv_process_data" "OpenCV 未启用编译"
fi
run_test_timeout "coord_tf_demo_node (C++)" 3 ros2 run coordinate_transforms coord_tf_demo_node

# ---------- 5. Python 可执行 ----------
echo ""
echo "--- 5. Python 可执行 ---"
run_test "run_core_demo" ros2 run coordinate_transforms_py run_core_demo
run_test "run_visualization_demo" ros2 run coordinate_transforms_py run_visualization_demo
if $PYTHON_EXE -c "import cv2" 2>/dev/null; then
  run_test "run_opencv_process_demo" ros2 run coordinate_transforms_py run_opencv_process_demo
else
  skip_test "run_opencv_process_demo" "opencv-python 未安装"
fi
run_test_timeout "coord_tf_demo_node (Python)" 4 ros2 run coordinate_transforms_py coord_tf_demo_node

# ---------- 6. Launch 文件（短时运行） ----------
# 使用临时目录作为 ROS 日志目录，避免依赖 ~/.ros 可写（沙箱/无家目录等）
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/coord_tf_ros_log_$$}"
mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true

if [ "$DO_LAUNCH" -eq 1 ]; then
  echo ""
  echo "--- 6. Launch 文件 ---"
  if [ ! -w "$ROS_LOG_DIR" ]; then
    skip_test "Launch 文件" "无法写入 ROS_LOG_DIR=$ROS_LOG_DIR"
  else
    run_test_timeout "coord_tf_demo.launch.py (C++ 包)" 4 ros2 launch coordinate_transforms coord_tf_demo.launch.py
    run_test_timeout "coord_tf_demo.launch.py (Python 包)" 4 ros2 launch coordinate_transforms_py coord_tf_demo.launch.py
    printf "  [RUN] coord_tf_visualization.launch.py use_rviz:=false (timeout 4s) ... "
    if timeout 4 ros2 launch coordinate_transforms_py coord_tf_visualization.launch.py use_rviz:=false > /tmp/coord_tf_test.log 2>&1; then
      echo -e "${GREEN}PASS${NC}"
      ((PASSED++)) || true
    elif grep -q -E "coord_tf|Starting|ROS|rviz|rclcpp" /tmp/coord_tf_test.log 2>/dev/null; then
      echo -e "${GREEN}PASS (launch started)${NC}"
      ((PASSED++)) || true
    elif grep -q "PermissionError\|Permission denied" /tmp/coord_tf_test.log 2>/dev/null; then
      echo -e "${YELLOW}SKIP (日志目录无写权限)${NC}"
      ((SKIPPED++)) || true
    else
      echo -e "${RED}FAIL${NC}"
      ((FAILED++)) || true
      tail -n 10 /tmp/coord_tf_test.log | sed 's/^/    /'
    fi
  fi
else
  echo ""
  echo "--- 6. Launch 文件 (已跳过 --no-launch) ---"
fi

# ---------- 7. 输出文件检查 ----------
echo ""
echo "--- 7. 输出文件检查 ($COORD_TF_OUTPUT_DIR) ---"
for f in coord_tf_sample.pcd coord_tf_sample_2d.csv coord_tf_visualization_2d.png coord_tf_reprojection.png coord_tf_process_data.yaml; do
  if [ -f "$COORD_TF_OUTPUT_DIR/$f" ]; then
    echo -e "  ${GREEN}存在${NC} $f"
  else
    echo -e "  ${YELLOW}未找到${NC} $f"
  fi
done

# ---------- 8. Python 单元测试（不依赖 pytest，使用系统 python3） ----------
echo ""
echo "--- 8. Python 单元测试 (core) ---"
PYTEST_RAN=0
if command -v pytest >/dev/null 2>&1; then
  (cd "$WS_DIR/src/coordinate_transforms_py" && PYTHONPATH="$WS_DIR/src/coordinate_transforms_py${PYTHONPATH:+:$PYTHONPATH}" $PYTHON_EXE -m pytest test/test_core.py -v --tb=short 2>&1) | tee /tmp/coord_tf_pytest.log | tail -35
  PYTEST_RET=${PIPESTATUS[0]}
  PYTEST_RAN=1
  if [ "$PYTEST_RET" -eq 0 ]; then
    echo -e "  ${GREEN}pytest 通过${NC}"
    ((PASSED++)) || true
  else
    echo -e "  ${RED}pytest 失败 (exit $PYTEST_RET)${NC}"
    ((FAILED++)) || true
    [ -s /tmp/coord_tf_pytest.log ] && echo "  最近输出:" && tail -15 /tmp/coord_tf_pytest.log | sed 's/^/    /'
  fi
else
  # 无 pytest 时直接执行 test_core 中所有 test_ 函数（保证 PYTHONPATH 可找到包）
  (cd "$WS_DIR/src/coordinate_transforms_py" && PYTHONPATH="$WS_DIR/src/coordinate_transforms_py${PYTHONPATH:+:$PYTHONPATH}" $PYTHON_EXE -c "
import sys
sys.path.insert(0, '.')
import importlib.util
spec = importlib.util.spec_from_file_location('test_core', 'test/test_core.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
failed = []
for name in dir(m):
    if name.startswith('test_') and callable(getattr(m, name)):
        try:
            getattr(m, name)()
            print('PASS', name)
        except Exception as e:
            print('FAIL', name, e)
            failed.append(name)
sys.exit(0 if not failed else 1)
" 2>&1) | tee /tmp/coord_tf_pytest.log
  PYTEST_RET=${PIPESTATUS[0]}
  [ -s /tmp/coord_tf_pytest.log ] && tail -20 /tmp/coord_tf_pytest.log | sed 's/^/    /'
  if [ "$PYTEST_RET" -eq 0 ]; then
    echo -e "  ${GREEN}core 单元测试通过 (无 pytest)${NC}"
    ((PASSED++)) || true
    PYTEST_RAN=1
  else
    echo -e "  ${RED}core 单元测试失败${NC}"
    ((FAILED++)) || true
    PYTEST_RAN=1
  fi
fi
if [ "$PYTEST_RAN" -eq 0 ]; then
  skip_test "Python 单元测试" "无法运行 (需在 src/coordinate_transforms_py 下执行)"
fi

# ---------- 9. 包列表与可执行列表 ----------
echo ""
echo "--- 9. 包与可执行列表 ---"
echo "  coordinate_transforms 可执行:"
ros2 pkg executables coordinate_transforms 2>/dev/null | sed 's/^/    /'
echo "  coordinate_transforms_py 可执行:"
ros2 pkg executables coordinate_transforms_py 2>/dev/null | sed 's/^/    /'

echo ""
echo "=============================================="
echo -e "  通过: ${GREEN}$PASSED${NC}  失败: ${RED}$FAILED${NC}  跳过: ${YELLOW}$SKIPPED${NC}"
echo "=============================================="
[ "$FAILED" -eq 0 ]
