#!/usr/bin/env bash
# =============================================================================
# env_bootstrap.sh — aubo_e5_jazzy_ws 全新环境 自检 + 部署
# =============================================================================
# 用法：
#   scripts/env_bootstrap.sh check    # 只自检，零改动；退出码 = 缺失项数（0=全绿）
#   scripts/env_bootstrap.sh install  # 安装缺失项（幂等：已有的一律跳过）
#   scripts/env_bootstrap.sh all      # install + check
#   SMOKE=1 scripts/env_bootstrap.sh all   # 末尾追加一次 mock 冒烟（五节点 Active）
#
# 覆盖面（与 AGENTS.md「依赖分层 venv-first」、requirements.txt、testing.md §1 同源）：
#   1. 系统：Ubuntu 24.04；build-essential / cmake / git / nlohmann-json3-dev / libeigen3-dev / udev
#   2. ROS 2 Jazzy apt 源 + ros-jazzy 包清单（moveit 2.12.4 全家 / MTC 0.1.8 / ros2_control / rosbag2）
#   3. aubo_py3.12 venv（--system-site-packages）+ requirements.txt 钉版 + rembg --no-deps 特例
#   4. IMU udev 规则（CH340/CH343 → /dev/imu）+ dialout 组
#   5. SMOKE=1 时：mock 起 harvest_system，验五节点 Active
#
# 约定：
#   * 清单即事实源，不依赖 rosdep；改依赖必须四处同步——package.xml、
#     requirements.txt、本脚本清单、docs/testing.md §1。
#   * 真机运动 / SetIO 授权不在本脚本范围；本脚本不 rosdep init。
#   * torch 按requirements.txt 钉版走 PyPI 默认轮（Linux 即 CUDA 版，约 2GB+）；
#     纯 CPU 机器自行改用 cpu index-url，勿降 numpy。
#   * 中国大陆网络可用 ROS_APT_DEB_URL 环境变量指向镜像 deb 覆盖默认官方源。
# =============================================================================

set -o pipefail

MODE="${1:-check}"
SMOKE="${SMOKE:-0}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$WS_DIR/aubo_py3.12"
UDEV_SRC="$WS_DIR/src/serial_imu/udev/99-imu-usb-serial.rules"
UDEV_DST="/etc/udev/rules.d/99-imu-usb-serial.rules"
FAIL=0
SUDO=""
if [ "$(id -u)" -ne 0 ] && command -v sudo >/dev/null 2>&1; then SUDO="sudo"; fi

log() { printf '\033[1;34m[bootstrap]\033[0m %s\n' "$*"; }
ok() { printf '  \033[32m✓\033[0m %s\n' "$*"; }
miss() { printf '  \033[33m✗ %s\033[0m\n' "$*"; FAIL=$((FAIL + 1)); }
die() { printf '\033[1;31m[bootstrap] 致命：%s\033[0m\n' "$*"; exit 1; }

# ---------------- 清单（事实源） ----------------
# 系统 apt
SYS_APT_PKGS=(
  build-essential cmake git curl ca-certificates
  python3-pip python3.12-venv python3-colcon-common-extensions python3-pytest
  nlohmann-json3-dev libeigen3-dev udev software-properties-common
)

# ROS Jazzy apt（rosdep 键的手工映射；ros-base 已含 rclcpp/rclpy/launch/常用消息）
ROS_APT_PKGS=(
  ros-jazzy-ros-base
  # 构建 / lint 门（colcon test 过门用；testing.md §0）
  ros-jazzy-ament-cmake ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common
  ros-jazzy-ament-cmake-pytest ros-jazzy-ament-cmake-copyright ros-jazzy-ament-cmake-cpplint
  ros-jazzy-ament-cmake-lint-cmake ros-jazzy-ament-cmake-uncrustify ros-jazzy-ament-cmake-xmllint
  ros-jazzy-ament-flake8 ros-jazzy-ament-pep257
  # ros2_control 全家（controller_manager/hardware_interface/JTC/JSB/realtime_tools）
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers
  # 常用运行件
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
  ros-jazzy-rviz2 ros-jazzy-imu-tools ros-jazzy-cv-bridge ros-jazzy-diagnostic-updater
  ros-jazzy-camera-calibration-parsers ros-jazzy-pcl-conversions ros-jazzy-generate-parameter-library
  ros-jazzy-urdf ros-jazzy-tf2 ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-ros
  ros-jazzy-message-filters ros-jazzy-sensor-msgs-py ros-jazzy-vision-msgs
  ros-jazzy-example-interfaces ros-jazzy-launch-param-builder
  # 过程录制 / 回读（observability recorder/bag_reader 走 rosbag2_py + mcap）
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap
  # MoveIt（2026-09-15 起全 apt：moveit 2.12.4 + MTC 0.1.8，勿再源码构建）
  ros-jazzy-moveit ros-jazzy-moveit-servo ros-jazzy-moveit-ros-perception
  ros-jazzy-pilz-industrial-motion-planner ros-jazzy-moveit-planners-stomp
  ros-jazzy-moveit-task-constructor-core ros-jazzy-moveit-task-constructor-msgs
  ros-jazzy-moveit-task-constructor-capabilities ros-jazzy-moveit-configs-utils
)

# venv 内应可导入的第三方库（requirements.txt 钉版 + ROS apt 注入的 serial/cv2 桥）
VENV_IMPORTS=(numpy scipy yaml cv2 torch torchvision open3d fastapi uvicorn httpx onnxruntime rembg serial PIL matplotlib ultralytics)
NUMPY_PIN="1.26.4"
REMBG_PIN="2.0.61"

dpkg_has() { dpkg -s "$1" >/dev/null 2>&1; }

# ---------------- ROS apt 源（缺 /opt/ros/jazzy 时才动） ----------------
install_ros_source() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    ok "ROS 2 Jazzy 已在 /opt/ros"
    return 0
  fi
  log "未检测到 /opt/ros/jazzy，配置 ROS 2 apt 源（官方 ros-apt-source 流程）"
  [ -f /etc/os-release ] || die "读不到 /etc/os-release"
  . /etc/os-release
  [ "$VERSION_CODENAME" = "noble" ] || die "需要 Ubuntu 24.04 (noble)，当前 $PRETTY_NAME"
  $SUDO apt-get update -qq || die "apt-get update 失败，先手工修网络/源"
  $SUDO apt-get install -y -qq software-properties-common curl ca-certificates \
    >/dev/null || die "基础工具安装失败"
  $SUDO add-apt-repository -y universe >/dev/null || die "universe 仓库启用失败"
  local url="${ROS_APT_DEB_URL:-}"
  if [ -z "$url" ]; then
    local tag
    tag="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -oP '"tag_name": "\K[^"]+' )" || die "取 ros-apt-source 最新版本号失败（可用 ROS_APT_DEB_URL 直指镜像 deb）"
    url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${tag}/ros2-apt-source_${tag}.${VERSION_CODENAME}_all.deb"
  fi
  curl -fsSL -o /tmp/ros2-apt-source.deb "$url" || die "下载 ros-apt-source deb 失败：$url"
  $SUDO dpkg -i /tmp/ros2-apt-source.deb || die "安装 ros-apt-source 失败"
  $SUDO apt-get update || die "apt-get update 失败"
  ok "ROS 2 apt 源已配置"
}

# ---------------- check ----------------
check_os() {
  log "① 系统与 ROS"
  if [ -f /etc/os-release ]; then
    . /etc/os-release
    case "$VERSION_ID" in
      24.*) ok "OS：$PRETTY_NAME" ;;
      *) miss "OS 需要 Ubuntu 24.04，当前 $PRETTY_NAME" ;;
    esac
  else
    miss "读不到 /etc/os-release"
  fi
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    ok "/opt/ros/jazzy 存在"
  else
    miss "/opt/ros/jazzy 不存在（ROS 2 Jazzy 未装）"
  fi
  if command -v colcon >/dev/null 2>&1; then
    ok "colcon：$(command -v colcon)"
  else
    miss "colcon 不在 PATH（python3-colcon-common-extensions）"
  fi
}

check_apt() {
  log "② 系统 apt 包"
  local p
  for p in "${SYS_APT_PKGS[@]}"; do
    dpkg_has "$p" && ok "$p" || miss "$p 未安装"
  done
  log "③ ROS Jazzy apt 包（${#ROS_APT_PKGS[@]} 个）"
  local p
  for p in "${ROS_APT_PKGS[@]}"; do
    if [ "$p" = ros-jazzy-ros-base ] && [ -f /opt/ros/jazzy/setup.bash ]; then
      ok "$p（元包，功能已由 /opt/ros/jazzy 组件提供）"
    elif dpkg_has "$p"; then
      ok "$p"
    else
      miss "$p 未安装"
    fi
  done
}

check_venv() {
  log "④ venv（aubo_py3.12）"
  if [ ! -x "$VENV/bin/python" ]; then
    miss "venv 不存在（install 模式将创建）"
    return
  fi
  ok "venv python：$("$VENV/bin/python" -V 2>&1)"
  if grep -q 'include-system-site-packages = true' "$VENV/pyvenv.cfg" 2>/dev/null; then
    ok "include-system-site-packages = true（serial/cv2 桥来自 apt 注入）"
  else
    miss "pyvenv.cfg 缺 include-system-site-packages = true（venv 须带 --system-site-packages 重建）"
  fi
  local lib
  for lib in "${VENV_IMPORTS[@]}"; do
    if "$VENV/bin/python" -c "import $lib" >/dev/null 2>&1; then
      ok "import $lib"
    else
      miss "venv 无法 import $lib"
    fi
  done
  if "$VENV/bin/python" -c "import numpy; assert numpy.__version__ == '$NUMPY_PIN', numpy.__version__" >/dev/null 2>&1; then
    ok "numpy == $NUMPY_PIN（cv_bridge ABI 约束）"
  else
    miss "numpy != $NUMPY_PIN（会被 cv_bridge 打穿，见 requirements.txt 头注）"
  fi
}

check_udev() {
  log "⑤ IMU udev + 权限"
  if [ ! -f "$UDEV_SRC" ]; then
    miss "仓库内规则文件缺失：$UDEV_SRC"
  elif [ -f "$UDEV_DST" ] && diff -q "$UDEV_SRC" "$UDEV_DST" >/dev/null 2>&1; then
    ok "udev 规则已装（$UDEV_DST）"
  else
    miss "udev 规则未装或与仓库不一致（install 模式覆盖）"
  fi
  if id -nG | tr ' ' '\n' | grep -qx dialout; then
    ok "用户已在 dialout 组"
  else
    miss "用户不在 dialout 组（串口打不开；install 后需重新登录）"
  fi
}

run_smoke() {
  log "⑦ mock 冒烟（SMOKE=1，约 2 分钟）"
  set +u
  source /opt/ros/jazzy/setup.bash || { miss "source Jazzy 失败"; return; }
  export PYTHONPATH="$VENV/lib/python3.12/site-packages:${PYTHONPATH:-}"
  source "$WS_DIR/install/setup.bash" || { miss "source 本仓 install 失败（先 colcon build）"; return; }
  export QT_QPA_PLATFORM=offscreen
  pgrep -f 'peach_scene_perception_node|peach_arm|move_group' >/dev/null 2>&1 \
    && { miss "已有栈实例在跑，拒起（先 pgrep -af 清理）"; return; }
  if [ ! -d "$WS_DIR/install" ]; then
    miss "本仓未构建（先在 $WS_DIR 跑 colcon build）"
    return
  fi
  ros2 launch peach_supervisor harvest_system.launch.py hardware_mode:=mock camera_enabled:=false \
    > /tmp/env_bootstrap_smoke.log 2>&1 &
  local lp=$! i st=0
  for i in $(seq 1 45); do
    sleep 2
    [ "$(ros2 lifecycle get /peach_arm 2>/dev/null)" = "active [3]" ] && st=1 && break
  done
  if [ "$st" = 1 ]; then
    ok "peach_arm active [3]（全链五节点托起，详见 /tmp/env_bootstrap_smoke.log）"
  else
    miss "90s 内 manipulation 未 Active，看 /tmp/env_bootstrap_smoke.log"
  fi
  kill -INT "$lp" 2>/dev/null; sleep 10; kill -9 "$lp" 2>/dev/null
  pkill -9 -f 'harvest_system.launch' 2>/dev/null; sleep 2
  pkill -9 -f 'robot_state_publisher|ros2_control_node|move_group|controller_manager|joint_state_publisher|extrinsics_publisher|serial_imu|peach_' 2>/dev/null
  return 0
}

# ---------------- install ----------------
install_pkgs() {
  install_ros_source
  local missing=() p
  for p in "${SYS_APT_PKGS[@]}"; do dpkg_has "$p" || missing+=("$p"); done
  for p in "${ROS_APT_PKGS[@]}"; do dpkg_has "$p" || missing+=("$p"); done
  if [ "${#missing[@]}" -gt 0 ]; then
    log "apt 安装缺失项（${#missing[@]} 个）"
    $SUDO apt-get install -y "${missing[@]}" || die "apt 安装失败：${missing[*]}"
    ok "apt 缺失项已装"
  else
    ok "apt 清单全齐，无需安装"
  fi
}

install_venv() {
  if [ ! -x "$VENV/bin/python" ]; then
    log "创建 venv（--system-site-packages，与本机历史配置一致）"
    python3 -m venv --system-site-packages "$VENV" || die "venv 创建失败（python3.12-venv 装了吗）"
    ok "venv 已创建：$VENV"
  else
    ok "venv 已存在"
  fi
  # 幂等短路：导入全绿且 numpy 达钉版则跳过重装（避免 2GB 级 torch 重复下载）
  local need=0 lib
  for lib in "${VENV_IMPORTS[@]}"; do
    "$VENV/bin/python" -c "import $lib" >/dev/null 2>&1 || need=1
  done
  if [ "$need" = 0 ] && \
    "$VENV/bin/python" -c "import numpy; assert numpy.__version__ == '$NUMPY_PIN'" >/dev/null 2>&1; then
    ok "venv 依赖导入全绿且 numpy == $NUMPY_PIN，跳过 pip 重装"
    return 0
  fi
  log "pip 按 requirements.txt 钉版安装（--ignore-installed 压过 apt 同名包）"
  "$VENV/bin/pip" install --ignore-installed -r "$WS_DIR/requirements.txt" \
    || die "requirements.txt 安装失败（大包 torch/open3d 注意网络与代理）"
  if ! "$VENV/bin/python" -c 'import rembg' >/dev/null 2>&1; then
    log "rembg 特例：--no-deps 安装（其元数据会把 numpy 拉到 2.x）"
    "$VENV/bin/pip" install --no-deps "rembg==$REMBG_PIN" || die "rembg 安装失败"
  fi
  ok "venv 依赖就绪"
}

install_udev() {
  if [ ! -f "$UDEV_SRC" ]; then
    die "仓库内规则文件缺失：$UDEV_SRC"
  fi
  if [ ! -f "$UDEV_DST" ] || ! diff -q "$UDEV_SRC" "$UDEV_DST" >/dev/null 2>&1; then
    $SUDO cp "$UDEV_SRC" "$UDEV_DST" || die "udev 规则拷贝失败"
    $SUDO udevadm control --reload-rules || die "udevadm reload 失败"
    ok "udev 规则已安装并重载（插拔 IMU 后 /dev/imu 出现）"
  else
    ok "udev 规则已是最新"
  fi
  if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
    $SUDO usermod -aG dialout "$(id -un)" || die "加入 dialout 组失败"
    ok "已加入 dialout 组——须重新登录（或 newgrp dialout）后生效"
  else
    ok "dialout 组已具备"
  fi
}

# ---------------- 主流程 ----------------
case "$MODE" in
  check)
    check_os; check_apt; check_venv; check_udev
    [ "$SMOKE" = 1 ] && run_smoke
    if [ "$FAIL" -eq 0 ]; then
      log "自检全绿 ✓"
    else
      log "自检完成：$FAIL 项缺失（install 模式可补齐）"
    fi
    exit "$FAIL"
    ;;
  install)
    install_pkgs; install_venv; install_udev
    log "部署完成。下一步：cd $WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    log "验收入口：docs/testing.md §1（冒烟）/ §4（真机，须另行授权）"
    ;;
  all)
    install_pkgs; install_venv; install_udev
    check_os; check_apt; check_venv; check_udev
    [ "$SMOKE" = 1 ] && run_smoke
    if [ "$FAIL" -eq 0 ]; then log "部署 + 自检全绿 ✓"; else log "部署完成，自检仍有 $FAIL 项缺失"; fi
    exit "$FAIL"
    ;;
  *)
    sed -n '2,25p' "${BASH_SOURCE[0]}"; exit 2
    ;;
esac
