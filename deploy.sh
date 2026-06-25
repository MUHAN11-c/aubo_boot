#!/bin/bash
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="humble"
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'

ok()   { echo -e "${GREEN}  ✓${NC} $1"; }
skip() { echo -e "${YELLOW}  ⏭${NC} $1"; }

echo -e "${GREEN}== aubo_boot 环境部署 ==${NC}"

# ── 系统包 ──────────────────────────────────────────────
echo -e "\n${BLUE}[1] 系统包${NC}"

SYSTEM_PKGS="
    curl wget gpg build-essential git vim htop btop tmux zsh
    unzip p7zip-full net-tools openssh-server ca-certificates
    flameshot ripgrep fd-find fzf tldr tree jq
    terminator python3-pip python3-venv
    python3-colcon-common-extensions python3-rosdep python3-vcstool
    ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-moveit
    ros-${ROS_DISTRO}-rosbridge-suite ros-${ROS_DISTRO}-tf2-web-republisher
    ros-${ROS_DISTRO}-web-video-server ros-${ROS_DISTRO}-image-transport-plugins
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
    ros-${ROS_DISTRO}-foxglove-bridge
    libopencv-dev libeigen3-dev ninja-build
"

MISSING=""
for p in $SYSTEM_PKGS; do
    dpkg -l "$p" 2>/dev/null | grep -q '^ii' || MISSING="$MISSING $p"
done

if [ -z "$MISSING" ]; then
    skip "系统包已全部安装"
else
    echo "$MISSING" | xargs sudo apt install -y
    ok "系统包安装完成"
fi

# ── CUDA Toolkit ─────────────────────────────────────────
echo -e "\n${BLUE}[2] CUDA Toolkit${NC}"

if command -v nvidia-smi &>/dev/null; then
    if [ -x /usr/local/cuda/bin/nvcc ] || command -v nvcc &>/dev/null; then
        NVCC=/usr/local/cuda/bin/nvcc
        command -v nvcc &>/dev/null && NVCC=nvcc
        skip "CUDA Toolkit 已安装"
    else
        wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -O /tmp/cuda-keyring.deb
        sudo dpkg -i /tmp/cuda-keyring.deb 2>/dev/null || true
        sudo apt update -qq
        sudo apt install -y cuda-toolkit
        ok "CUDA Toolkit 安装完成"
    fi
else
    skip "无 GPU，跳过 CUDA Toolkit"
fi

# ── rosdep ──────────────────────────────────────────────
echo -e "\n${BLUE}[3] rosdep 依赖${NC}"
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths "${PROJECT_ROOT}/aubo_ros2_ws/src" --ignore-src -r -y 2>/dev/null || true
ok "rosdep 完成"

# ── Python 依赖 ─────────────────────────────────────────
echo -e "\n${BLUE}[4] Python 依赖 (pip3 install --user)${NC}"

pip3 install --user --upgrade pip 'packaging>=23' 'setuptools<80,>=70' -q
pip3 install --user \
    httpx websockets Flask fastapi uvicorn pydantic \
    'numpy==1.23.4' opencv-python 'scipy>=1.10' \
    Pillow PyYAML trimesh tqdm matplotlib

if command -v nvidia-smi &>/dev/null; then
    pip3 install --user torch torchvision torchaudio \
        --index-url https://download.pytorch.org/whl/cu130
    pip3 install --user open3d ultralytics onnxruntime-gpu
    ok "GPU 库安装完成"
else
    pip3 install --user torch torchvision torchaudio \
        --index-url https://download.pytorch.org/whl/cpu
    pip3 install --user open3d ultralytics onnxruntime
    ok "CPU 库安装完成"
fi
ok "Python 依赖已就绪"

# ── 激活脚本 ────────────────────────────────────────────
echo -e "\n${BLUE}[5] 激活脚本${NC}"

ACTIVATE="${PROJECT_ROOT}/activate.sh"
if [ -f "$ACTIVATE" ]; then
    skip "activate.sh 已存在"
else
    cat > "$ACTIVATE" << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/aubo_ros2_ws/install/setup.bash" 2>/dev/null || true
export PATH="/usr/local/cuda/bin:$HOME/.local/bin:$PATH"
echo "✅ aubo_boot 环境就绪"
EOF
    chmod +x "$ACTIVATE"
    ok "$ACTIVATE"
fi

# ── 完成 ────────────────────────────────────────────────
echo ""
echo -e "${GREEN}== 环境部署完成 ==${NC}"
echo ""
echo -e "  激活: ${GREEN}source ${ACTIVATE}${NC}"
echo -e "  编译: ${GREEN}bash ${PROJECT_ROOT}/build.sh${NC}"
