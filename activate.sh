#!/bin/bash
source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/aubo_ros2_ws/install/setup.bash" 2>/dev/null || true
export PATH="/usr/local/cuda/bin:$HOME/.local/bin:$PATH"
echo "✅ aubo_boot 环境就绪"
