#!/bin/bash
# GraspNet ROS2 快速测试脚本

set -e

echo "=========================================="
echo "GraspNet ROS2 快速测试"
echo "=========================================="

# 激活 ros2_env 环境
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    echo "激活 ros2_env 环境..."
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate ros2_env
    echo "Python 版本: $(python --version)"
    echo "Python 路径: $(which python)"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    echo "激活 ros2_env 环境..."
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
    conda activate ros2_env
    echo "Python 版本: $(python --version)"
    echo "Python 路径: $(which python)"
else
    echo "警告：未找到 conda，尝试直接运行（可能失败）"
fi

# 检查是否在正确的目录
if [ ! -f "package.xml" ]; then
    echo "错误：请在包根目录运行此脚本"
    exit 1
fi

# 获取工作空间路径
WS_DIR=$(dirname $(dirname $(readlink -f $0)))
echo "工作空间目录: $WS_DIR"

# Source ROS2 环境
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo "Source ROS2 环境..."
    source "$WS_DIR/install/setup.bash"
else
    echo "警告：未找到 install/setup.bash，请先编译包"
    echo "运行: cd $WS_DIR && colcon build --packages-select graspnet_ros2"
    exit 1
fi

# 运行测试
echo ""
echo "运行测试节点..."
ros2 launch graspnet_ros2 test.launch.py

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
