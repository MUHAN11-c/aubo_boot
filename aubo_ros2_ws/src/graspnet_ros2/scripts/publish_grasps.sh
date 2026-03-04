#!/bin/bash
# GraspNet 抓取发布脚本
# 手动触发抓取结果发布到 RViz2

echo "=========================================="
echo "  GraspNet 抓取发布客户端"
echo "=========================================="
echo ""
echo "正在调用 /publish_grasps 服务..."
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 方法1：尝试使用独立脚本（推荐，无依赖问题）
if [ -f "$SCRIPT_DIR/publish_grasps_client.py" ]; then
    echo "使用独立 Python 脚本..."
    python3 "$SCRIPT_DIR/publish_grasps_client.py"
    exit_code=$?
else
    # 方法2：使用 ros2 run（需要正确安装包）
    echo "使用 ros2 run..."
    ros2 run graspnet_ros2 publish_grasps_client
    exit_code=$?
fi

if [ $exit_code -eq 0 ]; then
    echo ""
    echo "✓ 发布成功！请检查 RViz2 查看结果。"
else
    echo ""
    echo "✗ 发布失败，请检查日志。"
fi

exit $exit_code
