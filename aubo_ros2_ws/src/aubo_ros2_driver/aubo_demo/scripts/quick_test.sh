#!/bin/bash
# 快速位姿对比测试脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=====================================================================${NC}"
echo -e "${BLUE}  🤖 位姿对比工具 - 快速测试${NC}"
echo -e "${BLUE}=====================================================================${NC}"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 检查是否在 ROS 2 环境中
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS 2 环境未初始化${NC}"
    echo -e "${YELLOW}请先运行: source /home/mu/IVG/aubo_ros2_ws/install/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} ROS 2 环境已初始化 (ROS_DISTRO: $ROS_DISTRO)"

# 检查必要的话题
echo ""
echo -e "${BLUE}检查必要的话题和服务...${NC}"

# 检查 /demo_robot_status 话题
if ros2 topic list 2>/dev/null | grep -q "/demo_robot_status"; then
    echo -e "${GREEN}✓${NC} /demo_robot_status 话题存在"
else
    echo -e "${RED}✗${NC} /demo_robot_status 话题不存在"
    echo -e "${YELLOW}  请确保 robot_status_publisher 节点正在运行${NC}"
fi

# 检查 /move_to_pose 服务
if ros2 service list 2>/dev/null | grep -q "/move_to_pose"; then
    echo -e "${GREEN}✓${NC} /move_to_pose 服务存在"
else
    echo -e "${YELLOW}⚠${NC}  /move_to_pose 服务不存在 (自动对比工具需要)"
fi

# 选择测试工具
echo ""
echo -e "${BLUE}=====================================================================${NC}"
echo -e "${BLUE}请选择测试工具:${NC}"
echo -e "${BLUE}=====================================================================${NC}"
echo ""
echo "  1) 手动对比工具 (compare_pose.py)"
echo "     - 手动控制机械臂移动"
echo "     - 适合测试任意运动"
echo ""
echo "  2) 自动对比工具 (compare_pose_auto.py)"
echo "     - 自动调用 MoveIt2 移动机械臂"
echo "     - 适合精度测试"
echo "     - 需要 /move_to_pose 服务"
echo ""
echo "  3) 批量测试工具 (batch_pose_test.py)"
echo "     - 批量测试多个位姿"
echo "     - 生成测试报告"
echo "     - 需要 /move_to_pose 服务"
echo ""
echo "  4) MoveIt2 验证工具 (validate_moveit_poses.py) ⭐"
echo "     - 从 JSON 文件读取位姿"
echo "     - 验证运动到位判断"
echo "     - 验证 MoveIt2 位姿精度"
echo "     - 需要 /move_to_pose 服务"
echo ""
echo "  5) 退出"
echo ""

read -p "请选择 (1-5): " choice

case $choice in
    1)
        echo ""
        echo -e "${GREEN}启动手动对比工具...${NC}"
        python3 compare_pose.py
        ;;
    2)
        echo ""
        echo -e "${GREEN}启动自动对比工具...${NC}"
        python3 compare_pose_auto.py
        ;;
    3)
        echo ""
        echo -e "${GREEN}启动批量测试工具...${NC}"
        echo ""
        read -p "测试次数 (默认 5): " test_count
        test_count=${test_count:-5}
        python3 batch_pose_test.py --ros-args -p test_count:=$test_count
        ;;
    4)
        echo ""
        echo -e "${GREEN}启动 MoveIt2 验证工具...${NC}"
        echo ""
        read -p "JSON 文件路径 (留空使用默认): " json_file
        if [ -z "$json_file" ]; then
            python3 validate_moveit_poses.py
        else
            python3 validate_moveit_poses.py --ros-args -p json_file:="$json_file"
        fi
        ;;
    5)
        echo ""
        echo -e "${BLUE}退出${NC}"
        exit 0
        ;;
    *)
        echo ""
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${BLUE}=====================================================================${NC}"
echo -e "${GREEN}测试完成!${NC}"
echo -e "${BLUE}=====================================================================${NC}"
