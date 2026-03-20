#!/bin/bash

# 日志测试脚本
# 用于测试 execute_grasp_pose_worker 的日志输出

echo "================================================"
echo "Execute Grasp Pose Worker - 日志测试脚本"
echo "================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 环境未 source${NC}"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "/home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash" ]; then
    echo -e "${RED}❌ 工作空间未构建${NC}"
    echo "请先运行: cd /home/mu/IVG2.0/aubo_ros2_ws && colcon build"
    exit 1
fi

source /home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash

echo -e "${GREEN}✓ ROS2 环境已准备${NC}"
echo ""

# 检查服务是否可用
echo "检查服务可用性..."
if ros2 service list | grep -q "/execute_single_grasp"; then
    echo -e "${GREEN}✓ /execute_single_grasp 服务可用${NC}"
else
    echo -e "${YELLOW}⚠ /execute_single_grasp 服务不可用${NC}"
    echo "  请在另一个终端运行："
    echo "  ros2 launch demo_driver execute_grasp_pose_worker.launch.py"
    echo ""
    read -p "服务已启动？按 Enter 继续测试... "
fi

echo ""
echo "================================================"
echo "测试 1: 单次抓取（使用参数常量）"
echo "================================================"
echo ""
echo -e "${BLUE}观察以下日志输出：${NC}"
echo "  1. 服务请求的框图"
echo "  2. '使用参数常量抓取位姿' 信息"
echo "  3. 抓取周期的 8 个步骤"
echo "  4. 每步的成功标记 ✓"
echo "  5. 最终的成功框图"
echo ""
read -p "按 Enter 开始测试... "

ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose \
  "{object_id: 'test', use_visual_estimation: false}"

echo ""
echo -e "${GREEN}✓ 测试 1 完成${NC}"
echo ""
read -p "按 Enter 继续下一个测试... "

echo ""
echo "================================================"
echo "测试 2: 单次抓取（使用视觉估计 - 模拟）"
echo "================================================"
echo ""
echo -e "${BLUE}观察以下日志输出：${NC}"
echo "  1. 视觉估计服务调用的详细日志"
echo "  2. 服务等待和响应的步骤"
echo "  3. 位姿更新信息"
echo ""
echo -e "${YELLOW}注意：如果视觉服务未运行，会看到错误日志${NC}"
echo ""
read -p "按 Enter 开始测试... "

ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose \
  "{object_id: 'default', use_visual_estimation: true}"

echo ""
echo -e "${GREEN}✓ 测试 2 完成${NC}"
echo ""
read -p "按 Enter 继续下一个测试... "

echo ""
echo "================================================"
echo "测试 3: 启动循环抓取"
echo "================================================"
echo ""
echo -e "${BLUE}观察以下日志输出：${NC}"
echo "  1. 循环线程启动的框图"
echo "  2. 循环配置信息"
echo "  3. 每次循环的编号和日志"
echo ""
echo -e "${YELLOW}注意：循环将持续运行，按 Ctrl+C 或运行测试 4 停止${NC}"
echo ""
read -p "按 Enter 开始测试（将在5秒后自动停止）... "

# 启动循环
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}" &
CALL_PID=$!

# 等待5秒
echo "循环运行中..."
sleep 5

echo ""
echo "================================================"
echo "测试 4: 停止循环抓取"
echo "================================================"
echo ""
read -p "按 Enter 停止循环... "

ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"

# 等待之前的调用完成
wait $CALL_PID 2>/dev/null

echo ""
echo -e "${GREEN}✓ 测试 4 完成${NC}"
echo ""

echo ""
echo "================================================"
echo "测试 5: 重复启动（应该看到警告）"
echo "================================================"
echo ""
echo -e "${BLUE}观察以下日志输出：${NC}"
echo "  1. '循环抓取已在运行中' 警告"
echo ""
read -p "按 Enter 开始测试... "

# 先启动循环
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}" &
sleep 1

# 尝试再次启动（应该失败）
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}"

# 停止循环
sleep 1
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"

echo ""
echo -e "${GREEN}✓ 测试 5 完成${NC}"
echo ""

echo ""
echo "================================================"
echo "所有测试完成！"
echo "================================================"
echo ""
echo -e "${GREEN}日志特性验证：${NC}"
echo "  ✓ 框图和分隔线正确显示"
echo "  ✓ 步骤编号和进度清晰"
echo "  ✓ 成功/警告/错误标记明显"
echo "  ✓ 详细参数信息输出"
echo ""
echo -e "${BLUE}查看完整日志文档：${NC}"
echo "  /home/mu/IVG2.0/DETAILED_LOGGING_GUIDE.md"
echo ""
echo -e "${BLUE}实时监控日志：${NC}"
echo "  ros2 run rqt_console rqt_console"
echo "  或"
echo "  ros2 topic echo /rosout | grep execute_grasp_pose_worker"
echo ""
