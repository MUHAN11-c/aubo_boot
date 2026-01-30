#!/bin/bash

###############################################################################
# Visual Pose Estimation Python - Web UI 安装检查脚本
# 用于验证所有依赖和配置是否正确
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Web UI 安装检查${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查计数
PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0

# 检查函数
check_pass() {
    echo -e "${GREEN}✓ $1${NC}"
    ((PASS_COUNT++))
}

check_fail() {
    echo -e "${RED}✗ $1${NC}"
    ((FAIL_COUNT++))
}

check_warn() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ((WARN_COUNT++))
}

# 1. 检查文件完整性
echo -e "${BLUE}[1/7] 检查文件完整性...${NC}"
FILES=(
    "index.html"
    "README.md"
    "requirements.txt"
    "start_web_ui.sh"
    "stop_web_ui.sh"
    "scripts/http_bridge_server.py"
    "PROJECT_SUMMARY.md"
    "QUICK_REFERENCE.md"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        check_pass "$file 存在"
    else
        check_fail "$file 不存在"
    fi
done
echo ""

# 2. 检查目录结构
echo -e "${BLUE}[2/7] 检查目录结构...${NC}"
DIRS=(
    "scripts"
    "configs"
    "docs"
    "web_ui/resources"
)

for dir in "${DIRS[@]}"; do
    if [ -d "$dir" ]; then
        check_pass "$dir/ 目录存在"
    else
        check_fail "$dir/ 目录不存在"
    fi
done
echo ""

# 3. 检查文件权限
echo -e "${BLUE}[3/7] 检查文件权限...${NC}"
EXECUTABLE_FILES=(
    "start_web_ui.sh"
    "stop_web_ui.sh"
    "scripts/http_bridge_server.py"
)

for file in "${EXECUTABLE_FILES[@]}"; do
    if [ -x "$file" ]; then
        check_pass "$file 可执行"
    else
        check_fail "$file 不可执行"
        echo -e "${YELLOW}   修复: chmod +x $file${NC}"
    fi
done
echo ""

# 4. 检查Python环境
echo -e "${BLUE}[4/7] 检查Python环境...${NC}"

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | awk '{print $2}')
    check_pass "Python3 已安装 (版本: $PYTHON_VERSION)"
else
    check_fail "Python3 未安装"
fi

# 检查Python包
PYTHON_PACKAGES=(
    "cv2:OpenCV"
    "numpy:NumPy"
)

for pkg in "${PYTHON_PACKAGES[@]}"; do
    IFS=':' read -r module name <<< "$pkg"
    if python3 -c "import $module" 2>/dev/null; then
        VERSION=$(python3 -c "import $module; print($module.__version__)" 2>/dev/null || echo "unknown")
        check_pass "$name 已安装 (版本: $VERSION)"
    else
        check_warn "$name 未安装 (运行时需要)"
    fi
done
echo ""

# 5. 检查ROS2环境
echo -e "${BLUE}[5/7] 检查ROS2环境...${NC}"

if [ -n "$ROS_DISTRO" ]; then
    check_pass "ROS2 环境已设置 (发行版: $ROS_DISTRO)"
else
    check_warn "ROS2 环境未设置 (运行时需要 source)"
fi

# 检查ROS2 Python包
if python3 -c "import rclpy" 2>/dev/null; then
    check_pass "rclpy 已安装"
else
    check_warn "rclpy 未安装 (运行时需要)"
fi

if python3 -c "import cv_bridge" 2>/dev/null; then
    check_pass "cv_bridge 已安装"
else
    check_warn "cv_bridge 未安装 (运行时需要)"
fi
echo ""

# 6. 检查端口可用性
echo -e "${BLUE}[6/7] 检查端口可用性...${NC}"

PORT=8089
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    PID=$(lsof -ti:$PORT)
    check_warn "端口 $PORT 已被占用 (PID: $PID)"
    echo -e "${YELLOW}   修复: ./stop_web_ui.sh${NC}"
else
    check_pass "端口 $PORT 可用"
fi
echo ""

# 7. 检查ROS2服务
echo -e "${BLUE}[7/7] 检查ROS2服务...${NC}"

if command -v ros2 &> /dev/null; then
    if ros2 service list 2>/dev/null | grep -q "/estimate_pose"; then
        check_pass "/estimate_pose 服务运行中"
    else
        check_warn "/estimate_pose 服务未运行 (运行时需要)"
    fi
    
    if ros2 service list 2>/dev/null | grep -q "/list_templates"; then
        check_pass "/list_templates 服务运行中"
    else
        check_warn "/list_templates 服务未运行 (运行时需要)"
    fi
    
    if ros2 service list 2>/dev/null | grep -q "/standardize_template"; then
        check_pass "/standardize_template 服务运行中"
    else
        check_warn "/standardize_template 服务未运行 (运行时需要)"
    fi
else
    check_warn "ros2 命令不可用 (运行时需要)"
fi
echo ""

# 总结
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  检查结果总结${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}通过: $PASS_COUNT${NC}"
echo -e "${YELLOW}警告: $WARN_COUNT${NC}"
echo -e "${RED}失败: $FAIL_COUNT${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ 所有必需组件检查通过！${NC}"
    echo ""
    if [ $WARN_COUNT -gt 0 ]; then
        echo -e "${YELLOW}注意: 存在 $WARN_COUNT 个警告项${NC}"
        echo -e "${YELLOW}这些警告通常在运行时才需要，不影响安装${NC}"
    fi
    echo ""
    echo -e "${BLUE}下一步:${NC}"
    echo -e "  1. 启动ROS2节点: ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py"
    echo -e "  2. 启动Web UI: ./start_web_ui.sh"
    echo -e "  3. 访问: ${GREEN}http://localhost:8089${NC}"
    exit 0
else
    echo -e "${RED}✗ 发现 $FAIL_COUNT 个问题需要解决${NC}"
    echo ""
    echo -e "${BLUE}建议:${NC}"
    echo -e "  1. 检查上述失败项"
    echo -e "  2. 确保所有文件已正确复制"
    echo -e "  3. 运行修复命令（如果有）"
    echo -e "  4. 重新运行此检查脚本"
    exit 1
fi
