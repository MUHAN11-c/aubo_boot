#!/bin/bash

###############################################################################
# Visual Pose Estimation Python - Web UI 停止脚本
# 用于停止HTTP桥接服务器
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WEB_UI_DIR="${SCRIPT_DIR}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  停止 Web UI 服务${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 从PID文件读取进程ID
PID_FILE="${WEB_UI_DIR}/.web_ui.pid"

if [ -f "${PID_FILE}" ]; then
    PID=$(cat "${PID_FILE}")
    
    if ps -p $PID > /dev/null 2>&1; then
        echo -e "${YELLOW}正在停止进程 (PID: ${PID})...${NC}"
        kill $PID 2>/dev/null
        sleep 2
        
        # 检查进程是否已停止
        if ps -p $PID > /dev/null 2>&1; then
            echo -e "${YELLOW}进程未响应，强制终止...${NC}"
            kill -9 $PID 2>/dev/null
            sleep 1
        fi
        
        echo -e "${GREEN}✓ 进程已停止${NC}"
    else
        echo -e "${YELLOW}进程 (PID: ${PID}) 不存在${NC}"
    fi
    
    rm -f "${PID_FILE}"
else
    echo -e "${YELLOW}未找到PID文件，尝试按端口查找...${NC}"
fi

# 按端口查找并终止
PORT=8089
echo -e "${BLUE}检查端口 ${PORT}...${NC}"

if lsof -Pi :${PORT} -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    PID=$(lsof -ti:${PORT})
    echo -e "${YELLOW}找到占用端口的进程 (PID: ${PID})${NC}"
    kill -9 $PID 2>/dev/null
    sleep 1
    echo -e "${GREEN}✓ 端口已清理${NC}"
else
    echo -e "${GREEN}✓ 端口 ${PORT} 未被占用${NC}"
fi

# 查找所有相关Python进程
echo ""
echo -e "${BLUE}查找相关Python进程...${NC}"
PIDS=$(pgrep -f "http_bridge_server.py")

if [ ! -z "$PIDS" ]; then
    echo -e "${YELLOW}找到相关进程:${NC}"
    echo "$PIDS"
    echo ""
    echo -e "${YELLOW}正在终止这些进程...${NC}"
    echo "$PIDS" | xargs kill -9 2>/dev/null
    echo -e "${GREEN}✓ 进程已清理${NC}"
else
    echo -e "${GREEN}✓ 没有找到相关进程${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Web UI 服务已停止${NC}"
echo -e "${GREEN}========================================${NC}"
