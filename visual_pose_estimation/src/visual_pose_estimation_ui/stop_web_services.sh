#!/bin/bash

echo "🛑 停止Web服务..."

# 停止端口8088的Web UI服务器
echo "停止端口8088的Web UI服务器..."
pkill -f "python3 -m http.server 8088"

# 停止端口8088的算法服务器
echo "停止端口8088的算法服务器..."
pkill -f "algorithm_server.py"

# 等待一下让进程完全退出
sleep 2

# 检查是否还有相关进程
remaining=$(ps aux | grep -E "(python3.*http.server|algorithm_server.py)" | grep -v grep | wc -l)

if [ $remaining -eq 0 ]; then
    echo "✅ 所有Web服务已成功停止"
else
    echo "⚠️  还有 $remaining 个相关进程在运行"
    echo "正在运行的进程："
    ps aux | grep -E "(python3.*http.server|algorithm_server.py)" | grep -v grep
fi

echo "🌐 服务状态检查："
echo "端口8088: $(netstat -tlnp 2>/dev/null | grep :8088 || echo '未监听')"
echo "端口8088: $(netstat -tlnp 2>/dev/null | grep :8088 || echo '未监听')"


