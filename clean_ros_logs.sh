#!/bin/bash
# ROS2 日志清理脚本
# 用法: ./clean_ros_logs.sh [days]
# 如果不指定天数，默认清理超过1天的日志

LOG_DIR="/home/mu/.ros/log"
DAYS=${1:-1}

if [ ! -d "$LOG_DIR" ]; then
    echo "日志目录不存在: $LOG_DIR"
    exit 1
fi

echo "清理 $LOG_DIR 中超过 ${DAYS} 天的日志文件..."
echo "清理前大小: $(du -sh $LOG_DIR | cut -f1)"

# 删除超过指定天数的日志文件
find "$LOG_DIR" -type f -mtime +$DAYS -delete

# 删除空目录
find "$LOG_DIR" -type d -empty -delete

echo "清理后大小: $(du -sh $LOG_DIR | cut -f1)"
echo "当前日志文件数量: $(find $LOG_DIR -type f | wc -l)"
echo "✅ 清理完成！"
