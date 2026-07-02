# 结构化系统日志消息
#
# 用途：将 IVG 系统中各节点的关键日志事件（调试/信息/警告/错误）以结构化格式发布，
#       供 Web Dashboard 日志面板实时展示，并支持 rosbag 录制后的离线分析喵~
# 发布者：各节点的结构化日志发布器，通常在关键操作（工具切换/抓取执行/视觉推理）的
#         开始和结束时发布喵~
# 订阅者：Web Dashboard 日志面板（通过 websocket gateway 推送到浏览器）、
#         监控系统的日志聚合节点喵~
#

# 日志消息的时间戳，记录日志事件发生的实际时刻（非发布时刻），
# 使用 builtin_interfaces/Time 以纳秒精度标记喵~
builtin_interfaces/Time timestamp

# 日志级别常量：调试信息，仅开发/调试时需要，生产环境通常屏蔽喵~
uint8 LEVEL_DEBUG=0
# 日志级别常量：一般信息，正常操作的事件记录（如 "工具切换完成"）喵~
uint8 LEVEL_INFO=1
# 日志级别常量：警告，潜在问题但不影响当前操作（如 "接近关节限位"）喵~
uint8 LEVEL_WARN=2
# 日志级别常量：错误，操作失败或异常状态需要人工介入（如 "SDK 连接断开"）喵~
uint8 LEVEL_ERROR=3

# 日志级别值：取值为 LEVEL_DEBUG / LEVEL_INFO / LEVEL_WARN / LEVEL_ERROR 之一喵~
uint8 level
# 来源节点名称，与 ROS 2 节点名一致，用于定位日志来源喵~
string node_name
# 日志内容主体，纯文本描述，不包含二进制或特殊编码数据喵~
string message
# 来源文件路径（可选），可用于在 IDE 中快速定位代码位置；字符串为空时表示该信息不可用喵~
string file
# 来源行号（可选），配合 file 字段定位代码行；值为 -1 时表示该信息不可用喵~
int32 line
