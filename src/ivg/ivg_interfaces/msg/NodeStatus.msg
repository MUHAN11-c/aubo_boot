# 节点状态消息（系统健康监控）
#
# 用途：提供 IVG 演示栈中各 ROS 2 节点的健康状态和心跳信息，用于系统级健康监控面板，
#       当某节点出现故障（如 crash、网络断连、SDK 失联）时及时发现和报警
# 发布者：ivg_demo_services 的 system_monitor_node，以 ~1Hz 汇总发布受监控节点状态
# 订阅者：Web Dashboard 监控面板
#

# ROS 2 节点名称，与 ros2 node list 输出一致，用于唯一标识消息来源
string node_name
# 节点功能分类标签："driver" / "camera" / "vision" / "grasp" / "tool" / "web" / "system"，
# 用于监控面板分类显示和分组告警
string node_type
# 节点健康状态："online"（正常运行）/ "degraded"（功能降级或部分异常）/
# "offline"（节点已停止或失联）/ "error"（节点运行但存在不可恢复错误）
string status
# 错误描述信息：当 status 为 "error" 时填写具体的错误信息；正常状态时为空字符串
string error_message
# 上次心跳时间戳：节点最后一次发布此消息的时间，监控面板通过判断该时间距当前的差值
# 来检测节点是否超时失联（通常超时阈值为 3 倍发布间隔）
builtin_interfaces/Time last_heartbeat
# 自节点状态变为 "online" 以来的累计运行秒数
float64 uptime_sec
