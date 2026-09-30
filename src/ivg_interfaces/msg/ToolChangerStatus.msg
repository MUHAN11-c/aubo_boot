# 快换盘状态消息
#
# 用途：提供 AUBO 快换盘（Quick Tool Changer）的当前状态，包括已安装的工具标识、
#       锁定状态和工具参数等，用于工具管理流程（切换/安装/卸载）的决策依据
# 发布者：tool_changer 的 gripper_swap_worker 节点（~10Hz），IO 轮询后检测状态变化并发布
# 订阅者：gripper_swap_worker（工具切换流程）、scene_attach_worker、Web Dashboard 工具面板
#

# 标准消息头：时间戳标记状态采集时刻，frame_id 为快换盘坐标系（如 "tool_changer_link"）
std_msgs/Header header
# 当前连接的工具 ID，如 "gripper0"、"gripper2"、"suction_v1" 等，
# 与 tools.yaml 中的 tool_id 字段一致，空字符串表示未安装工具
string tool_id
# 当前连接的工具名称（人类可读），用于前端显示和日志记录
string tool_name
# 工具功能类型分类："gripper"（夹爪型）/ "suction"（吸盘型）/ "other"（其他），
# 用于切换策略的差异化处理（如 Gripper 使用 IO 控制开闭，Suction 使用气阀控制）
string tool_type
# 工具连接状态：true=工具已锁紧（锁紧到位且限位开关触发），false=未锁定或未安装工具
bool is_connected
# 工具参数 JSON 字符串，包含工具特有的配置信息（如夹爪行程、最大夹持力、IO 引脚映射等），
# 解析格式与 tools.yaml 定义一致
string tool_parameters
