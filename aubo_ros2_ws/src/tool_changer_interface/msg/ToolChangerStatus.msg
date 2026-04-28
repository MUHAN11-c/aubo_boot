# 快换盘状态
std_msgs/Header header
string tool_id           # 当前工具ID（如 "gripper0", "gripper2"）
string tool_name         # 当前工具名称
string tool_type         # 工具类型（gripper / suction / other）
bool is_connected        # 工具连接状态（true=已锁定）
string tool_parameters   # 工具参数（JSON 格式）
