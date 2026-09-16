# 操作台使能广播（清洁重写轮契约，阶段 3 接线；阶段 2 臂侧先订阅）。
# 话题：/peach/batch/enables（reliable, transient_local, depth 1）
# 生产：peach_supervisor（SetEnables 服务后广播）    消费：peach_arm、peach_recorder
# 意图源=大脑；臂侧命令门为强制点。臂侧未收到过本话题时以本地参数为准
# （旧栈兼容）；一旦收到，话题值覆盖本地参数并打日志。

std_msgs/Header header        # stamp=变更时刻；frame_id=来源（supervisor）
bool execution                # TRANSIT/PREGRASP 级运动使能
bool grasp                    # CONTACT 级使能
bool tool                     # TOOL 级（SetIO）使能
string reason                 # 人读原因（审计镜像）
