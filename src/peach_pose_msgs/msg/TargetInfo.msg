# TargetInfo.msg — 目标注册表条目（peach_motion TargetRegistry 的单个目标档案）
string target_id              # 持久 ID（T%03d）
geometry_msgs/Point position  # 融合位置（base_link，米）
string kind                   # 'bag'（袋装桃）| 'fruit'（裸果）
uint8 status                  # 状态机当前状态（见下）
uint8 STATUS_NEW = 0          # 新注册
uint8 STATUS_OBSERVING = 1    # 观测中（未稳定）
uint8 STATUS_READY = 2        # 可处理（连续 ACCEPT 稳定）
uint8 STATUS_EXECUTING = 3    # 正在执行接近/插入
uint8 STATUS_DONE = 4         # 已处理完成（不再调度）
uint8 STATUS_FAILED = 5       # 处理失败（记原因，不再调度）
uint8 STATUS_LOST = 6         # 超时未再见（保留档案）
uint32 attempts               # 已尝试次数
string last_result            # 最近一次结果/失败原因
builtin_interfaces/Time first_seen   # 首次观测时刻
builtin_interfaces/Time last_seen    # 最近观测时刻
