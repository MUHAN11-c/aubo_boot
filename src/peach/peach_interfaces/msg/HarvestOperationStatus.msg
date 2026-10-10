# 【预留】采摘 → 外部导航：臂占用与放行。剪切/撤退完成前不得让底盘 resume。
# 话题：/peach/navigation/arm_status（清单 reserved_interfaces）
# 生产/消费：固定座核心栈均无。随 NavigateToWorksite 一并归档。

std_msgs/Header header             # stamp=发布时刻
string target_id               # 当前作业目标
string site_id                 # 作业位点
uint8 phase                    # 见下
uint8 IDLE=0                   # 臂空闲
uint8 BUSY=1                   # 臂占用（观察/规划）
uint8 CONTACT=2                # 接触中
uint8 ARM_CLEAR=3              # 臂已撤离可放行底盘
uint8 COMPLETED=4              # 本颗完成
uint8 ABANDONED=5              # 放弃
bool resume_allowed            # true=允许导航恢复行驶
string message                 # 人读说明
