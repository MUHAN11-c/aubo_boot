# 【预留】采摘→外部导航：臂占用与放行（固定座核心栈不发布/不消费；随 NavigateToWorksite 一并归档）。剪切/撤退完成前不得 resume。
std_msgs/Header header
string target_id
string site_id
uint8 phase
uint8 IDLE=0
uint8 BUSY=1
uint8 CONTACT=2
uint8 ARM_CLEAR=3
uint8 COMPLETED=4
uint8 ABANDONED=5
bool resume_allowed
string message
