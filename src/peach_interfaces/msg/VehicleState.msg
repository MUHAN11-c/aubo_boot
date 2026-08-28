# 外部导航→采摘：车辆到位且静止。导航拥有路径主权。
std_msgs/Header header
string target_id
string site_id
bool matched
bool arrived
bool stationary
bool fresh                    # 状态在 freshness_s 内
float32 freshness_s
string message
