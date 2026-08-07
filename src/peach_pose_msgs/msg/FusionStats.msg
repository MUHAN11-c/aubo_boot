# FusionStats.msg — peach_recon 时序融合的逐帧统计（过程分析用）
std_msgs/Header header
string target_id        # 融合对象
uint32 frame_seq        # 融合器内部帧序号
uint32 accepted         # 累计被接受的观测数
uint32 rejected         # 累计被判离群的观测数
uint32 cluster_size     # 当前一致簇大小（DBSCAN）
float32 pos_std_m       # 当前融合位置标准差（米，收敛性指标）
float32 axis_std_deg    # 当前融合轴向标准差（度，收敛性指标）
