# 重建结构化心跳：绑定态、机位数、视角基线、TF 健康。技能用来看覆盖够不够。
# 话题：/peach/reconstruction/diagnostics（reliable, transient_local, depth 1；约 1 Hz）
# 生产：peach_target_reconstruction    消费：peach_manipulation、peach_observability
# 调度不订本话题。TSDF/ICP/逐机位明细在 /peach/reconstruction/diagnostics_debug（String JSON）。
# 无效标量填 -1；消费方把 <0 视为无数据。

std_msgs/Header header         # stamp=发布时刻，frame_id=base_frame（center/方向坐标系）

string harvest_run_id          # 所属批次；空=无批次
string selected_target_id      # 感知侧选中目标（镜像）；空=未选
string state                   # 重建状态机短名：IDLE / COLLECTING / READY / …
string target_id               # 当前绑定目标；空=未绑定

float64[3] target_center_base  # 目标中心 base 系 [m]；未绑定填 [-1, -1, -1]

int32 captured_views           # 已采机位数（机位聚类口径，不是原始帧数）
int32 rejected_views           # 质量门拒绝帧数
int32 tf_failures              # 精确 stamp TF 查询失败次数
float64 tf_latency_ms          # 最近一次 TF 查询墙钟耗时 [ms]；未测过 -1

float64 valid_depth_ratio      # 机位代表平均有效深度比；覆盖无效回退最近帧；无帧 -1
float64 max_baseline_deg       # 机位间最大基线角 [deg]；覆盖无效 -1
float64 mean_nearest_baseline_deg  # 最近邻基线角均值 [deg]；覆盖无效 -1

geometry_msgs/Vector3[] view_directions  # 各机位「目标→相机」单位方向（base 系）；无效时发空数组盖旧值
