# 重建节点结构化诊断核心（/peach/reconstruction/diagnostics，transient_local 闩锁，1Hz 心跳）。
# 只承载消费方决策所需的结构化字段：编排器用到时刻、approach_grasp 读视角覆盖、
# Web 做只读镜像；tsdf/registration/overlap/refined 明细与逐机位完整信息保留在
# 调试 JSON 话题 /peach/reconstruction/diagnostics_debug，不进本消息。
# 无效标量统一填 -1（沿用 BagFitting 惯例）；消费方把 <0 视为"无数据"。

std_msgs/Header header  # stamp=发布时刻，frame_id=base_frame（center/方向所属坐标系）

string harvest_run_id        # 所属批次（空=无批次）
string selected_target_id    # 感知侧选中目标（空=未选）
string state                 # 重建状态机状态名（IDLE/COLLECTING/READY/...）
string target_id             # 当前绑定目标（空=未绑定）

# 目标中心（base 系 [m]）；未绑定填 [-1, -1, -1]
float64[3] target_center_base

int32 captured_views         # 已采机位数（按机位聚类口径，同 diagnostics.captured_views）
int32 rejected_views         # 质量门拒绝帧数
int32 tf_failures            # TF 查询失败次数
float64 tf_latency_ms        # 最近一次 TF 查询墙钟耗时 [ms]；未测过 -1

# 机位代表平均有效深度比（view_coverage.valid_depth_ratio_mean）；
# 覆盖无效时回退最近帧 valid_depth_ratio；无帧 -1
float64 valid_depth_ratio

# 视角基线指标（按机位聚类）；覆盖无效（无目标中心/无有效方向）填 -1
float64 max_baseline_deg
float64 mean_nearest_baseline_deg

# 各机位代表"目标→相机"单位方向（base 系），供球面自适应视点规划；
# 覆盖无效时为空数组（与 refined_pose 发空数组覆盖的闩锁惯例一致）
geometry_msgs/Vector3[] view_directions
