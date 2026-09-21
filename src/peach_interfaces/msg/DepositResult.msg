# 卸果站结果（M15 2026-09-20 修注释：预留消息，当前零生产零消费）。
# 决策 0012 保留：卸果站关节未标定（M8），标定后随卸果动作携带；
# W7 起 ExecuteTarget.Result 已不再嵌本消息（顶层镜像删除）。
# 到期无人接线则随下轮接口清理删除（E2E 审查 L 级跟踪）。

bool deposited                 # 已放到卸果站
string reason                  # 失败/跳过原因（pending_m8_unload_pose 等）；成功可空
