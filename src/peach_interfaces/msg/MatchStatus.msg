# 身份匹配结果。感知对检测做马氏门控 + 全局一对一后写入 BagGraspCandidate.match_status。
# AMBIGUOUS 不强制合并，下游须当新身份或等待下一帧，不要自行拼 ID。

uint8 OK=0          # 与已有稳定 ID 唯一匹配
uint8 NEW=1         # 新目标，已分配新 ID
uint8 AMBIGUOUS=2   # 多个已有 ID 都过门，拒绝合并
uint8 REJECTED=3    # 质量不够或几何拒绝，不入库

uint8 status        # 上列常量
