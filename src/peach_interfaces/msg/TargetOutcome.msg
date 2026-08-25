# 单目标终局，写入账本与 HarvestSummary.outcomes。
# 数值须与 ExecuteTarget.Result 的 SUCCEEDED..CANCELED 及 harvest_fsm 映射表一致。

uint8 SUCCEEDED=0            # FULL 成功
uint8 SKIPPED_QUALITY=1      # 质量不够（观察失败、建模不够、质量门）
uint8 SKIPPED_UNREACHABLE=2  # 运动学/碰撞不可达
uint8 FAILED=3               # 接触或工具失败
uint8 CANCELED=4             # 取消或人工 SKIP

string target_id
uint8 outcome                # 上列常量
string reason                # 稳定原因码（observe_failed:... / build_timeout:... 等）
float32 quality_score        # 终局质量分；未评则 0
builtin_interfaces/Duration elapsed  # 该目标墙钟时长
