# 单颗终局，写入账本与 HarvestSummary.outcomes。
# outcome 数值须与 ExecuteTarget.Result 的 SUCCEEDED..CANCELED 及 harvest_fsm 映射表一致。
# 周期 SUCCEEDED 不等于产品采摘成功；后者看 HarvestResult.grasped / harvest_confirmed。

uint8 SUCCEEDED=0            # 周期完成
uint8 SKIPPED_QUALITY=1      # 质量不够（观察失败、建模不够、质量门）
uint8 SKIPPED_UNREACHABLE=2  # 运动学/碰撞不可达
uint8 FAILED=3               # 接触或工具失败
uint8 CANCELED=4             # 取消或人工 SKIP

string target_id                           # 本颗稳定 ID
uint8 outcome                              # 上列常量
string reason                              # 稳定原因码（observe_failed:… / build_timeout:… 等），不是自由散文
float32 quality_score                      # 终局质量分；未评则 0
builtin_interfaces/Duration elapsed        # 该目标墙钟时长
