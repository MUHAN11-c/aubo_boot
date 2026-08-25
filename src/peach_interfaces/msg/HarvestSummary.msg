# 一批次结算摘要。RunHarvest 的 Result.summary 携带；账本 ledger.json 同源字段。
# skipped_* / failed / canceled 与 TargetOutcome 枚举对应，不要混用。

string run_id
uint32 discovered           # 发现并锁定的目标数
uint32 attempted            # 实际派过的目标数
uint32 succeeded            # TargetOutcome.SUCCEEDED
uint32 skipped_quality      # SKIPPED_QUALITY（含观察失败、建模不够）
uint32 skipped_unreachable  # SKIPPED_UNREACHABLE
uint32 failed               # FAILED
uint32 canceled             # CANCELED（含人工 SKIP 记入的取消）
builtin_interfaces/Duration elapsed  # 批次墙钟时长
peach_interfaces/TargetOutcome[] outcomes  # 按尝试顺序
