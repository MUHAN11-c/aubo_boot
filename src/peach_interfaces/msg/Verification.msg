# ExecuteTarget 周期校验块（技能写入 Result.verification）。
# passed=本周期过门；harvest_confirmed 才表示果已带走。failure_code 用 FailureCode.*。

bool passed                    # 本周期软件门通过
bool commanded                 # 已发出接触/切割指令
bool confirmed                 # 有切断证据
bool harvest_confirmed         # 切断且撤退确认（与 HarvestResult.grasped 同口径）
string reason                  # 未过门原因
uint32 failure_code            # FailureCode.*；0=NONE
