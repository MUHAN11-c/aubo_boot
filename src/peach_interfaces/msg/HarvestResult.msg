# ExecuteTarget 接触段的抓取结果（技能写入 Result.harvest）。
# grasped 仅在切断与撤退均确认时为 true；规划占位不得置 grasped。

bool grasped                 # 切断且撤退确认（产品成功）
bool commanded               # 已发出接触/切割指令
bool confirmed               # 有切断证据
string reason
float32 quality_score
uint8 completion_level
uint8 LEVEL_NONE=0
uint8 LEVEL_PREGRASP_VERIFIED=1
uint8 LEVEL_SLEEVE_COMPLETED=2
uint8 LEVEL_CUT_COMMAND_ACCEPTED=3
uint8 LEVEL_CUT_CONFIRMED=4
uint8 LEVEL_RETREAT_CONFIRMED=5
uint8 LEVEL_HARVEST_CONFIRMED=6
