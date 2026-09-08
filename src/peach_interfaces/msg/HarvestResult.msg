# ExecuteTarget 接触段抓取结果（技能写入 Result.harvest）。
# grasped 仅在切断与撤退均确认时为 true；规划占位、PREGRASP_ONLY 干跑不得置 grasped。

bool grasped                   # 切断且撤退确认（产品成功）
bool commanded                 # 已发出接触/切割指令
bool confirmed                 # 有切断证据（无硬件反馈时不得仅凭指令置 true）
string reason                  # 未 grasped 时的原因码；成功可空
float32 quality_score          # 本颗质量分；未评则 0
uint8 completion_level         # 最高到达档（见下）；与 ExecuteTarget.Result 同枚举

uint8 LEVEL_NONE=0                   # 尚未过预抓取验证
uint8 LEVEL_PREGRASP_VERIFIED=1      # 预抓取停位已验证
uint8 LEVEL_SLEEVE_COMPLETED=2       # 套入完成
uint8 LEVEL_CUT_COMMAND_ACCEPTED=3   # 刀指令已受理
uint8 LEVEL_CUT_CONFIRMED=4          # 切断已确认
uint8 LEVEL_RETREAT_CONFIRMED=5      # 撤退已确认
uint8 LEVEL_HARVEST_CONFIRMED=6      # 采摘确认（切断+撤退）
