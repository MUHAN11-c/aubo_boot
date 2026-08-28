# ExecuteTarget 周期校验。技能在 Result.verification 写入。
# passed 表示本周期过门；harvest_confirmed 才表示果已带走。

bool passed
bool commanded
bool confirmed
bool harvest_confirmed
string reason
uint32 failure_code
