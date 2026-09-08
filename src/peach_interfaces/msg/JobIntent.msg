# 批次意图枚举。RunHarvest.goal.intent 取本常量（动作文件不重复定义，须与此对齐）。
# 独立话题未接线；调度只从 RunHarvest goal 读取。不要另发 JobIntent 话题。

uint8 PICK_ALL=0           # 锁定集内逐颗采（可被 execution_enabled=false 截成只扫）
uint8 PICK_SELECTED=1      # 只采 target_ids 指定目标
uint8 SURVEY_ONLY=2        # 只扫一场后结算，不派目标

uint8 intent               # PICK_ALL / PICK_SELECTED / SURVEY_ONLY
string[] target_ids        # PICK_SELECTED 的允许列表；空=未限定（PICK_ALL 忽略）
