# 批次意图。RunHarvest.goal.intent 取本枚举（动作定义未重复常量，须与此对齐）。
# 契约预留的独立话题尚未接线；执行器从 RunHarvest goal 读取 intent。

uint8 PICK_ALL=0       # 锁定集内逐颗采（可被 execution_enabled=false 截成只扫）
uint8 PICK_SELECTED=1  # 只采 goal.target_ids 指定目标
uint8 SURVEY_ONLY=2    # 只扫一场后结算，不派目标

uint8 intent           # PICK_ALL / PICK_SELECTED / SURVEY_ONLY
string[] target_ids    # PICK_SELECTED 的允许列表；空=未限定（PICK_ALL 忽略）
