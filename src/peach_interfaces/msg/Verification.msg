# ExecuteTarget 周期校验。技能在 Result.verification 写入；编排器据此记账，不二次做视觉判定。

bool passed    # 本周期是否视为过门（抓取+卸果策略下的综合）
string reason  # 未过门原因；过门可为空
