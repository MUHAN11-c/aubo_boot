# ExecuteTarget 接触段的抓取结果（技能写入 Result.harvest）。
# 默认 tool=false 时 grasped 仍可能为规划成功的占位，须结合 tool_enabled 解读。

bool grasped           # 工具动作判定已抓住（或规划占位成功）
string reason          # 失败/跳过原因码；成功可为空
float32 quality_score  # 技能侧质量分；未评则 0
