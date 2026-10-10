# 接触许可令牌：大脑（视觉融合 + 动态预算）→ 臂侧命令门的接触复检输入。
# 随 ExecuteTarget goal 传递（清洁重写轮契约，阶段 2 接线）。
# allowed 数学与现行 GraspDecision 完全同源（预算原值）；臂侧只验
# 目标绑定（target_id）、新鲜度（valid_until 冻结不续签 + model_stamp）
# 与档位，不重算几何——单一许可源在融合侧。

string target_id                     # 绑定目标；空或不符=令牌无效（防旧决策授权新目标）
builtin_interfaces/Time valid_until  # GraspDecision.valid_until 原值；过期不得执行（心跳不续签）
builtin_interfaces/Time model_stamp   # 融合模型时刻；CONTACT/TOOL 级复检新鲜度
bool allowed                          # 套入/剪切许可（动态预算派生；原值数学）
float64 radial_margin_m               # 径向余量 [m]；负值不可套
float64 axial_margin_m                # 轴向余量 [m]；负值不可剪
string model_fingerprint              # 模型指纹（版本归因）
string reason                         # 许可/拒绝原因（稳定短语）
