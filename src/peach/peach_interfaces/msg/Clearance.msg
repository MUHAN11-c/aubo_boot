# 接触许可令牌：大脑（视觉融合 + 动态预算）→ 臂侧命令门的接触复检输入。
# 随 ExecuteTarget goal 传递（清洁重写轮契约，阶段 2 接线）。
# allowed 原样来自 GraspDecision；臂侧核目标绑定、冻结期限与当前绑定模型
# 的身份/相应能力。valid_until>0 时不叠加短图像新鲜窗；无截止才回退
# model_stamp，未来超过0.5s的戳始终拒绝。余量仅记录，不重算几何。

string target_id                     # 绑定目标；空或不符=令牌无效（防旧决策授权新目标）
builtin_interfaces/Time valid_until  # GraspDecision.valid_until 原值；过期不得执行（心跳不续签）
builtin_interfaces/Time model_stamp   # 融合模型时刻；拒绝未来戳，valid_until=0 时回退此戳的新鲜度窗
bool allowed                          # 套入/剪切许可（动态预算派生；原值数学）
float64 radial_margin_m               # 静态径向余量 [m]，初步查看标记，只记录。接触时弹性会动，目前不拦套入
float64 axial_margin_m                # 轴向余量 [m]；记录给机械，负值不拦剪切
string model_fingerprint              # 当前填 model_revision；非空时须与绑定模型修订一致
string reason                         # 许可/拒绝原因（稳定短语）
