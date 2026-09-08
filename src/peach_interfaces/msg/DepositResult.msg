# ExecuteTarget 卸果结果（技能写入 Result.deposit）。
# 卸果站关节未标定（M8）时系统最高到完成采摘、不卸果：deposited=false。

bool deposited                 # 已放到卸果站
string reason                  # 失败/跳过原因（pending_m8_unload_pose 等）；成功可空
