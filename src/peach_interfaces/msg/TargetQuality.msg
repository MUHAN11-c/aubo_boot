# 重建/建模质量档。BuildTargetModel.Result.quality_level 与 TargetModel.quality.level 同枚举。

uint8 LOW=0                  # 不够接触（视角少、RMSE 大）
uint8 MEDIUM=1               # 可观察/可预抓取，套入仍看 GraspDecision.allowed
uint8 HIGH=2                 # 高质量融合
uint8 HIGH_CONFIDENCE=3      # 高置信（机位与内点均充足）

uint8 level                  # LOW..HIGH_CONFIDENCE
float32 score                # 连续分，通常 0..1；未评则 0
string reason                # 档位原因（视角不够、RMSE 过高等）
