# BuildTargetModel 终局模型。动作 Result.model 携带；shape 来自重建 ShapeHypothesis。

std_msgs/Header header
string target_id
uint32 scene_epoch                 # 绑定的场景世代；与 BeginScene 对齐
peach_interfaces/ShapeHypothesis shape
peach_interfaces/TargetQuality quality
uint32 view_count                  # finalize 时已采合格视角数
bool accepted                      # 是否过质量门并接受本模型
string message                     # 拒绝/降级原因；接受时可为空
