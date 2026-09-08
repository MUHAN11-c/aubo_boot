# BagFitting 数组。感知 diagnostics 与重建 refined_diagnostics 共用。
# 无效标量在元素内填 -1。闩锁话题无数据时发空数组盖旧值。
# 话题见 BagFitting.msg 文件头。

std_msgs/Header header                 # 与元素同时刻
BagFitting[] fittings                  # 本帧/本次精化诊断；空=无数据
