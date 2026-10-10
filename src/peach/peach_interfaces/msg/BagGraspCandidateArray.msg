# BagGraspCandidate 数组。感知 initial_pose 与重建 refined_pose 共用。
# 闩锁话题在无有效几何时发空数组，覆盖陈旧候选；后启动订阅者不要当「上一颗还在」。
# 话题见 BagGraspCandidate.msg 文件头。

std_msgs/Header header                 # 与元素 header 同时刻；无候选时仍发布以盖旧值
BagGraspCandidate[] candidates         # 本帧/本次融合的候选；空=无效
