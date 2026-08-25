# BagGraspCandidate 数组。感知 initial_pose 与重建 refined_pose 共用本类型。
# 闩锁话题在无有效几何时发空数组，覆盖陈旧候选，后启动订阅者不要当「上一颗还在」。

std_msgs/Header header
BagGraspCandidate[] candidates
