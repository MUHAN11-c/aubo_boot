# 感知与推理

## 处理链

```text
RGB-D + CameraInfo + TF
  → decode / frame-age gate / motion gate
  → bounded latest-frame worker
  → YOLO detector
  → FastSAM（默认）/ SAM（显式权重分支）
  → geometry / identity / lock set
  → observations + masks + debug
```

## 关键实现

- `peach_scene_perception`：LifecycleNode + SingleThreadedExecutor + bounded worker。
- `peach_vision_core`：纯核 geometry/runtime/domain，零 ROS import。
- 模型：`peach_scene_perception/model/{best.pt,FastSAM-s.pt,mobile_sam.pt}`；配置使用 `$(find-pkg-share peach_scene_perception)`。
- 参数：`config/scene_perception.yaml`；scene_perception、target_reconstruction 与 scene_obstacles 均已迁移 GPL。
- 历史性能（2026-10-27）：离线基准 13.74 FPS，p50 71.87 ms，p95 78.90 ms（8 帧 + 2 warmup）；真实相机 20 s 验证中 camera/depth/joints 均 fresh。
- 失败码：ROS 边界以 `FailureCode` 为准；零 ROS 纯核仍有附常量语义的数值输出，节点会再次组装最终能力，见 [错误码说明](08-failure-codes.md)。

## 已知边界

以下为 2026-10-27 历史验证，不代表本次重新运行：

- 真实相机 + RViz 测试中 `rviz_alive=true`，scene 干净退出。
- 该轮 `move_group` shutdown `-11` 已记录；当前 bringup 提供仅作用于 move_group 的退出兼容参数，仍须逐轮检查退出结果。

身份注册表的 position/axis/diameter EMA 用于关联与记忆；当前候选保持当次曝光几何，不拿身份 EMA 覆写候选轴。重建/臂侧的阶段能力和身份/时效门见各包文档，不能将 margin 数值直接当作唯一许可。
