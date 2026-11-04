# 感知与推理

## 处理链

```text
RGB-D + CameraInfo + TF
  → decode / frame-age gate / motion gate
  → bounded latest-frame worker
  → YOLO detector
  → MobileSAM segmenter (cached encoder fallback)
  → geometry / identity / lock set
  → observations + masks + debug
```

## 关键实现

- `peach_scene_perception`：LifecycleNode + SingleThreadedExecutor + bounded worker。
- `peach_vision_core`：纯核 geometry/runtime/domain，零 ROS import。
- 模型：`peach_scene_perception/model/{best.pt,mobile_sam.pt}`；配置使用 `$(find-pkg-share peach_scene_perception)`。
- 参数：`config/scene_perception.yaml`；scene_perception、target_reconstruction 与 scene_obstacles 均已迁移 GPL。
- 性能：最终离线基准 13.74 FPS，p50 71.87 ms，p95 78.90 ms（8 帧 + 2 warmup）；真实相机 20 s 验证中 camera/depth/joints 均 fresh。
- 失败码：`FailureCode` 已有 supervisor 侧枚举化；感知/重建纯核的裸数字迁移仍待完成。

## 已知边界

- 真实相机 + RViz 测试中 `rviz_alive=true`，scene 干净退出。
- `move_group` shutdown `-11` 是外部 MoveIt 问题，不属于感知模块。
