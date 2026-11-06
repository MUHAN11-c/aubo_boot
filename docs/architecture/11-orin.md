# Jetson Orin NX 部署档案

## 状态

- 历史基准（2026-10-27）：x86 + 真实帧 **13.74 FPS / p50 71.87 ms / p95 78.90 ms**（8 帧 + 2 warmup）。
- Orin NX 现场数据尚未采集；不在文档中伪造 FPS。
- TensorRT engine 需在目标机生成；当前代码只提供可选的 `yolo_half`，不把 FP16 当默认。

## profile

[scene_perception_orin.yaml](../../src/peach/peach_scene_perception/config/scene_perception_orin.yaml) 是可选 overlay：

- `yolo_half: true`（仅已验证 FP16 的设备）；
- 关闭 debug/mask/detection cloud 发布；
- `sync_queue_size: 2`，降低队列延迟；
- 保留 4 帧收齐窗，避免改单果周期语义。

```bash
ros2 launch peach_scene_perception scene_perception.launch.py \
  params_overlay:=/path/to/scene_perception_orin.yaml \
  autostart:=true
```

## 验收

1. 目标机跑 `benchmark_perception.py`，记录 FPS/p50/p95。
2. 真实相机 + mock 臂 15 s soak，camera/depth/joints fresh。
3. 校准身份单轮 Build→GraspDecision→预抓取通过。
4. 若 FP16 精度/速度不达标，回退 `yolo_half: false`，不要改默认产品 YAML。
