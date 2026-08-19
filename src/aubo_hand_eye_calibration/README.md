# aubo_hand_eye_calibration

眼在手上标定：采集标定板、解外参、落盘，并由 `extrinsics_publisher` 发静态 TF。

感知与重建用**精确时间戳**查 `base_link ← camera`；外参 TF 是这条链的一部分。标定坏了，世界系身份和 ICP 初值都会偏。

## 节点

- `extrinsics_publisher`：读 yaml，发静态 TF；可 `reload`
- 标定 server：动作/服务跑采集与求解

bringup 默认可开外参发布。全标定流程单独 launch，不要在日常采摘里自动跑。
